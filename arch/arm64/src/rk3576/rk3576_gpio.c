/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_gpio.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_gpio.h"
#include "rk3576_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of GPIO pins supported */

#define RK3576_GPIO_NPINS          (RK3576_GPIO_BANKS * RK3576_PINS_PER_BANK)

/* Maximum number of GPIO interrupts */

#define RK3576_GPIO_NIRQS          RK3576_GPIO_NPINS

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-pin interrupt state */

struct rk3576_gpio_irq_s
{
  xcpt_t handler;               /* Interrupt handler */
  void   *arg;                  /* Handler argument */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* GPIO bank base addresses */

const uint32_t g_gpio_base[RK3576_GPIO_BANKS] =
{
  RK3576_GPIO0_ADDR,            /* GPIO0: 0xff750000 */
  RK3576_GPIO1_ADDR,            /* GPIO1: 0xff760000 */
  RK3576_GPIO2_ADDR,            /* GPIO2: 0xff770000 */
  RK3576_GPIO3_ADDR,            /* GPIO3: 0xff780000 */
  RK3576_GPIO4_ADDR,            /* GPIO4: 0xff790000 */
};

/* Per-pin interrupt handlers */

static struct rk3576_gpio_irq_s g_gpio_irqs[RK3576_GPIO_NIRQS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_pin_validate
 *
 * Description:
 *   Validate pin number.
 *
 ****************************************************************************/

static inline int rk3576_pin_validate(unsigned int pin)
{
  if (pin >= RK3576_GPIO_NPINS)
    {
      gpioerr("GPIO: invalid pin %u\n", pin);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_gpio_get_base
 *
 * Description:
 *   Get the base address for a GPIO bank.
 *
 ****************************************************************************/

static inline uint32_t rk3576_gpio_get_base(unsigned int bank)
{
  DEBUGASSERT(bank < RK3576_GPIO_BANKS);
  return g_gpio_base[bank];
}

/****************************************************************************
 * Name: rk3576_gpio_get_port_reg
 *
 * Description:
 *   Get the external port register address for a specific port.
 *
 ****************************************************************************/

static inline uint32_t rk3576_gpio_get_port_reg(unsigned int bank,
                                                 unsigned int port)
{
  return rk3576_gpio_get_base(bank) + GPIO_EXT_PORTA + (port * 4);
}

/****************************************************************************
 * Name: rk3576_gpio_port_read
 *
 * Description:
 *   Read all 8 pins of a port.
 *
 ****************************************************************************/

static uint8_t rk3576_gpio_port_read(unsigned int bank, unsigned int port)
{
  uint32_t reg = rk3576_gpio_get_port_reg(bank, port);
  return (uint8_t)(getreg32(reg) & 0xff);
}

/****************************************************************************
 * Name: rk3576_gpio_irq_dispatch
 *
 * Description:
 *   Dispatch GPIO interrupt to per-pin handlers.
 *   Called from the GPIO bank interrupt handler.
 *
 ****************************************************************************/

void rk3576_gpio_irq_dispatch(int bank, int irq, void *context)
{
  uint32_t base = rk3576_gpio_get_base(bank);
  uint32_t status;
  int pin;

  /* Read and clear interrupt status */

  status = getreg32(base + GPIO_INT_STATUS);
  if (status == 0)
    {
      return;
    }

  /* Clear all pending interrupts */

  putreg32(status, base + GPIO_PORT_EOI);

  /* Dispatch per-pin handlers */

  for (pin = 0; pin < RK3576_PINS_PER_BANK; pin++)
    {
      if (status & (1u << pin))
        {
          int global_pin = bank * RK3576_PINS_PER_BANK + pin;

          if (g_gpio_irqs[global_pin].handler != NULL)
            {
              g_gpio_irqs[global_pin].handler(irq, context,
                                               g_gpio_irqs[global_pin].arg);
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_gpio_init
 *
 * Description:
 *   Initialize the GPIO subsystem. Called from arm64_chip_boot().
 *
 ****************************************************************************/

void rk3576_gpio_init(void)
{
  int bank;

  /* Clear all interrupt handlers */

  memset(g_gpio_irqs, 0, sizeof(g_gpio_irqs));

  /* Disable all interrupts on all banks */

  for (bank = 0; bank < RK3576_GPIO_BANKS; bank++)
    {
      uint32_t base = rk3576_gpio_get_base(bank);
      putreg32(0x00000000, base + GPIO_INT_EN);
      putreg32(0xffffffff, base + GPIO_INT_MASK);
      putreg32(0x00000000, base + GPIO_DEBOUNCE);
    }

  gpioinfo("GPIO: initialized %d banks, %d pins\n",
           RK3576_GPIO_BANKS, RK3576_GPIO_NPINS);
}

/****************************************************************************
 * Name: rk3576_gpio_direction_input
 *
 * Description:
 *   Configure a GPIO pin as input.
 *
 ****************************************************************************/

void rk3576_gpio_direction_input(unsigned int pin)
{
  unsigned int bank;
  unsigned int offset;
  uint32_t base;
  irqstate_t flags;
  uint32_t reg;

  if (rk3576_pin_validate(pin) < 0)
    {
      return;
    }

  bank = RK3576_PIN_BANK(pin);
  offset = RK3576_PIN_OFFSET(pin);
  base = rk3576_gpio_get_base(bank);

  flags = up_irq_save();

  /* Clear direction bit (input = 0) */

  reg = getreg32(base + GPIO_SWPORTA_DDR);
  reg &= ~(1u << offset);
  putreg32(reg, base + GPIO_SWPORTA_DDR);

  up_irq_restore(flags);

  gpioinfo("GPIO%d pin %u: input\n", bank, offset);
}

/****************************************************************************
 * Name: rk3576_gpio_direction_output
 *
 * Description:
 *   Configure a GPIO pin as output with initial value.
 *
 ****************************************************************************/

void rk3576_gpio_direction_output(unsigned int pin, int value)
{
  unsigned int bank;
  unsigned int offset;
  uint32_t base;
  irqstate_t flags;
  uint32_t reg;

  if (rk3576_pin_validate(pin) < 0)
    {
      return;
    }

  bank = RK3576_PIN_BANK(pin);
  offset = RK3576_PIN_OFFSET(pin);
  base = rk3576_gpio_get_base(bank);

  flags = up_irq_save();

  /* Set output value first (avoid glitch) */

  reg = getreg32(base + GPIO_SWPORTA_DR);
  if (value)
    {
      reg |= (1u << offset);
    }
  else
    {
      reg &= ~(1u << offset);
    }

  putreg32(reg, base + GPIO_SWPORTA_DR);

  /* Set direction bit (output = 1) */

  reg = getreg32(base + GPIO_SWPORTA_DDR);
  reg |= (1u << offset);
  putreg32(reg, base + GPIO_SWPORTA_DDR);

  up_irq_restore(flags);

  gpioinfo("GPIO%d pin %u: output %d\n", bank, offset, value);
}

/****************************************************************************
 * Name: rk3576_gpio_get_value
 *
 * Description:
 *   Read the current value of a GPIO pin.
 *
 ****************************************************************************/

int rk3576_gpio_get_value(unsigned int pin)
{
  unsigned int bank;
  unsigned int port;
  unsigned int offset;
  uint8_t portval;

  if (rk3576_pin_validate(pin) < 0)
    {
      return -EINVAL;
    }

  bank = RK3576_PIN_BANK(pin);
  port = RK3576_PIN_PORT(pin);
  offset = RK3576_PIN_OFFSET(pin);

  portval = rk3576_gpio_port_read(bank, port);
  return (portval >> offset) & 1;
}

/****************************************************************************
 * Name: rk3576_gpio_set_value
 *
 * Description:
 *   Set the output value of a GPIO pin.
 *
 ****************************************************************************/

void rk3576_gpio_set_value(unsigned int pin, int value)
{
  unsigned int bank;
  unsigned int offset;
  uint32_t base;
  irqstate_t flags;
  uint32_t reg;

  if (rk3576_pin_validate(pin) < 0)
    {
      return;
    }

  bank = RK3576_PIN_BANK(pin);
  offset = RK3576_PIN_OFFSET(pin);
  base = rk3576_gpio_get_base(bank);

  flags = up_irq_save();

  reg = getreg32(base + GPIO_SWPORTA_DR);
  if (value)
    {
      reg |= (1u << offset);
    }
  else
    {
      reg &= ~(1u << offset);
    }

  putreg32(reg, base + GPIO_SWPORTA_DR);

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: rk3576_gpio_set_debounce
 *
 * Description:
 *   Enable or disable debounce on a GPIO pin.
 *
 ****************************************************************************/

void rk3576_gpio_set_debounce(unsigned int pin, bool enable)
{
  unsigned int bank;
  unsigned int offset;
  uint32_t base;
  irqstate_t flags;
  uint32_t reg;

  if (rk3576_pin_validate(pin) < 0)
    {
      return;
    }

  bank = RK3576_PIN_BANK(pin);
  offset = RK3576_PIN_OFFSET(pin);
  base = rk3576_gpio_get_base(bank);

  flags = up_irq_save();

  reg = getreg32(base + GPIO_DEBOUNCE);
  if (enable)
    {
      reg |= (1u << offset);
    }
  else
    {
      reg &= ~(1u << offset);
    }

  putreg32(reg, base + GPIO_DEBOUNCE);

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: rk3576_gpio_irq_attach
 *
 * Description:
 *   Attach an interrupt handler to a GPIO pin.
 *
 ****************************************************************************/

int rk3576_gpio_irq_attach(unsigned int pin, xcpt_t handler, void *arg)
{
  irqstate_t flags;
  int ret;

  ret = rk3576_pin_validate(pin);
  if (ret < 0)
    {
      return ret;
    }

  flags = up_irq_save();

  g_gpio_irqs[pin].handler = handler;
  g_gpio_irqs[pin].arg = arg;

  up_irq_restore(flags);

  gpioinfo("GPIO pin %u: attached handler %p\n", pin, handler);
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpio_irq_detach
 *
 * Description:
 *   Detach the interrupt handler from a GPIO pin.
 *
 ****************************************************************************/

void rk3576_gpio_irq_detach(unsigned int pin)
{
  irqstate_t flags;

  if (rk3576_pin_validate(pin) < 0)
    {
      return;
    }

  /* Disable interrupt first */

  rk3576_gpio_irq_disable(pin);

  flags = up_irq_save();

  g_gpio_irqs[pin].handler = NULL;
  g_gpio_irqs[pin].arg = NULL;

  up_irq_restore(flags);
}

/****************************************************************************
 * Name: rk3576_gpio_irq_settype
 *
 * Description:
 *   Set the interrupt trigger type for a GPIO pin.
 *
 ****************************************************************************/

int rk3576_gpio_irq_settype(unsigned int pin, unsigned int type)
{
  unsigned int bank;
  unsigned int offset;
  uint32_t base;
  irqstate_t flags;
  uint32_t int_type;
  uint32_t int_polarity;
  int ret;

  ret = rk3576_pin_validate(pin);
  if (ret < 0)
    {
      return ret;
    }

  bank = RK3576_PIN_BANK(pin);
  offset = RK3576_PIN_OFFSET(pin);
  base = rk3576_gpio_get_base(bank);

  flags = up_irq_save();

  /* Disable interrupt during configuration */

  putreg32(0, base + GPIO_INT_EN);

  /* Read current settings */

  int_type = getreg32(base + GPIO_INT_TYPE);
  int_polarity = getreg32(base + GPIO_INT_POLARITY);

  switch (type)
    {
      case GPIO_IRQTYPE_RISING:
        int_type |= (1u << offset);        /* Edge triggered */
        int_polarity |= (1u << offset);    /* Rising / high */
        break;

      case GPIO_IRQTYPE_FALLING:
        int_type |= (1u << offset);        /* Edge triggered */
        int_polarity &= ~(1u << offset);   /* Falling / low */
        break;

      case GPIO_IRQTYPE_BOTHEDGE:
        /* Use INT_TYPE bit = 0 (level) and handle both in handler */
        int_type &= ~(1u << offset);
        int_polarity |= (1u << offset);
        break;

      case GPIO_IRQTYPE_HIGHLEVEL:
        int_type &= ~(1u << offset);       /* Level triggered */
        int_polarity |= (1u << offset);    /* High level */
        break;

      case GPIO_IRQTYPE_LOWLEVEL:
        int_type &= ~(1u << offset);       /* Level triggered */
        int_polarity &= ~(1u << offset);   /* Low level */
        break;

      case GPIO_IRQTYPE_NONE:
      default:
        break;
    }

  putreg32(int_type, base + GPIO_INT_TYPE);
  putreg32(int_polarity, base + GPIO_INT_POLARITY);

  up_irq_restore(flags);

  gpioinfo("GPIO pin %u: irq type %u\n", pin, type);
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpio_irq_enable
 *
 * Description:
 *   Enable interrupt on a GPIO pin.
 *
 ****************************************************************************/

int rk3576_gpio_irq_enable(unsigned int pin)
{
  unsigned int bank;
  unsigned int offset;
  uint32_t base;
  irqstate_t flags;
  uint32_t reg;
  int ret;

  ret = rk3576_pin_validate(pin);
  if (ret < 0)
    {
      return ret;
    }

  bank = RK3576_PIN_BANK(pin);
  offset = RK3576_PIN_OFFSET(pin);
  base = rk3576_gpio_get_base(bank);

  flags = up_irq_save();

  /* Unmask the interrupt */

  reg = getreg32(base + GPIO_INT_MASK);
  reg &= ~(1u << offset);
  putreg32(reg, base + GPIO_INT_MASK);

  /* Enable the interrupt */

  reg = getreg32(base + GPIO_INT_EN);
  reg |= (1u << offset);
  putreg32(reg, base + GPIO_INT_EN);

  up_irq_restore(flags);

  gpioinfo("GPIO pin %u: irq enabled\n", pin);
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpio_irq_disable
 *
 * Description:
 *   Disable interrupt on a GPIO pin.
 *
 ****************************************************************************/

int rk3576_gpio_irq_disable(unsigned int pin)
{
  unsigned int bank;
  unsigned int offset;
  uint32_t base;
  irqstate_t flags;
  uint32_t reg;
  int ret;

  ret = rk3576_pin_validate(pin);
  if (ret < 0)
    {
      return ret;
    }

  bank = RK3576_PIN_BANK(pin);
  offset = RK3576_PIN_OFFSET(pin);
  base = rk3576_gpio_get_base(bank);

  flags = up_irq_save();

  /* Disable the interrupt */

  reg = getreg32(base + GPIO_INT_EN);
  reg &= ~(1u << offset);
  putreg32(reg, base + GPIO_INT_EN);

  /* Mask the interrupt */

  reg = getreg32(base + GPIO_INT_MASK);
  reg |= (1u << offset);
  putreg32(reg, base + GPIO_INT_MASK);

  /* Clear any pending status */

  putreg32(1u << offset, base + GPIO_PORT_EOI);

  up_irq_restore(flags);

  gpioinfo("GPIO pin %u: irq disabled\n", pin);
  return OK;
}

/****************************************************************************
 * Name: rk3576_gpio_set_function
 *
 * Description:
 *   Set pin function (alternate function). This is a placeholder -
 *   actual IOMUX register configuration depends on the specific pin
 *   and function mapping. For now, this sets the pin to GPIO mode.
 *
 ****************************************************************************/

void rk3576_gpio_set_function(unsigned int pin, unsigned int function)
{
  /* For now, just log the request.
   * Full IOMUX configuration will be added when the TRM IOMUX
   * register details are fully mapped.
   */

  gpioinfo("GPIO pin %u: function %u (IOMUX not yet implemented)\n",
           pin, function);
}

/****************************************************************************
 * Name: rk3576_gpio_set_pull
 *
 * Description:
 *   Set pull-up/pull-down for a GPIO pin.
 *   Placeholder - actual IOMUX pull register configuration
 *   depends on the specific pin.
 *
 ****************************************************************************/

void rk3576_gpio_set_pull(unsigned int pin, unsigned int pull)
{
  gpioinfo("GPIO pin %u: pull %u (IOMUX not yet implemented)\n",
           pin, pull);
}

/****************************************************************************
 * Name: rk3576_gpio_set_drive
 *
 * Description:
 *   Set drive strength for a GPIO pin.
 *   Placeholder - actual IOMUX drive register configuration
 *   depends on the specific pin.
 *
 ****************************************************************************/

void rk3576_gpio_set_drive(unsigned int pin, unsigned int drive)
{
  gpioinfo("GPIO pin %u: drive %u (IOMUX not yet implemented)\n",
           pin, drive);
}
