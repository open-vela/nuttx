/****************************************************************************
 * arch/risc-v/src/common/espressif/esp_gpio.c
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

/* Config */

#include <nuttx/config.h>

/* Libc */

#include <assert.h>
#include <debug.h>
#include <stdint.h>
#include <sys/types.h>

/* NuttX */

#include <arch/irq.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

/* Arch */

#include "riscv_internal.h"

#include "esp_gpio.h"
#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
#  include "esp_irq.h"
#endif

/* HAL */

#include "esp_rom_gpio.h"
#include "hal/gpio_hal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static gpio_hal_context_t g_gpio_hal =
{
  .dev = GPIO_HAL_GET_HW(GPIO_PORT_0)
};

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
static int g_gpio_cpuint;

struct esp_gpio_irqhandler_s
{
  xcpt_t handler;
  void *arg;
};

static struct esp_gpio_irqhandler_s
  g_gpio_irqhandlers[SOC_GPIO_PIN_COUNT];

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpio_dispatch
 *
 * Description:
 *   Second level dispatch for GPIO interrupt handling.
 *
 * Input Parameters:
 *   irq           - GPIO IRQ number.
 *   status        - Value from the GPIO interrupt status clear register.
 *   regs          - Saved CPU context.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
static void gpio_dispatch(int irq, uint32_t status, uint32_t *regs)
{
  int i;

  /* Check set bits in the status register */

  while ((i = __builtin_ffs(status)) > 0)
    {
      int gpioirq = irq + i - 1;
      int pin = ESP_IRQ2PIN(gpioirq);
      xcpt_t handler = g_gpio_irqhandlers[pin].handler;
      void *arg = g_gpio_irqhandlers[pin].arg;

      /* GPIO pin IRQs are second-level IRQs.  They cannot be placed in the
       * 17-entry RISC-V minimal vector table, so dispatch them through the
       * dedicated GPIO handler table instead of irq_dispatch().
       */

      if (handler != NULL)
        {
          handler(gpioirq, regs, arg);
        }

      status >>= i;
    }
}
#endif

/****************************************************************************
 * Name: gpio_interrupt
 *
 * Description:
 *   GPIO interrupt handler.
 *
 * Input Parameters:
 *   irq           - Identifier of the interrupt request.
 *   context       - Context data from the ISR.
 *   arg           - Opaque pointer to the internal driver state structure.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
static int gpio_interrupt(int irq, void *context, void *arg)
{
  int i;
  uint32_t status;
#ifdef CONFIG_ESPRESSIF_ESP32P4
  uint32_t status_high;
#endif
  uint32_t intr_bitmask;
  int cpu = this_cpu();

  /* Read the lower GPIO interrupt status */

  gpio_hal_get_intr_status(&g_gpio_hal, cpu, &status);
  intr_bitmask = status;

  while ((i = __builtin_ffs(intr_bitmask)) > 0)
    {
      gpio_hal_clear_intr_status_bit(&g_gpio_hal, (i - 1));
      intr_bitmask >>= i;
    }

  /* Dispatch pending interrupts in the lower GPIO status register */

  gpio_dispatch(ESP_FIRST_GPIOIRQ, status, (uint32_t *)context);
#ifdef CONFIG_ESPRESSIF_ESP32P4
  /* ESP32-P4 has a second status register for GPIO32 and above.  The
   * interrupt source is shared with the lower GPIO status, but the two
   * status registers must be read, cleared, and dispatched separately.
   */

  gpio_hal_get_intr_status_high(&g_gpio_hal, cpu, &status_high);
  intr_bitmask = status_high;

  while ((i = __builtin_ffs(intr_bitmask)) > 0)
    {
      gpio_hal_clear_intr_status_bit(&g_gpio_hal, 32 + (i - 1));
      intr_bitmask >>= i;
    }

  gpio_dispatch(ESP_FIRST_GPIOIRQ + 32, status_high,
                (uint32_t *)context);
#endif

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_configgpio
 *
 * Description:
 *   Configure a GPIO pin based on encoded pin attributes.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *   attr          - Attributes to be configured for the selected GPIO pin.
 *                   The following attributes are accepted:
 *                   - Direction (OUTPUT or INPUT)
 *                   - Pull (PULLUP, PULLDOWN or OPENDRAIN)
 *                   - Function (if not provided, assume function GPIO by
 *                     default)
 *                   - Drive strength (if not provided, assume DRIVE_2 by
 *                     default)
 *
 * Returned Value:
 *   Zero (OK) on success, or -1 (ERROR) in case of failure.
 *
 ****************************************************************************/

int esp_configgpio(int pin, gpio_pinattr_t attr)
{
  DEBUGASSERT(pin >= 0 && pin < SOC_GPIO_PIN_COUNT);

  /* Handle input pins */

  if ((attr & INPUT) != 0)
    {
      gpio_hal_input_enable(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_input_disable(&g_gpio_hal, pin);
    }

  if ((attr & OPEN_DRAIN) != 0)
    {
      gpio_hal_od_enable(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_od_disable(&g_gpio_hal, pin);
    }

  if ((attr & OUTPUT) != 0)
    {
      gpio_hal_output_enable(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_output_disable(&g_gpio_hal, pin);
    }

  if ((attr & PULLUP) != 0)
    {
      gpio_hal_pullup_en(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_pullup_dis(&g_gpio_hal, pin);
    }

  if ((attr & PULLDOWN) != 0)
    {
      gpio_hal_pulldown_en(&g_gpio_hal, pin);
    }
  else
    {
      gpio_hal_pulldown_dis(&g_gpio_hal, pin);
    }

  if ((attr & DRIVE_MASK) != 0)
    {
      uint32_t val = ((attr & DRIVE_MASK) >> DRIVE_SHIFT) - 1;
      gpio_hal_set_drive_capability(&g_gpio_hal, pin, val);
    }
  else
    {
      gpio_hal_set_drive_capability(&g_gpio_hal, pin,
                                    GPIO_DRIVE_CAP_DEFAULT);
    }

  if ((attr & FUNCTION_MASK) != 0)
    {
      uint32_t val = ((attr & FUNCTION_MASK) >> FUNCTION_SHIFT) - 1;
#ifdef CONFIG_ESPRESSIF_ESP32P4
      gpio_hal_func_sel(&g_gpio_hal, pin, val);
#else
      gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], val);
#endif
    }
  else
    {
#ifdef CONFIG_ESPRESSIF_ESP32P4
      gpio_hal_func_sel(&g_gpio_hal, pin, PIN_FUNC_GPIO);
#else
      gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[pin], PIN_FUNC_GPIO);
#endif
    }

  return OK;
}

/****************************************************************************
 * Name: esp_gpioirqattach
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
int esp_gpioirqattach(int irq, xcpt_t isr, void *arg)
{
  int pin;
  irqstate_t flags;

  DEBUGASSERT(irq >= ESP_FIRST_GPIOIRQ && irq <= ESP_LAST_GPIOIRQ);
  pin = ESP_IRQ2PIN(irq);

  flags = enter_critical_section();
  g_gpio_irqhandlers[pin].handler = isr;
  g_gpio_irqhandlers[pin].arg = arg;
  leave_critical_section(flags);

  return OK;
}
#endif

/****************************************************************************
 * Name: esp_gpio_matrix_in
 *
 * Description:
 *   Set GPIO input to a signal.
 *   NOTE: one GPIO can receive inputs from several signals.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *                   - If pin == 0x3c, cancel input to the signal, input 0
 *                     to signal.
 *                   - If pin == 0x3a, input nothing to signal.
 *                   - If pin == 0x38, cancel input to the signal, input 1
 *                     to signal.
 *   signal_idx    - Signal index.
 *   inv           - Flag indicating whether the signal is inverted.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpio_matrix_in(uint32_t pin, uint32_t signal_idx, bool inv)
{
  esp_rom_gpio_connect_in_signal(pin, signal_idx, inv);
}

/****************************************************************************
 * Name: esp_gpio_matrix_out
 *
 * Description:
 *   Set signal output to GPIO.
 *   NOTE: one signal can output to several GPIOs.
 *
 * Input Parameters:
 *   pin           - GPIO pin to be configured.
 *   signal_idx    - Signal index.
 *                   - If signal_idx == 0x100, cancel output to the GPIO.
 *   out_inv       - Flag indicating whether the signal output is inverted.
 *   oen_inv       - Flag indicating whether the signal output enable is
 *                   inverted.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpio_matrix_out(uint32_t pin, uint32_t signal_idx, bool out_inv,
                         bool oen_inv)
{
  esp_rom_gpio_connect_out_signal(pin, signal_idx, out_inv, oen_inv);
}

/****************************************************************************
 * Name: esp_gpiowrite
 *
 * Description:
 *   Write one or zero to the selected GPIO pin
 *
 * Input Parameters:
 *   pin           - GPIO pin to be modified.
 *   value         - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void esp_gpiowrite(int pin, bool value)
{
  DEBUGASSERT(pin >= 0 && pin <= SOC_GPIO_PIN_COUNT);

  gpio_hal_set_level(&g_gpio_hal, pin, value);
}

/****************************************************************************
 * Name: esp_gpioread
 *
 * Description:
 *   Read one or zero from the selected GPIO pin
 *
 * Input Parameters:
 *   pin           - GPIO pin to be read.
 *
 * Returned Value:
 *   The boolean representation of the input value (true/false).
 *
 ****************************************************************************/

bool esp_gpioread(int pin)
{
  DEBUGASSERT(pin >= 0 && pin <= SOC_GPIO_PIN_COUNT);

  return gpio_hal_get_level(&g_gpio_hal, pin) != 0;
}

/****************************************************************************
 * Name: esp_gpioirqinitialize
 *
 * Description:
 *   Initialize logic to support a second level of interrupt decoding for
 *   GPIO pins.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
void esp_gpioirqinitialize(void)
{
  /* Setup the GPIO interrupt. */

  g_gpio_cpuint = esp_setup_irq(GPIO_INTR0_SOURCE,
                                ESP_IRQ_PRIORITY_DEFAULT,
                                ESP_IRQ_TRIGGER_LEVEL,
                                gpio_interrupt,
                                NULL);
  DEBUGASSERT(g_gpio_cpuint >= 0);

  /* Keep the CPU interrupt disabled until a GPIO client has configured and
   * enabled a pin interrupt.  At this stage the GPIO status registers and
   * the second-level NuttX handlers are not ready yet.
   */
}
#endif

/****************************************************************************
 * Name: esp_gpioirqenable
 *
 * Description:
 *   Enable the interrupt for specified GPIO IRQ
 *
 * Input Parameters:
 *   irq           - GPIO IRQ number to be enabled.
 *   intrtype      - Interrupt type to be enabled.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
void esp_gpioirqenable(int irq, gpio_intrtype_t intrtype)
{
  uintptr_t regaddr;
  uint32_t regval;
  int pin;
  int cpu;

  DEBUGASSERT(irq >= ESP_FIRST_GPIOIRQ && irq <= ESP_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP_IRQ2PIN(irq);

  /* Disable the GPIO interrupt during the configuration. */

  up_disable_irq(ESP_IRQ_GPIO_INTR0);

  /* Enable interrupt for this pin on the current core */

  cpu = this_cpu();
  gpio_hal_set_intr_type(&g_gpio_hal, pin, intrtype);
  gpio_hal_intr_enable_on_core(&g_gpio_hal, pin, cpu);

  /* Configuration done. Re-enable the GPIO interrupt. */

  up_enable_irq(ESP_IRQ_GPIO_INTR0);
}
#endif

/****************************************************************************
 * Name: esp_gpioirqdisable
 *
 * Description:
 *   Disable the interrupt for specified GPIO IRQ
 *
 * Input Parameters:
 *   irq           - GPIO IRQ number to be disabled.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
void esp_gpioirqdisable(int irq)
{
  uintptr_t regaddr;
  uint32_t regval;
  int pin;

  DEBUGASSERT(irq >= ESP_FIRST_GPIOIRQ && irq <= ESP_LAST_GPIOIRQ);

  /* Convert the IRQ number to a pin number */

  pin = ESP_IRQ2PIN(irq);

  /* Disable the GPIO interrupt during the configuration. */

  up_disable_irq(ESP_IRQ_GPIO_INTR0);

  /* Disable the interrupt for this pin */

  gpio_hal_intr_disable(&g_gpio_hal, pin);

  /* Configuration done. Re-enable the GPIO interrupt. */

  up_enable_irq(ESP_IRQ_GPIO_INTR0);
}
#endif

/****************************************************************************
 * Name: esp_gpioirqclear
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_GPIO_IRQ
void esp_gpioirqclear(int irq)
{
  int pin;

  DEBUGASSERT(irq >= ESP_FIRST_GPIOIRQ && irq <= ESP_LAST_GPIOIRQ);

  pin = ESP_IRQ2PIN(irq);
  gpio_hal_clear_intr_status_bit(&g_gpio_hal, pin);
}
#endif
