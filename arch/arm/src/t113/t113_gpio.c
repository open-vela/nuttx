/****************************************************************************
 * arch/arm/src/t113/t113_gpio.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#ifdef CONFIG_T113_AMP
#  include <nuttx/hwspinlock/hwspinlock.h>
#  include "t113_hwspinlock.h"
#else
#  include <nuttx/spinlock.h>
#endif

#include "arm_internal.h"
#include "hardware/t113_gpio.h"
#include "t113_gpio.h"

#ifdef CONFIG_T113_GPIO_IRQ
#  include <nuttx/arch.h>
#  include <nuttx/irq.h>
#  include <nuttx/spinlock.h>
#  include <arch/t113/irq.h>
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_T113_AMP
static FAR struct hwspinlock_dev_s *g_gpio_hwlock;
#else
static spinlock_t g_gpio_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline irqstate_t gpio_lock(void)
{
#ifdef CONFIG_T113_AMP
  if (g_gpio_hwlock != NULL)
    {
      return hwspin_lock_irqsave(g_gpio_hwlock);
    }

  return up_irq_save();
#else
  return spin_lock_irqsave(&g_gpio_lock);
#endif
}

static inline void gpio_unlock(irqstate_t flags)
{
#ifdef CONFIG_T113_AMP
  if (g_gpio_hwlock != NULL)
    {
      hwspin_unlock_restore(g_gpio_hwlock, flags);
      return;
    }

  up_irq_restore(flags);
#else
  spin_unlock_irqrestore(&g_gpio_lock, flags);
#endif
}

static inline void gpio_rmw(uint32_t reg, uint32_t clr, uint32_t set)
{
  irqstate_t flags;
  uint32_t   val;

  flags = gpio_lock();
  val = getreg32(reg);
  val &= ~clr;
  val |= set;
  putreg32(val, reg);
  gpio_unlock(flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_gpio_init(void)
{
#ifdef CONFIG_T113_AMP
  if (g_gpio_hwlock == NULL)
    {
      g_gpio_hwlock = t113_hwspinlock_get(T113_HWLOCK_ID_GPIO);
    }
#endif
}

/****************************************************************************
 * Name: t113_gpio_config
 *
 * Description:
 *   Configure a GPIO pin using the encoded pin descriptor from
 *   t113_gpio.h.  Sets function, pull, and drive level.
 *
 ****************************************************************************/

int t113_gpio_config(uint16_t pinset)
{
  uint32_t port = T113_GPIO_PORT(pinset);
  uint32_t pin  = T113_GPIO_PINNO(pinset);
  uint32_t func = T113_GPIO_FUNC(pinset);
  uint32_t pull = T113_GPIO_PULL_VAL(pinset);
  uint32_t drv  = T113_GPIO_DRV_VAL(pinset);
  uint32_t reg;

  reg = T113_PIO_CFG(port, pin >> 3);
  gpio_rmw(reg, 0xf << ((pin & 7) * 4),
           (func & 0xf) << ((pin & 7) * 4));

  if (pin < 16)
    {
      reg = T113_PIO_PULL0(port);
    }
  else
    {
      reg = T113_PIO_PULL1(port);
    }

  gpio_rmw(reg, 0x3 << ((pin & 15) * 2),
           (pull & 0x3) << ((pin & 15) * 2));

  if (pin < 16)
    {
      reg = T113_PIO_DRV0(port);
    }
  else
    {
      reg = T113_PIO_DRV1(port);
    }

  gpio_rmw(reg, 0x3 << ((pin & 15) * 2),
           (drv & 0x3) << ((pin & 15) * 2));

  return OK;
}

/****************************************************************************
 * Name: t113_gpio_write
 *
 * Description:
 *   Write a value to a GPIO output pin.
 *
 ****************************************************************************/

void t113_gpio_write(uint16_t pinset, bool value)
{
  uint32_t port = T113_GPIO_PORT(pinset);
  uint32_t pin  = T113_GPIO_PINNO(pinset);
  uint32_t reg  = T113_PIO_DAT(port);

  if (value)
    {
      gpio_rmw(reg, 0, 1u << pin);
    }
  else
    {
      gpio_rmw(reg, 1u << pin, 0);
    }
}

/****************************************************************************
 * Name: t113_gpio_read
 *
 * Description:
 *   Read the current state of a GPIO pin.
 *
 ****************************************************************************/

bool t113_gpio_read(uint16_t pinset)
{
  uint32_t port = T113_GPIO_PORT(pinset);
  uint32_t pin  = T113_GPIO_PINNO(pinset);

  return (getreg32(T113_PIO_DAT(port)) >> pin) & 1;
}

/****************************************************************************
 * GPIO Interrupt Framework (PD port only)
 ****************************************************************************/

#ifdef CONFIG_T113_GPIO_IRQ

#define T113_PD_NPINS  T113_GPIO_NPINS_D  /* 23 */

struct t113_pd_slot_s
{
  xcpt_t isr;
  FAR void *arg;
};

static struct t113_pd_slot_s g_pd_slots[T113_PD_NPINS];
static spinlock_t g_pd_lock = SP_UNLOCKED;
static bool g_pd_irq_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int t113_pd_isr(int irq, FAR void *context, FAR void *arg)
{
  uint32_t status = getreg32(T113_PD_EINT_STATUS);
  uint32_t pending;
  unsigned int pin;

  UNUSED(irq);
  UNUSED(arg);

  /* W1C first.  If a new edge arrives on the same pin during dispatch,
   * hardware re-latches the status bit and a fresh IRQ fires afterward.
   * Doing the clear after dispatch would drop such edges.
   */

  putreg32(status, T113_PD_EINT_STATUS);

  pending = status;
  while (pending != 0)
    {
      pin = __builtin_ctz(pending);
      pending &= ~(1u << pin);

      if (pin < T113_PD_NPINS && g_pd_slots[pin].isr != NULL)
        {
          g_pd_slots[pin].isr(T113_IRQ_PD_EINT, context,
                              g_pd_slots[pin].arg);
        }
    }

  return OK;
}

static int t113_pd_set_trigger(unsigned int pin, uint8_t trigger)
{
  uintptr_t reg;
  unsigned int shift;
  uint32_t val;

  if (pin < 8)
    {
      reg = T113_PD_EINT_CFG0;
      shift = pin * 4;
    }
  else if (pin < 16)
    {
      reg = T113_PD_EINT_CFG1;
      shift = (pin - 8) * 4;
    }
  else if (pin < T113_PD_NPINS)
    {
      reg = T113_PD_EINT_CFG2;
      shift = (pin - 16) * 4;
    }
  else
    {
      return -EINVAL;
    }

  val = getreg32(reg);
  val &= ~(0xfu << shift);
  val |= ((uint32_t)trigger & 0xfu) << shift;
  putreg32(val, reg);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_gpio_irq_initialize(void)
{
  irqstate_t flags;

  flags = spin_lock_irqsave(&g_pd_lock);

  if (g_pd_irq_initialized)
    {
      spin_unlock_irqrestore(&g_pd_lock, flags);
      return;
    }

  /* Mask and clear all PD EINT sources */

  putreg32(0, T113_PD_EINT_CTL);
  putreg32(0xffffffff, T113_PD_EINT_STATUS);

  memset(g_pd_slots, 0, sizeof(g_pd_slots));
  g_pd_irq_initialized = true;

  spin_unlock_irqrestore(&g_pd_lock, flags);

  /* Attach the port-level handler and enable in GIC */

  irq_attach(T113_IRQ_PD_EINT, t113_pd_isr, NULL);
  up_enable_irq(T113_IRQ_PD_EINT);
}

int t113_gpio_irq_attach(uint16_t pinset, uint8_t trigger,
                         xcpt_t isr, FAR void *arg)
{
  unsigned int port = T113_GPIO_PORT(pinset);
  unsigned int pin  = T113_GPIO_PINNO(pinset);
  irqstate_t flags;
  int ret;

  if (port != T113_GPIO_PORTD || pin >= T113_PD_NPINS ||
      trigger > T113_EINT_MODE_BOTH || isr == NULL)
    {
      return -EINVAL;
    }

  if (!g_pd_irq_initialized)
    {
      t113_gpio_irq_initialize();
    }

  flags = spin_lock_irqsave(&g_pd_lock);

  if (g_pd_slots[pin].isr != NULL)
    {
      spin_unlock_irqrestore(&g_pd_lock, flags);
      return -EALREADY;
    }

  g_pd_slots[pin].isr = isr;
  g_pd_slots[pin].arg = arg;

  ret = t113_pd_set_trigger(pin, trigger);

  /* Clear stale pending status for this pin before the caller unmasks. */

  putreg32(1u << pin, T113_PD_EINT_STATUS);

  spin_unlock_irqrestore(&g_pd_lock, flags);
  return ret;
}

void t113_gpio_irq_detach(uint16_t pinset)
{
  unsigned int port = T113_GPIO_PORT(pinset);
  unsigned int pin  = T113_GPIO_PINNO(pinset);
  irqstate_t flags;
  uint32_t val;

  if (port != T113_GPIO_PORTD || pin >= T113_PD_NPINS)
    {
      return;
    }

  flags = spin_lock_irqsave(&g_pd_lock);

  val = getreg32(T113_PD_EINT_CTL);
  val &= ~(1u << pin);
  putreg32(val, T113_PD_EINT_CTL);

  g_pd_slots[pin].isr = NULL;
  g_pd_slots[pin].arg = NULL;

  spin_unlock_irqrestore(&g_pd_lock, flags);
}

void t113_gpio_irq_enable(uint16_t pinset, bool enable)
{
  unsigned int port = T113_GPIO_PORT(pinset);
  unsigned int pin  = T113_GPIO_PINNO(pinset);
  irqstate_t flags;
  uint32_t val;

  if (port != T113_GPIO_PORTD || pin >= T113_PD_NPINS)
    {
      return;
    }

  flags = spin_lock_irqsave(&g_pd_lock);

  val = getreg32(T113_PD_EINT_CTL);
  if (enable)
    {
      /* Clear any stale pending before unmasking. */

      putreg32(1u << pin, T113_PD_EINT_STATUS);
      val |= (1u << pin);
    }
  else
    {
      val &= ~(1u << pin);
    }

  putreg32(val, T113_PD_EINT_CTL);

  spin_unlock_irqrestore(&g_pd_lock, flags);
}

#endif /* CONFIG_T113_GPIO_IRQ */
