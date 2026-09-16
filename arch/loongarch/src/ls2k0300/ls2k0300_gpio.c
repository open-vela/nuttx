/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_gpio.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/spinlock.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_gpio.h"

#if defined(CONFIG_LS2K0300_GPIO)

struct ls2k0300_gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t pin;
  pin_interrupt_t callback;
};

static uintptr_t g_gpio_base = LS2K0300_GPIO_BASE;

static const struct gpio_operations_s g_gpio_input_ops;
static const struct gpio_operations_s g_gpio_output_ops;
static const struct gpio_operations_s g_gpio_int_ops;

static void gpio_set_reg(uintptr_t base, uint32_t offset, int pin,
                         uint8_t val)
{
  putreg8(val, base + offset + pin);
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t gpio_get_reg(uintptr_t base, uint32_t offset, int pin)
{
  return getreg8(base + offset + pin);
}

static int ls2k0300_gpio_input_read(struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct ls2k0300_gpio_dev_s *priv =
      (FAR struct ls2k0300_gpio_dev_s *)dev;

  *value = gpio_get_reg(g_gpio_base, LS2K0300_GPIO_IN, priv->pin) & 1;
  return OK;
}

static int ls2k0300_gpio_output_read(struct gpio_dev_s *dev,
                                    FAR bool *value)
{
  FAR struct ls2k0300_gpio_dev_s *priv =
      (FAR struct ls2k0300_gpio_dev_s *)dev;

  *value = gpio_get_reg(g_gpio_base, LS2K0300_GPIO_OUT, priv->pin) & 1;
  return OK;
}

static int ls2k0300_gpio_output_write(struct gpio_dev_s *dev, bool value)
{
  FAR struct ls2k0300_gpio_dev_s *priv =
      (FAR struct ls2k0300_gpio_dev_s *)dev;

  gpio_set_reg(g_gpio_base, LS2K0300_GPIO_OUT, priv->pin, value ? 1 : 0);
  return OK;
}

static int ls2k0300_gpio_int_read(struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct ls2k0300_gpio_dev_s *priv =
      (FAR struct ls2k0300_gpio_dev_s *)dev;

  *value = gpio_get_reg(g_gpio_base, LS2K0300_GPIO_IN, priv->pin) & 1;
  return OK;
}

static int ls2k0300_gpio_int_attach(struct gpio_dev_s *dev,
                                   pin_interrupt_t callback)
{
  FAR struct ls2k0300_gpio_dev_s *priv =
      (FAR struct ls2k0300_gpio_dev_s *)dev;

  priv->callback = callback;
  return OK;
}

static int ls2k0300_gpio_int_enable(struct gpio_dev_s *dev, bool enable)
{
  FAR struct ls2k0300_gpio_dev_s *priv =
      (FAR struct ls2k0300_gpio_dev_s *)dev;

  gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, enable ? 1 : 0);
  return OK;
}

static int ls2k0300_gpio_setpintype(struct gpio_dev_s *dev,
                                   enum gpio_pintype_e pintype)
{
  FAR struct ls2k0300_gpio_dev_s *priv =
      (FAR struct ls2k0300_gpio_dev_s *)dev;

  switch (pintype)
    {
      case GPIO_INPUT_PIN:
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, 0);
        break;

      case GPIO_OUTPUT_PIN:
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, priv->pin, 0);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, 0);
        break;

      case GPIO_INTERRUPT_RISING_PIN:
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQDUL, priv->pin, 0);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQPOL, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQEDG, priv->pin, 1);
        break;

      case GPIO_INTERRUPT_FALLING_PIN:
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQDUL, priv->pin, 0);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQPOL, priv->pin, 0);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQEDG, priv->pin, 1);
        break;

      case GPIO_INTERRUPT_BOTH_PIN:
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQDUL, priv->pin, 1);
        break;

      case GPIO_INTERRUPT_HIGH_PIN:
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQDUL, priv->pin, 0);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQPOL, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQEDG, priv->pin, 0);
        break;

      case GPIO_INTERRUPT_LOW_PIN:
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQ, priv->pin, 1);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQDUL, priv->pin, 0);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQPOL, priv->pin, 0);
        gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQEDG, priv->pin, 0);
        break;

      default:
        return -EINVAL;
    }

  switch (pintype)
    {
      case GPIO_INPUT_PIN:
      case GPIO_INPUT_PIN_PULLUP:
      case GPIO_INPUT_PIN_PULLDOWN:
        dev->gp_ops = &g_gpio_input_ops;
        break;

      case GPIO_OUTPUT_PIN:
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        dev->gp_ops = &g_gpio_output_ops;
        break;

      case GPIO_INTERRUPT_PIN:
      case GPIO_INTERRUPT_HIGH_PIN:
      case GPIO_INTERRUPT_LOW_PIN:
      case GPIO_INTERRUPT_RISING_PIN:
      case GPIO_INTERRUPT_FALLING_PIN:
      case GPIO_INTERRUPT_BOTH_PIN:
      case GPIO_INTERRUPT_PIN_WAKEUP:
      case GPIO_INTERRUPT_HIGH_PIN_WAKEUP:
      case GPIO_INTERRUPT_LOW_PIN_WAKEUP:
      case GPIO_INTERRUPT_RISING_PIN_WAKEUP:
      case GPIO_INTERRUPT_FALLING_PIN_WAKEUP:
      case GPIO_INTERRUPT_BOTH_PIN_WAKEUP:
        dev->gp_ops = &g_gpio_int_ops;
        break;

      default:
        break;
    }

  dev->gp_pintype = (uint8_t)pintype;
  return OK;
}

static const struct gpio_operations_s g_gpio_input_ops =
{
  .go_read       = ls2k0300_gpio_input_read,
  .go_write      = NULL,
  .go_attach     = NULL,
  .go_enable     = NULL,
  .go_setpintype = ls2k0300_gpio_setpintype,
};

static const struct gpio_operations_s g_gpio_output_ops =
{
  .go_read       = ls2k0300_gpio_output_read,
  .go_write      = ls2k0300_gpio_output_write,
  .go_attach     = NULL,
  .go_enable     = NULL,
  .go_setpintype = ls2k0300_gpio_setpintype,
};

static const struct gpio_operations_s g_gpio_int_ops =
{
  .go_read       = ls2k0300_gpio_int_read,
  .go_write      = NULL,
  .go_attach     = ls2k0300_gpio_int_attach,
  .go_enable     = ls2k0300_gpio_int_enable,
  .go_setpintype = ls2k0300_gpio_setpintype,
};

static struct ls2k0300_gpio_dev_s g_gpio_dev[LS2K0300_GPIO_NPINS];

#ifdef CONFIG_LS2K0300_GPIO_IRQ
static const int g_gpio_irqs[] =
{
  LS2K0300_IRQ_GPIO,
  LS2K0300_IRQ_GPIO1,
  LS2K0300_IRQ_GPIO2,
  LS2K0300_IRQ_GPIO3,
  LS2K0300_IRQ_GPIO4,
  LS2K0300_IRQ_GPIO5,
  LS2K0300_IRQ_GPIO6,
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GPIO_IRQ_GROUPS  7

static const int g_gpio_irq_group_start[] =
{
  0, 16, 32, 48, 64, 80, 96
};

static const int g_gpio_irq_group_count[] =
{
  16, 16, 16, 16, 16, 16, 10
};

static int ls2k0300_gpio_irq_handler(int irq, FAR void *context,
                                    FAR void *arg)
{
  int group = (int)(uintptr_t)arg;
  int start;
  int count;
  int i;

  if (group < 0 || group >= GPIO_IRQ_GROUPS)
    {
      return OK;
    }

  start = g_gpio_irq_group_start[group];
  count = g_gpio_irq_group_count[group];

  for (i = start; i < start + count && i < LS2K0300_GPIO_NPINS; i++)
    {
      if (gpio_get_reg(g_gpio_base, LS2K0300_GPIO_IRQ, i) == 1 &&
          gpio_get_reg(g_gpio_base, LS2K0300_GPIO_IRQSTA, i) == 1)
        {
          gpio_set_reg(g_gpio_base, LS2K0300_GPIO_IRQCLR, i, 1);

          if (g_gpio_dev[i].callback != NULL)
            {
              g_gpio_dev[i].callback(&g_gpio_dev[i].gpio, i);
            }
        }
    }

  return OK;
}

#endif

int ls2k0300_gpio_config_pin(int pin, enum gpio_pintype_e pintype)
{
  if (pin < 0 || pin >= LS2K0300_GPIO_NPINS)
    {
      return -EINVAL;
    }

  switch (pintype)
    {
      case GPIO_INPUT_PIN:
      case GPIO_INPUT_PIN_PULLUP:
      case GPIO_INPUT_PIN_PULLDOWN:
        g_gpio_dev[pin].gpio.gp_ops = &g_gpio_input_ops;
        break;

      case GPIO_OUTPUT_PIN:
      case GPIO_OUTPUT_PIN_OPENDRAIN:
        g_gpio_dev[pin].gpio.gp_ops = &g_gpio_output_ops;
        break;

      case GPIO_INTERRUPT_PIN:
      case GPIO_INTERRUPT_HIGH_PIN:
      case GPIO_INTERRUPT_LOW_PIN:
      case GPIO_INTERRUPT_RISING_PIN:
      case GPIO_INTERRUPT_FALLING_PIN:
      case GPIO_INTERRUPT_BOTH_PIN:
        g_gpio_dev[pin].gpio.gp_ops = &g_gpio_int_ops;
        break;

      default:
        return -EINVAL;
    }

  return ls2k0300_gpio_setpintype(&g_gpio_dev[pin].gpio, pintype);
}

int ls2k0300_gpio_write(int pin, bool value)
{
  if (pin < 0 || pin >= LS2K0300_GPIO_NPINS)
    {
      return -EINVAL;
    }

  gpio_set_reg(g_gpio_base, LS2K0300_GPIO_OUT, pin, value ? 1 : 0);
  return OK;
}

int ls2k0300_gpio_read(int pin, FAR bool *value)
{
  if (pin < 0 || pin >= LS2K0300_GPIO_NPINS)
    {
      return -EINVAL;
    }

  *value = gpio_get_reg(g_gpio_base, LS2K0300_GPIO_IN, pin) & 1;
  return OK;
}

int ls2k0300_gpio_set_direction(int pin, bool input)
{
  if (pin < 0 || pin >= LS2K0300_GPIO_NPINS)
    {
      return -EINVAL;
    }

  gpio_set_reg(g_gpio_base, LS2K0300_GPIO_DIR, pin, input ? 1 : 0);
  return OK;
}

int ls2k0300_gpio_initialize(void)
{
  int i;
  int ret;
  int minor = 0;

  for (i = 0; i < LS2K0300_GPIO_NPINS; i++)
    {
      g_gpio_dev[i].pin = (uint8_t)i;
      g_gpio_dev[i].callback = NULL;
      g_gpio_dev[i].gpio.gp_pintype = (uint8_t)GPIO_INPUT_PIN;
      ls2k0300_gpio_config_pin(i, GPIO_INPUT_PIN);
    }

#ifdef CONFIG_LS2K0300_GPIO_IRQ
  for (i = 0; i < GPIO_IRQ_GROUPS; i++)
    {
      ret = irq_attach(g_gpio_irqs[i], ls2k0300_gpio_irq_handler,
                       (FAR void *)(uintptr_t)i);
      if (ret < 0)
        {
          gpioerr("ERROR: irq_attach failed for irq %d: %d\n",
                  g_gpio_irqs[i], ret);
          return ret;
        }

      up_enable_irq(g_gpio_irqs[i]);
    }
#endif

  for (i = 0; i < LS2K0300_GPIO_NPINS; i++)
    {
      ret = gpio_pin_register(&g_gpio_dev[i].gpio, minor);
      if (ret < 0)
        {
          gpioerr("ERROR: gpio_pin_register failed for pin %d: %d\n",
                  i, ret);
          continue;
        }

      minor++;
    }

  return OK;
}

#endif
