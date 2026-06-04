/****************************************************************************
 * arch/arm/src/t113/t113_lradc.c
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
 * T113-S3 / R528 LRADC driver.  Registers one 6-bit key-scan channel as
 * /dev/lradc0 using the standard NuttX upper-half ADC framework.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "arm_internal.h"
#include "t113_lradc.h"
#include "hardware/t113_lradc.h"
#include "hardware/t113_ccu.h"

#ifdef CONFIG_T113_LRADC

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_lradc_s
{
  FAR const struct adc_callback_s *cb;
  t113_lradc_hook_t                hook;
  FAR void                        *hook_priv;
  bool                             attached;
  bool                             enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  t113_lradc_bind(FAR struct adc_dev_s *dev,
                            FAR const struct adc_callback_s *callback);
static void t113_lradc_reset(FAR struct adc_dev_s *dev);
static int  t113_lradc_setup(FAR struct adc_dev_s *dev);
static void t113_lradc_shutdown(FAR struct adc_dev_s *dev);
static void t113_lradc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  t113_lradc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                             unsigned long arg);
static int  t113_lradc_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_lradc_ops =
{
  .ao_bind     = t113_lradc_bind,
  .ao_reset    = t113_lradc_reset,
  .ao_setup    = t113_lradc_setup,
  .ao_shutdown = t113_lradc_shutdown,
  .ao_rxint    = t113_lradc_rxint,
  .ao_ioctl    = t113_lradc_ioctl,
};

static struct t113_lradc_s g_lradc_priv;

static struct adc_dev_s g_lradc_dev =
{
  .ad_ops  = &g_lradc_ops,
  .ad_priv = &g_lradc_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_lradc_clk_enable
 *
 * Description:
 *   De-assert the LRADC reset and enable its bus clock gate via the CCU
 *   BGR register.  Writes LRADC_RST | LRADC_GATING_EN in a single write.
 *
 ****************************************************************************/

static void t113_lradc_clk_enable(void)
{
  putreg32(T113_LRADC_BGR_RST | T113_LRADC_BGR_GATING,
           T113_CCU_LRADC_BGR);
}

/****************************************************************************
 * Name: t113_lradc_bind
 ****************************************************************************/

static int t113_lradc_bind(FAR struct adc_dev_s *dev,
                           FAR const struct adc_callback_s *callback)
{
  FAR struct t113_lradc_s *priv = (FAR struct t113_lradc_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: t113_lradc_reset
 *
 * Description:
 *   Enable the clock and program the CTRL register for key-scan:
 *   250 Hz sample rate, LEVELB=0, HOLD_EN, normal key mode,
 *   channel 0, LRADC enabled.  All interrupts remain masked until
 *   t113_lradc_rxint(true) is called by the upper half.
 *
 ****************************************************************************/

static void t113_lradc_reset(FAR struct adc_dev_s *dev)
{
  t113_lradc_clk_enable();

  /* CTRL = FIRST_DLY(0) | CHAN0 | KEY_MODE_NORM | HOLD_EN
   *        | LEVELB(0) | SRATE_250HZ | EN.
   * Expanding: (HOLD_EN & ~LEVELB & ~SRATE) | FIRST_DLY | CHAN |
   * KEY_MODE | EN evaluates to HOLD_EN | EN with all other fields
   * cleared to 0.
   */

  putreg32(T113_LRADC_CTRL_HOLD_EN | T113_LRADC_CTRL_EN,
           T113_LRADC_CTRL);

  /* Mask all interrupts; clear any pending status (W1C). */

  putreg32(0, T113_LRADC_INTC);
  putreg32(T113_LRADC_INTS_ADC0_DATA |
           T113_LRADC_INTS_ADC0_DOWN |
           T113_LRADC_INTS_ADC0_UP,
           T113_LRADC_INTS);
}

/****************************************************************************
 * Name: t113_lradc_setup
 *
 * Description:
 *   Attach the ISR (idempotent) and enable the GIC line.  Interrupts remain
 *   masked at the peripheral until ao_rxint is called.
 *
 ****************************************************************************/

static int t113_lradc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct t113_lradc_s *priv = (FAR struct t113_lradc_s *)dev->ad_priv;
  int ret;

  if (!priv->attached)
    {
      ret = irq_attach(T113_IRQ_LRADC, t113_lradc_interrupt, dev);
      if (ret < 0)
        {
          aerr("ERROR: irq_attach failed: %d\n", ret);
          return ret;
        }

      priv->attached = true;
    }

  t113_lradc_reset(dev);
  up_enable_irq(T113_IRQ_LRADC);

  return OK;
}

/****************************************************************************
 * Name: t113_lradc_shutdown
 ****************************************************************************/

static void t113_lradc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct t113_lradc_s *priv = (FAR struct t113_lradc_s *)dev->ad_priv;

  up_disable_irq(T113_IRQ_LRADC);

  /* Mask all peripheral interrupts and disable the ADC. */

  putreg32(0, T113_LRADC_INTC);
  putreg32(0, T113_LRADC_CTRL);

  priv->enabled = false;
}

/****************************************************************************
 * Name: t113_lradc_rxint
 *
 * Description:
 *   Enable or disable LRADC data/up/down interrupts at the peripheral.
 *
 ****************************************************************************/

static void t113_lradc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct t113_lradc_s *priv = (FAR struct t113_lradc_s *)dev->ad_priv;

  if (enable)
    {
      putreg32(T113_LRADC_INTC_ADC0_DATA_EN |
               T113_LRADC_INTC_ADC0_DOWN_EN |
               T113_LRADC_INTC_ADC0_UP_EN,
               T113_LRADC_INTC);
    }
  else
    {
      putreg32(0, T113_LRADC_INTC);
    }

  priv->enabled = enable;
}

/****************************************************************************
 * Name: t113_lradc_ioctl
 ****************************************************************************/

static int t113_lradc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                            unsigned long arg)
{
  UNUSED(dev);
  UNUSED(cmd);
  UNUSED(arg);
  return -ENOTTY;
}

/****************************************************************************
 * Name: t113_lradc_interrupt
 *
 * Description:
 *   LRADC ISR.  On each interrupt, read INTS, acknowledge (W1C), read the
 *   6-bit DATA0 sample, and push the raw value to the upper half via the
 *   bound callback.  Called for both key-down/key-up edges and periodic
 *   DATA events.
 *
 ****************************************************************************/

static int t113_lradc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct adc_dev_s    *dev  = (FAR struct adc_dev_s *)arg;
  FAR struct t113_lradc_s *priv = (FAR struct t113_lradc_s *)dev->ad_priv;
  uint32_t status;
  uint32_t data;

  UNUSED(irq);
  UNUSED(context);

  status = getreg32(T113_LRADC_INTS);
  data   = getreg32(T113_LRADC_DATA0) & T113_LRADC_DATA_MASK;

  /* Acknowledge all latched events. */

  putreg32(status, T113_LRADC_INTS);

  if ((status & (T113_LRADC_INTS_ADC0_DATA |
                 T113_LRADC_INTS_ADC0_DOWN |
                 T113_LRADC_INTS_ADC0_UP)) != 0)
    {
      if (priv->cb != NULL && priv->cb->au_receive != NULL)
        {
          priv->cb->au_receive(dev, 0, (int32_t)data);
        }

      if (priv->hook != NULL)
        {
          priv->hook(status, data, priv->hook_priv);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_lradc_initialize
 *
 * Description:
 *   Register the LRADC lower-half driver with the NuttX ADC upper half as
 *   /dev/lradc0.  Must be called during board bringup before any user
 *   task opens the device.
 *
 ****************************************************************************/

void t113_lradc_initialize(void)
{
  int ret = adc_register("/dev/lradc0", &g_lradc_dev);
  if (ret < 0)
    {
      aerr("ERROR: adc_register /dev/lradc0 failed: %d\n", ret);
    }
}

/****************************************************************************
 * Name: t113_lradc_register_hook
 ****************************************************************************/

void t113_lradc_register_hook(t113_lradc_hook_t hook, FAR void *priv)
{
  irqstate_t flags = enter_critical_section();

  g_lradc_priv.hook      = hook;
  g_lradc_priv.hook_priv = priv;

  leave_critical_section(flags);
}

#endif /* CONFIG_T113_LRADC */
