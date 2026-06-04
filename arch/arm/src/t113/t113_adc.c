/****************************************************************************
 * arch/arm/src/t113/t113_adc.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/t113_adc.h"
#include "hardware/t113_ccu.h"
#include "t113_ccu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CCU GPADC bus-gate/reset register bits.
 * Register is T113_CCU_GPADC_BGR at CCU offset 0x9ec.
 */

#define T113_CCU_GPADC_GATING       (1u << 0)
#define T113_CCU_GPADC_RST_DEASSERT (1u << 16)

/* Only CH0 is bonded out on T113-S3 (pin PC9). */

#define T113_GPADC_DEFAULT_CH       0
#define T113_GPADC_DEFAULT_MASK     (1u << T113_GPADC_DEFAULT_CH)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_adc_s
{
  const struct adc_ops_s          *ops;
  FAR const struct adc_callback_s *cb;
  uint8_t                          chanmask;
  bool                             rxenabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  t113_adc_bind(FAR struct adc_dev_s *dev,
                          FAR const struct adc_callback_s *callback);
static void t113_adc_reset(FAR struct adc_dev_s *dev);
static int  t113_adc_setup(FAR struct adc_dev_s *dev);
static void t113_adc_shutdown(FAR struct adc_dev_s *dev);
static void t113_adc_rxint(FAR struct adc_dev_s *dev, bool enable);
static int  t113_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                           unsigned long arg);
static int  t113_adc_interrupt(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct adc_ops_s g_adc_ops =
{
  .ao_bind     = t113_adc_bind,
  .ao_reset    = t113_adc_reset,
  .ao_setup    = t113_adc_setup,
  .ao_shutdown = t113_adc_shutdown,
  .ao_rxint    = t113_adc_rxint,
  .ao_ioctl    = t113_adc_ioctl,
};

static struct t113_adc_s g_adc_priv =
{
  .ops       = &g_adc_ops,
  .cb        = NULL,
  .chanmask  = T113_GPADC_DEFAULT_MASK,
  .rxenabled = false,
};

static struct adc_dev_s g_adc_dev =
{
  .ad_ops  = &g_adc_ops,
  .ad_priv = &g_adc_priv,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_adc_clk_enable
 *
 * Description:
 *   Deassert GPADC reset and ungate the APB bus clock via the CCU.
 *
 ****************************************************************************/

static void t113_adc_clk_enable(void)
{
  /* Deassert GPADC reset and enable clock via CCU.  Use t113_ccu_modify
   * so concurrent CCU writers (other drivers, AMP siblings) cannot drop
   * bits in T113_CCU_GPADC_BGR.
   */

  t113_ccu_modify(T113_CCU_GPADC_BGR, 0,
                  T113_CCU_GPADC_RST_DEASSERT | T113_CCU_GPADC_GATING);
}

/****************************************************************************
 * Name: t113_adc_clk_disable
 ****************************************************************************/

static void t113_adc_clk_disable(void)
{
  uint32_t val;

  val  = getreg32(T113_CCU_GPADC_BGR);
  val &= ~(T113_CCU_GPADC_RST_DEASSERT | T113_CCU_GPADC_GATING);
  putreg32(val, T113_CCU_GPADC_BGR);
}

/****************************************************************************
 * Name: t113_adc_set_rate
 *
 * Description:
 *   Program the sample-rate divider.  sample_rate = clk_in / (div + 1).
 *
 ****************************************************************************/

static void t113_adc_set_rate(uint32_t clk_in, uint32_t sample_rate)
{
  uint32_t div;
  uint32_t val;

  if (sample_rate == 0 || sample_rate > clk_in)
    {
      sample_rate = T113_GPADC_DEFAULT_SR_HZ;
    }

  div = (clk_in / sample_rate) - 1;

  val  = getreg32(T113_GPADC_SR);
  val &= ~T113_GPADC_SR_DIV_MASK;
  val |= (div << T113_GPADC_SR_DIV_SHIFT) & T113_GPADC_SR_DIV_MASK;
  putreg32(val, T113_GPADC_SR);
}

/****************************************************************************
 * Name: t113_adc_reset
 *
 * Description:
 *   Reset the hardware to a known state: clocks ungated, core disabled,
 *   FIFO flushed.  Called early and on error.
 *
 ****************************************************************************/

static void t113_adc_reset(FAR struct adc_dev_s *dev)
{
  t113_adc_clk_enable();

  /* Disable core so register writes take effect cleanly. */

  putreg32(0, T113_GPADC_CTRL);

  /* Disable every interrupt source, then flush the FIFO. */

  putreg32(0, T113_GPADC_FIFO_INTC);
  putreg32(0, T113_GPADC_DATA_INTC);
  putreg32(0, T113_GPADC_DATAL_INTC);
  putreg32(0, T113_GPADC_DATAH_INTC);

  putreg32(T113_GPADC_FIFO_FLUSH, T113_GPADC_FIFO_INTC);

  /* Clear any stale pending flags (W1C). */

  putreg32(T113_GPADC_FIFO_DATA_PEND | T113_GPADC_FIFO_OVER_PEND,
           T113_GPADC_FIFO_INTS);
  putreg32(0xff, T113_GPADC_DATA_INTS);

  /* Deselect all channels. */

  putreg32(0, T113_GPADC_CS_EN);
}

/****************************************************************************
 * Name: t113_adc_bind
 *
 * Description:
 *   Store the upper-half callback table so the ISR can deliver samples.
 *
 ****************************************************************************/

static int t113_adc_bind(FAR struct adc_dev_s *dev,
                         FAR const struct adc_callback_s *callback)
{
  FAR struct t113_adc_s *priv = (FAR struct t113_adc_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;
  return OK;
}

/****************************************************************************
 * Name: t113_adc_setup
 *
 * Description:
 *   Program the hardware, attach the IRQ, and enable per-channel
 *   data-ready interrupts.  Called the first time /dev/adc0 is opened.
 *
 ****************************************************************************/

static int t113_adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct t113_adc_s *priv = (FAR struct t113_adc_s *)dev->ad_priv;
  uint32_t ctrl;
  int ret;
  int i;

  t113_adc_reset(dev);

  ret = irq_attach(T113_IRQ_GPADC, t113_adc_interrupt, priv);
  if (ret < 0)
    {
      aerr("ERROR: irq_attach(%d) failed: %d\n", T113_IRQ_GPADC, ret);
      return ret;
    }

  /* Sample rate: 24 MHz HOSC divided to a safe default. */

  t113_adc_set_rate(T113_GPADC_CLK_HZ, T113_GPADC_DEFAULT_SR_HZ);

  /* Enable channel(s) selected via the active mask. */

  putreg32(priv->chanmask, T113_GPADC_CS_EN);

  /* Arm per-channel data-ready interrupts for the enabled channels. */

  for (i = 0; i < T113_GPADC_NCHANNELS; i++)
    {
      if (priv->chanmask & (1u << i))
        {
          uint32_t dic = getreg32(T113_GPADC_DATA_INTC);
          dic |= T113_GPADC_DATA_IRQ(i);
          putreg32(dic, T113_GPADC_DATA_INTC);
        }
    }

  /* Program CTRL per T113-S3 User Manual v1.1 section 9.8.6.2
   * (GP_CTRL table):
   *
   *   [31:24] FIRST_DLY     = 8  -> hardware discards the first 8
   *                                 samples of each channel's first
   *                                 activation.  At the default 2 us
   *                                 TACQ this covers ~16 us of LDO
   *                                 warm-up and sample-cap pre-charge
   *                                 settling.
   *   [23]    AUTOCALI_EN   = 1  -> datasheet default; auto-calibrate
   *                                 on every ADC_EN transition.  The
   *                                 previous driver cleared this bit
   *                                 because it assigned to CTRL without
   *                                 preserving the reset value.
   *   [19:18] Work mode     = 10 -> continuous, matches the NuttX
   *                                 upper-half FIFO semantics;
   *                                 ANIOC_TRIGGER switches to single.
   *   [17]    CALI_EN       = 1  -> manual-trigger calibration, the
   *                                 hardware self-clears when done.
   *   [16]    ADC_EN        = 1  -> enable function.
   *   [0]     LDO_EN        = 1  -> enable analog LDO.
   */

  ctrl  = T113_GPADC_CTRL_LDO_EN;
  ctrl |= T113_GPADC_CTRL_AUTOCALI_EN;
  ctrl |= (8u << T113_GPADC_CTRL_FIRST_DLY_SHIFT);
  ctrl |= T113_GPADC_CTRL_CALI_EN;
  ctrl |= T113_GPADC_CTRL_MODE_CONT;
  ctrl |= T113_GPADC_CTRL_ADC_EN;
  putreg32(ctrl, T113_GPADC_CTRL);

  up_enable_irq(T113_IRQ_GPADC);

  ainfo("T113 GPADC setup: ctrl=0x%08lx mask=0x%02x irq=%d\n",
        (unsigned long)ctrl, priv->chanmask, T113_IRQ_GPADC);

  return OK;
}

/****************************************************************************
 * Name: t113_adc_shutdown
 ****************************************************************************/

static void t113_adc_shutdown(FAR struct adc_dev_s *dev)
{
  up_disable_irq(T113_IRQ_GPADC);
  irq_detach(T113_IRQ_GPADC);

  putreg32(0, T113_GPADC_CTRL);
  putreg32(0, T113_GPADC_DATA_INTC);
  putreg32(0, T113_GPADC_FIFO_INTC);

  t113_adc_clk_disable();
}

/****************************************************************************
 * Name: t113_adc_rxint
 *
 * Description:
 *   Enable or disable the per-channel data-ready interrupt sources while
 *   the upper half flow-controls the sample stream.  The GIC-level IRQ
 *   line stays attached; only the device-level enables toggle.
 *
 ****************************************************************************/

static void t113_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct t113_adc_s *priv = (FAR struct t113_adc_s *)dev->ad_priv;
  irqstate_t flags;
  uint32_t dic;
  int i;

  flags = enter_critical_section();

  if (enable)
    {
      dic = getreg32(T113_GPADC_DATA_INTC);
      for (i = 0; i < T113_GPADC_NCHANNELS; i++)
        {
          if (priv->chanmask & (1u << i))
            {
              dic |= T113_GPADC_DATA_IRQ(i);
            }
        }

      putreg32(dic, T113_GPADC_DATA_INTC);
    }
  else
    {
      putreg32(0, T113_GPADC_DATA_INTC);
    }

  priv->rxenabled = enable;
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: t113_adc_ioctl
 *
 * Description:
 *   Handle analog device ioctl commands.  ANIOC_TRIGGER starts a single
 *   conversion in single-shot mode - used by apps/examples/adc with
 *   CONFIG_EXAMPLES_ADC_SWTRIG.
 *
 ****************************************************************************/

static int t113_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                          unsigned long arg)
{
  FAR struct t113_adc_s *priv = (FAR struct t113_adc_s *)dev->ad_priv;
  uint32_t ctrl;

  UNUSED(arg);

  switch (cmd)
    {
      case ANIOC_TRIGGER:

        /* Switch to single-shot and re-assert ADC_EN to kick one pass.
         * In single-shot the core clears ADC_EN automatically when the
         * conversion completes; toggling ADC_EN here acts as the
         * software trigger.
         */

        ctrl  = getreg32(T113_GPADC_CTRL);
        ctrl &= ~(T113_GPADC_CTRL_MODE_MASK | T113_GPADC_CTRL_ADC_EN);
        ctrl |= T113_GPADC_CTRL_MODE_SINGLE;
        putreg32(ctrl, T113_GPADC_CTRL);

        ctrl |= T113_GPADC_CTRL_ADC_EN;
        putreg32(ctrl, T113_GPADC_CTRL);
        return OK;

      case ANIOC_GET_NCHANNELS:
        {
          int n = 0;
          int i;

          for (i = 0; i < T113_GPADC_NCHANNELS; i++)
            {
              if (priv->chanmask & (1u << i))
                {
                  n++;
                }
            }

          return n;
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: t113_adc_interrupt
 *
 * Description:
 *   GPADC IRQ handler.  For every channel flagged in DATA_INTS, read the
 *   corresponding CH_DATA register, clear the pending bit (W1C), and hand
 *   the sample to the upper half via au_receive().  Also drains the
 *   FIFO_INTS overrun / data flags to keep the controller healthy.
 *
 ****************************************************************************/

static int t113_adc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct t113_adc_s *priv = (FAR struct t113_adc_s *)arg;
  uint32_t pending;
  uint32_t fifo_pending;
  uint32_t enabled;
  int i;

  DEBUGASSERT(priv != NULL);

  pending = getreg32(T113_GPADC_DATA_INTS);
  enabled = getreg32(T113_GPADC_DATA_INTC);

  if (pending != 0)
    {
      putreg32(pending, T113_GPADC_DATA_INTS);
    }

  for (i = 0; i < T113_GPADC_NCHANNELS; i++)
    {
      uint32_t bit = T113_GPADC_DATA_IRQ(i);

      if ((pending & enabled & bit) == 0)
        {
          continue;
        }

      if (priv->cb != NULL && priv->cb->au_receive != NULL)
        {
          int32_t sample = (int32_t)(getreg32(T113_GPADC_CH_DATA(i)) &
                                     T113_GPADC_DATA_MASK);
          priv->cb->au_receive(&g_adc_dev, (uint8_t)i, sample);
        }
    }

  /* Drain FIFO status bits (data-ready / overrun) so a stray FIFO-mode
   * configuration does not leave the line asserted.
   */

  fifo_pending = getreg32(T113_GPADC_FIFO_INTS);
  if (fifo_pending != 0)
    {
      putreg32(fifo_pending, T113_GPADC_FIFO_INTS);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_adc_initialize
 *
 * Description:
 *   Register /dev/adc0 backed by the T113-S3 GPADC.  Called from
 *   t113_bringup() under CONFIG_T113_GPADC.
 *
 ****************************************************************************/

void t113_adc_initialize(void)
{
  int ret;

  ret = adc_register("/dev/adc0", &g_adc_dev);
  if (ret < 0)
    {
      aerr("ERROR: adc_register failed: %d\n", ret);
    }
}
