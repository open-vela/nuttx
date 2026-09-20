/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_adc.c
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
#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_adc.h"
#include "ls2k0300_clockconfig.h"

#if defined(CONFIG_LS2K0300_ADC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ADC_TARGET_CLK_HZ       10000000UL

struct ls2k0300_adc_dev_s
{
  FAR const struct adc_callback_s *cb;
  uintptr_t base;
  uint8_t nchannels;
  uint8_t channels[ADC_CHANNEL_COUNT];
  bool rxint_enabled;
};

static uint32_t adc_getreg(FAR struct ls2k0300_adc_dev_s *priv,
                           uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void adc_putreg(FAR struct ls2k0300_adc_dev_s *priv,
                       uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static void adc_modifyreg(FAR struct ls2k0300_adc_dev_s *priv,
                          uint32_t offset, uint32_t clearbits,
                          uint32_t setbits)
{
  uint32_t regval = getreg32(priv->base + offset);
  regval &= ~clearbits;
  regval |= setbits;
  putreg32(regval, priv->base + offset);
}

static void adc_regular_channel_config(FAR struct ls2k0300_adc_dev_s *priv,
                                       uint8_t channel, uint8_t rank,
                                       uint8_t sampletime)
{
  uint32_t regval;
  uint32_t mask;

  if (channel <= 9)
    {
      regval = adc_getreg(priv, LS2K0300_ADC_SMPR2);
      mask = ADC_SMPR_SMP_MASK << (3 * channel);
      regval &= ~mask;
      regval |= (uint32_t)sampletime << (3 * channel);
      adc_putreg(priv, LS2K0300_ADC_SMPR2, regval);
    }
  else
    {
      regval = adc_getreg(priv, LS2K0300_ADC_SMPR1);
      mask = ADC_SMPR_SMP_MASK << (3 * (channel - 10));
      regval &= ~mask;
      regval |= (uint32_t)sampletime << (3 * (channel - 10));
      adc_putreg(priv, LS2K0300_ADC_SMPR1, regval);
    }

  if (rank < 7)
    {
      regval = adc_getreg(priv, LS2K0300_ADC_SQR3);
      mask = ADC_SQR3_SQ_MASK << (5 * (rank - 1));
      regval &= ~mask;
      regval |= (uint32_t)channel << (5 * (rank - 1));
      adc_putreg(priv, LS2K0300_ADC_SQR3, regval);
    }
  else if (rank < 13)
    {
      regval = adc_getreg(priv, LS2K0300_ADC_SQR2);
      mask = ADC_SQR2_SQ_MASK << (5 * (rank - 7));
      regval &= ~mask;
      regval |= (uint32_t)channel << (5 * (rank - 7));
      adc_putreg(priv, LS2K0300_ADC_SQR2, regval);
    }
  else
    {
      regval = adc_getreg(priv, LS2K0300_ADC_SQR1);
      mask = ADC_SQR1_SQ_MASK << (5 * (rank - 13));
      regval &= ~mask;
      regval |= (uint32_t)channel << (5 * (rank - 13));
      adc_putreg(priv, LS2K0300_ADC_SQR1, regval);
    }
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void adc_software_start_conv(FAR struct ls2k0300_adc_dev_s *priv)
{
  uint32_t sr;
  uint32_t cr2;

  sr = adc_getreg(priv, LS2K0300_ADC_SR);
  sr &= ~ADC_SR_EOC;
  adc_putreg(priv, LS2K0300_ADC_SR, sr);

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 |= ADC_CR2_SWSTART | ADC_CR2_EXTTRIG;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);
}

static bool adc_eoc_check(FAR struct ls2k0300_adc_dev_s *priv)
{
  int timeout = 10000;

  while (timeout-- > 0)
    {
      uint32_t sr = adc_getreg(priv, LS2K0300_ADC_SR);
      if (sr & ADC_SR_EOC)
        {
          return true;
        }

      up_udelay(1);
    }

  return false;
}

static int adc_read_channel(FAR struct ls2k0300_adc_dev_s *priv,
                            uint8_t channel)
{
  uint32_t data;
  uint32_t sqr1;

  sqr1 = adc_getreg(priv, LS2K0300_ADC_SQR1);
  sqr1 &= ADC_SQR1_L_MASK;
  sqr1 |= (0 << ADC_SQR1_L_SHIFT);
  adc_putreg(priv, LS2K0300_ADC_SQR1, sqr1);

  adc_regular_channel_config(priv, channel, 1, ADC_SAMPLETIME_64CYCLES);
  adc_software_start_conv(priv);

  if (!adc_eoc_check(priv))
    {
      aerr("ADC channel %d conversion timeout\n", channel);
      return -EIO;
    }

  data = adc_getreg(priv, LS2K0300_ADC_DR) & ADC_DATA_MASK;
  return (int)data;
}

static void adc_hw_init(FAR struct ls2k0300_adc_dev_s *priv)
{
  uint32_t cr1;
  uint32_t cr2;
  uint32_t sqr1;
  uint32_t apb_freq;
  uint32_t clkdiv;
  uint32_t clkdiv_lo;
  uint32_t clkdiv_hi;
  int timeout;

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 &= ~ADC_CR2_ADON;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);

  up_udelay(10);

  apb_freq = ls2k0300_get_apbclk();
  if (apb_freq == 0)
    {
      apb_freq = 200000000UL;
    }

  clkdiv = (apb_freq / (2 * ADC_TARGET_CLK_HZ));
  if (clkdiv > 0)
    {
      clkdiv--;
    }

  if (clkdiv > 1023)
    {
      clkdiv = 1023;
    }

  clkdiv_lo = clkdiv & 0x3f;
  clkdiv_hi = (clkdiv >> 6) & 0xf;

  cr1 = adc_getreg(priv, LS2K0300_ADC_CR1);
  cr1 &= ADC_CR1_CLEAR_MASK;
  cr1 |= ADC_CR1_SCAN;
  cr1 |= (clkdiv_lo << ADC_CR1_CLKDIV_SHIFT);
  adc_putreg(priv, LS2K0300_ADC_CR1, cr1);

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 &= ADC_CR2_CLEAR_MASK;
  cr2 |= ADC_EXTSEL_SWSTART;
  cr2 |= (clkdiv_hi << ADC_CR2_CLKDIV_HI_SHIFT);
  cr2 |= ADC_CR2_CLKMASK;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);

  sqr1 = adc_getreg(priv, LS2K0300_ADC_SQR1);
  sqr1 &= ADC_SQR1_CLEAR_MASK;
  sqr1 |= ((priv->nchannels - 1) << ADC_SQR1_L_SHIFT);
  adc_putreg(priv, LS2K0300_ADC_SQR1, sqr1);

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 |= ADC_CR2_ADON;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);

  up_udelay(100);

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 &= ~ADC_CR2_ADON;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);

  up_udelay(10);

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 |= ADC_CR2_ADON;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);

  up_udelay(100);

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 |= ADC_CR2_RSTCAL;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);

  timeout = 100000;
  while (timeout-- > 0)
    {
      cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
      if (!(cr2 & ADC_CR2_RSTCAL))
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout <= 0)
    {
      aerr("ADC RSTCAL timeout\n");
    }

  cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
  cr2 |= ADC_CR2_CAL;
  adc_putreg(priv, LS2K0300_ADC_CR2, cr2);

  timeout = 100000;
  while (timeout-- > 0)
    {
      cr2 = adc_getreg(priv, LS2K0300_ADC_CR2);
      if (!(cr2 & ADC_CR2_CAL))
        {
          break;
        }

      up_udelay(1);
    }

  if (timeout <= 0)
    {
      aerr("ADC CAL timeout\n");
    }

  ainfo("ADC initialized: apb=%lu clkdiv=%lu adc_clk=%lu\n",
        (unsigned long)apb_freq, (unsigned long)clkdiv,
        (unsigned long)(apb_freq / (clkdiv + 1) / 2));
}

static int ls2k0300_adc_bind(FAR struct adc_dev_s *dev,
                            FAR const struct adc_callback_s *callback)
{
  FAR struct ls2k0300_adc_dev_s *priv =
      (FAR struct ls2k0300_adc_dev_s *)dev->ad_priv;

  DEBUGASSERT(priv != NULL);
  priv->cb = callback;

  return OK;
}

static void ls2k0300_adc_reset(FAR struct adc_dev_s *dev)
{
  FAR struct ls2k0300_adc_dev_s *priv =
      (FAR struct ls2k0300_adc_dev_s *)dev->ad_priv;

  adc_hw_init(priv);
}

static int ls2k0300_adc_setup(FAR struct adc_dev_s *dev)
{
  FAR struct ls2k0300_adc_dev_s *priv =
      (FAR struct ls2k0300_adc_dev_s *)dev->ad_priv;

  adc_hw_init(priv);
  return OK;
}

static void ls2k0300_adc_shutdown(FAR struct adc_dev_s *dev)
{
  FAR struct ls2k0300_adc_dev_s *priv =
      (FAR struct ls2k0300_adc_dev_s *)dev->ad_priv;

  adc_modifyreg(priv, LS2K0300_ADC_CR2, ADC_CR2_ADON, 0);
}

static void ls2k0300_adc_rxint(FAR struct adc_dev_s *dev, bool enable)
{
  FAR struct ls2k0300_adc_dev_s *priv =
      (FAR struct ls2k0300_adc_dev_s *)dev->ad_priv;

  priv->rxint_enabled = enable;
}

static int ls2k0300_adc_ioctl(FAR struct adc_dev_s *dev, int cmd,
                             unsigned long arg)
{
  FAR struct ls2k0300_adc_dev_s *priv =
      (FAR struct ls2k0300_adc_dev_s *)dev->ad_priv;
  int ret = OK;

  switch (cmd)
    {
      case ANIOC_TRIGGER:
        {
          int i;

          if (priv->cb == NULL)
            {
              ret = -EIO;
              break;
            }

          for (i = 0; i < priv->nchannels; i++)
            {
              int32_t data = adc_read_channel(priv, priv->channels[i]);
              if (data < 0)
                {
                  ret = data;
                  break;
                }

              priv->cb->au_receive(dev, priv->channels[i], data);
            }
        }
        break;

      case ANIOC_GET_NCHANNELS:
        {
          ret = priv->nchannels;
        }
        break;

      default:
        {
          ret = -ENOTTY;
        }
        break;
    }

  return ret;
}

static const struct adc_ops_s g_adcops =
{
  ls2k0300_adc_bind,
  ls2k0300_adc_reset,
  ls2k0300_adc_setup,
  ls2k0300_adc_shutdown,
  ls2k0300_adc_rxint,
  ls2k0300_adc_ioctl,
};

static struct ls2k0300_adc_dev_s g_adcpriv =
{
  .base = LS2K0300_ADC_BASE,
  .nchannels = ADC_CHANNEL_COUNT,
  .channels =
    {
      0, 1, 2, 3, 4, 5, 6, 7
    },
};

static struct adc_dev_s g_adcdev =
{
  &g_adcops,
  &g_adcpriv,
};

FAR struct adc_dev_s *ls2k0300_adc_initialize(void)
{
  g_adcdev.ad_recv.af_fifosize = ADC_CHANNEL_COUNT + 1;
  return &g_adcdev;
}

#endif
