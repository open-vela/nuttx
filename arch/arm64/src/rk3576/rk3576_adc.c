/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_adc.c
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
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_adc.h"
#include "rk3576_adc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Conversion timeout (microseconds) */

#define SARADC_CONV_TIMEOUT_US      1000

/* Calibration samples */

#define SARADC_CAL_SAMPLES          16

/* Register access helpers */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SARADC base address */

const uint32_t g_saradc_base = RK3576_SARADC_ADDR;

/* Number of available channels */

const int g_saradc_channels = RK3576_ADC_MAX_CHANNELS;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_adc_channel_validate
 *
 * Description:
 *   Validate ADC channel number.
 *
 ****************************************************************************/

static inline int rk3576_adc_channel_validate(int channel)
{
  if (channel < 0 || channel >= g_saradc_channels)
    {
      aerr("ADC: invalid channel %d\n", channel);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_adc_wait_idle
 *
 * Description:
 *   Wait for ADC to become idle.
 *
 ****************************************************************************/

static int rk3576_adc_wait_idle(void)
{
  int timeout = SARADC_CONV_TIMEOUT_US;

  while (timeout--)
    {
      if (!(getreg32(g_saradc_base + SARADC_STAS) & SARADC_STAS_BUSY))
        {
          return OK;
        }

      up_udelay(1);
    }

  aerr("ADC: timeout waiting for idle\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_adc_wait_eoc
 *
 * Description:
 *   Wait for end of conversion.
 *
 ****************************************************************************/

static int rk3576_adc_wait_eoc(void)
{
  int timeout = SARADC_CONV_TIMEOUT_US;

  while (timeout--)
    {
      if (getreg32(g_saradc_base + SARADC_INTSTS) & SARADC_INTSTS_EOC)
        {
          /* Clear the interrupt status */

          putreg32(SARADC_INTSTS_EOC, g_saradc_base + SARADC_INTSTS);
          return OK;
        }

      up_udelay(1);
    }

  aerr("ADC: timeout waiting for EOC\n");
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_adc_startConversion
 *
 * Description:
 *   Start a single ADC conversion on the specified channel.
 *
 ****************************************************************************/

static int rk3576_adc_start_conversion(int channel)
{
  uint32_t ctrl;
  int ret;

  /* Wait for ADC to be idle */

  ret = rk3576_adc_wait_idle();
  if (ret < 0)
    {
      return ret;
    }

  /* Clear any pending interrupts */

  putreg32(SARADC_INTSTS_EOC | SARADC_INTSTS_OVF, g_saradc_base + SARADC_INTSTS);

  /* Configure and start conversion */

  ctrl = SARADC_CTRL_EN |
         SARADC_CTRL_START |
         SARADC_CTRL_SINGLE |
         SARADC_CTRL_CH(channel);

  putreg32(ctrl, g_saradc_base + SARADC_CTRL);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_adc_init
 *
 * Description:
 *   Initialize the SARADC controller.
 *   Resets the ADC, disables interrupts, and prepares for operation.
 *
 ****************************************************************************/

void rk3576_adc_init(void)
{
  /* Reset the ADC */

  putreg32(SARADC_CTRL_RESET, g_saradc_base + SARADC_CTRL);
  up_udelay(10);

  /* Disable ADC */

  putreg32(0, g_saradc_base + SARADC_CTRL);

  /* Clear all interrupts */

  putreg32(SARADC_INTSTS_EOC | SARADC_INTSTS_OVF, g_saradc_base + SARADC_INTSTS);

  /* Disable interrupts */

  putreg32(0, g_saradc_base + SARADC_INTEN);

  /* Configure delay:
   * - Power-down delay: minimal
   * - Conversion delay: adequate for 12-bit accuracy
   */

  putreg32((0x20 << SARADC_DLY_PD_SHIFT) |
           (0x20 << SARADC_DLY_CONVERT_SHIFT),
           g_saradc_base + SARADC_DLY);

  /* Enable the ADC */

  putreg32(SARADC_CTRL_EN, g_saradc_base + SARADC_CTRL);

  ainfo("ADC: initialized %d channels\n", g_saradc_channels);
}

/****************************************************************************
 * Name: rk3576_adc_read
 *
 * Description:
 *   Read a single ADC conversion from the specified channel.
 *   Returns the raw 12-bit value (0-4095).
 *
 ****************************************************************************/

int rk3576_adc_read(int channel)
{
  uint32_t data;
  int ret;

  ret = rk3576_adc_channel_validate(channel);
  if (ret < 0)
    {
      return ret;
    }

  /* Start conversion */

  ret = rk3576_adc_start_conversion(channel);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for completion */

  ret = rk3576_adc_wait_eoc();
  if (ret < 0)
    {
      return ret;
    }

  /* Read the data from the channel register */

  data = getreg32(g_saradc_base + SARADC_DATA0 + (channel * 4));

  return (int)(data & SARADC_DATA_MASK);
}

/****************************************************************************
 * Name: rk3576_adc_read_mv
 *
 * Description:
 *   Read ADC and return value in millivolts.
 *
 ****************************************************************************/

int rk3576_adc_read_mv(int channel)
{
  int raw;
  int mv;

  raw = rk3576_adc_read(channel);
  if (raw < 0)
    {
      return raw;
    }

  /* Convert to millivolts:
   * mv = (raw * VREF) / (2^resolution - 1)
   *    = (raw * 1800) / 4095
   */

  mv = (raw * SARADC_VREF_MV) / ((1 << SARADC_RESOLUTION) - 1);

  return mv;
}

/****************************************************************************
 * Name: rk3576_adc_read_voltage
 *
 * Description:
 *   Read ADC and return raw value and voltage in millivolts.
 *
 ****************************************************************************/

int rk3576_adc_read_voltage(int channel, int *mv)
{
  int raw;

  if (mv == NULL)
    {
      return -EINVAL;
    }

  raw = rk3576_adc_read(channel);
  if (raw < 0)
    {
      return raw;
    }

  *mv = (raw * SARADC_VREF_MV) / ((1 << SARADC_RESOLUTION) - 1);

  return raw;
}

/****************************************************************************
 * Name: rk3576_adc_read_avg
 *
 * Description:
 *   Read ADC multiple times and return averaged raw value.
 *   Improves accuracy by averaging out noise.
 *
 ****************************************************************************/

int rk3576_adc_read_avg(int channel, int samples)
{
  int total = 0;
  int valid = 0;
  int i;
  int val;

  if (samples <= 0)
    {
      samples = 1;
    }

  for (i = 0; i < samples; i++)
    {
      val = rk3576_adc_read(channel);
      if (val >= 0)
        {
          total += val;
          valid++;
        }
    }

  if (valid == 0)
    {
      return -EIO;
    }

  return total / valid;
}

/****************************************************************************
 * Name: rk3576_adc_set_channel
 *
 * Description:
 *   Pre-select a channel for the next conversion.
 *   Useful for setting up DMA or continuous mode.
 *
 ****************************************************************************/

int rk3576_adc_set_channel(int channel)
{
  uint32_t ctrl;
  int ret;

  ret = rk3576_adc_channel_validate(channel);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait for ADC to be idle */

  ret = rk3576_adc_wait_idle();
  if (ret < 0)
    {
      return ret;
    }

  /* Update channel select in control register */

  ctrl = getreg32(g_saradc_base + SARADC_CTRL);
  ctrl &= ~SARADC_CTRL_CH_MASK;
  ctrl |= SARADC_CTRL_CH(channel);
  putreg32(ctrl, g_saradc_base + SARADC_CTRL);

  return OK;
}
