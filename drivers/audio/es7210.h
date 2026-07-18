/****************************************************************************
 * drivers/audio/es7210.h
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

#ifndef __DRIVERS_AUDIO_ES7210_H
#define __DRIVERS_AUDIO_ES7210_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/audio/audio.h>
#include <nuttx/audio/es7210.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mutex.h>
#include <nuttx/queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ES7210 Register Addresses */

#define ES7210_RESET_REG00          0x00 /* Reset control */
#define ES7210_CLK_ON_REG01         0x01 /* Clock manager 1 */
#define ES7210_MCLK_CTL_REG02       0x02 /* Master clock control */
#define ES7210_MST_CLK_CTL_REG03    0x03 /* Master clock divider */
#define ES7210_MST_LRCDIVH_REG04    0x04 /* LRCK divider high */
#define ES7210_MST_LRCDIVL_REG05    0x05 /* LRCK divider low */
#define ES7210_DIGITAL_PDN_REG06    0x06 /* Digital power down */
#define ES7210_ADC_OSR_REG07        0x07 /* ADC over-sampling ratio */
#define ES7210_MODE_CFG_REG08       0x08 /* Mode config */
#define ES7210_TCT0_CHPINI_REG09    0x09 /* Time control 0 */
#define ES7210_TCT1_CHPINI_REG0A    0x0A /* Time control 1 */
#define ES7210_CHIP_STA_REG0B       0x0B /* Chip status */
#define ES7210_IRQ_CTL_REG0C        0x0C /* IRQ control */
#define ES7210_MISC_CTL_REG0D       0x0D /* Misc control */
#define ES7210_DMIC_CTL_REG10       0x10 /* DMIC control */
#define ES7210_SDP_CFG1_REG11       0x11 /* SDP config 1 */
#define ES7210_SDP_CFG2_REG12       0x12 /* SDP config 2 */
#define ES7210_ADC_AUTOMUTE_REG13   0x13 /* ADC automute */
#define ES7210_ADC34_MUTE_REG14     0x14 /* ADC34 mute control */
#define ES7210_ADC12_MUTE_REG15     0x15 /* ADC12 mute control */
#define ES7210_ALC_SEL_REG16        0x16 /* ALC select */
#define ES7210_ALC_COM_CFG1_REG17   0x17 /* ALC common config 1 */
#define ES7210_ALC_COM_CFG2_REG18   0x18 /* ALC common config 2 */
#define ES7210_ALC1_MAX_GAIN_REG1A  0x1A /* ALC1 max gain */
#define ES7210_ALC1_MIN_GAIN_REG1B  0x1B /* ALC1 min gain */
#define ES7210_ALC1_LVL_REG1C       0x1C /* ALC1 level */
#define ES7210_ALC2_MAX_GAIN_REG1D  0x1D /* ALC2 max gain */
#define ES7210_ALC2_MIN_GAIN_REG1E  0x1E /* ALC2 min gain */
#define ES7210_ALC2_LVL_REG1F       0x1F /* ALC2 level */
#define ES7210_ADC1_HPF_REG20       0x20 /* ADC1 HPF coefficient */
#define ES7210_ADC2_HPF_REG21       0x21 /* ADC2 HPF coefficient */
#define ES7210_ADC3_HPF_REG22       0x22 /* ADC3 HPF coefficient */
#define ES7210_ADC4_HPF_REG23       0x23 /* ADC4 HPF coefficient */
#define ES7210_ALC4_MIN_GAIN_REG24  0x24 /* ALC4 min gain */
#define ES7210_ALC4_LVL_REG25       0x25 /* ALC4 level */
#define ES7210_ADC12_HPF2_REG2A     0x2A /* ADC12 HPF coeff 2 */
#define ES7210_ADC12_HPF1_REG2B     0x2B /* ADC12 HPF coeff 1 */
#define ES7210_ADC34_HPF2_REG2C     0x2C /* ADC34 HPF coeff 2 */
#define ES7210_ADC34_HPF1_REG2D     0x2D /* ADC34 HPF coeff 1 */
#define ES7210_ANALOG_SYS_REG40     0x40 /* Analog system */
#define ES7210_MICBIAS12_REG41      0x41 /* MIC bias 12 */
#define ES7210_MICBIAS34_REG42      0x42 /* MIC bias 34 */
#define ES7210_ADC1_GAIN_REG43      0x43 /* ADC1 PGA gain */
#define ES7210_ADC2_GAIN_REG44      0x44 /* ADC2 PGA gain */
#define ES7210_ADC3_GAIN_REG45      0x45 /* ADC3 PGA gain */
#define ES7210_ADC4_GAIN_REG46      0x46 /* ADC4 PGA gain */
#define ES7210_MIC1_POWER_REG47     0x47 /* MIC1 power */
#define ES7210_MIC2_POWER_REG48     0x48 /* MIC2 power */
#define ES7210_MIC3_POWER_REG49     0x49 /* MIC3 power */
#define ES7210_MIC4_POWER_REG4A     0x4A /* MIC4 power */
#define ES7210_MIC12_POWER_REG4B    0x4B /* MIC12 power */
#define ES7210_MIC34_POWER_REG4C    0x4C /* MIC34 power */

/* SDP Format */

#define ES7210_SDP_FMT_I2S          0x00
#define ES7210_SDP_FMT_LJ           0x01
#define ES7210_SDP_FMT_DSP_A        0x03
#define ES7210_SDP_FMT_DSP_B        0x13

/* SDP Word Length */

#define ES7210_SDP_WL_16BIT         0x03
#define ES7210_SDP_WL_18BIT         0x02
#define ES7210_SDP_WL_20BIT         0x01
#define ES7210_SDP_WL_24BIT         0x00
#define ES7210_SDP_WL_32BIT         0x04

/* MIC Gain (dB) */

#define ES7210_MIC_GAIN_0DB         0x00
#define ES7210_MIC_GAIN_3DB         0x01
#define ES7210_MIC_GAIN_6DB         0x02
#define ES7210_MIC_GAIN_9DB         0x03
#define ES7210_MIC_GAIN_12DB        0x04
#define ES7210_MIC_GAIN_15DB        0x05
#define ES7210_MIC_GAIN_18DB        0x06
#define ES7210_MIC_GAIN_21DB        0x07
#define ES7210_MIC_GAIN_24DB        0x08
#define ES7210_MIC_GAIN_27DB        0x09
#define ES7210_MIC_GAIN_30DB        0x0A
#define ES7210_MIC_GAIN_33DB        0x0B
#define ES7210_MIC_GAIN_34_5DB      0x0C
#define ES7210_MIC_GAIN_36DB        0x0D
#define ES7210_MIC_GAIN_37_5DB      0x0E

/* PGA gain register bit 4: analog front-end power enable */

#define ES7210_ADC_PGA_POWER_ON     0x10

/* Reset register values */

#define ES7210_RESET_CMD            0xFF /* Software reset command */
#define ES7210_RESET_NORMAL         0x41 /* Normal operation after reset */
#define ES7210_CLOCK_ENABLE_ALL     0x20 /* Enable clocks for ADC1-4 */
#define ES7210_DEVICE_ON            0x41 /* Device enable */

/* SDP configuration */

#define ES7210_SDP_I2S_16BIT        0x60 /* I2S format, 16-bit */
#define ES7210_SDP_TDM              0x02 /* Four-channel TDM mode */

/* Analog system */

#define ES7210_VMID_SELECT          0x43 /* VMID voltage selection */
#define ES7210_MICBIAS_2V87         0x70 /* MIC bias 2.87V */

/* MIC / ADC power */

#define ES7210_MIC_POWER_ON         0x08 /* Individual MIC power on */
#define ES7210_MIC_POWER_DOWN       0xff /* Individual MIC power down */
#define ES7210_MIC_ADC_PGA_ON       0x00 /* MIC bias + ADC + PGA on */

/* MCLK / clock */

#define ES7210_MCLK_ADC_DIV1_DLL    0xC1 /* ADC_DIV=1, doubler, DLL */
#define ES7210_DIGITAL_POWER_ON     0x00 /* DLL and ADC digital power on */

/* Mute control */

#define ES7210_ADC_MUTE             0x03 /* Mute both ADC channels */
#define ES7210_ADC_UNMUTE           0x00 /* Unmute both ADC channels */

/* Default configuration */

#define ES7210_DEFAULT_SAMPRATE     16000
#define ES7210_DEFAULT_NCHANNELS    4
#define ES7210_DEFAULT_BPSAMP       16

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct es7210_dev_s
{
  /* We are an audio lower-half driver.  The structure also holds the
   * codec-specific state used by the lower half.
   */

  struct audio_lowerhalf_s dev;

  /* Our specific driver data goes here */

  FAR struct i2c_master_s  *i2c;        /* I2C driver to use */
  FAR struct i2s_dev_s     *i2s;        /* I2S driver to use */
  struct es7210_lower_s    lower;       /* Platform-specific config */
  mutex_t                  devlock;     /* Assures mutually exclusive
                                         * access to driver */

  /* Driver state */

  uint32_t                 samprate;    /* Sample rate in samples/sec */
  uint8_t                  nchannels;   /* Number of TDM channels (4) */
  uint8_t                  bpsamp;      /* Bits per sample (16) */
  volatile bool            running;     /* True: Worker thread is running */
  volatile bool            paused;      /* True: Playing is paused */
  bool                     reserved;    /* True: Device is reserved */

  /* Buffer management */

  struct dq_queue_s        pendq;       /* Queue of pending buffers to be
                                         * received */
};

#endif /* __DRIVERS_AUDIO_ES7210_H */
