/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_adc.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_ADC_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_ADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SARADC channel count */

#define RK3576_ADC_MAX_CHANNELS     6

/* SARADC register offsets (Rockchip standard) */

#define SARADC_DATA0               0x00   /* Channel 0 data (R) */
#define SARADC_DATA1               0x04   /* Channel 1 data (R) */
#define SARADC_DATA2               0x08   /* Channel 2 data (R) */
#define SARADC_DATA3               0x0c   /* Channel 3 data (R) */
#define SARADC_DATA4               0x10   /* Channel 4 data (R) */
#define SARADC_DATA5               0x14   /* Channel 5 data (R) */
#define SARADC_DLY                 0x30   /* Delay control (R/W) */
#define SARADC_CTRL                0x34   /* Control register (R/W) */
#define SARADC_DMSR                0x38   /* DMA control (R/W) */
#define SARADC_INTEN               0x3c   /* Interrupt enable (R/W) */
#define SARADC_INTSTS              0x40   /* Interrupt status (W1C) */
#define SARADC_STAS                0x44   /* Status register (R) */

/* SARADC_CTRL bit definitions */

#define SARADC_CTRL_EN             (1 << 0)   /* ADC enable */
#define SARADC_CTRL_START          (1 << 1)   /* Start conversion */
#define SARADC_CTRL_RESET          (1 << 2)   /* ADC reset */
#define SARADC_CTRL_CH_SHIFT       4
#define SARADC_CTRL_CH_MASK        (0x7 << SARADC_CTRL_CH_SHIFT)
#define SARADC_CTRL_CH(ch)         ((ch) << SARADC_CTRL_CH_SHIFT)
#define SARADC_CTRL_CONT           (1 << 8)   /* Continuous mode */
#define SARADC_CTRL_SINGLE         (0 << 8)   /* Single conversion */

/* SARADC_INTEN bit definitions */

#define SARADC_INTEN_EOC_EN        (1 << 0)   /* End of conversion IRQ */

/* SARADC_INTSTS bit definitions */

#define SARADC_INTSTS_EOC          (1 << 0)   /* End of conversion (W1C) */
#define SARADC_INTSTS_OVF          (1 << 1)   /* Data overflow (W1C) */

/* SARADC_STAS bit definitions */

#define SARADC_STAS_BUSY           (1 << 0)   /* ADC busy */

/* SARADC_DLY bit definitions */

#define SARADC_DLY_PD_SHIFT        0
#define SARADC_DLY_PD_MASK         (0xff << SARADC_DLY_PD_SHIFT)
#define SARADC_DLY_CONVERT_SHIFT   8
#define SARADC_DLY_CONVERT_MASK    (0xff << SARADC_DLY_CONVERT_SHIFT)

/* SARADC data mask (12-bit ADC) */

#define SARADC_DATA_MASK           0xfff

/* SARADC reference voltage (from hardware) */

#define SARADC_VREF_MV             1800   /* 1.8V reference in mV */

/* SARADC resolution */

#define SARADC_RESOLUTION          12     /* 12-bit ADC */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* SARADC base address */

extern const uint32_t g_saradc_base;

/* Number of available channels */

extern const int g_saradc_channels;

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_ADC_H */
