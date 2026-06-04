/****************************************************************************
 * arch/arm/src/t113/hardware/t113_lradc.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_LRADC_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_LRADC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T113-S3 / R528 LRADC - 6-bit low resolution ADC intended for resistor
 * ladder key scanning.  T113-S3 and R528 are the same silicon; the LRADC
 * block is present on this die even though the T113-S3 public datasheet
 * only documents the GPADC and TPADC blocks.
 */

#define T113_LRADC_BASE             0x02009800

/* Register offsets */

#define T113_LRADC_CTRL_OFFSET      0x00  /* Control register */
#define T113_LRADC_INTC_OFFSET      0x04  /* Interrupt control register */
#define T113_LRADC_INTS_OFFSET      0x08  /* Interrupt status register */
#define T113_LRADC_DATA0_OFFSET     0x0c  /* Data register (channel 0) */
#define T113_LRADC_REV_OFFSET       0x100 /* Revision register */

#define T113_LRADC_CTRL             (T113_LRADC_BASE + T113_LRADC_CTRL_OFFSET)
#define T113_LRADC_INTC             (T113_LRADC_BASE + T113_LRADC_INTC_OFFSET)
#define T113_LRADC_INTS             (T113_LRADC_BASE + T113_LRADC_INTS_OFFSET)
#define T113_LRADC_DATA0            (T113_LRADC_BASE + T113_LRADC_DATA0_OFFSET)

/* CTRL register bit/field encodings.
 *  bit0      LRADC enable
 *  bits3:2   sample rate (00=250Hz 01=125Hz 10=62.5Hz 11=32Hz)
 *  bits5:4   trigger level B (0 = default, lowest)
 *  bit7      hold key enable
 *  bits13:12 key mode (00 = normal)
 *  bits23:22 channel select (00 = channel 0)
 *  bits31:24 first-convert delay
 */

#define T113_LRADC_CTRL_EN              (1 << 0)
#define T113_LRADC_CTRL_SRATE_SHIFT     2
#define T113_LRADC_CTRL_SRATE_MASK      (3 << 2)
#define T113_LRADC_CTRL_SRATE_250HZ     (0 << 2)
#define T113_LRADC_CTRL_SRATE_125HZ     (1 << 2)
#define T113_LRADC_CTRL_SRATE_62HZ      (2 << 2)
#define T113_LRADC_CTRL_SRATE_32HZ      (3 << 2)
#define T113_LRADC_CTRL_LEVELB_MASK     (3 << 4)
#define T113_LRADC_CTRL_HOLD_EN         (1 << 7)
#define T113_LRADC_CTRL_KEY_MODE_SHIFT  12
#define T113_LRADC_CTRL_KEY_MODE_MASK   (3 << 12)
#define T113_LRADC_CTRL_KEY_MODE_NORM   (0 << 12)
#define T113_LRADC_CTRL_CHAN_SHIFT      22
#define T113_LRADC_CTRL_CHAN_MASK       (3 << 22)
#define T113_LRADC_CTRL_CHAN0           (0 << 22)
#define T113_LRADC_CTRL_FIRST_DLY_SHIFT 24
#define T113_LRADC_CTRL_FIRST_DLY_MASK  (0xff << 24)

/* INTC register bits */

#define T113_LRADC_INTC_ADC0_DATA_EN (1 << 0)  /* data available IRQ enable */
#define T113_LRADC_INTC_ADC0_DOWN_EN (1 << 1)  /* key-down edge IRQ enable */
#define T113_LRADC_INTC_ADC0_UP_EN   (1 << 4)  /* key-up edge IRQ enable */

/* INTS register bits (W1C) */

#define T113_LRADC_INTS_ADC0_DATA   (1 << 0)   /* data pending */
#define T113_LRADC_INTS_ADC0_DOWN   (1 << 1)   /* key-down pending */
#define T113_LRADC_INTS_ADC0_UP     (1 << 4)   /* key-up pending */

/* DATA0 register: 6-bit result in bits[5:0]; full-scale = 63 at idle.
 * Vendor HAL reads the entire 24-bit field; only the low 6 bits are
 * meaningful ADC data.
 */

#define T113_LRADC_DATA_MASK        0x3f
#define T113_LRADC_MAX_VALUE        0x3f

/* LRADC BGR (Bus Gating Reset) register bits - the register address is
 * T113_CCU_LRADC_BGR in t113_ccu.h (CCU_BASE + 0x0a9c).
 */

#define T113_LRADC_BGR_RST          (1 << 16)  /* de-assert reset */
#define T113_LRADC_BGR_GATING       (1 << 0)   /* enable bus clock gate */

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_LRADC_H */
