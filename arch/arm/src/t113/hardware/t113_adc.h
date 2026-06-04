/****************************************************************************
 * arch/arm/src/t113/hardware/t113_adc.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_ADC_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_ADC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T113-S3 GPADC - 12-bit SAR, single independent channel (GPADC0, pin C9).
 * The register file supports up to four channels in silicon, but on T113-S3
 * only CH0 is bonded out.  HOSC 24 MHz is the default input clock.
 * Vref = AVCC (nominally 1.8 V on the EVB).
 */

#define T113_GPADC_BASE             0x02009000

#define T113_GPADC_SR               (T113_GPADC_BASE + 0x00)
#define T113_GPADC_CTRL             (T113_GPADC_BASE + 0x04)
#define T113_GPADC_CS_EN            (T113_GPADC_BASE + 0x08)
#define T113_GPADC_FIFO_INTC        (T113_GPADC_BASE + 0x0c)
#define T113_GPADC_FIFO_INTS        (T113_GPADC_BASE + 0x10)
#define T113_GPADC_FIFO_DATA        (T113_GPADC_BASE + 0x14)
#define T113_GPADC_CB_DATA          (T113_GPADC_BASE + 0x18)
#define T113_GPADC_DATAL_INTC       (T113_GPADC_BASE + 0x20)
#define T113_GPADC_DATAH_INTC       (T113_GPADC_BASE + 0x24)
#define T113_GPADC_DATA_INTC        (T113_GPADC_BASE + 0x28)
#define T113_GPADC_DATAL_INTS       (T113_GPADC_BASE + 0x30)
#define T113_GPADC_DATAH_INTS       (T113_GPADC_BASE + 0x34)
#define T113_GPADC_DATA_INTS        (T113_GPADC_BASE + 0x38)

/* Per-channel data register: read-only, 12-bit conversion result. */

#define T113_GPADC_CH_DATA(n)       (T113_GPADC_BASE + 0x80 + (n) * 4)

/* SR register - sample-rate divider.  Divider field at bits [31:16];
 * sample_rate = clk_in / (div + 1).
 */

#define T113_GPADC_SR_DIV_SHIFT     16
#define T113_GPADC_SR_DIV_MASK      (0xffffu << 16)

/* CTRL register bits. */

#define T113_GPADC_CTRL_LDO_EN      (1 << 0)       /* Analog LDO enable    */
#define T113_GPADC_CTRL_ADC_EN      (1 << 16)      /* GPADC function EN    */
#define T113_GPADC_CTRL_CALI_EN     (1 << 17)      /* Calibration enable   */
#define T113_GPADC_CTRL_AUTOCALI_EN (1 << 23)      /* Auto-cal on EN       */
#define T113_GPADC_CTRL_MODE_SHIFT  18
#define T113_GPADC_CTRL_MODE_MASK   (3u << 18)
#define T113_GPADC_CTRL_MODE_SINGLE (0u << 18)     /* 00: single shot      */
#define T113_GPADC_CTRL_MODE_CYCLE  (1u << 18)     /* 01: single cycle     */
#define T113_GPADC_CTRL_MODE_CONT   (2u << 18)     /* 10: continuous       */
#define T113_GPADC_CTRL_MODE_BURST  (3u << 18)     /* 11: burst            */
#define T113_GPADC_CTRL_FIRST_DLY_SHIFT 24
#define T113_GPADC_CTRL_FIRST_DLY_MASK  (0xffu << 24)

/* CS_EN register - channel select (bits 0..7) and compare enable
 * (bits 16..23).  Bit N = enable channel N for conversion.
 */

#define T113_GPADC_CS_EN_CH(n)      (1u << (n))
#define T113_GPADC_CS_EN_CMP(n)     (1u << (16 + (n)))

/* FIFO_INTC - FIFO interrupt enables. */

#define T113_GPADC_FIFO_FLUSH       (1u << 4)
#define T113_GPADC_FIFO_DATA_IRQ_EN (1u << 16)     /* FIFO-data IRQ enable */
#define T113_GPADC_FIFO_OVER_IRQ_EN (1u << 17)     /* FIFO-overrun IRQ en  */

/* FIFO_INTS - FIFO interrupt status (W1C for pending bits). */

#define T113_GPADC_FIFO_CNT_SHIFT   8
#define T113_GPADC_FIFO_CNT_MASK    (0x3fu << 8)
#define T113_GPADC_FIFO_DATA_PEND   (1u << 16)
#define T113_GPADC_FIFO_OVER_PEND   (1u << 17)

/* FIFO_DATA register - 12-bit sample. */

#define T113_GPADC_FIFO_DATA_MASK   0xfffu

/* Per-channel data-interrupt register (DATA_INTC): bit N = enable IRQ
 * when CH(N) conversion completes.  DATA_INTS mirrors W1C pending bits.
 */

#define T113_GPADC_DATA_IRQ(n)      (1u << (n))

/* Data mask - 12-bit resolution. */

#define T113_GPADC_DATA_MASK        0xfffu

/* Reference voltage and input range.  T113-S3 GPADC0 nominal range is
 * 0..VOL_RANGE at 12-bit resolution.
 */

#define T113_GPADC_VREF_MV          1800
#define T113_GPADC_MAX_VALUE        4095

/* T113-S3 exposes one usable GPADC channel (CH0 on pin PC9). */

#define T113_GPADC_NCHANNELS        1

/* Sample-rate clock source - HOSC 24 MHz. */

#define T113_GPADC_CLK_HZ           24000000u
#define T113_GPADC_DEFAULT_SR_HZ    1000u

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_ADC_H */
