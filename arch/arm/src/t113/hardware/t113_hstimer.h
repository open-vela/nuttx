/****************************************************************************
 * arch/arm/src/t113/hardware/t113_hstimer.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_HSTIMER_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_HSTIMER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HSTimer base address */

#define T113_HSTIMER_BASE             0x03008000

/* Per-channel register offset: channel 0 at +0x20, channel 1 at +0x40 */

#define T113_HSTIMER_CH_OFFSET(n)     (0x20 + (n) * 0x20)

/* Global registers */

#define T113_HSTIMER_IRQ_EN           (T113_HSTIMER_BASE + 0x0000)
#define T113_HSTIMER_IRQ_STAS         (T113_HSTIMER_BASE + 0x0004)

/* Per-channel registers */

#define T113_HSTIMER_CTRL(n)          (T113_HSTIMER_BASE + \
                                       T113_HSTIMER_CH_OFFSET(n) + 0x00)
#define T113_HSTIMER_INTV_LO(n)       (T113_HSTIMER_BASE + \
                                       T113_HSTIMER_CH_OFFSET(n) + 0x04)
#define T113_HSTIMER_INTV_HI(n)       (T113_HSTIMER_BASE + \
                                       T113_HSTIMER_CH_OFFSET(n) + 0x08)
#define T113_HSTIMER_CURNT_LO(n)      (T113_HSTIMER_BASE + \
                                       T113_HSTIMER_CH_OFFSET(n) + 0x0c)
#define T113_HSTIMER_CURNT_HI(n)      (T113_HSTIMER_BASE + \
                                       T113_HSTIMER_CH_OFFSET(n) + 0x10)

/* HS_TMR_IRQ_EN_REG bit definitions */

#define HSTIMER_IRQ_EN_TMR(n)         (1 << (n))   /* bit0=TMR0, bit1=TMR1 */

/* HS_TMR_IRQ_STAS_REG bit definitions (write-1-to-clear) */

#define HSTIMER_IRQ_PEND_TMR(n)       (1 << (n))   /* bit0=TMR0, bit1=TMR1 */

/* HS_TMRn_CTRL_REG bit definitions */

#define HSTIMER_CTRL_EN               (1 << 0)     /* Timer enable */
#define HSTIMER_CTRL_RELOAD           (1 << 1)     /* Reload interval (W1S) */
#define HSTIMER_CTRL_CLK_PRESCALE_SHIFT 4
#define HSTIMER_CTRL_CLK_PRESCALE_MASK  (0x7 << 4) /* bits[6:4] */
#define HSTIMER_CTRL_CLK_PRESCALE(n)  ((n) << 4)   /* 0=/1 .. 4=/16 */
#define HSTIMER_CTRL_MODE_PERIODIC    (0 << 7)     /* Continuous mode */
#define HSTIMER_CTRL_MODE_ONESHOT     (1 << 7)     /* One-shot mode */
#define HSTIMER_CTRL_TEST             (1 << 31)    /* Test mode */

/* Prescaler values: divider = 1 << prescale_field (max /16 = field 4) */

#define HSTIMER_PRESCALE_1            0   /* /1 */
#define HSTIMER_PRESCALE_2            1   /* /2 */
#define HSTIMER_PRESCALE_4            2   /* /4 */
#define HSTIMER_PRESCALE_8            3   /* /8 */
#define HSTIMER_PRESCALE_16           4   /* /16 */

/* 56-bit counter: high part uses bits[23:0] */

#define HSTIMER_INTV_HI_MASK          0x00ffffff
#define HSTIMER_CURNT_HI_MASK         0x00ffffff

/* Maximum 56-bit interval value */

#define HSTIMER_MAX_COUNT             0x00ffffffffffffffull

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_HSTIMER_H */
