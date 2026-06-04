/****************************************************************************
 * arch/arm/src/t113/hardware/t113_pwm.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_PWM_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_PWM_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T113 has 8 PWM channels (PWM0-PWM7).
 * Per-channel registers at PWM_BASE + 0x100 + N*0x20.
 */

#define T113_PWM_BASE           0x02000c00

#define T113_PWM_PIER           (T113_PWM_BASE + 0x00)
#define T113_PWM_PISR           (T113_PWM_BASE + 0x04)
#define T113_PWM_PCCR01         (T113_PWM_BASE + 0x20)
#define T113_PWM_PCCR23         (T113_PWM_BASE + 0x24)
#define T113_PWM_PCCR45         (T113_PWM_BASE + 0x28)
#define T113_PWM_PCCR67         (T113_PWM_BASE + 0x2c)
#define T113_PWM_PCGR           (T113_PWM_BASE + 0x40)
#define T113_PWM_PER            (T113_PWM_BASE + 0x80)

/* Per-channel registers: base + 0x100 + ch * 0x20 */

#define T113_PWM_CH_BASE(ch)    (T113_PWM_BASE + 0x100 + (ch) * 0x20)
#define T113_PWM_PCR(ch)        (T113_PWM_CH_BASE(ch) + 0x00)
#define T113_PWM_PPR(ch)        (T113_PWM_CH_BASE(ch) + 0x04)
#define T113_PWM_PCNTR(ch)      (T113_PWM_CH_BASE(ch) + 0x08)

/* PCR bits */

#define T113_PWM_PCR_PRESCAL_MASK  0xff
#define T113_PWM_PCR_ACT_STA       (1 << 8)

/* PPR bits:
 *   [31:16] = entire period cycles
 *   [15:0]  = active cycles
 */

#define T113_PWM_PPR_PERIOD_SHIFT   16
#define T113_PWM_PPR_ACTIVE_SHIFT   0

/* PER bits: bit N = enable channel N */

/* PCGR bits: bit N = clock gate for channel N */

/* PCCR bits: [8:7] = clock source, [4] = clock gating, [3:0] = divider M
 *   clock source: 0=HOSC(24MHz), 1=APB0
 */

#define T113_PWM_PCCR_SRC_SHIFT    7
#define T113_PWM_PCCR_SRC_MASK     (0x3 << T113_PWM_PCCR_SRC_SHIFT)
#define T113_PWM_PCCR_CLK_GATING   (1 << 4)
#define T113_PWM_PCCR_DIV_SHIFT    0
#define T113_PWM_PCCR_DIV_MASK     0x0f

#define T113_PWM_NCHANNELS         8

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_PWM_H */
