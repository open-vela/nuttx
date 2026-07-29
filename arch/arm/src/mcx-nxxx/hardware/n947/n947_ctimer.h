/****************************************************************************
 * arch/arm/src/mcx-nxxx/hardware/n947/n947_ctimer.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_CTIMER_H
#define __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_CTIMER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define N947_CTIMER_IR_OFFSET           0x0000
#define N947_CTIMER_TCR_OFFSET          0x0004
#define N947_CTIMER_TC_OFFSET           0x0008
#define N947_CTIMER_PR_OFFSET           0x000c
#define N947_CTIMER_PC_OFFSET           0x0010
#define N947_CTIMER_MCR_OFFSET          0x0014
#define N947_CTIMER_MR_OFFSET(n)        (0x0018 + ((n) << 2))
#define N947_CTIMER_CCR_OFFSET          0x0028
#define N947_CTIMER_CR_OFFSET(n)        (0x002c + ((n) << 2))
#define N947_CTIMER_EMR_OFFSET          0x003c
#define N947_CTIMER_CTCR_OFFSET         0x0070
#define N947_CTIMER_PWMC_OFFSET         0x0074
#define N947_CTIMER_MSR_OFFSET(n)       (0x0078 + ((n) << 2))

/* Interrupt register */

#define CTIMER_IR_MRINT(n)              (1u << (n))
#define CTIMER_IR_CRINT(n)              (1u << (4 + (n)))
#define CTIMER_IR_ALL                   0xffu

/* Timer control register */

#define CTIMER_TCR_CEN                  (1u << 0)
#define CTIMER_TCR_CRST                 (1u << 1)
#define CTIMER_TCR_AGCEN                (1u << 4)
#define CTIMER_TCR_ATCEN                (1u << 5)

/* Match control register */

#define CTIMER_MCR_MRI(n)               (1u << ((n) * 3))
#define CTIMER_MCR_MRR(n)               (1u << (((n) * 3) + 1))
#define CTIMER_MCR_MRS(n)               (1u << (((n) * 3) + 2))
#define CTIMER_MCR_MR_MASK(n)           (7u << ((n) * 3))
#define CTIMER_MCR_MRRL(n)              (1u << (24 + (n)))

/* External match register */

#define CTIMER_EMR_EM(n)                (1u << (n))
#define CTIMER_EMR_EMC_SHIFT(n)         (4 + ((n) * 2))
#define CTIMER_EMR_EMC_MASK(n)          (3u << CTIMER_EMR_EMC_SHIFT(n))
#define CTIMER_EMR_EMC_NOACTION         0u
#define CTIMER_EMR_EMC_CLEAR            1u
#define CTIMER_EMR_EMC_SET              2u
#define CTIMER_EMR_EMC_TOGGLE           3u

/* Count control register */

#define CTIMER_CTCR_CTMODE_MASK         (3u << 0)
#define CTIMER_CTCR_CTMODE_TIMER        (0u << 0)
#define CTIMER_CTCR_CINSEL_MASK         (3u << 2)

/* PWM control register */

#define CTIMER_PWMC_PWMEN(n)            (1u << (n))

/* This BSP selects FRO_HF as CTIMER functional clock.  nxxx_clockconfig()
 * programs FRO_HF to 48 MHz and enables it for peripheral use.
 */

#define N947_CTIMER_FRO_HF_FREQ         48000000u

#endif /* __ARCH_ARM_SRC_MCX_NXXX_HARDWARE_N947_CTIMER_H */
