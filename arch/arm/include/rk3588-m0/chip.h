/****************************************************************************
 * arch/arm/include/rk3588-m0/chip.h
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

#ifndef __ARCH_ARM_INCLUDE_RK3588_M0_CHIP_H
#define __ARCH_ARM_INCLUDE_RK3588_M0_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The RK3588 PMU Cortex-M0 ("PMU_M0") is one of three M0 subsystems in the
 * SoC.  Per RK3588 TRM Table 9-1 (MCU Subsystem Configurations) it has:
 *
 *   TCM      16KB
 *   CACHE    16KB
 *   IRQ_NUM  32        <- number of NVIC external interrupts
 *   INTMUX   yes       <- SoC interrupts are multiplexed onto the IRQ lines
 *
 * Priorities: ARMv6-M implements 2 bits of priority (4 levels) in the upper
 * bits of each 8-bit priority field.
 */

#define NVIC_SYSH_PRIORITY_MIN     0xc0 /* All bits set in minimum priority */
#define NVIC_SYSH_PRIORITY_DEFAULT 0x80 /* Midpoint is the default */
#define NVIC_SYSH_PRIORITY_MAX     0x00 /* Zero is maximum priority */
#define NVIC_SYSH_PRIORITY_STEP    0x40 /* Two bits of interrupt priority */

#endif /* __ARCH_ARM_INCLUDE_RK3588_M0_CHIP_H */
