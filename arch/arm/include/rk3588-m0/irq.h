/****************************************************************************
 * arch/arm/include/rk3588-m0/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_RK3588_M0_IRQ_H
#define __ARCH_ARM_INCLUDE_RK3588_M0_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/rk3588-m0/chip.h>

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* IRQ numbers map directly to exception vector numbers, which in turn map to
 * bits in the NVIC.
 *
 * Processor exceptions (vectors 0-15)
 */

#define RK3588M0_IRQ_RESERVED    (0) /* Reserved vector (only with CONFIG_DEBUG_FEATURES) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handled as an IRQ) */
#define RK3588M0_IRQ_NMI         (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define RK3588M0_IRQ_HARDFAULT   (3) /* Vector  3: Hard fault */
                                     /* Vectors 4-10: Reserved */
#define RK3588M0_IRQ_SVCALL     (11) /* Vector 11: SVC call */
                                     /* Vectors 12-13: Reserved */
#define RK3588M0_IRQ_PENDSV     (14) /* Vector 14: Pendable system service request */
#define RK3588M0_IRQ_SYSTICK    (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).
 *
 * PMU_M0 has 32 IRQ lines (TRM Table 9-1).  The sources are listed in TRM
 * Table 9-7 "PMU_M0 Interrupt"; IRQ 16-23 are the INTMUX outputs, through
 * which the 512 SoC interrupts are multiplexed (64 per line).
 */

#define RK3588M0_IRQ_EXTINT     (16) /* Vector number of the first external IRQ */

#define RK3588M0_IRQ_PMIC       (16) /* PMIC */
#define RK3588M0_IRQ_SDMMC_DET  (17) /* SDMMC_DETECTN */
#define RK3588M0_IRQ_UART0      (18) /* UART0 */
#define RK3588M0_IRQ_GPIO0      (19) /* GPIO0 */
#define RK3588M0_IRQ_GPIO0_EXP  (20) /* GPIO0_EXP */
#define RK3588M0_IRQ_I2C0       (21) /* I2C0 */
#define RK3588M0_IRQ_PDM0       (23) /* PDM0 */
#define RK3588M0_IRQ_PWM0       (27) /* PWM0 */

/* The INTMUX outputs, i.e. external lines 16-23.  Each carries 64 of the SoC's
 * 512 interrupt sources, numbered as in TRM Table 1-3: line 0 below takes
 * source ids 0-63, line 1 takes 64-127, and so on up to 448-511.  Sources
 * 0-255 arrive through INTMUX0 and 256-511 through INTMUX1, but from the NVIC's
 * point of view the eight lines are contiguous.
 *
 * Use RK3588M0_INTMUX_EXTINT_OF(id) to find the line for a source id, then add
 * RK3588M0_IRQ_EXTINT to get the vector number these constants use.
 */

#define RK3588M0_IRQ_INTMUX(n)  (RK3588M0_IRQ_EXTINT + 16 + (n)) /* n = 0..7 */

#define RK3588M0_IRQ_WDT_PMU    (28) /* WDT_PMU */
#define RK3588M0_IRQ_TIMER1_PMU (29) /* TIMER1_PMU */
#define RK3588M0_IRQ_TIMER0_PMU (30) /* TIMER0_PMU */

#define RK3588M0_IRQ_NEXTINT    (32) /* 32 external interrupts */

#define NR_IRQS                 (RK3588M0_IRQ_EXTINT + RK3588M0_IRQ_NEXTINT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_ARM_INCLUDE_RK3588_M0_IRQ_H */
