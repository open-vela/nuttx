/****************************************************************************
 * arch/xtensa/include/t113/irq.h
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

#ifndef __ARCH_XTENSA_INCLUDE_T113_IRQ_H
#define __ARCH_XTENSA_INCLUDE_T113_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/xtensa/core.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers for internal interrupts that the NuttX xtensa core dispatches
 * like peripheral interrupts.  These numbers are referenced by name from
 * arch/xtensa/src/common (XTENSA_IRQ_SYSCALL, XTENSA_IRQ_SWINT).
 *
 * The two CCOMPARE timers map to XCHAL_TIMER0_INTERRUPT=2 and
 * XCHAL_TIMER1_INTERRUPT=1 in HiFi4 silicon.  These are Xtensa-internal
 * interrupts and unrelated to the SoC peripheral TIMER IP listed below.
 */

#define XTENSA_IRQ_TIMER0           0  /* CCOMPARE0 */
#define XTENSA_IRQ_TIMER1           1  /* CCOMPARE1 */
#define XTENSA_IRQ_SYSCALL          2  /* EXCCAUSE=syscall pseudo-IRQ */
#define XTENSA_IRQ_SWINT            3  /* Software interrupt (logical slot) */

/* MSGBOX is hard-wired to Xtensa external interrupt 3 (EXTERN_LEVEL,
 * BInterrupt[1]).  NuttX context switching uses the SYSCALL exception
 * (EXCCAUSE=8), not an Xtensa software-interrupt bit, so the SWINT logical
 * slot (index 3) is unused and is reused here for the MSGBOX handler.
 * T113_MSGBOX_INT is the Xtensa INTERRUPT-register bit; XTENSA_IRQ_MSGBOX
 * is the NuttX logical IRQ the driver attaches to.
 */

#define T113_MSGBOX_INT             3  /* Xtensa INT bit (silicon-fixed) */
#define XTENSA_IRQ_MSGBOX           XTENSA_IRQ_SWINT

#define XTENSA_NIRQ_INTERNAL        4
#define XTENSA_IRQ_FIRSTPERIPH      4

/* R_INTC source IDs.
 *
 * The DSP-side R_INTC controller (base 0x01700800) fans the SoC peripheral
 * interrupt sources into a single Xtensa INT line (bit 20, level 1).  It has
 * its OWN source numbering, distinct from the ARM GIC SPI numbers -- the two
 * MUST NOT be conflated.  Source numbers for the sun8iw20 DSP peripheral
 * set:
 *
 *   UART0..3       = source 1..4
 *   GPIO B/C/D/E/F/G = source 40/42/44/46/48/50
 *   source ceiling = 88, fan-in = Xtensa INT bit 20
 *
 * UART2 = source 3 is confirmed firing on hardware.  Each peripheral's NuttX
 * IRQ number is T113_IRQ_FIRST + R_INTC source, so up_enable_irq() can
 * recover the source as (irq - T113_IRQ_FIRST) and the dispatcher maps
 * PEND bit -> source -> IRQ with the same offset.
 *
 * Only peripherals with a vendor-defined R_INTC source for sun8iw20 DSP are
 * listed.  TWI/SPI/PWM/audio have NO DSP R_INTC source in this silicon's
 * vendor tree and are intentionally absent.  MSGBOX (XEA2 int 3) and DMA
 * (XEA2 int 8) are Xtensa-internal interrupts, not R_INTC sources, and are
 * likewise not listed here.
 */

#define T113_IRQ_FIRST              XTENSA_IRQ_FIRSTPERIPH

#define T113_IRQ_UART0              (T113_IRQ_FIRST +  1)  /* R_INTC src 1 */
#define T113_IRQ_UART1              (T113_IRQ_FIRST +  2)  /* R_INTC src 2 */
#define T113_IRQ_UART2              (T113_IRQ_FIRST +  3)  /* R_INTC src 3 (console) */
#define T113_IRQ_UART3              (T113_IRQ_FIRST +  4)  /* R_INTC src 4 */
#define T113_IRQ_GPIOB              (T113_IRQ_FIRST + 40)  /* R_INTC src 40 */
#define T113_IRQ_GPIOC              (T113_IRQ_FIRST + 42)  /* R_INTC src 42 */
#define T113_IRQ_GPIOD              (T113_IRQ_FIRST + 44)  /* R_INTC src 44 */
#define T113_IRQ_GPIOE              (T113_IRQ_FIRST + 46)  /* R_INTC src 46 */
#define T113_IRQ_GPIOF              (T113_IRQ_FIRST + 48)  /* R_INTC src 48 */
#define T113_IRQ_GPIOG              (T113_IRQ_FIRST + 50)  /* R_INTC src 50 */

/* Source ceiling per intc-sun8iw20.h (SUNXI_RINTC_IRQ_SOURCE_MAX = 88).  The
 * NuttX IRQ range therefore runs T113_IRQ_FIRST .. +87 even though only
 * the sources above carry a named symbol.
 */

#define T113_IRQ_DSP_INTC_LAST      (T113_IRQ_FIRST + 87)

/* Aliases for the CCOMPARE-based Xtensa-internal timer IRQs.  These remain
 * the system-tick driver's IRQs and are NOT to be confused with the SoC
 * peripheral TIMER IP at SOCTIMER0/SOCTIMER1 above.
 */

#define T113_IRQ_TIMER0             XTENSA_IRQ_TIMER0
#define T113_IRQ_TIMER1             XTENSA_IRQ_TIMER1

#define NR_IRQS                          (T113_IRQ_DSP_INTC_LAST + 1)

#endif /* __ARCH_XTENSA_INCLUDE_T113_IRQ_H */
