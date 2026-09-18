/****************************************************************************
 * arch/arm/include/bk7258/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_ARM_INCLUDE_BK7258_IRQ_H
#define __ARCH_ARM_INCLUDE_BK7258_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Processor Exceptions (vectors 0-15) */

#define BK7258_IRQ_RESERVED    (0)  /* Reserved (CONFIG_DEBUG_FEATURES) */
                                    /* Vector 0: Reset stack pointer value */
                                    /* Vector 1: Reset (not handled as IRQ) */
#define BK7258_IRQ_NMI         (2)  /* Vector 2:  NMI */
#define BK7258_IRQ_HARDFAULT   (3)  /* Vector 3:  Hard fault */
#define BK7258_IRQ_MEMFAULT    (4)  /* Vector 4:  MPU fault */
#define BK7258_IRQ_BUSFAULT    (5)  /* Vector 5:  Bus fault */
#define BK7258_IRQ_USAGEFAULT  (6)  /* Vector 6:  Usage fault */
#define BK7258_IRQ_SECUREFAULT (7)  /* Vector 7:  Secure fault (Armv8-M TZ) */
                                    /* Vectors 8-10: Reserved */
#define BK7258_IRQ_SVCALL      (11) /* Vector 11: SVC call */
#define BK7258_IRQ_DBGMONITOR  (12) /* Vector 12: Debug Monitor */
                                    /* Vector 13: Reserved */
#define BK7258_IRQ_PENDSV      (14) /* Vector 14: PendSV */
#define BK7258_IRQ_SYSTICK     (15) /* Vector 15: SysTick */
#define BK7258_IRQ_EXTINT      (16) /* Vector 16: First external interrupt */

/* External interrupts (NVIC), from ARMINO icu_map.h (64 sources, GROUP0/1).
 * NuttX IRQ number = BK7258_IRQ_EXTINT + INT_SRC index.
 */

#define BK7258_IRQ_DMA0        (BK7258_IRQ_EXTINT + 0)
#define BK7258_IRQ_ENCP        (BK7258_IRQ_EXTINT + 1)
#define BK7258_IRQ_ENCS        (BK7258_IRQ_EXTINT + 2)
#define BK7258_IRQ_TIMER0      (BK7258_IRQ_EXTINT + 3)
#define BK7258_IRQ_UART0       (BK7258_IRQ_EXTINT + 4)
#define BK7258_IRQ_PWM0        (BK7258_IRQ_EXTINT + 5)
#define BK7258_IRQ_I2C0        (BK7258_IRQ_EXTINT + 6)
#define BK7258_IRQ_SPI0        (BK7258_IRQ_EXTINT + 7)
#define BK7258_IRQ_SARADC      (BK7258_IRQ_EXTINT + 8)
#define BK7258_IRQ_IRDA        (BK7258_IRQ_EXTINT + 9)
#define BK7258_IRQ_SDIO        (BK7258_IRQ_EXTINT + 10)
#define BK7258_IRQ_GDMA        (BK7258_IRQ_EXTINT + 11)
#define BK7258_IRQ_LA          (BK7258_IRQ_EXTINT + 12)
#define BK7258_IRQ_TIMER1      (BK7258_IRQ_EXTINT + 13)
#define BK7258_IRQ_I2C1        (BK7258_IRQ_EXTINT + 14)
#define BK7258_IRQ_UART1       (BK7258_IRQ_EXTINT + 15)
#define BK7258_IRQ_UART2       (BK7258_IRQ_EXTINT + 16)
#define BK7258_IRQ_SPI1        (BK7258_IRQ_EXTINT + 17)
#define BK7258_IRQ_CAN         (BK7258_IRQ_EXTINT + 18)
#define BK7258_IRQ_USB         (BK7258_IRQ_EXTINT + 19)
#define BK7258_IRQ_QSPI0       (BK7258_IRQ_EXTINT + 20)
#define BK7258_IRQ_CKMN        (BK7258_IRQ_EXTINT + 21)
#define BK7258_IRQ_SBC         (BK7258_IRQ_EXTINT + 22)
#define BK7258_IRQ_AUDIO       (BK7258_IRQ_EXTINT + 23)
#define BK7258_IRQ_I2S0        (BK7258_IRQ_EXTINT + 24)
#define BK7258_IRQ_JPEG_ENC    (BK7258_IRQ_EXTINT + 25)
#define BK7258_IRQ_JPEG_DEC    (BK7258_IRQ_EXTINT + 26)
#define BK7258_IRQ_LCD         (BK7258_IRQ_EXTINT + 27)
#define BK7258_IRQ_DMA2D       (BK7258_IRQ_EXTINT + 28)
#define BK7258_IRQ_GPIO        (BK7258_IRQ_EXTINT + 55)
#define BK7258_IRQ_RTC         (BK7258_IRQ_EXTINT + 54)
#define BK7258_IRQ_MAILBOX     (BK7258_IRQ_EXTINT + 63)

/* Total number of external interrupts (INT_SRC 0..63) and total IRQs */

#define BK7258_IRQ_NEXTINT     (64)
#define NR_IRQS                (BK7258_IRQ_EXTINT + BK7258_IRQ_NEXTINT)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
typedef void (*vic_vector_t)(uint32_t *regs);
#endif

#endif /* __ARCH_ARM_INCLUDE_BK7258_IRQ_H */
