/****************************************************************************
 * arch/loongarch/include/ls2k0300/irq.h
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

#ifndef __ARCH_LOONGARCH_INCLUDE_LS2K0300_IRQ_H
#define __ARCH_LOONGARCH_INCLUDE_LS2K0300_IRQ_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LS2K0300_IRQ_PERI_START    (LOONGARCH_IRQ_ASYNC + 1)

#define LS2K0300_IRQ_ICU_START     LS2K0300_IRQ_PERI_START
#define LS2K0300_IRQ_ICU_COUNT     64

#define LS2K0300_IRQ_EIOINTC_START (LS2K0300_IRQ_ICU_START + LS2K0300_IRQ_ICU_COUNT)
#define LS2K0300_IRQ_EIOINTC_COUNT 128

#define LS2K0300_IRQ_ICU(n)        (LS2K0300_IRQ_ICU_START + (n))
#define LS2K0300_IRQ_EIOINTC(n)    (LS2K0300_IRQ_EIOINTC_START + (n))

#define LS2K0300_IRQ_UART0         LS2K0300_IRQ_ICU(0)
#define LS2K0300_IRQ_UART1         LS2K0300_IRQ_ICU(1)
#define LS2K0300_IRQ_UART2         LS2K0300_IRQ_ICU(2)
#define LS2K0300_IRQ_UART3         LS2K0300_IRQ_ICU(3)

#define LS2K0300_IRQ_I2C0          LS2K0300_IRQ_ICU(4)
#define LS2K0300_IRQ_I2C1          LS2K0300_IRQ_ICU(4)
#define LS2K0300_IRQ_I2C2          LS2K0300_IRQ_ICU(5)
#define LS2K0300_IRQ_I2C3          LS2K0300_IRQ_ICU(5)

#define LS2K0300_IRQ_CAN0          LS2K0300_IRQ_ICU(8)
#define LS2K0300_IRQ_CAN1          LS2K0300_IRQ_ICU(9)
#define LS2K0300_IRQ_CAN2          LS2K0300_IRQ_ICU(10)
#define LS2K0300_IRQ_CAN3          LS2K0300_IRQ_ICU(11)

#define LS2K0300_IRQ_I2S           LS2K0300_IRQ_ICU(12)

#define LS2K0300_IRQ_PWM0          LS2K0300_IRQ_ICU(16)
#define LS2K0300_IRQ_PWM1          LS2K0300_IRQ_ICU(16)
#define LS2K0300_IRQ_PWM2          LS2K0300_IRQ_ICU(17)
#define LS2K0300_IRQ_PWM3          LS2K0300_IRQ_ICU(17)

#define LS2K0300_IRQ_IODMA0        LS2K0300_IRQ_ICU(23)

#define LS2K0300_IRQ_RTC           LS2K0300_IRQ_ICU(41)

#define LS2K0300_IRQ_SDIO0         LS2K0300_IRQ_ICU(31)
#define LS2K0300_IRQ_SDIO1         LS2K0300_IRQ_ICU(32)

#define LS2K0300_IRQ_SPI0          LS2K0300_IRQ_ICU(44)
#define LS2K0300_IRQ_SPI1          LS2K0300_IRQ_ICU(45)

#define LS2K0300_IRQ_EHCI          LS2K0300_IRQ_ICU(46)
#define LS2K0300_IRQ_OHCI          LS2K0300_IRQ_ICU(47)
#define LS2K0300_IRQ_OTG           LS2K0300_IRQ_ICU(48)

#define LS2K0300_IRQ_GMAC0         LS2K0300_IRQ_ICU(49)
#define LS2K0300_IRQ_GMAC1         LS2K0300_IRQ_ICU(50)

#define LS2K0300_IRQ_DC            LS2K0300_IRQ_ICU(51)
#define LS2K0300_IRQ_TSENSOR       LS2K0300_IRQ_ICU(52)

#define LS2K0300_IRQ_GPIO          LS2K0300_IRQ_ICU(53)
#define LS2K0300_IRQ_GPIO1         LS2K0300_IRQ_ICU(54)
#define LS2K0300_IRQ_GPIO2         LS2K0300_IRQ_ICU(55)
#define LS2K0300_IRQ_GPIO3         LS2K0300_IRQ_ICU(56)
#define LS2K0300_IRQ_GPIO4         LS2K0300_IRQ_ICU(57)
#define LS2K0300_IRQ_GPIO5         LS2K0300_IRQ_ICU(58)
#define LS2K0300_IRQ_GPIO6         LS2K0300_IRQ_ICU(59)

#define NR_IRQS                   (LS2K0300_IRQ_EIOINTC_START + LS2K0300_IRQ_EIOINTC_COUNT)

#endif /* __ARCH_LOONGARCH_INCLUDE_LS2K0300_IRQ_H */
