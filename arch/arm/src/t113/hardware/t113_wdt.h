/****************************************************************************
 * arch/arm/src/t113/hardware/t113_wdt.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_WDT_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_WDT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer/Watchdog base address (TIMER module contains WDT sub-block) */

#define T113_TIMER_BASE         0x02050000

/* Watchdog register offsets from T113_TIMER_BASE */

#define T113_WDOG_IRQ_EN_REG    (T113_TIMER_BASE + 0x00a0)
#define T113_WDOG_IRQ_STA_REG   (T113_TIMER_BASE + 0x00a4)
#define T113_WDOG_SOFT_RST_REG  (T113_TIMER_BASE + 0x00a8)
#define T113_WDOG_CTRL_REG      (T113_TIMER_BASE + 0x00b0)
#define T113_WDOG_CFG_REG       (T113_TIMER_BASE + 0x00b4)
#define T113_WDOG_MODE_REG      (T113_TIMER_BASE + 0x00b8)

/* WDOG_SOFT_RST_REG key (must be written with upper 16 bits = 0x16aa) */

#define T113_WDOG_SOFT_RST_KEY  (0x16aa << 16)

/* WDOG_CTRL_REG bits */

#define T113_WDOG_CTRL_KEY      (0xa57 << 1)
#define T113_WDOG_CTRL_RESTART  (T113_WDOG_CTRL_KEY | 1)

/* WDOG_CFG_REG bits */

#define T113_WDOG_CFG_RESET     0x01
#define T113_WDOG_CFG_IRQ       0x02

/* WDOG_MODE_REG bits - key 0x16AA in [31:16] required for writes */

#define T113_WDOG_MODE_KEY      (0x16aa << 16)
#define T113_WDOG_MODE_EN       (1 << 0)

/* WDOG_MODE_REG bit[7:4] = interval select:
 *   0=0.5s 1=1s 2=2s 3=3s 4=4s 5=5s 6=6s
 *   7=8s 8=10s 9=12s 10=14s 11=16s
 */

#define T113_WDOG_MODE_INTV_SHIFT  4
#define T113_WDOG_MODE_INTV_MASK   (0xf << T113_WDOG_MODE_INTV_SHIFT)

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_WDT_H */
