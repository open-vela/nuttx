/****************************************************************************
 * arch/arm/src/t113/hardware/t113_rtc.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_RTC_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_RTC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_RTC_BASE           0x07090000

/* LOSC control */

#define T113_RTC_LOSC_CTRL      (T113_RTC_BASE + 0x0000)
#define T113_RTC_LOSC_MAGIC     0x16aa0000
#define T113_RTC_LOSC_EXT_SRC   (1 << 0)
#define T113_RTC_LOSC_HMS_ACC   (1 << 8)
#define T113_RTC_LOSC_YMD_ACC   (1 << 7)

/* Date/Time registers (YMD = year/month/day, HMS = hour/min/sec) */

#define T113_RTC_YMD            (T113_RTC_BASE + 0x0010)
#define T113_RTC_HMS            (T113_RTC_BASE + 0x0014)

/* YMD bit fields:
 *   [23:16] = year (0-255, offset from YEAR_BASE)
 *   [11:8]  = month (1-12)
 *   [4:0]   = day (1-31)
 *   [15]    = leap year indicator (read-only)
 */

#define T113_RTC_YMD_YEAR_SHIFT 16
#define T113_RTC_YMD_YEAR_MASK  (0xff << 16)
#define T113_RTC_YMD_MON_SHIFT  8
#define T113_RTC_YMD_MON_MASK   (0x0f << 8)
#define T113_RTC_YMD_DAY_SHIFT  0
#define T113_RTC_YMD_DAY_MASK   (0x1f << 0)

/* HMS bit fields:
 *   [20:16] = hour (0-23)
 *   [13:8]  = minute (0-59)
 *   [5:0]   = second (0-59)
 */

#define T113_RTC_HMS_HOUR_SHIFT 16
#define T113_RTC_HMS_HOUR_MASK  (0x1f << 16)
#define T113_RTC_HMS_MIN_SHIFT  8
#define T113_RTC_HMS_MIN_MASK   (0x3f << 8)
#define T113_RTC_HMS_SEC_SHIFT  0
#define T113_RTC_HMS_SEC_MASK   (0x3f << 0)

/* Alarm counter registers (simplified timer mode) */

#define T113_RTC_ALRM_COUNTER   (T113_RTC_BASE + 0x0020)
#define T113_RTC_ALRM_CURRENT   (T113_RTC_BASE + 0x0024)
#define T113_RTC_ALRM_EN        (T113_RTC_BASE + 0x0028)
#define T113_RTC_ALRM_EN_CNT    (1 << 0)
#define T113_RTC_ALRM_IRQ_EN    (T113_RTC_BASE + 0x002c)
#define T113_RTC_ALRM_IRQ_CNT   (1 << 0)
#define T113_RTC_ALRM_IRQ_STA   (T113_RTC_BASE + 0x0030)
#define T113_RTC_ALRM_IRQ_PEND  (1 << 0)

/* Alarm wakeup output */

#define T113_RTC_ALARM_CONFIG   (T113_RTC_BASE + 0x0050)
#define T113_RTC_ALRM_WAKEUP_EN (1 << 0)

/* General purpose data registers (used for reset cause, etc.) */

#define T113_RTC_GP_DATA(n)     (T113_RTC_BASE + 0x100 + (n) * 4)

/* Year base for T113 RTC (hardware counts 0-63 offset from this) */

#define T113_RTC_YEAR_BASE      1970

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_RTC_H */
