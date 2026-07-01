/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_rtc.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_RTC_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RTC register offsets */

#define RTC_SECONDS                0x0000   /* Seconds (0-59) */
#define RTC_MINUTES                0x0004   /* Minutes (0-59) */
#define RTC_HOURS                  0x0008   /* Hours (0-23) */
#define RTC_DAYS                   0x000c   /* Day of month (1-31) */
#define RTC_MONTHS                 0x0010   /* Month (1-12) */
#define RTC_YEARS                  0x0014   /* Years (0-99) */
#define RTC_WEEKDAY                0x0018   /* Day of week (0-6) */
#define RTC_CONTROL                0x001c   /* Control register */
#define RTC_STATUS                 0x0020   /* Status register */
#define RTC_ALARM_SEC              0x0024   /* Alarm seconds */
#define RTC_ALARM_MIN              0x0028   /* Alarm minutes */
#define RTC_ALARM_HOUR             0x002c   /* Alarm hours */
#define RTC_ALARM_DAY              0x0030   /* Alarm day */
#define RTC_ALARM_MON              0x0034   /* Alarm month */
#define RTC_ALARM_YEAR             0x0038   /* Alarm year */
#define RTC_INTEN                  0x003c   /* Interrupt enable */
#define RTC_INTSTS                 0x0040   /* Interrupt status */
#define RTC_COMP                   0x0044   /* Compensation register */

/****************************************************************************
 * RTC_CONTROL bit definitions
 ****************************************************************************/

#define RTC_CONTROL_EN             (1 << 0)   /* RTC enable */
#define RTC_CONTROL_HOUR24         (1 << 1)   /* 24-hour mode */
#define RTC_CONTROL_INT_EN         (1 << 2)   /* Interrupt enable */

/****************************************************************************
 * RTC_STATUS bit definitions
 ****************************************************************************/

#define RTC_STATUS_BUSY            (1 << 0)   /* RTC busy */
#define RTC_STATUS_ALARMPENDING    (1 << 1)   /* Alarm pending */

/****************************************************************************
 * RTC_INTEN bit definitions
 ****************************************************************************/

#define RTC_INTEN_SEC_EN           (1 << 0)   /* Seconds interrupt */
#define RTC_INTEN_MIN_EN           (1 << 1)   /* Minutes interrupt */
#define RTC_INTEN_HOUR_EN          (1 << 2)   /* Hours interrupt */
#define RTC_INTEN_DAY_EN           (1 << 3)   /* Day interrupt */
#define RTC_INTEN_ALARM_EN         (1 << 4)   /* Alarm interrupt */

/****************************************************************************
 * RTC_INTSTS bit definitions
 ****************************************************************************/

#define RTC_INTSTS_SEC             (1 << 0)   /* Seconds event */
#define RTC_INTSTS_MIN             (1 << 1)   /* Minutes event */
#define RTC_INTSTS_HOUR            (1 << 2)   /* Hours event */
#define RTC_INTSTS_DAY             (1 << 3)   /* Day event */
#define RTC_INTSTS_ALARM           (1 << 4)   /* Alarm event */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_rtc_base;

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_RTC_H */
