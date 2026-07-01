/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_rtc.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_RTC_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <time.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_rtc_init(void);

/* Time get/set */

int  rk3576_rtc_get_time(struct tm *tm);
int  rk3576_rtc_set_time(const struct tm *tm);

/* Alarm */

int  rk3576_rtc_set_alarm(const struct tm *tm);
int  rk3576_rtc_cancel_alarm(void);

/* Interrupt */

void rk3576_rtc_enable_irq(int sources);
void rk3576_rtc_disable_irq(int sources);
void rk3576_rtc_clear_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_RTC_H */
