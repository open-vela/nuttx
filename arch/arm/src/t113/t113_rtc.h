/****************************************************************************
 * arch/arm/src/t113/t113_rtc.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_RTC_H
#define __ARCH_ARM_SRC_T113_T113_RTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: t113_rtc_initialize
 *
 * Description:
 *   Register the RTC lower-half driver under /dev/rtc0.  Called from
 *   board bring-up after CONFIG_T113_RTC has been validated.
 *
 ****************************************************************************/

#ifdef CONFIG_T113_RTC
void t113_rtc_initialize(void);
#endif

/****************************************************************************
 * Chip-level RTC helpers (no driver framework dependency).
 *
 * Available whenever CONFIG_T113_RTC or CONFIG_T113_FEL_RESCUE_THRESHOLD>0
 * pulls t113_rtc.c into the build.  Implemented with raw GP_DATA / YMD /
 * HMS / LOSC_CTRL register access via the LOSC ACC handshake - usable
 * from t113_boardinitialize before scheduler/IRQ are up.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: t113_rtc_early_init
 *
 * Description:
 *   Seed the RTC with a known-good initial date/time on first power-on.
 *   The RTC YMD/HMS registers are UDF after POR; this routine writes
 *   1970-01-01 00:00:00 via the LOSC ACC handshake so that subsequent
 *   delta calculations are well defined.  Idempotent: callers should
 *   only invoke when GP_DATA[1] indicates the counter has not yet been
 *   initialised by t113_fel_rescue.
 *
 *   Also stamps GP_DATA[2] with the seeded time (= 0 in coarse-seconds
 *   space, i.e. 1970-01-01 00:00:00) as the initial boot timestamp.
 *
 *   Safe to call from t113_boardinitialize: uses busy-wait spin (no
 *   up_udelay) and touches only RTC MMIO.  If the LOSC ACC handshake
 *   times out the helper returns silently leaving GP_DATA[2]=0 so the
 *   next boot retries the seed cleanly.
 *
 ****************************************************************************/

void t113_rtc_early_init(void);

/****************************************************************************
 * Name: t113_rtc_get_seconds
 *
 * Description:
 *   Read the current RTC YMD/HMS and fold them into a coarse second
 *   count.  The conversion uses a fixed 31-day month / 365-day year
 *   approximation: only delta comparisons within a small window are
 *   correct.  Do not use as an absolute wall-clock value.
 *
 ****************************************************************************/

uint32_t t113_rtc_get_seconds(void);

/****************************************************************************
 * Name: t113_rtc_get_boottime
 *
 * Description:
 *   Read RTC GP_DATA[2] which holds the coarse seconds value at the
 *   start of the previous boot (or last settime).  Returns 0 if no
 *   stamp has been recorded.
 *
 ****************************************************************************/

uint32_t t113_rtc_get_boottime(void);

/****************************************************************************
 * Name: t113_rtc_set_boottime
 *
 * Description:
 *   Stamp RTC GP_DATA[2] with the supplied coarse-seconds value,
 *   marking "now" as the new boot/sync reference point.
 *
 *   Caller supplies the seconds (computed from RTC registers, or from
 *   a target struct rtc_time, or as a literal seed value) so this
 *   helper can be used both right after writing the RTC (where a
 *   read-back may still see the stale, not-yet-latched value from the
 *   LOSC clock domain) and from a normal boot path.
 *
 *   Called by:
 *     - t113_fel_rescue with the result of t113_rtc_get_seconds();
 *     - t113_rtc_settime (internally) with the seconds value derived
 *       from the target rtc_time so the FEL-rescue delta does not
 *       jump epochs across a user date -s;
 *     - t113_rtc_early_init with the literal seed (0 = 1970-01-01).
 *
 ****************************************************************************/

void t113_rtc_set_boottime(uint32_t seconds);

#endif /* __ARCH_ARM_SRC_T113_T113_RTC_H */
