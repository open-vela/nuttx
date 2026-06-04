/****************************************************************************
 * arch/arm/src/t113/t113_fel.c
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

/****************************************************************************
 * FEL rescue
 *
 * Detects abnormal-reboot storms early in arm_boot() and forces the
 * device into BROM FEL mode after CONFIG_T113_FEL_RESCUE_THRESHOLD
 * consecutive abnormal resets, so a wedged device can always be
 * recovered with `xfel write` instead of needing a JLink.
 *
 * Storage (RTC VDD_RTC domain, survives WDT, cleared by POR):
 *   GP_DATA[0]  boot marker (USER/FEL/abnormal) - owned by
 *               t113_systemreset.c
 *   GP_DATA[1]  counter:
 *                 [31:16] = 0xC0DE (initialised magic)
 *                 [15: 0] = abnormal-reboot count
 *   GP_DATA[2]  last boot RTC timestamp - owned by RTC helpers
 *               (t113_rtc_get/set_boottime)
 *
 * This file holds the *policy* only.  All RTC register access lives in
 * t113_rtc.c and is reached through the chip-level helpers declared in
 * t113_rtc.h.
 *
 * Counting rule:
 *   - cold POR / VDD_RTC lost (magic word missing in GP_DATA[1])
 *     => NOT counted; this is the very first boot, not a crash event.
 *     Just seed the magic + boottime and return.
 *   - "abnormal" = magic present AND boot marker is NOT the USER
 *     magic (silent WDT, ASSERT, PANIC, xfel reset without USER
 *     marker, ...).  Implies the system booted at least once before.
 *   - delta(now, last_boottime) > HEALTHY_WINDOW_SEC => system stayed
 *     up long enough to be considered cooled-down, counter resets to 1
 *   - delta <= HEALTHY_WINDOW_SEC => short-cycle reboot, counter
 *     increments
 *   - count >= THRESHOLD => enter FEL via the same path as
 *     board_reset(99)
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/board.h>

#include "arm_internal.h"
#include "hardware/t113_rtc.h"
#include "t113_rtc.h"
#include "t113_fel.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FEL_RESCUE_MAGIC        0xC0DE0000u
#define FEL_RESCUE_MAGIC_MASK   0xFFFF0000u
#define FEL_RESCUE_COUNT_MASK   0x0000FFFFu

/* Marker values written by board_reset(); must mirror the encoding in
 * t113_systemreset.c.  Kept private here to keep this file independent.
 */

#define BOOT_MARKER_USER        0x5AA50000u
#define BOOT_MARKER_FEL         0x5AA50001u

/* Window within which two boots are still considered "the same crash
 * loop".  If the system stayed up longer than this between resets we
 * treat it as healthy and restart the count.
 */

#define HEALTHY_WINDOW_SEC      30u

#if !defined(CONFIG_T113_FEL_RESCUE_THRESHOLD)
#  error "t113_fel_rescue.c built without CONFIG_T113_FEL_RESCUE_THRESHOLD"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_fel_rescue
 ****************************************************************************/

void t113_fel_rescue(void)
{
  uint32_t marker;
  uint32_t reg1;
  uint32_t now;
  uint32_t last;
  uint32_t count;

  marker = getreg32(T113_RTC_GP_DATA(0));
  reg1   = getreg32(T113_RTC_GP_DATA(1));

  /* The marker is consumed every boot - clear it now so a cold POR on
   * the next reset does not see a stale value.
   */

  putreg32(0, T113_RTC_GP_DATA(0));

  if ((reg1 & FEL_RESCUE_MAGIC_MASK) != FEL_RESCUE_MAGIC)
    {
      /* Cold POR / VDD_RTC was lost: this is the first ever boot of
       * this RTC domain, NOT a crash event.  Seed the wall clock,
       * prime the magic word, stamp GP_DATA[2], and return without
       * entering the count path.  The next non-USER reboot - which
       * by definition cannot happen until the system has booted at
       * least once - will be the first one that counts.
       */

      t113_rtc_early_init();
      now = t113_rtc_get_seconds();
      t113_rtc_set_boottime(now);
      putreg32(FEL_RESCUE_MAGIC, T113_RTC_GP_DATA(1));
      return;
    }

  last  = t113_rtc_get_boottime();
  count = reg1 & FEL_RESCUE_COUNT_MASK;

  /* Stamp the new boot reference so the next boot's delta is measured
   * from this moment.  Done before the USER short-circuit so that even
   * a chain of clean reboots keeps GP_DATA[2] fresh.  Single read of
   * the RTC, single write of GP_DATA[2].
   */

  now = t113_rtc_get_seconds();
  t113_rtc_set_boottime(now);

  /* User-initiated reboots are not crash events: counter unchanged. */

  if (marker == BOOT_MARKER_USER)
    {
      return;
    }

  /* Abnormal path: ASSERT, PANIC, silent WDT, xfel reset without USER
   * marker, ... - initialised is true here, so the system has booted
   * at least once before.
   */

  if ((now - last) > HEALTHY_WINDOW_SEC)
    {
      /* System stayed up long enough to count as recovered. */

      count = 1;
    }
  else
    {
      /* Short-cycle reboot - same crash loop. */

      count = count + 1;
    }

  if (CONFIG_T113_FEL_RESCUE_THRESHOLD > 0 &&
      count >= (uint32_t)CONFIG_T113_FEL_RESCUE_THRESHOLD)
    {
      /* Reset counter to a known state before bouncing into FEL so the
       * first boot after `xfel write` does not immediately re-trip.
       */

      putreg32(FEL_RESCUE_MAGIC, T113_RTC_GP_DATA(1));

      /* Re-enter via board_reset(99) - same WDT path used elsewhere
       * for the FEL transition; does not return.
       */

      board_reset(99);

      /* Unreachable */

      for (; ; );
    }

  putreg32(FEL_RESCUE_MAGIC | (count & FEL_RESCUE_COUNT_MASK),
           T113_RTC_GP_DATA(1));
}
