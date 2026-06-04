/****************************************************************************
 * arch/arm/src/t113/t113_rtc.c
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
 * The file is split in two blocks selected by independent Kconfig
 * options.  At least one must be enabled for this file to be linked
 * (the build system gates compilation accordingly):
 *
 *   - Chip-level RTC helpers (raw GP_DATA / YMD / HMS / LOSC_CTRL
 *     access via the LOSC ACC handshake).  Compiled when either
 *     CONFIG_T113_RTC or CONFIG_T113_FEL_RESCUE_THRESHOLD>0 is set.
 *     Usable from t113_boardinitialize before scheduler/IRQ are up.
 *
 *   - rtc_lowerhalf driver (NuttX RTC framework).  Compiled when
 *     CONFIG_T113_RTC is set.  Hooks into the helpers for the
 *     settime -> GP_DATA[2] sync.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#ifdef CONFIG_T113_RTC
#  include <time.h>
#  include <debug.h>
#  include <nuttx/arch.h>
#  include <nuttx/spinlock.h>
#  include <nuttx/timers/rtc.h>
#endif

#include "arm_internal.h"
#include "hardware/t113_rtc.h"
#include "hardware/t113_ccu.h"
#include "t113_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_RTC_HELPERS_ENABLED \
  (defined(CONFIG_T113_RTC) || \
   (defined(CONFIG_T113_FEL_RESCUE_THRESHOLD) && \
    CONFIG_T113_FEL_RESCUE_THRESHOLD > 0))

/* ===========================================================
 * Block A: Chip-level RTC helpers
 *
 *   Compiled whenever the file is built.  Pure register access,
 *   no dependency on the rtc_lowerhalf framework.  Safe to call
 *   from t113_boardinitialize (before nx_start).
 * ===========================================================
 */

#if T113_RTC_HELPERS_ENABLED

/****************************************************************************
 * Name: rtc_acc_write_spin
 *
 * Description:
 *   LOSC ACC handshake variant usable from t113_boardinitialize, before
 *   the system tick is running and up_udelay() can be called.  Spins
 *   on the ACC bit instead of timed wait; the LOSC domain latches
 *   within a few 32 kHz cycles (~ tens of microseconds), so a coarse
 *   spin loop is sufficient.
 *
 *   Returns 0 on success, -ETIMEDOUT if the LOSC domain never
 *   acknowledged the write request.  On timeout the data register is
 *   NOT written, so a stalled LOSC produces a clean failure rather
 *   than a silently dropped write that would leave RTC YMD/HMS in
 *   undefined state.
 *
 ****************************************************************************/

static int rtc_acc_write_spin(uint32_t acc_bit, uint32_t reg, uint32_t val)
{
  uint32_t ctrl = getreg32(T113_RTC_LOSC_CTRL) &
                  ~(T113_RTC_LOSC_YMD_ACC | T113_RTC_LOSC_HMS_ACC);
  int i;

  putreg32(T113_RTC_LOSC_MAGIC | ctrl | acc_bit, T113_RTC_LOSC_CTRL);

  /* CPU-cycle bound, NOT a wall-clock bound.  The body's getreg32 hits
   * volatile MMIO and cannot be elided, so the loop count is the only
   * upper limit.  A single LOSC tick is ~31 us; legitimate latches
   * complete in tens of microseconds.  1e6 iterations gives well over
   * an order of magnitude of margin even when this runs in boot0 SPL
   * before t113_clk_init() has ramped the CPU to 1.2 GHz.
   */

  for (i = 0; i < 1000000; i++)
    {
      if ((getreg32(T113_RTC_LOSC_CTRL) & acc_bit) == 0)
        {
          putreg32(val, reg);
          return 0;
        }
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: t113_rtc_get_seconds
 ****************************************************************************/

uint32_t t113_rtc_get_seconds(void)
{
  uint32_t ymd1;
  uint32_t ymd2;
  uint32_t hms;
  uint32_t year;
  uint32_t mon;
  uint32_t day;
  uint32_t hour;
  uint32_t min;
  uint32_t sec;

  /* Double-read YMD around a single HMS read to defend against the
   * day-boundary rollover (23:59:59 -> 00:00:00, YMD advances).
   */

  do
    {
      ymd1 = getreg32(T113_RTC_YMD);
      hms  = getreg32(T113_RTC_HMS);
      ymd2 = getreg32(T113_RTC_YMD);
    }
  while (ymd1 != ymd2);

  year = (ymd1 & T113_RTC_YMD_YEAR_MASK) >> T113_RTC_YMD_YEAR_SHIFT;
  mon  = (ymd1 & T113_RTC_YMD_MON_MASK)  >> T113_RTC_YMD_MON_SHIFT;
  day  =  ymd1 & T113_RTC_YMD_DAY_MASK;
  hour = (hms  & T113_RTC_HMS_HOUR_MASK) >> T113_RTC_HMS_HOUR_SHIFT;
  min  = (hms  & T113_RTC_HMS_MIN_MASK)  >> T113_RTC_HMS_MIN_SHIFT;
  sec  =  hms  & T113_RTC_HMS_SEC_MASK;

  /* Coarse 31-day-month / 365-day-year fold.  The result is NOT a
   * true Unix epoch - it only needs to be monotonic enough for short
   * delta comparisons across a single boot cycle.  Wraps at year 137
   * (uint32).  A reading that straddles the wrap makes (now - last)
   * resolve to a huge positive value (>> HEALTHY_WINDOW_SEC), which
   * the rescue path treats as "cooled down" and resets the counter
   * to 1 - the safe direction (false negative on counting, never a
   * false positive on FEL trigger).
   */

  return ((((year * 365 + mon * 31 + day) * 24 + hour) * 60 + min) * 60
          + sec);
}

/****************************************************************************
 * Name: t113_rtc_get_boottime
 ****************************************************************************/

uint32_t t113_rtc_get_boottime(void)
{
  return getreg32(T113_RTC_GP_DATA(2));
}

/****************************************************************************
 * Name: t113_rtc_set_boottime
 ****************************************************************************/

void t113_rtc_set_boottime(uint32_t seconds)
{
  putreg32(seconds, T113_RTC_GP_DATA(2));
}

/****************************************************************************
 * Name: t113_rtc_early_init
 ****************************************************************************/

void t113_rtc_early_init(void)
{
  /* Seed YMD = 1970-01-01 (year offset 0, month 1, day 1).  MON=1
   * makes havesettime() report "set" so NuttX's RTC framework can
   * sync system time off the seeded value rather than failing.  The
   * user is free to override later via date -s.
   *
   * If the LOSC ACC handshake times out (clock stalled), bail out:
   * leaving GP_DATA[2] = 0 is the documented "no prior boot
   * timestamp" value for t113_fel_rescue, so the next boot retries
   * the seed cleanly.  Better than stamping a bogus timestamp from
   * undefined RTC state.
   */

  if (rtc_acc_write_spin(T113_RTC_LOSC_YMD_ACC, T113_RTC_YMD,
                         (0u << T113_RTC_YMD_YEAR_SHIFT) |
                         (1u << T113_RTC_YMD_MON_SHIFT)  |
                         (1u << T113_RTC_YMD_DAY_SHIFT)) < 0)
    {
      return;
    }

  /* Seed HMS = 00:00:00 */

  if (rtc_acc_write_spin(T113_RTC_LOSC_HMS_ACC, T113_RTC_HMS, 0) < 0)
    {
      return;
    }

  /* Stamp GP_DATA[2] with the seeded "now" so the very first delta
   * compare in t113_fel_rescue has a sane reference point.  Use the
   * literal seed value (mirroring t113_rtc_get_seconds() formula
   * applied to year=0, mon=1, day=1, 00:00:00) instead of reading
   * back YMD/HMS, which the LOSC domain may not have latched yet.
   */

  t113_rtc_set_boottime(((1u * 31u + 1u) * 24u * 60u * 60u));
}

#endif /* T113_RTC_HELPERS_ENABLED */

/* ===========================================================
 * Block B: rtc_lowerhalf driver (NuttX RTC framework)
 * ===========================================================
 */

#ifdef CONFIG_T113_RTC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum microseconds to wait for a LOSC_CTRL ACC bit to clear after
 * requesting a YMD/HMS write.  One LOSC tick is ~31 us; 5 ms gives
 * the 32 kHz domain ample time while still bounding settime() latency.
 */

#define ACC_TIMEOUT_US 5000

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int t113_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                           FAR struct rtc_time *rtctime);
static int t113_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                            FAR const struct rtc_time *rtctime);
static bool t113_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = t113_rtc_rdtime,
  .settime     = t113_rtc_settime,
  .havesettime = t113_rtc_havesettime,
};

static struct rtc_lowerhalf_s g_rtc_lowerhalf =
{
  .ops = &g_rtc_ops,
};

/* Serializes the multi-step LOSC ACC handshake in t113_rtc_settime so two
 * concurrent settime() callers cannot interleave YMD/HMS write requests
 * (which would corrupt LOSC_CTRL or drop a write).
 */

static spinlock_t g_rtc_settime_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int t113_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                           FAR struct rtc_time *rtctime)
{
  uint32_t ymd;
  uint32_t ymd1;
  uint32_t ymd2;
  uint32_t hms;

  /* Read date and time registers.
   *
   * The dangerous tearing is the day-boundary rollover
   * (23:59:59 -> 00:00:00) where HMS wraps and YMD advances.  Guard
   * by double-reading YMD around a single HMS read; if YMD changed,
   * the HMS read straddled the rollover and must be retried.
   */

  do
    {
      ymd1 = getreg32(T113_RTC_YMD);
      hms  = getreg32(T113_RTC_HMS);
      ymd2 = getreg32(T113_RTC_YMD);
    }
  while (ymd1 != ymd2);

  ymd = ymd1;

  rtctime->tm_sec  = (hms & T113_RTC_HMS_SEC_MASK) >>
                     T113_RTC_HMS_SEC_SHIFT;
  rtctime->tm_min  = (hms & T113_RTC_HMS_MIN_MASK) >>
                     T113_RTC_HMS_MIN_SHIFT;
  rtctime->tm_hour = (hms & T113_RTC_HMS_HOUR_MASK) >>
                     T113_RTC_HMS_HOUR_SHIFT;
  rtctime->tm_mday = (ymd & T113_RTC_YMD_DAY_MASK) >>
                     T113_RTC_YMD_DAY_SHIFT;
  rtctime->tm_mon  = ((ymd & T113_RTC_YMD_MON_MASK) >>
                     T113_RTC_YMD_MON_SHIFT) - 1;
  rtctime->tm_year = ((ymd & T113_RTC_YMD_YEAR_MASK) >>
                     T113_RTC_YMD_YEAR_SHIFT) +
                     T113_RTC_YEAR_BASE - 1900;

  return OK;
}

static int t113_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                            FAR const struct rtc_time *rtctime)
{
  irqstate_t flags;
  uint32_t ymd;
  uint32_t hms;
  uint32_t ctrl;
  int year;
  int i;
  int ret = OK;

  /* Validate ranges before composing register values. */

  if (rtctime->tm_mon  < 0 || rtctime->tm_mon  > 11 ||
      rtctime->tm_mday < 1 || rtctime->tm_mday > 31 ||
      rtctime->tm_hour < 0 || rtctime->tm_hour > 23 ||
      rtctime->tm_min  < 0 || rtctime->tm_min  > 59 ||
      rtctime->tm_sec  < 0 || rtctime->tm_sec  > 60)
    {
      return -EINVAL;
    }

  year = (rtctime->tm_year + 1900) - T113_RTC_YEAR_BASE;
  if (year < 0 || year > 255)
    {
      return -EINVAL;
    }

  ymd = ((uint32_t)year << T113_RTC_YMD_YEAR_SHIFT) |
        ((uint32_t)(rtctime->tm_mon + 1) << T113_RTC_YMD_MON_SHIFT) |
        ((uint32_t)rtctime->tm_mday << T113_RTC_YMD_DAY_SHIFT);

  hms = ((uint32_t)rtctime->tm_hour << T113_RTC_HMS_HOUR_SHIFT) |
        ((uint32_t)rtctime->tm_min << T113_RTC_HMS_MIN_SHIFT) |
        ((uint32_t)rtctime->tm_sec << T113_RTC_HMS_SEC_SHIFT);

  /* The YMD/HMS counters live in the 32 kHz LOSC clock domain.  Each
   * AHB write must be acknowledged by the LOSC domain before the next
   * can be issued; the ACC bit in LOSC_CTRL is set by software to
   * request the write and cleared by hardware once it is latched.
   *
   * Sequence (per datasheet): set ACC with MAGIC -> poll ACC clear ->
   * write data register.  Back-to-back writes without the poll are
   * silently dropped.
   *
   * Two callers must not interleave: a second YMD/HMS_ACC request
   * issued while the first is still pending will overwrite LOSC_CTRL
   * and drop one of the writes.  Serialize the whole sequence.
   */

  flags = spin_lock_irqsave(&g_rtc_settime_lock);

  ctrl = getreg32(T113_RTC_LOSC_CTRL) & ~(T113_RTC_LOSC_YMD_ACC |
                                          T113_RTC_LOSC_HMS_ACC);

  /* Request YMD write */

  putreg32(T113_RTC_LOSC_MAGIC | ctrl | T113_RTC_LOSC_YMD_ACC,
           T113_RTC_LOSC_CTRL);
  for (i = 0; i < ACC_TIMEOUT_US; i++)
    {
      if ((getreg32(T113_RTC_LOSC_CTRL) & T113_RTC_LOSC_YMD_ACC) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (i >= ACC_TIMEOUT_US)
    {
      ret = -ETIMEDOUT;
      goto unlock_out;
    }

  putreg32(ymd, T113_RTC_YMD);

  /* Request HMS write */

  putreg32(T113_RTC_LOSC_MAGIC | ctrl | T113_RTC_LOSC_HMS_ACC,
           T113_RTC_LOSC_CTRL);
  for (i = 0; i < ACC_TIMEOUT_US; i++)
    {
      if ((getreg32(T113_RTC_LOSC_CTRL) & T113_RTC_LOSC_HMS_ACC) == 0)
        {
          break;
        }

      up_udelay(1);
    }

  if (i >= ACC_TIMEOUT_US)
    {
      ret = -ETIMEDOUT;
      goto unlock_out;
    }

  putreg32(hms, T113_RTC_HMS);

  /* Keep GP_DATA[2] (the FEL-rescue boot timestamp) in lockstep with
   * the wall clock: settime() jumps RTC to a new epoch, so the next
   * boot's delta comparison must see the new epoch as the reference,
   * otherwise the elapsed-time guard would mis-classify a short
   * crash loop as cooled-down.
   *
   * Compute the stamp directly from the target rtctime instead of
   * reading the YMD/HMS registers back: the LOSC-domain latch is
   * asynchronous, and a read immediately after the HMS write can
   * still return the old value for ~1 LOSC tick, which would defeat
   * the hook entirely.  Formula must mirror t113_rtc_get_seconds() -
   * `mon + 1` here converts struct rtc_time's 0-11 month to the raw
   * YMD register encoding (1-12), which is what get_seconds reads.
   *
   * Done inside the spinlock so two concurrent settime() callers can't
   * race on GP_DATA[2] either.
   */

  t113_rtc_set_boottime(((((uint32_t)year * 365 +
                           (uint32_t)(rtctime->tm_mon + 1) * 31 +
                           (uint32_t)rtctime->tm_mday) * 24 +
                          (uint32_t)rtctime->tm_hour) * 60 +
                         (uint32_t)rtctime->tm_min) * 60 +
                        (uint32_t)rtctime->tm_sec);

unlock_out:
  spin_unlock_irqrestore(&g_rtc_settime_lock, flags);
  return ret;
}

static bool t113_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  /* Month == 0 means "never set": the hardware resets to month 0,
   * and a valid month is 1-12 (MON_SHIFT field).  Year == 0 is a
   * valid value (1970 = YEAR_BASE) and cannot be used as the
   * sentinel.
   */

  uint32_t ymd = getreg32(T113_RTC_YMD);
  return (ymd & T113_RTC_YMD_MON_MASK) != 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_rtc_initialize(void)
{
  rtc_initialize(0, &g_rtc_lowerhalf);
}

int up_rtc_initialize(void)
{
  g_rtc_enabled = true;
  return OK;
}

int up_rtc_getdatetime(FAR struct tm *tp)
{
  struct rtc_time rtctime;
  t113_rtc_rdtime(&g_rtc_lowerhalf, &rtctime);
  tp->tm_sec  = rtctime.tm_sec;
  tp->tm_min  = rtctime.tm_min;
  tp->tm_hour = rtctime.tm_hour;
  tp->tm_mday = rtctime.tm_mday;
  tp->tm_mon  = rtctime.tm_mon;
  tp->tm_year = rtctime.tm_year;

  /* Fields not provided by the hardware must be explicitly zeroed
   * to avoid leaking stack garbage into callers such as mktime().
   */

  tp->tm_wday  = 0;
  tp->tm_yday  = 0;
  tp->tm_isdst = 0;
  return OK;
}

#ifndef CONFIG_RTC_HIRES
int up_rtc_settime(FAR const struct timespec *ts)
{
  struct rtc_time rtctime;
  struct tm t;

  gmtime_r(&ts->tv_sec, &t);
  rtctime.tm_sec  = t.tm_sec;
  rtctime.tm_min  = t.tm_min;
  rtctime.tm_hour = t.tm_hour;
  rtctime.tm_mday = t.tm_mday;
  rtctime.tm_mon  = t.tm_mon;
  rtctime.tm_year = t.tm_year;

  return t113_rtc_settime(&g_rtc_lowerhalf, &rtctime);
}
#endif

#endif /* CONFIG_T113_RTC */
