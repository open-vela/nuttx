/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_rtc.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>
#include <nuttx/timers/rtc.h>
#include <nuttx/timers/arch_rtc.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_rtc.h"

#if defined(CONFIG_LS2K0300_RTC)

struct ls2k0300_rtc_lowerhalf_s
{
  const struct rtc_ops_s *ops;
  uintptr_t base;
  bool haveset;
};

static uint32_t rtc_getreg(FAR struct ls2k0300_rtc_lowerhalf_s *priv,
                           uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void rtc_putreg(FAR struct ls2k0300_rtc_lowerhalf_s *priv,
                       uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/* Lazy repair state: the counter is only re-armed after the board bringup
 * has had a chance to run the enable/load sequence late in boot.
 */

static bool g_rtc_late_init;
static int g_rtc_repairs;
static clock_t g_rtc_repair_tick;

static bool ls2k0300_rtc_time_valid(uint32_t read0, uint32_t read1);
static void ls2k0300_rtc_hw_init(FAR struct ls2k0300_rtc_lowerhalf_s *priv);

static int ls2k0300_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                              FAR struct rtc_time *rtctime)
{
  FAR struct ls2k0300_rtc_lowerhalf_s *priv =
      (FAR struct ls2k0300_rtc_lowerhalf_s *)lower;
  uint32_t toy_read0;
  uint32_t toy_read1;
  irqstate_t flags;

  memset(rtctime, 0, sizeof(struct rtc_time));

  flags = enter_critical_section();

  toy_read1 = rtc_getreg(priv, LS2K0300_RTC_TOY_READ1_REG);
  toy_read0 = rtc_getreg(priv, LS2K0300_RTC_TOY_READ0_REG);

  leave_critical_section(flags);

  /* If the TOY still holds an invalid value once boot has settled, the
   * early load never took effect.  Re-arm it here so the clock starts
   * without anyone having to run `date -s`.  Attempts are spaced a second
   * apart and capped, so a dead crystal cannot stall every read.
   */

  if (g_rtc_late_init && !ls2k0300_rtc_time_valid(toy_read0, toy_read1))
    {
      clock_t now = clock_systime_ticks();

      if (g_rtc_repairs < 3 &&
          (g_rtc_repairs == 0 ||
           now - g_rtc_repair_tick >= MSEC2TICK(1000)))
        {
          g_rtc_repairs++;
          g_rtc_repair_tick = now;

          ls2k0300_rtc_hw_init(priv);

          flags = enter_critical_section();
          toy_read1 = rtc_getreg(priv, LS2K0300_RTC_TOY_READ1_REG);
          toy_read0 = rtc_getreg(priv, LS2K0300_RTC_TOY_READ0_REG);
          leave_critical_section(flags);
        }
    }

  rtctime->tm_sec  = (toy_read0 >> RTC_TOY_SEC_SHIFT) & RTC_TOY_SEC_MASK;
  rtctime->tm_min  = (toy_read0 >> RTC_TOY_MIN_SHIFT) & RTC_TOY_MIN_MASK;
  rtctime->tm_hour = (toy_read0 >> RTC_TOY_HOUR_SHIFT) & RTC_TOY_HOUR_MASK;
  rtctime->tm_mday = (toy_read0 >> RTC_TOY_DAY_SHIFT) & RTC_TOY_DAY_MASK;
  rtctime->tm_mon  = ((toy_read0 >> RTC_TOY_MON_SHIFT) &
                       RTC_TOY_MON_MASK) - 1;
  rtctime->tm_year = toy_read1;

  if (rtctime->tm_mon < 0 || rtctime->tm_mon > 11 ||
      rtctime->tm_mday < 1 || rtctime->tm_mday > 31 ||
      rtctime->tm_hour > 23 || rtctime->tm_min > 59 ||
      rtctime->tm_sec > 61)
    {
      rtctime->tm_sec  = 0;
      rtctime->tm_min  = 0;
      rtctime->tm_hour = 0;
      rtctime->tm_mday = 1;
      rtctime->tm_mon  = 0;
      rtctime->tm_year = 0;
    }

  rtctime->tm_wday  = -1;
  rtctime->tm_yday  = -1;
  rtctime->tm_isdst = -1;
#ifdef CONFIG_RTC_HIRES
  rtctime->tm_nsec  = 0;
#endif

  return OK;
}

static int ls2k0300_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                               FAR const struct rtc_time *rtctime)
{
  FAR struct ls2k0300_rtc_lowerhalf_s *priv =
      (FAR struct ls2k0300_rtc_lowerhalf_s *)lower;
  uint32_t toy_write0;
  irqstate_t flags;

  toy_write0  = (rtctime->tm_sec << RTC_TOY_SEC_SHIFT);
  toy_write0 |= (rtctime->tm_min << RTC_TOY_MIN_SHIFT);
  toy_write0 |= (rtctime->tm_hour << RTC_TOY_HOUR_SHIFT);
  toy_write0 |= (rtctime->tm_mday << RTC_TOY_DAY_SHIFT);
  toy_write0 |= ((rtctime->tm_mon + 1) << RTC_TOY_MON_SHIFT);

  flags = enter_critical_section();

  rtc_putreg(priv, LS2K0300_RTC_TOY_WRITE0_REG, toy_write0);
  rtc_putreg(priv, LS2K0300_RTC_TOY_WRITE1_REG, (uint32_t)rtctime->tm_year);

  leave_critical_section(flags);

  priv->haveset = true;

  return OK;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool ls2k0300_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  FAR struct ls2k0300_rtc_lowerhalf_s *priv =
      (FAR struct ls2k0300_rtc_lowerhalf_s *)lower;

  return priv->haveset;
}

#ifdef CONFIG_RTC_ALARM
static int ls2k0300_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower,
                                FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct ls2k0300_rtc_lowerhalf_s *priv =
      (FAR struct ls2k0300_rtc_lowerhalf_s *)lower;
  uint32_t val;
  irqstate_t flags;

  val  = (alarminfo->time.tm_sec << RTC_TOY_MATCH_SEC_SHIFT);
  val |= (alarminfo->time.tm_min << RTC_TOY_MATCH_MIN_SHIFT);
  val |= (alarminfo->time.tm_hour << RTC_TOY_MATCH_HOUR_SHIFT);
  val |= (alarminfo->time.tm_mday << RTC_TOY_MATCH_DAY_SHIFT);
  val |= ((alarminfo->time.tm_mon + 1) << RTC_TOY_MATCH_MON_SHIFT);
  val |= ((alarminfo->time.tm_year & RTC_TOY_MATCH_YEAR_MASK)
          << RTC_TOY_MATCH_YEAR_SHIFT);

  flags = enter_critical_section();
  rtc_putreg(priv, LS2K0300_RTC_TOY_MATCH0_REG, val);
  leave_critical_section(flags);

  return OK;
}

static int ls2k0300_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                                   int alarmid)
{
  FAR struct ls2k0300_rtc_lowerhalf_s *priv =
      (FAR struct ls2k0300_rtc_lowerhalf_s *)lower;

  rtc_putreg(priv, LS2K0300_RTC_TOY_MATCH0_REG, 0);

  return OK;
}

static int ls2k0300_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                               FAR struct lower_rdalarm_s *alarminfo)
{
  FAR struct ls2k0300_rtc_lowerhalf_s *priv =
      (FAR struct ls2k0300_rtc_lowerhalf_s *)lower;
  uint32_t val;
  irqstate_t flags;

  flags = enter_critical_section();
  val = rtc_getreg(priv, LS2K0300_RTC_TOY_MATCH0_REG);
  leave_critical_section(flags);

  alarminfo->time->tm_sec  = (val >> RTC_TOY_MATCH_SEC_SHIFT) &
                              RTC_TOY_MATCH_SEC_MASK;
  alarminfo->time->tm_min  = (val >> RTC_TOY_MATCH_MIN_SHIFT) &
                              RTC_TOY_MATCH_MIN_MASK;
  alarminfo->time->tm_hour = (val >> RTC_TOY_MATCH_HOUR_SHIFT) &
                              RTC_TOY_MATCH_HOUR_MASK;
  alarminfo->time->tm_mday = (val >> RTC_TOY_MATCH_DAY_SHIFT) &
                              RTC_TOY_MATCH_DAY_MASK;
  alarminfo->time->tm_mon  = ((val >> RTC_TOY_MATCH_MON_SHIFT) &
                              RTC_TOY_MATCH_MON_MASK) - 1;
  alarminfo->time->tm_year = (val >> RTC_TOY_MATCH_YEAR_SHIFT) &
                              RTC_TOY_MATCH_YEAR_MASK;

  return OK;
}
#endif

static const struct rtc_ops_s g_rtc_ops =
{
  .rdtime      = ls2k0300_rtc_rdtime,
  .settime     = ls2k0300_rtc_settime,
  .havesettime = ls2k0300_rtc_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = ls2k0300_rtc_setalarm,
  .cancelalarm = ls2k0300_rtc_cancelalarm,
  .rdalarm     = ls2k0300_rtc_rdalarm,
#endif
};

static struct ls2k0300_rtc_lowerhalf_s g_rtc_lowerhalf =
{
  .ops    = &g_rtc_ops,
  .base   = LS2K0300_RTC_BASE,
  .haveset = false,
};

/* A TOY value is only meaningful if the fields decode to a real date.  The
 * counter reads back as zero (1900-01-01) when it has never been loaded, and
 * writing such a value back leaves the clock frozen.
 */

static bool ls2k0300_rtc_time_valid(uint32_t read0, uint32_t read1)
{
  uint32_t mon  = (read0 >> RTC_TOY_MON_SHIFT) & RTC_TOY_MON_MASK;
  uint32_t day  = (read0 >> RTC_TOY_DAY_SHIFT) & RTC_TOY_DAY_MASK;
  uint32_t hour = (read0 >> RTC_TOY_HOUR_SHIFT) & RTC_TOY_HOUR_MASK;
  uint32_t min  = (read0 >> RTC_TOY_MIN_SHIFT) & RTC_TOY_MIN_MASK;
  uint32_t sec  = (read0 >> RTC_TOY_SEC_SHIFT) & RTC_TOY_SEC_MASK;

  if (read1 < 100 || read1 > 200)
    {
      return false;
    }

  if (mon < 1 || mon > 12 || day < 1 || day > 31 ||
      hour > 23 || min > 59 || sec > 59)
    {
      return false;
    }

  return true;
}

static void ls2k0300_rtc_hw_init(FAR struct ls2k0300_rtc_lowerhalf_s *priv)
{
  uint32_t ctrl = 0;
  uint32_t read0;
  uint32_t read1;
  int i;

  rtc_putreg(priv, LS2K0300_RTC_TOY_TRIM_REG, 0x0);
  rtc_putreg(priv, LS2K0300_RTC_TRIM_REG, 0x0);

  /* The trim writes are asynchronous; let them land before the control
   * register is touched.
   */

  up_mdelay(1);

  /* RTCEN runs the seconds counter, TOYEN the time-of-year counter and
   * EO the 32.768 kHz oscillator.  This register also carries read-only
   * status bits and reads of this block are not always reliable on this
   * SoC, so write the wanted value instead of read-modify-write.
   */

  rtc_putreg(priv, LS2K0300_RTC_CTRL_REG,
             RTC_CTRL_RTC_ENABLE_BIT | RTC_CTRL_TOY_ENABLE_BIT |
             RTC_CTRL_OSC_ENABLE_BIT);

  /* The crystal takes time to start, and a TOY write issued before the
   * 32.768 kHz clock runs is dropped.  Give it a moment, but do not trust
   * the status bit: the load below is what actually proves the RTC runs.
   */

  for (i = 0; i < 100; i++)
    {
      ctrl = rtc_getreg(priv, LS2K0300_RTC_CTRL_REG);
      if ((ctrl & RTC_CTRL_OSC_READY_BIT) != 0)
        {
          break;
        }

      up_mdelay(1);
    }

  /* Writing TOY_WRITE also loads the counter; until that happens the TOY
   * holds its reset value and RTC_RD_TIME reads back a clock that never
   * moves -- which is why the time only started ticking after a manual
   * RTC_SET_TIME.  Reload whatever the counter already holds so a
   * battery-backed board keeps its time.
   */

  read1 = rtc_getreg(priv, LS2K0300_RTC_TOY_READ1_REG);
  read0 = rtc_getreg(priv, LS2K0300_RTC_TOY_READ0_REG);
  if (!ls2k0300_rtc_time_valid(read0, read1))
    {
      /* Fresh boot with nothing retained (or garbage): give the counter
       * a defined start (2025-01-01 00:00:00) rather than a frozen zero.
       */

      read0 = (1 << RTC_TOY_DAY_SHIFT) | (1 << RTC_TOY_MON_SHIFT);
      read1 = 2025 - 1900;
    }

  /* The load is dropped while the oscillator is still spinning up, so
   * retry until the counter actually holds a value instead of giving up
   * after one attempt.  Checking the readback rather than a status bit
   * keeps the wait proportional to how long the crystal actually needs.
   */

  for (i = 0; i < 100; i++)
    {
      rtc_putreg(priv, LS2K0300_RTC_TOY_WRITE0_REG, read0);
      rtc_putreg(priv, LS2K0300_RTC_TOY_WRITE1_REG, read1);
      up_mdelay(10);

      if (rtc_getreg(priv, LS2K0300_RTC_TOY_READ0_REG) != 0 &&
          rtc_getreg(priv, LS2K0300_RTC_TOY_READ1_REG) != 0)
        {
          break;
        }
    }

  if (i == 100)
    {
      rtcerr("ERROR: TOY counter did not load, ctrl=%08lx\n",
             (unsigned long)rtc_getreg(priv, LS2K0300_RTC_CTRL_REG));
    }
}

int ls2k0300_rtc_initialize(void)
{
  FAR struct rtc_lowerhalf_s *lower;
  int ret;

  ls2k0300_rtc_hw_init(&g_rtc_lowerhalf);

  lower = (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
  up_rtc_set_lowerhalf(lower, true);

  ret = rtc_initialize(0, lower);
  if (ret < 0)
    {
      rtcerr("ERROR: rtc_initialize failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_rtc_hw_reinit
 *
 * Description:
 *   Re-run the hardware enable/load sequence.  up_rtc_initialize() is
 *   called from clock_initialize() before the SoC clocks are fully set
 *   up, so the oscillator may not be running yet and the TOY load can be
 *   dropped.  The board bringup calls this once boot has progressed far
 *   enough for the write to stick.
 *
 ****************************************************************************/

void ls2k0300_rtc_hw_reinit(void)
{
  ls2k0300_rtc_hw_init(&g_rtc_lowerhalf);
  g_rtc_late_init = true;
}

#ifdef CONFIG_RTC_DATETIME
int up_rtc_getdatetime(FAR struct tm *tp)
{
  FAR struct rtc_lowerhalf_s *lower =
      (FAR struct rtc_lowerhalf_s *)&g_rtc_lowerhalf;
  struct rtc_time rtctime;
  int ret;

  memset(&rtctime, 0, sizeof(struct rtc_time));

  ret = lower->ops->rdtime(lower, &rtctime);
  if (ret == OK)
    {
      tp->tm_sec   = rtctime.tm_sec;
      tp->tm_min   = rtctime.tm_min;
      tp->tm_hour  = rtctime.tm_hour;
      tp->tm_mday  = rtctime.tm_mday;
      tp->tm_mon   = rtctime.tm_mon;
      tp->tm_year  = rtctime.tm_year;
      tp->tm_wday  = rtctime.tm_wday;
      tp->tm_yday  = rtctime.tm_yday;
      tp->tm_isdst = rtctime.tm_isdst;
      tp->tm_gmtoff = rtctime.tm_gmtoff;
      tp->tm_zone  = rtctime.tm_zone;
    }

  return ret;
}
#endif

#ifdef CONFIG_RTC
int up_rtc_initialize(void)
{
  return ls2k0300_rtc_initialize();
}
#endif

#endif
