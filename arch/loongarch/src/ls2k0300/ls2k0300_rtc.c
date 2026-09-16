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

static void ls2k0300_rtc_hw_init(FAR struct ls2k0300_rtc_lowerhalf_s *priv)
{
  uint32_t ctrl;

  rtc_putreg(priv, LS2K0300_RTC_TOY_TRIM_REG, 0x0);
  rtc_putreg(priv, LS2K0300_RTC_TRIM_REG, 0x0);

  ctrl = rtc_getreg(priv, LS2K0300_RTC_CTRL_REG);
  ctrl |= RTC_CTRL_TOY_ENABLE_BIT | RTC_CTRL_OSC_ENABLE_BIT;
  rtc_putreg(priv, LS2K0300_RTC_CTRL_REG, ctrl);
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
