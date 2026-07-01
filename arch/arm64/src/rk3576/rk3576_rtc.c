/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_rtc.c
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
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_rtc.h"
#include "rk3576_rtc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Days in each month (non-leap year) */

static const uint8_t g_days_in_month[] =
{
  31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* RTC base address */

const uint32_t g_rtc_base = RK3576_RTC_ADDR;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_rtc_is_leap_year
 *
 * Description:
 *   Check if a year is a leap year.
 *
 ****************************************************************************/

static inline bool rk3576_rtc_is_leap_year(int year)
{
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

/****************************************************************************
 * Name: rk3576_rtc_get_days_in_month
 *
 * Description:
 *   Get the number of days in a month.
 *
 ****************************************************************************/

static inline int rk3576_rtc_get_days_in_month(int year, int month)
{
  int days = g_days_in_month[month - 1];

  if (month == 2 && rk3576_rtc_is_leap_year(year))
    {
      days++;
    }

  return days;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_rtc_init
 *
 * Description:
 *   Initialize RTC. Enable 24-hour mode.
 *
 ****************************************************************************/

void rk3576_rtc_init(void)
{
  uint32_t ctrl;

  /* Enable RTC with 24-hour mode */

  ctrl = RTC_CONTROL_EN | RTC_CONTROL_HOUR24;
  putreg32(ctrl, g_rtc_base + RTC_CONTROL);

  /* Clear any pending interrupts */

  putreg32(0xffffffff, g_rtc_base + RTC_INTSTS);

  /* Disable all interrupts */

  putreg32(0, g_rtc_base + RTC_INTEN);

  rtcinfo("RTC: initialized\n");
}

/****************************************************************************
 * Name: rk3576_rtc_get_time
 *
 * Description:
 *   Get current time from RTC.
 *
 ****************************************************************************/

int rk3576_rtc_get_time(struct tm *tm)
{
  if (tm == NULL)
    {
      return -EINVAL;
    }

  /* Wait for RTC not busy */

  {
    int timeout = 100;
    while (timeout--)
      {
        if (!(getreg32(g_rtc_base + RTC_STATUS) & RTC_STATUS_BUSY))
          {
            break;
          }

        up_udelay(10);
      }
  }

  /* Read time registers atomically by retrying if values change */

  {
    uint32_t sec1, sec2, min1, min2, hr1, hr2;
    int retries = 3;

    do
      {
        sec1 = getreg32(g_rtc_base + RTC_SECONDS) & 0x3f;
        min1 = getreg32(g_rtc_base + RTC_MINUTES) & 0x3f;
        hr1  = getreg32(g_rtc_base + RTC_HOURS) & 0x1f;

        sec2 = getreg32(g_rtc_base + RTC_SECONDS) & 0x3f;
        min2 = getreg32(g_rtc_base + RTC_MINUTES) & 0x3f;
        hr2  = getreg32(g_rtc_base + RTC_HOURS) & 0x1f;
      }
    while ((sec1 != sec2 || min1 != min2 || hr1 != hr2) && --retries > 0);

    tm->tm_sec = (int)sec2;
    tm->tm_min = (int)min2;
    tm->tm_hour = (int)hr2;
  }
  tm->tm_mday = (int)(getreg32(g_rtc_base + RTC_DAYS) & 0x1f);
  tm->tm_mon = (int)(getreg32(g_rtc_base + RTC_MONTHS) & 0x0f) - 1;
  tm->tm_year = (int)(getreg32(g_rtc_base + RTC_YEARS) & 0xff);
  tm->tm_wday = (int)(getreg32(g_rtc_base + RTC_WEEKDAY) & 0x07);

  /* Adjust year (RTC stores 0-99, need to add 1900 or 2000) */

  if (tm->tm_year < 70)
    {
      tm->tm_year += 100;   /* 2000-2069 */
    }
  else
    {
      tm->tm_year += 0;     /* 1970-1999 */
    }

  /* Calculate day of year */

  tm->tm_yday = 0;
  for (int m = 1; m < tm->tm_mon + 1; m++)
    {
      tm->tm_yday += rk3576_rtc_get_days_in_month(tm->tm_year + 1900, m);
    }

  tm->tm_yday += tm->tm_mday - 1;

  /* Daylight savings - not used */

  tm->tm_isdst = 0;

  rtcinfo("RTC: %04d-%02d-%02d %02d:%02d:%02d\n",
          tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
          tm->tm_hour, tm->tm_min, tm->tm_sec);

  return OK;
}

/****************************************************************************
 * Name: rk3576_rtc_set_time
 *
 * Description:
 *   Set RTC time.
 *
 ****************************************************************************/

int rk3576_rtc_set_time(const struct tm *tm)
{
  int year;

  if (tm == NULL)
    {
      return -EINVAL;
    }

  year = tm->tm_year + 1900;

  /* Validate values */

  if (tm->tm_sec < 0 || tm->tm_sec > 59 ||
      tm->tm_min < 0 || tm->tm_min > 59 ||
      tm->tm_hour < 0 || tm->tm_hour > 23 ||
      tm->tm_mday < 1 || tm->tm_mday > 31 ||
      tm->tm_mon < 0 || tm->tm_mon > 11 ||
      year < 2000 || year > 2099)
    {
      return -EINVAL;
    }

  /* Disable RTC during update */

  putreg32(0, g_rtc_base + RTC_CONTROL);

  /* Write time registers */

  putreg32((uint32_t)tm->tm_sec, g_rtc_base + RTC_SECONDS);
  putreg32((uint32_t)tm->tm_min, g_rtc_base + RTC_MINUTES);
  putreg32((uint32_t)tm->tm_hour, g_rtc_base + RTC_HOURS);
  putreg32((uint32_t)tm->tm_mday, g_rtc_base + RTC_DAYS);
  putreg32((uint32_t)(tm->tm_mon + 1), g_rtc_base + RTC_MONTHS);
  putreg32((uint32_t)(year % 100), g_rtc_base + RTC_YEARS);
  putreg32((uint32_t)tm->tm_wday, g_rtc_base + RTC_WEEKDAY);

  /* Re-enable RTC */

  putreg32(RTC_CONTROL_EN | RTC_CONTROL_HOUR24, g_rtc_base + RTC_CONTROL);

  rtcinfo("RTC: set to %04d-%02d-%02d %02d:%02d:%02d\n",
          year, tm->tm_mon + 1, tm->tm_mday,
          tm->tm_hour, tm->tm_min, tm->tm_sec);

  return OK;
}

/****************************************************************************
 * Name: rk3576_rtc_set_alarm
 *
 * Description:
 *   Set RTC alarm.
 *
 ****************************************************************************/

int rk3576_rtc_set_alarm(const struct tm *tm)
{
  int year;

  if (tm == NULL)
    {
      return -EINVAL;
    }

  year = tm->tm_year + 1900;

  /* Write alarm registers */

  putreg32((uint32_t)tm->tm_sec, g_rtc_base + RTC_ALARM_SEC);
  putreg32((uint32_t)tm->tm_min, g_rtc_base + RTC_ALARM_MIN);
  putreg32((uint32_t)tm->tm_hour, g_rtc_base + RTC_ALARM_HOUR);
  putreg32((uint32_t)tm->tm_mday, g_rtc_base + RTC_ALARM_DAY);
  putreg32((uint32_t)(tm->tm_mon + 1), g_rtc_base + RTC_ALARM_MON);
  putreg32((uint32_t)(year % 100), g_rtc_base + RTC_ALARM_YEAR);

  /* Enable alarm interrupt */

  {
    uint32_t inten = getreg32(g_rtc_base + RTC_INTEN);
    inten |= RTC_INTEN_ALARM_EN;
    putreg32(inten, g_rtc_base + RTC_INTEN);
  }

  rtcinfo("RTC: alarm set to %04d-%02d-%02d %02d:%02d:%02d\n",
          year, tm->tm_mon + 1, tm->tm_mday,
          tm->tm_hour, tm->tm_min, tm->tm_sec);

  return OK;
}

/****************************************************************************
 * Name: rk3576_rtc_cancel_alarm
 *
 * Description:
 *   Cancel RTC alarm.
 *
 ****************************************************************************/

int rk3576_rtc_cancel_alarm(void)
{
  uint32_t inten = getreg32(g_rtc_base + RTC_INTEN);
  inten &= ~RTC_INTEN_ALARM_EN;
  putreg32(inten, g_rtc_base + RTC_INTEN);

  rtcinfo("RTC: alarm cancelled\n");
  return OK;
}

/****************************************************************************
 * Name: rk3576_rtc_enable_irq
 *
 * Description:
 *   Enable RTC interrupt sources.
 *
 ****************************************************************************/

void rk3576_rtc_enable_irq(int sources)
{
  uint32_t inten = getreg32(g_rtc_base + RTC_INTEN);
  inten |= (uint32_t)sources;
  putreg32(inten, g_rtc_base + RTC_INTEN);
}

/****************************************************************************
 * Name: rk3576_rtc_disable_irq
 *
 * Description:
 *   Disable RTC interrupt sources.
 *
 ****************************************************************************/

void rk3576_rtc_disable_irq(int sources)
{
  uint32_t inten = getreg32(g_rtc_base + RTC_INTEN);
  inten &= ~(uint32_t)sources;
  putreg32(inten, g_rtc_base + RTC_INTEN);
}

/****************************************************************************
 * Name: rk3576_rtc_clear_irq
 *
 * Description:
 *   Clear all pending RTC interrupts.
 *
 ****************************************************************************/

void rk3576_rtc_clear_irq(void)
{
  putreg32(0xffffffff, g_rtc_base + RTC_INTSTS);
}
