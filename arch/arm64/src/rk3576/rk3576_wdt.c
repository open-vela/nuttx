/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_wdt.c
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
#include "hardware/rk3576_wdt.h"
#include "rk3576_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Watchdog clock frequency (24MHz) */

#define WDT_CLK_FREQ               24000000

/* Watchdog counter resolution */

#define WDT_FREQ_1HZ               0   /* Divisor 256 */
#define WDT_FREQ_1KHZ              1   /* Divisor 128 */
#define WDT_FREQ_1MHZ              2   /* Divisor 64 */
#define WDT_FREQ_2MHZ              3   /* Divisor 32 */
#define WDT_FREQ_12MHZ             4   /* Divisor 2 */
#define WDT_FREQ_24MHZ             5   /* Divisor 1 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Watchdog base address */

const uint32_t g_wdt_base = RK3576_WDT_ADDR;

/* Watchdog state */

static bool g_wdt_enabled = false;
static int g_wdt_mode = RK3576_WDT_MODE_RESET;
static uint32_t g_wdt_timeout_ms = 0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_wdt_calc_torr
 *
 * Description:
 *   Calculate timeout range register value for the given timeout.
 *   TORR defines the upper 4 bits of the timeout counter.
 *   Timeout = (0xFFFFFFF * (TORR+1) * prescaler) / CLK_FREQ
 *
 ****************************************************************************/

static int rk3576_wdt_calc_torr(uint32_t timeout_ms)
{
  int torr;

  /* Find TORR value (upper 4 bits of 28-bit counter) */
  /* Counter = (0xFFFFFFF) >> (16 - TORR) */
  /* Timeout = Counter / CLK_FREQ * 1000 ms */

  for (torr = 0; torr <= 15; torr++)
    {
      uint32_t counter = 0xfffffff >> (16 - torr);
      uint32_t timeout_us = (uint32_t)((uint64_t)counter * 1000000 / WDT_CLK_FREQ);

      if (timeout_us >= timeout_ms * 1000)
        {
          return torr;
        }
    }

  /* Use maximum timeout */

  return 15;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_wdt_init
 *
 * Description:
 *   Initialize watchdog timer. Disable watchdog.
 *
 ****************************************************************************/

void rk3576_wdt_init(void)
{
  /* Disable watchdog */

  putreg32(0, g_wdt_base + WDT_CR);

  /* Clear any pending status */

  putreg32(0, g_wdt_base + WDT_STAT);

  g_wdt_enabled = false;

  wdinfo("WDT: initialized (disabled)\n");
}

/****************************************************************************
 * Name: rk3576_wdt_start
 *
 * Description:
 *   Start watchdog with the specified timeout.
 *   If timeout is 0, uses the previously configured timeout.
 *
 ****************************************************************************/

void rk3576_wdt_start(uint32_t timeout_ms)
{
  uint32_t cr;
  int torr;

  if (timeout_ms > 0)
    {
      g_wdt_timeout_ms = timeout_ms;
    }

  /* Calculate timeout range */

  torr = rk3576_wdt_calc_torr(g_wdt_timeout_ms);
  putreg32(torr, g_wdt_base + WDT_TORR);

  /* Configure control register */

  cr = WDT_CR_WDT_EN;

  if (g_wdt_mode == RK3576_WDT_MODE_RESET)
    {
      cr |= WDT_CR_RST_EN;
    }
  else
    {
      cr |= WDT_CR_IRQ_EN;
    }

  putreg32(cr, g_wdt_base + WDT_CR);

  /* Restart the counter */

  putreg32(WDT_CRR_RESTART, g_wdt_base + WDT_CRR);

  g_wdt_enabled = true;

  wdinfo("WDT: started, timeout %u ms\n", g_wdt_timeout_ms);
}

/****************************************************************************
 * Name: rk3576_wdt_stop
 *
 * Description:
 *   Stop watchdog timer.
 *
 ****************************************************************************/

void rk3576_wdt_stop(void)
{
  putreg32(0, g_wdt_base + WDT_CR);

  g_wdt_enabled = false;

  wdinfo("WDT: stopped\n");
}

/****************************************************************************
 * Name: rk3576_wdt_feed
 *
 * Description:
 *   Feed (kick) the watchdog to prevent reset.
 *
 ****************************************************************************/

void rk3576_wdt_feed(void)
{
  putreg32(WDT_CRR_RESTART, g_wdt_base + WDT_CRR);
}

/****************************************************************************
 * Name: rk3576_wdt_set_timeout
 *
 * Description:
 *   Set watchdog timeout. Does not start the watchdog.
 *
 ****************************************************************************/

int rk3576_wdt_set_timeout(uint32_t timeout_ms)
{
  if (timeout_ms < WDT_TIMEOUT_MIN || timeout_ms > WDT_TIMEOUT_MAX * 1000)
    {
      wdinfo("WDT: timeout %u ms out of range\n", timeout_ms);
      return -EINVAL;
    }

  g_wdt_timeout_ms = timeout_ms;

  wdinfo("WDT: timeout set to %u ms\n", timeout_ms);
  return OK;
}

/****************************************************************************
 * Name: rk3576_wdt_set_mode
 *
 * Description:
 *   Set watchdog mode (reset or interrupt).
 *
 ****************************************************************************/

int rk3576_wdt_set_mode(int mode)
{
  if (mode != RK3576_WDT_MODE_RESET && mode != RK3576_WDT_MODE_INTERRUPT)
    {
      return -EINVAL;
    }

  g_wdt_mode = mode;

  /* Update if already running */

  if (g_wdt_enabled)
    {
      uint32_t cr = getreg32(g_wdt_base + WDT_CR);
      cr &= ~(WDT_CR_RST_EN | WDT_CR_IRQ_EN);

      if (mode == RK3576_WDT_MODE_RESET)
        {
          cr |= WDT_CR_RST_EN;
        }
      else
        {
          cr |= WDT_CR_IRQ_EN;
        }

      putreg32(cr, g_wdt_base + WDT_CR);
    }

  wdinfo("WDT: mode set to %s\n",
          mode == 0 ? "reset" : "interrupt");
  return OK;
}

/****************************************************************************
 * Name: rk3576_wdt_is_enabled
 *
 * Description:
 *   Check if watchdog is enabled.
 *
 ****************************************************************************/

int rk3576_wdt_is_enabled(void)
{
  return g_wdt_enabled ? 1 : 0;
}

/****************************************************************************
 * Name: rk3576_wdt_get_remaining
 *
 * Description:
 *   Get remaining time before watchdog timeout (in ms).
 *
 ****************************************************************************/

uint32_t rk3576_wdt_get_remaining(void)
{
  uint32_t counter;

  if (!g_wdt_enabled)
    {
      return 0;
    }

  counter = getreg32(g_wdt_base + WDT_CCVR);

  /* Convert counter to milliseconds */

  return (uint32_t)((uint64_t)counter * 1000 / WDT_CLK_FREQ);
}
