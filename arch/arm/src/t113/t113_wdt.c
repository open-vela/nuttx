/****************************************************************************
 * arch/arm/src/t113/t113_wdt.c
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

#include <stdint.h>
#include <inttypes.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/timers/watchdog.h>

#include "arm_internal.h"
#include "hardware/t113_wdt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Short aliases for readability */

#define WDOG_IRQ_EN_REG       T113_WDOG_IRQ_EN_REG
#define WDOG_IRQ_STA_REG      T113_WDOG_IRQ_STA_REG
#define WDOG_SOFT_RST_REG     T113_WDOG_SOFT_RST_REG
#define WDOG_CTRL_REG         T113_WDOG_CTRL_REG
#define WDOG_CFG_REG          T113_WDOG_CFG_REG
#define WDOG_MODE_REG         T113_WDOG_MODE_REG

#define WDOG_SOFT_RST_KEY     T113_WDOG_SOFT_RST_KEY
#define WDOG_CTRL_KEY         T113_WDOG_CTRL_KEY
#define WDOG_CTRL_RESTART     T113_WDOG_CTRL_RESTART
#define WDOG_CFG_RESET        T113_WDOG_CFG_RESET
#define WDOG_CFG_IRQ          T113_WDOG_CFG_IRQ
#define WDOG_MODE_EN          T113_WDOG_MODE_EN
#define WDOG_MODE_KEY         T113_WDOG_MODE_KEY

#define WDOG_MODE_INTV_SHIFT  T113_WDOG_MODE_INTV_SHIFT
#define WDOG_MODE_INTV_MASK   T113_WDOG_MODE_INTV_MASK

/* The 2-arg watchdog_register() call below cannot satisfy the AUTOMONITOR
 * wrapper signatures.  Guard against the misconfiguration at compile time.
 */

#if defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_ONESHOT) || \
    defined(CONFIG_WATCHDOG_AUTOMONITOR_BY_TIMER)
#  error "T113 WDT driver does not support AUTOMONITOR configurations"
#endif

static const uint16_t g_timeout_ms[] =
{
  500, 1000, 2000, 3000, 4000, 5000, 6000,
  8000, 10000, 12000, 14000, 16000
};

#define NTIMEOUTS (sizeof(g_timeout_ms) / sizeof(g_timeout_ms[0]))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_wdt_lowerhalf_s
{
  const struct watchdog_ops_s *ops;
  uint32_t timeout_ms;
  bool     started;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int t113_wdt_start(struct watchdog_lowerhalf_s *lower);
static int t113_wdt_stop(struct watchdog_lowerhalf_s *lower);
static int t113_wdt_keepalive(struct watchdog_lowerhalf_s *lower);
static int t113_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                              struct watchdog_status_s *status);
static int t113_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                               uint32_t timeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_wdtops =
{
  .start      = t113_wdt_start,
  .stop       = t113_wdt_stop,
  .keepalive  = t113_wdt_keepalive,
  .getstatus  = t113_wdt_getstatus,
  .settimeout = t113_wdt_settimeout,
  .capture    = NULL,
  .ioctl      = NULL,
};

static struct t113_wdt_lowerhalf_s g_wdt =
{
  .ops        = &g_wdtops,
  .timeout_ms = 6000,
  .started    = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int t113_wdt_intv_index(uint32_t timeout_ms)
{
  unsigned int i;

  /* Round up: return the first entry >= timeout_ms so the caller always
   * gets at least the grace period it asked for.  If the request exceeds
   * the largest supported interval, saturate at the last entry.
   */

  for (i = 0; i < NTIMEOUTS; i++)
    {
      if (g_timeout_ms[i] >= timeout_ms)
        {
          return (int)i;
        }
    }

  return (int)NTIMEOUTS - 1;
}

static int t113_wdt_start(struct watchdog_lowerhalf_s *lower)
{
  struct t113_wdt_lowerhalf_s *priv =
    (struct t113_wdt_lowerhalf_s *)lower;
  int idx = t113_wdt_intv_index(priv->timeout_ms);

  putreg32(WDOG_CFG_RESET, WDOG_CFG_REG);
  putreg32(WDOG_MODE_KEY |
           (idx << WDOG_MODE_INTV_SHIFT) |
           WDOG_MODE_EN,
           WDOG_MODE_REG);
  putreg32(WDOG_CTRL_RESTART, WDOG_CTRL_REG);

  priv->started = true;

  /* Surface the actual hardware programming.  If settimeout() silently
   * failed earlier, the timeout_ms used here will be the initializer
   * default (see g_wdt), which is the fingerprint of that misconfig.
   */

  wdinfo("WDT started: timeout=%" PRIu32 " ms, INTV idx=%d\n",
         priv->timeout_ms, idx);
  return OK;
}

static int t113_wdt_stop(struct watchdog_lowerhalf_s *lower)
{
  struct t113_wdt_lowerhalf_s *priv =
    (struct t113_wdt_lowerhalf_s *)lower;

  /* Key + EN=0 disables the counter; then W1C any pending IRQ and clear
   * the CFG register so the block is in a clean state for the next start.
   */

  putreg32(WDOG_MODE_KEY, WDOG_MODE_REG);
  putreg32(1, WDOG_IRQ_STA_REG);
  putreg32(0, WDOG_CFG_REG);

  priv->started = false;
  return OK;
}

static int t113_wdt_keepalive(struct watchdog_lowerhalf_s *lower)
{
  putreg32(WDOG_CTRL_RESTART, WDOG_CTRL_REG);
  return OK;
}

static int t113_wdt_getstatus(struct watchdog_lowerhalf_s *lower,
                              struct watchdog_status_s *status)
{
  struct t113_wdt_lowerhalf_s *priv =
    (struct t113_wdt_lowerhalf_s *)lower;

  status->flags = priv->started ? WDFLAGS_ACTIVE : 0;
  status->timeout = priv->timeout_ms;

  /* The T113 watchdog has no down-counter we can read back, so we cannot
   * report the real remaining time.  Report the full quantized timeout
   * while running (conservative) and zero when stopped.
   */

  status->timeleft = priv->started ? priv->timeout_ms : 0;
  return OK;
}

static int t113_wdt_settimeout(struct watchdog_lowerhalf_s *lower,
                               uint32_t timeout)
{
  struct t113_wdt_lowerhalf_s *priv =
    (struct t113_wdt_lowerhalf_s *)lower;
  int idx;

  if (timeout > 16000)
    {
      wderr("ERROR: timeout %" PRIu32 " ms exceeds hardware max "
            "(16000 ms), request rejected\n", timeout);
      return -ERANGE;
    }

  idx = t113_wdt_intv_index(timeout);
  priv->timeout_ms = g_timeout_ms[idx];

  if (priv->started)
    {
      t113_wdt_start(lower);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_wdt_initialize(void)
{
  watchdog_register("/dev/watchdog0",
                    (struct watchdog_lowerhalf_s *)&g_wdt);
}

