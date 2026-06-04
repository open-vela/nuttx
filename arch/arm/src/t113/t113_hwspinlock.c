/****************************************************************************
 * arch/arm/src/t113/t113_hwspinlock.c
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

/* T113-S3 hardware spinlock driver.
 *
 * Exposes the 32 lock units of the SPINLOCK module at 0x03005000 as
 * NuttX hwspinlock_dev_s instances via the generic framework defined in
 * include/nuttx/hwspinlock/hwspinlock.h.  Acquire is a TAS read (read 0
 * = lock granted, read 1 = busy); release is write 0.  Hooked from
 * t113_bringup.c when CONFIG_T113_HWSPINLOCK=y, after the CCU has been
 * initialized so the SPINLOCK clock and reset can be ungated.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/hwspinlock/hwspinlock.h>

#include <arch/barriers.h>

#include "arm_internal.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_hwspinlock.h"
#include "t113_hwspinlock.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static bool t113_hwspinlock_trylock(FAR struct hwspinlock_dev_s *dev);
static void t113_hwspinlock_unlock(FAR struct hwspinlock_dev_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct hwspinlock_ops_s g_t113_hwspinlock_ops =
{
  .trylock = t113_hwspinlock_trylock,
  .unlock  = t113_hwspinlock_unlock,
};

static struct hwspinlock_dev_s
  g_t113_hwspinlock_devs[T113_HWSPINLOCK_NUM_LOCKS];
static bool g_t113_hwspinlock_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool t113_hwspinlock_trylock(FAR struct hwspinlock_dev_s *dev)
{
  /* Read 0 means we just acquired the lock.  Read 1 means it was already
   * taken by someone else.  A successful acquire pairs with a DMB so that
   * subsequent loads/stores in the critical section cannot be reordered
   * before the read that took the lock.
   */

  if (getreg32(T113_HWSPINLOCK_LOCK(dev->id)) == 0)
    {
      UP_DMB();
      return true;
    }

  return false;
}

static void t113_hwspinlock_unlock(FAR struct hwspinlock_dev_s *dev)
{
  /* Ensure all stores in the critical section are globally visible
   * before releasing the lock.
   */

  UP_DMB();
  putreg32(0, T113_HWSPINLOCK_LOCK(dev->id));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_hwspinlock_initialize(void)
{
#ifndef CONFIG_T113_RPTUN_SLAVE
  uint32_t val;
#endif
  int i;

  if (g_t113_hwspinlock_ready)
    {
      return;
    }

#ifndef CONFIG_T113_RPTUN_SLAVE
  /* CCU bring-up per T113-S3 user manual: de-assert reset (bit 16) first,
   * then enable clock gate (bit 0).  The BGR register defaults to 0 at
   * cold boot, so the IP starts gated and held in reset.  Use RMW to
   * preserve any unrelated bits sibling drivers may add later.
   *
   * NOTE: This is the one and only CCU access in the T113 port that
   * does *not* go through the t113_ccu_* shared helpers.  We are about
   * to make the SPINLOCK module itself usable; in AMP builds the helpers
   * cannot acquire a hardware spinlock until that module is up, so a
   * circular dependency would break the boot path.  Bare RMW is safe
   * here because t113_hwspinlock_initialize() runs at board bringup on
   * a single CPU before any sibling image / secondary CPU is online.
   *
   * AMP slave skips this RMW: master has already powered the SPINLOCK
   * module.  Slave still constructs its own 32 lock dev_s instances
   * below so its drivers can acquire/release through the framework.
   */

  val = getreg32(T113_CCU_SPINLOCK_BGR);
  val |= T113_CCU_SPINLOCK_RST;
  putreg32(val, T113_CCU_SPINLOCK_BGR);
  up_udelay(1);

  val |= T113_CCU_SPINLOCK_GATING;
  putreg32(val, T113_CCU_SPINLOCK_BGR);
  up_udelay(1);
#endif

  /* Construct the 32 dev_s instances.  All locks share the same ops and
   * priority; only the id varies.
   */

  for (i = 0; i < T113_HWSPINLOCK_NUM_LOCKS; i++)
    {
      g_t113_hwspinlock_devs[i].id       = i;
      g_t113_hwspinlock_devs[i].priority = 0;
      g_t113_hwspinlock_devs[i].ops      = &g_t113_hwspinlock_ops;
    }

  g_t113_hwspinlock_ready = true;
}

FAR struct hwspinlock_dev_s *t113_hwspinlock_get(int id)
{
  if (!g_t113_hwspinlock_ready ||
      id < 0 || id >= T113_HWSPINLOCK_NUM_LOCKS)
    {
      return NULL;
    }

  return &g_t113_hwspinlock_devs[id];
}
