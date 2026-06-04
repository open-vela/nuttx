/****************************************************************************
 * arch/arm/src/t113/t113_ccu.c
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

/* Shared CCU register-protection layer.
 *
 * Every T113 driver that touches the Clock Control Unit (BGR / clock /
 * MBUS) goes through the t113_ccu_* helpers defined in t113_ccu.h.  The
 * helpers serialize concurrent read-modify-write traffic against the
 * shared CCU register block so that:
 *
 *   - On SMP, sibling drivers running on CPU0 and CPU1 cannot interleave
 *     RMW sequences and corrupt each other's bits.
 *
 *   - In AMP (CONFIG_T113_AMP=y) where each CPU runs a separate NuttX
 *     image with its own kernel state, the locking primitive switches to
 *     the SoC's hardware spinlock module so cross-image traffic is also
 *     serialized.
 *
 * Drivers do not see the lock; they only call the semantic helpers.
 * This mirrors the precedent set by include/nuttx/atomic.h.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#ifdef CONFIG_T113_AMP
#  include <nuttx/hwspinlock/hwspinlock.h>
#  include "t113_hwspinlock.h"
#else
#  include <nuttx/spinlock.h>
#endif

#include "arm_internal.h"
#include "t113_ccu.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_T113_AMP
/* Resolved at t113_ccu_init() time from the hwspinlock framework.  Until
 * that runs the helpers fall back to plain irq save/restore, which is
 * still correct for the boot path before SMP/AMP siblings are running.
 */

static FAR struct hwspinlock_dev_s *g_ccu_hwlock;
#else
static spinlock_t g_ccu_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline irqstate_t ccu_lock(void)
{
#ifdef CONFIG_T113_AMP
  if (g_ccu_hwlock != NULL)
    {
      return hwspin_lock_irqsave(g_ccu_hwlock);
    }

  /* hwspinlock not yet wired up - boot-time access on a single CPU. */

  return up_irq_save();
#else
  return spin_lock_irqsave(&g_ccu_lock);
#endif
}

static inline void ccu_unlock(irqstate_t flags)
{
#ifdef CONFIG_T113_AMP
  if (g_ccu_hwlock != NULL)
    {
      hwspin_unlock_restore(g_ccu_hwlock, flags);
      return;
    }

  up_irq_restore(flags);
#else
  spin_unlock_irqrestore(&g_ccu_lock, flags);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void t113_ccu_init(void)
{
#ifdef CONFIG_T113_AMP
  if (g_ccu_hwlock == NULL)
    {
      g_ccu_hwlock = t113_hwspinlock_get(T113_HWLOCK_ID_CCU);
    }
#endif
}

void t113_ccu_module_enable(uint32_t bgr_reg, uint32_t rst_bit,
                            uint32_t gate_bit)
{
  irqstate_t flags;
  uint32_t   val;

  /* Step 1: de-assert reset. */

  flags = ccu_lock();
  val = getreg32(bgr_reg);
  val |= rst_bit;
  putreg32(val, bgr_reg);
  ccu_unlock(flags);

  /* Step 2: brief delay outside the lock so we do not hold the CCU
   * critical section across an idle wait.
   */

  up_udelay(1);

  /* Step 3: enable bus clock gate. */

  flags = ccu_lock();
  val = getreg32(bgr_reg);
  val |= gate_bit;
  putreg32(val, bgr_reg);
  ccu_unlock(flags);
}

void t113_ccu_module_disable(uint32_t bgr_reg, uint32_t rst_bit,
                             uint32_t gate_bit)
{
  irqstate_t flags;
  uint32_t   val;

  flags = ccu_lock();
  val = getreg32(bgr_reg);
  val &= ~gate_bit;
  val &= ~rst_bit;
  putreg32(val, bgr_reg);
  ccu_unlock(flags);
}

void t113_ccu_clk_set(uint32_t clk_reg, uint32_t mask, uint32_t val)
{
  irqstate_t flags;
  uint32_t   reg;

  flags = ccu_lock();
  reg = getreg32(clk_reg);
  reg &= ~mask;
  reg |= (val & mask);
  putreg32(reg, clk_reg);
  ccu_unlock(flags);
}

void t113_ccu_modify(uint32_t reg, uint32_t clr, uint32_t set)
{
  irqstate_t flags;
  uint32_t   val;

  flags = ccu_lock();
  val = getreg32(reg);
  val &= ~clr;
  val |= set;
  putreg32(val, reg);
  ccu_unlock(flags);
}
