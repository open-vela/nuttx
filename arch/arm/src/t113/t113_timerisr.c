/****************************************************************************
 * arch/arm/src/t113/t113_timerisr.c
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

#if defined(CONFIG_ARCH_TRUSTZONE_SECURE) || \
    defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)

/****************************************************************************
 * SECURE / NONSECURE mode: use public arm_timer.c
 *
 * SECURE:    arm_timer_initialize(24000000) -- writes CNTFRQ, IRQ 29 (STM)
 * NONSECURE: arm_timer_initialize(0)        -- no CNTFRQ write, IRQ 30 (PTM)
 ****************************************************************************/

#include <nuttx/timers/arch_alarm.h>
#include "arm_timer.h"
#include "hardware/t113_clk.h"

#define T113_CNTFRQ T113_TIMER_FREQUENCY

void up_timer_initialize(void)
{
#ifdef CONFIG_ARCH_TRUSTZONE_SECURE
  up_alarm_set_lowerhalf(arm_timer_initialize(T113_CNTFRQ));
#else
  up_alarm_set_lowerhalf(arm_timer_initialize(0));
#endif
}

#else /* TRUSTZONE_DISABLED - local implementation with IRQ 29 */

/****************************************************************************
 * DISABLED mode: bare-metal secure without TrustZone declaration
 *
 * T113 Cortex-A7 resets into secure SVC mode. The physical timer fires
 * on secure PPI (IRQ 29), not non-secure PPI (IRQ 30). The public
 * arm_timer.c without TRUSTZONE_SECURE uses IRQ 30 which never fires
 * on T113. We implement the timer locally with the correct IRQ.
 ****************************************************************************/

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/bits.h>

#include <arch/irq.h>
#include <arch/arm_a_r/cp15.h>

#ifdef CONFIG_SCHED_TICKLESS
#  include <nuttx/timers/arch_alarm.h>
#  include <nuttx/timers/oneshot.h>
#  include <sys/param.h>
#endif

#include "hardware/t113_clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_CNTFRQ           T113_TIMER_FREQUENCY

/* T113 runs in secure mode, physical timer fires on IRQ 29 (secure PPI) */

#define GIC_IRQ_SEC_PHY_TIMER 29

#ifdef CONFIG_SCHED_TICKLESS
#  define CNT_CTL_ENABLE_BIT  0
#  define CNT_CTL_IMASK_BIT   1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
static uint32_t g_timer_reload;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
static int t113_timerisr(int irq, void *context, void *arg)
{
  uint64_t cntpct;

  /* Disable timer output (ENABLE=0, IMASK=0) so the IRQ line cannot
   * re-assert while we reconfigure.  ISB retires the write before the
   * next CP15 write is issued - without it, the comparator-reload is
   * not guaranteed to land before ENABLE=1 and the line may immediately
   * re-fire.
   */

  CP15_SET(CNTP_CTL, 0);
  UP_ISB();

  /* Program the next trigger using the absolute compare value
   * (CVAL = CNTPCT_now + reload).  CVAL is 64-bit monotonic and is
   * immune to the TVAL write-ordering race that produced observable
   * storm behaviour (IRQ 29 re-entry at us rate) under sustained
   * MUSB RX DMA load.
   */

  cntpct = CP15_GET64(CNTPCT);
  CP15_SET64(CNTP_CVAL, cntpct + g_timer_reload);
  UP_ISB();

  CP15_SET(CNTP_CTL, 1);
  UP_ISB();

  nxsched_process_timer();
  return OK;
}

#else /* CONFIG_SCHED_TICKLESS */

/****************************************************************************
 * Tickless oneshot lowerhalf using CP15 generic timer with IRQ 29
 *
 * T113 runs bare-metal in secure mode. The physical timer generates
 * secure PPI (IRQ 29), not non-secure PPI (IRQ 30). The common
 * arm_timer.c uses IRQ 30 (GIC_IRQ_PTM) without TRUSTZONE_SECURE,
 * which never fires on T113. We implement the oneshot lowerhalf
 * locally with the correct IRQ.
 ****************************************************************************/

static int t113_timer_interrupt(int irq, void *regs, void *arg)
{
  struct oneshot_lowerhalf_s *priv = (struct oneshot_lowerhalf_s *)arg;

  CP15_SET64(CNTP_CVAL, UINT64_MAX);

  oneshot_process_callback(priv);

  return OK;
}

static clkcnt_t t113_oneshot_max_delay(struct oneshot_lowerhalf_s *lower)
{
  return UINT64_MAX;
}

static clkcnt_t t113_oneshot_current(struct oneshot_lowerhalf_s *lower)
{
  return CP15_GET64(CNTPCT);
}

static void t113_oneshot_start_absolute(struct oneshot_lowerhalf_s *lower,
                                        clkcnt_t expected)
{
  CP15_SET64(CNTP_CVAL, expected);
}

static void t113_oneshot_start(struct oneshot_lowerhalf_s *lower,
                               clkcnt_t delta)
{
  /* Program next trigger using absolute compare (CVAL = CNTPCT_now + delta)
   * rather than TVAL.  TVAL has a write-ordering race under IRQ storms
   * that can cause sub-us IRQ re-entry.  CVAL is 64-bit monotonic so
   * no saturation is needed.
   */

  uint64_t now = CP15_GET64(CNTPCT);
  CP15_SET64(CNTP_CVAL, now + delta);
  CP15_MODIFY(BIT(CNT_CTL_ENABLE_BIT),
              BIT(CNT_CTL_ENABLE_BIT), CNTP_CTL);
  UP_ISB();
  CP15_MODIFY(0, BIT(CNT_CTL_IMASK_BIT), CNTP_CTL);
  UP_ISB();
}

static void t113_oneshot_cancel(struct oneshot_lowerhalf_s *lower)
{
  CP15_SET64(CNTP_CVAL, UINT64_MAX);
}

static const struct oneshot_operations_s g_t113_oneshot_ops =
{
  .current        = t113_oneshot_current,
  .start          = t113_oneshot_start,
  .start_absolute = t113_oneshot_start_absolute,
  .cancel         = t113_oneshot_cancel,
  .max_delay      = t113_oneshot_max_delay,
};

static struct oneshot_lowerhalf_s g_t113_oneshot_lowerhalf =
{
  .ops = &g_t113_oneshot_ops
};

static void t113_timer_hw_enable(void)
{
  /* Set absolute compare to max (no pending interrupt), enable timer,
   * unmask IRQ, then enable the per-CPU IRQ line at the GIC.
   */

  CP15_SET64(CNTP_CVAL, UINT64_MAX);
  CP15_MODIFY(BIT(CNT_CTL_ENABLE_BIT),
              BIT(CNT_CTL_ENABLE_BIT), CNTP_CTL);
  UP_ISB();
  CP15_MODIFY(0, BIT(CNT_CTL_IMASK_BIT), CNTP_CTL);
  UP_ISB();

  up_enable_irq(GIC_IRQ_SEC_PHY_TIMER);
}

static struct oneshot_lowerhalf_s *t113_timer_initialize(void)
{
  struct oneshot_lowerhalf_s *lower = &g_t113_oneshot_lowerhalf;

  t113_timer_hw_enable();

  oneshot_count_init(lower, CP15_GET(CNTFRQ));

  irq_attach(GIC_IRQ_SEC_PHY_TIMER, t113_timer_interrupt, lower);

  return lower;
}

#if !defined(CONFIG_UP)
static void t113_timer_initialize_per_cpu(void)
{
  t113_timer_hw_enable();
}
#endif /* !CONFIG_UP */

#endif /* CONFIG_SCHED_TICKLESS */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_initialize(void)
{
  CP15_SET(CNTFRQ, T113_CNTFRQ);

#ifdef CONFIG_SCHED_TICKLESS
  up_alarm_set_lowerhalf(t113_timer_initialize());
#else
  uint32_t cntfrq;

  cntfrq = CP15_GET(CNTFRQ);
  /* Avoid 64-bit division (__udivmoddi4 costs ~874B).
   * Requires USEC_PER_TICK to divide 1000000 evenly.
   */

  static_assert(1000000 % CONFIG_USEC_PER_TICK == 0,
                "USEC_PER_TICK must divide 1000000 evenly");
  g_timer_reload = cntfrq / (1000000 / CONFIG_USEC_PER_TICK);

  up_disable_irq(GIC_IRQ_SEC_PHY_TIMER);
  irq_attach(GIC_IRQ_SEC_PHY_TIMER, t113_timerisr, NULL);
  CP15_SET(CNTP_TVAL, g_timer_reload);
  CP15_SET(CNTP_CTL, 1);
  up_enable_irq(GIC_IRQ_SEC_PHY_TIMER);
#endif
}

#ifndef CONFIG_UP
void t113_timer_secondary_init(void)
{
  CP15_SET(CNTFRQ, T113_CNTFRQ);

#ifdef CONFIG_SCHED_TICKLESS
  t113_timer_initialize_per_cpu();
#else
  /* Program the first comparator using CVAL absolute compare, matching
   * the fix already in t113_timerisr() for the primary CPU.  TVAL
   * re-introduces the one-tick write-ordering race under IRQ storms.
   * Reuse g_timer_reload computed by the primary path.
   */

  uint64_t now = CP15_GET64(CNTPCT);
  CP15_SET64(CNTP_CVAL, now + g_timer_reload);
  CP15_SET(CNTP_CTL, 1);
  up_enable_irq(GIC_IRQ_SEC_PHY_TIMER);
#endif
}
#endif /* !CONFIG_UP */

#endif /* TRUSTZONE_SECURE || TRUSTZONE_NONSECURE */
