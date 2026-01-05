/****************************************************************************
 * sched/sched/sched_lock.c
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_LOCK_HISTORY
static bool g_no_lock_check = false;

static inline void record_sched_lock_stack(void)
{
  FAR struct tcb_s *rtcb = this_task();

  if (rtcb && rtcb->lock_hist)
    {

      FAR struct lock_stack_entry_s *entry =
        &rtcb->lock_hist[rtcb->lock_hist_idx];

#if CONFIG_SCHED_LOCK_HISTORY_DEPTH > 0
      entry->depth = sched_backtrace(rtcb->pid, entry->pc,
                                     CONFIG_SCHED_LOCK_HISTORY_DEPTH, 0);
#else
      entry->depth = 0;
#endif

      rtcb->lock_hist_idx++;
      if (rtcb->lock_hist_idx >= rtcb->lock_hist_max)
        {
          rtcb->lock_hist_idx = 0;
        }
    }
}

static void dump_sched_lock_history(FAR struct tcb_s *tcb)
{
  if (!tcb)
    {
      _alert("sched_lock: lockcount overflow but rtcb is NULL\n");
      return;
    }

  _alert("sched_lock: lockcount overflow pid=%d lockcount=%d\n",
         tcb->pid, tcb->lockcount);

  if (!tcb->lock_hist || tcb->lock_hist_max == 0)
    {
      _alert("sched_lock: lock history is not initialized\n");
      return;
    }

  _alert("sched_lock: lock_hist_idx=%u lock_hist_max=%u\n",
         (unsigned)tcb->lock_hist_idx, (unsigned)tcb->lock_hist_max);

  for (uint16_t n = 0; n < tcb->lock_hist_max; n++)
    {
      uint16_t pos = (uint16_t)((tcb->lock_hist_idx + tcb->lock_hist_max -
                                 (uint16_t)(n + 1)) %
                                tcb->lock_hist_max);
      FAR struct lock_stack_entry_s *entry = &tcb->lock_hist[pos];

      if (entry->depth <= 0)
        {
          continue;
        }

      _alert("  hist[%u] depth=%d\n", (unsigned)pos, entry->depth);
      for (int i = 0; i < entry->depth && i < CONFIG_SCHED_LOCK_HISTORY_DEPTH; i++)
        {
#if defined(CONFIG_ALLSYMS) && defined(CONFIG_LIBC_PRINT_EXTENSION)
          _alert("    %pS\n", entry->pc[i]);
#else
          _alert("    %p\n", entry->pc[i]);
#endif
        }
    }
}

void tcb_init_lock_history(FAR struct tcb_s *tcb)
{
  int count = CONFIG_SCHED_LOCK_HISTORY_COUNT;

  if (count <= 0)
    {
      count = MAX_LOCK_COUNT;
    }

  tcb->lock_hist_max = (uint16_t)count;
  tcb->lock_hist_idx = 0;

  if (count > 0)
    {
      tcb->lock_hist =
        kmm_zalloc(sizeof(struct lock_stack_entry_s) * (size_t)count);
    }
}

void tcb_uninit_lock_history(FAR struct tcb_s *tcb)
{
  if (tcb->lock_hist)
    {
      kmm_free(tcb->lock_hist);
      tcb->lock_hist = NULL;
    }
}
#endif

/****************************************************************************
 * Name:  sched_lock
 *
 * Description:
 *   This function disables context switching by disabling addition of
 *   new tasks to the g_readytorun task list.  The task that calls this
 *   function will be the only task that is allowed to run until it
 *   either calls  sched_unlock() (the appropriate number of times) or
 *   until it blocks itself.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   OK on success; ERROR on failure
 *
 ****************************************************************************/

#ifdef CONFIG_SMP

int sched_lock(void)
{
  FAR struct tcb_s *rtcb;

  /* If the CPU supports suppression of interprocessor interrupts, then
   * simple disabling interrupts will provide sufficient protection for
   * the following operation.
   */

  rtcb = this_task();

  /* Check for some special cases:  (1) rtcb may be NULL only during early
   * boot-up phases, and (2) sched_lock() should have no effect if called
   * from the interrupt level.
   */

  if (rtcb != NULL && !up_interrupt_context())
    {
      irqstate_t flags;

      /* Catch attempts to increment the lockcount beyond the range of the
       * integer type.
       */

#ifdef CONFIG_SCHED_LOCK_HISTORY
      if (rtcb->lockcount >= 32)
        {
          g_no_lock_check = true;
          dump_sched_lock_history(rtcb);
          assert(false); /* IGNORE */
        }
      if(!g_no_lock_check)
        {
          record_sched_lock_stack();
        }
#else
      DEBUGASSERT(rtcb->lockcount < MAX_LOCK_COUNT);
#endif

      flags = enter_critical_section();

      /* A counter is used to support locking.  This allows nested lock
       * operations on this thread
       */

      rtcb->lockcount++;

      /* Check if we just acquired the lock */

      if (rtcb->lockcount == 1)
        {
          /* Note that we have pre-emption locked */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
          nxsched_critmon_preemption(rtcb, true, return_address(0));
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          sched_note_premption(rtcb, true);
#endif
        }

      /* Move any tasks in the ready-to-run list to the pending task list
       * where they will not be available to run until the scheduler is
       * unlocked and nxsched_merge_pending() is called.
       */

      nxsched_merge_prioritized(list_readytorun(),
                                list_pendingtasks(),
                                TSTATE_TASK_PENDING);

      leave_critical_section(flags);
    }

  return OK;
}

#else /* CONFIG_SMP */

int sched_lock(void)
{
  FAR struct tcb_s *rtcb = this_task();

  /* Check for some special cases:  (1) rtcb may be NULL only during early
   * boot-up phases, and (2) sched_lock() should have no effect if called
   * from the interrupt level.
   */

  if (rtcb != NULL && !up_interrupt_context())
    {
      /* Catch attempts to increment the lockcount beyond the range of the
       * integer type.
       */

#ifdef CONFIG_SCHED_LOCK_HISTORY
      if (rtcb->lockcount >= SCHED_LOCK_HISTORY_THRESHOLD && !g_no_lock_check)
        {
          g_no_lock_check = true;
          dump_sched_lock_history(rtcb);
          assert(false); /* IGNORE */
        }
      if(!g_no_lock_check)
        {
          record_sched_lock_stack();
        }
#else
      DEBUGASSERT(rtcb->lockcount < MAX_LOCK_COUNT);
#endif

      /* A counter is used to support locking.  This allows nested lock
       * operations on this thread (on any CPU)
       */

      rtcb->lockcount++;

      /* Check if we just acquired the lock */

      if (rtcb->lockcount == 1)
        {
          /* Note that we have pre-emption locked */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
          nxsched_critmon_preemption(rtcb, true, return_address(0));
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION_PREEMPTION
          sched_note_premption(rtcb, true);
#endif
        }
    }

  return OK;
}

#endif /* CONFIG_SMP */
