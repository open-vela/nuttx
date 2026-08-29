/****************************************************************************
 * sched/sched/sched_critmonitor.c
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
#include <nuttx/sched_note.h>

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>
#include <time.h>

#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION > 0
#  define CHECK_PREEMPTION(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION) \
           { \
             CRITMONITOR_PANIC("PID %d hold sched lock too long %ju\n", \
                               pid, (uintmax_t)elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_PREEMPTION(pid, elapsed)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION > 0
#  define CHECK_CSECTION(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION) \
           { \
             CRITMONITOR_PANIC("PID %d hold critical section too long " \
                               "%ju\n", pid, (uintmax_t)elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_CSECTION(pid, elapsed)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0
#  define CHECK_BUSYWAIT(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT) \
           { \
             CRITMONITOR_PANIC("PID %d wait for critical section or spin" \
                               "lock too long %ju\n", \
                               pid, (uintmax_t)elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_BUSYWAIT(pid, elapsed)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD > 0
#  define CHECK_THREAD(pid, elapsed) \
     do \
       { \
         if (pid > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD) \
           { \
             CRITMONITOR_PANIC("PID %d execute too long %ju\n", \
                               pid, (uintmax_t)elapsed); \
           } \
       } \
     while (0)
#else
#  define CHECK_THREAD(pid, elapsed)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
static spinlock_t g_crimonitor_lock = SP_UNLOCKED;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Maximum time with pre-emption disabled or within critical section. */

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
DEFINE_PER_CPU_BSS(clock_t, g_premp_max);
#  define g_premp_max this_cpu_var(g_premp_max)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
DEFINE_PER_CPU_BSS(clock_t, g_crit_max);
#  define g_crit_max this_cpu_var(g_crit_max)
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0
DEFINE_PER_CPU_BSS(clock_t, g_busywait_max);
#  define g_busywait_max this_cpu_var(g_busywait_max)
DEFINE_PER_CPU_BSS(clock_t, g_busywait_total);
#  define g_busywait_total this_cpu_var(g_busywait_total)
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_critmon_cpuload
 *
 * Description:
 *   Update the running time of all running threads when switching threads
 *
 * Input Parameters:
 *   tcb   - The task that we are performing the load operations on.
 *   current - The current time
 *   tick - The ticks that we process in this cpuload.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
static void nxsched_critmon_cpuload(FAR struct tcb_s *tcb, clock_t current,
                                    clock_t tick)
{
  irqstate_t flags;
  int i;
  UNUSED(i);

  /* Update the cpuload of the thread ready to be suspended */

  flags = enter_critical_section();
  nxsched_process_taskload_ticks(tcb, tick);

  /* Update the cpuload of threads running on other CPUs */

#  ifdef CONFIG_SMP
  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      FAR struct tcb_s *rtcb = current_task(i);

      if (tcb->cpu == rtcb->cpu)
        {
          continue;
        }

      nxsched_process_taskload_ticks(rtcb, tick);

      /* Update start time, avoid repeated statistics when the next call */

      rtcb->run_start = current;
    }
#  endif

  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_critmon_preemption
 *
 * Description:
 *   Called when there is any change in pre-emptible state of a thread.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Never called from an interrupt handler
 *   - Caller is the address of the function that is changing the pre-emption
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0
void nxsched_critmon_preemption(FAR struct tcb_s *tcb, bool state,
                                FAR void *caller)
{
  clock_t current = perf_gettime();
  irqstate_t flags;

  flags = spin_lock_irqsave_notrace(&g_crimonitor_lock);

  /* Are we enabling or disabling pre-emption */

  if (state)
    {
      /* Disabling.. Save the thread start time */

      tcb->premp_start  = current;
      tcb->premp_caller = caller;
    }
  else
    {
      /* Re-enabling.. Check for the max elapsed time */

      clock_t elapsed = current - tcb->premp_start;

      if (elapsed > tcb->premp_max)
        {
          tcb->premp_max        = elapsed;
          tcb->premp_max_caller = tcb->premp_caller;
          CHECK_PREEMPTION(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_premp_max)
        {
          g_premp_max = elapsed;
        }
    }

  spin_unlock_irqrestore_notrace(&g_crimonitor_lock, flags);
}
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0 */

/****************************************************************************
 * Name: nxsched_critmon_csection
 *
 * Description:
 *   Called when a thread enters or leaves a critical section.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Never called from an interrupt handler
 *   - Caller is the address of the function that is entering the critical
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0
void nxsched_critmon_csection(FAR struct tcb_s *tcb, bool state,
                              FAR void *caller)
{
  clock_t current = perf_gettime();

  /* Are we entering or leaving the critical section? */

  if (state)
    {
      /* Entering... Save the start time. */

      tcb->crit_start  = current;
      tcb->crit_caller = caller;
    }
  else if (tcb->crit_caller != NULL)
    {
      /* Leaving .. Check for the max elapsed time */

      clock_t elapsed = current - tcb->crit_start;
      FAR void *caller = tcb->crit_caller;

      /* Clear the active marker before reporting.  Reporting can enter
       * another critical section through the syslog backend.
       */

      tcb->crit_caller = NULL;

      if (elapsed > tcb->crit_max)
        {
          tcb->crit_max        = elapsed;
          tcb->crit_max_caller = caller;
          CHECK_CSECTION(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_crit_max)
        {
          g_crit_max = elapsed;
        }
    }
}
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_CSECTION >= 0 */

/****************************************************************************
 * Name: nxsched_critmon_busywait
 *
 * Description:
 *   Called when a thread try to enter critical section（get spinlock） or
 *   successfully entered cirtical section.
 *
 * Assumptions:
 *   - Called before a critical section or within a critical section.
 *   - Might be called from an interrupt handler.
 *
 ****************************************************************************/

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0
void nxsched_critmon_busywait(bool state, FAR void *caller)
{
  FAR struct tcb_s *tcb = this_task();
  clock_t current       = perf_gettime();

  /* Are we busy waiting for critical section or spinlock? */

  if (state)
    {
      /* Waiting... Save the start time. */

      tcb->busywait_start  = current;
      tcb->busywait_caller = caller;
    }
  else
    {
      /* Entered critical section... Check for the max elapsed time */

      clock_t elapsed = current - tcb->busywait_start;

      if (elapsed > tcb->busywait_max)
        {
          tcb->busywait_max        = elapsed;
          tcb->busywait_max_caller = tcb->busywait_caller;
          CHECK_BUSYWAIT(tcb->pid, elapsed);
        }

      /* Check for the global max elapsed time */

      if (elapsed > g_busywait_max)
        {
          g_busywait_max = elapsed;
        }

      /* Update thread-level and cpu-level busywait */

      tcb->busywait_total   += elapsed;
      g_busywait_total += elapsed;
    }
}
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_BUSYWAIT >= 0 */

/****************************************************************************
 * Name: nxsched_suspend_critmon
 *
 * Description:
 *   Called when a thread suspends execution, perhaps terminating a
 *   critical section or a non-preemptible state.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Might be called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_suspend_critmon(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SCHED_INSTRUMENTATION_THREADTIME
  static clock_t threshold = CLOCK_MAX;
#endif

  clock_t current = perf_gettime();
  clock_t elapsed = current - tcb->run_start;

#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
  nxsched_critmon_cpuload(tcb, current, elapsed);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_THREADTIME
  if (threshold == CLOCK_MAX)
    {
      threshold =
        CONFIG_SCHED_INSTRUMENTATION_THREAD_RUNTIME_DURATION *
        (clock_t)perf_getfreq() / USEC_PER_SEC;
    }

  if (!is_idle_task(tcb) && elapsed > threshold)
    {
      clock_t us = elapsed * USEC_PER_SEC / perf_getfreq();
      sched_note_threadtime(us);
    }
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  tcb->run_time += elapsed;
  if (elapsed > tcb->run_max)
    {
      tcb->run_max = elapsed;
      CHECK_THREAD(tcb->pid, elapsed);
    }
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0

  /* Did this task disable preemption? */

  if (tcb->lockcount > 0)
    {
      /* Possibly re-enabling.. Check for the max elapsed time */

      elapsed = current - tcb->premp_start;
      if (elapsed > tcb->premp_max)
        {
          tcb->premp_max        = elapsed;
          tcb->premp_max_caller = tcb->premp_caller;
          CHECK_PREEMPTION(tcb->pid, elapsed);
        }

      if (elapsed > g_premp_max)
        {
          g_premp_max = elapsed;
        }
    }

#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION */
}

/****************************************************************************
 * Name: nxsched_resume_critmon
 *
 * Description:
 *   Called when a thread resumes execution, perhaps re-establishing a
 *   critical section or a non-pre-emptible state.
 *
 * Assumptions:
 *   - Called within a critical section.
 *   - Might be called from an interrupt handler
 *
 ****************************************************************************/

void nxsched_resume_critmon(FAR struct tcb_s *tcb)
{
  clock_t current = perf_gettime();

  UNUSED(current);

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_THREAD >= 0
  tcb->run_start = current;
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION >= 0

  /* Did this task disable pre-emption? */

  if (tcb->lockcount > 0)
    {
      /* Yes.. Save the start time */

      tcb->premp_start = current;
    }
#endif /* CONFIG_SCHED_CRITMONITOR_MAXTIME_PREEMPTION */
}

void nxsched_update_critmon(FAR struct tcb_s *tcb)
{
  clock_t current = perf_gettime();
  clock_t elapsed = current - tcb->run_start;

  if (tcb->task_state == TSTATE_TASK_RUNNING)
    {
#ifdef CONFIG_SCHED_CPULOAD_CRITMONITOR
      clock_t tick = elapsed * CLOCKS_PER_SEC / perf_getfreq();
      nxsched_process_taskload_ticks(tcb, tick);
#endif

      tcb->run_start = current;
      tcb->run_time += elapsed;
      if (elapsed > tcb->run_max)
        {
          tcb->run_max = elapsed;
          CHECK_THREAD(tcb->pid, elapsed);
        }
    }
}
