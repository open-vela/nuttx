/****************************************************************************
 * arch/arm64/src/common/arm64_cpustart.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>
#include <sched/sched.h>
#include <nuttx/cache.h>
#include <nuttx/spinlock.h>
#include <nuttx/init.h>

#include "init/init.h"
#include "arm64_arch.h"
#include "arm64_internal.h"
#include "arm64_gic.h"
#include "arm64_arch_timer.h"
#include "arm64_smp.h"
#include "arm64_cpu_psci.h"

#ifdef CONFIG_ARCH_HAVE_MPU
#include "arm64_mpu.h"
#else
#include "arm64_mmu.h"
#endif

/****************************************************************************
 * External data
 ****************************************************************************/

#ifdef CONFIG_ARM64_PSCI

/* AArch64 entry point of the kernel image.  __start is the A32 boot stub in
 * arm64_head.S, which is only valid for the primary CPU when it is entered
 * from a 32-bit boot loader.  A secondary CPU released by PSCI is already
 * running in AArch64, so it must enter at real_start instead.
 */

extern char real_start[];
#endif

/****************************************************************************
 * Public data
 ****************************************************************************/

uint64_t *const g_cpu_int_stacktop[CONFIG_NCPUS] =
{
  (uint64_t *)(g_interrupt_stack[0] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 1
  (uint64_t *)(g_interrupt_stack[1] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 2
  (uint64_t *)(g_interrupt_stack[2] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 3
  (uint64_t *)(g_interrupt_stack[3] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 4
  (uint64_t *)(g_interrupt_stack[4] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 5
  (uint64_t *)(g_interrupt_stack[5] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 6
  (uint64_t *)(g_interrupt_stack[6] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 7
  (uint64_t *)(g_interrupt_stack[7] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 8
#  error This logic needs to extended for CONFIG_NCPUS > 8
#endif /* CONFIG_NCPUS > 8 */
#endif /* CONFIG_NCPUS > 7 */
#endif /* CONFIG_NCPUS > 6 */
#endif /* CONFIG_NCPUS > 5 */
#endif /* CONFIG_NCPUS > 4 */
#endif /* CONFIG_NCPUS > 3 */
#endif /* CONFIG_NCPUS > 2 */
#endif /* CONFIG_NCPUS > 1 */
};

#ifdef CONFIG_ARM64_DECODEFIQ
uint64_t *const g_cpu_int_fiq_stacktop[CONFIG_NCPUS] =
{
  (uint64_t *)(g_interrupt_fiq_stack[0] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 1
  (uint64_t *)(g_interrupt_fiq_stack[1] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 2
  (uint64_t *)(g_interrupt_fiq_stack[2] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 3
  (uint64_t *)(g_interrupt_fiq_stack[3] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 4
  (uint64_t *)(g_interrupt_fiq_stack[4] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 5
  (uint64_t *)(g_interrupt_fiq_stack[5] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 6
  (uint64_t *)(g_interrupt_fiq_stack[6] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 7
  (uint64_t *)(g_interrupt_fiq_stack[7] + INTSTACK_SIZE),
#if CONFIG_NCPUS > 8
#  error This logic needs to extended for CONFIG_NCPUS > 8
#endif /* CONFIG_NCPUS > 8 */
#endif /* CONFIG_NCPUS > 7 */
#endif /* CONFIG_NCPUS > 6 */
#endif /* CONFIG_NCPUS > 5 */
#endif /* CONFIG_NCPUS > 4 */
#endif /* CONFIG_NCPUS > 3 */
#endif /* CONFIG_NCPUS > 2 */
#endif /* CONFIG_NCPUS > 1 */
};
#endif

/****************************************************************************
 * Private data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SMP
static void arm64_smp_init_top(void)
{
  struct tcb_s *tcb = current_task(this_cpu());

#ifndef CONFIG_SUPPRESS_INTERRUPTS
  arm64_gic_secondary_init();
  arm64_color_intstack();

  /* And finally, enable interrupts */

  up_irq_enable();
#endif

  arm64_timer_secondary_init();

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Notify that this CPU has started */

  sched_note_cpu_started(tcb);
#endif

  UNUSED(tcb);

  nx_idle_trampoline();
}
#endif

static void arm64_start_cpu(int cpu_num)
{
#ifdef CONFIG_ARM64_PSCI
  uint64_t cpu_mpid = arm64_get_mpid(cpu_num);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu_num);
#endif

#ifdef CONFIG_ARM64_PSCI
  if (psci_cpu_on(cpu_mpid, (uint64_t)real_start))
    {
      serr("Failed to boot secondary CPU core %d (MPID:%#lx)\n", cpu_num,
           cpu_mpid);
      return;
    }
#else
  UP_SEV();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   In an SMP configuration, only one CPU is initially active (CPU 0).
 *   System initialization occurs on that single thread. At the completion of
 *   the initialization of the OS, just before beginning normal multitasking,
 *   the additional CPUs would be started by calling this function.
 *
 *   Each CPU is provided the entry point to its IDLE task when started.  A
 *   TCB for each CPU's IDLE task has been initialized and placed in the
 *   CPU's g_assignedtasks[cpu] list.  No stack has been allocated or
 *   initialized.
 *
 *   The OS initialization logic calls this function repeatedly until each
 *   CPU has been started, 1 through (CONFIG_SMP_NCPUS-1).
 *
 * Input Parameters:
 *   cpu - The index of the CPU being started.  This will be a numeric
 *         value in the range of one to (CONFIG_SMP_NCPUS-1).
 *         (CPU 0 is already active)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_NCPUS && cpu != this_cpu());

#ifdef CONFIG_SCHED_INSTRUMENTATION

  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  arm64_start_cpu(cpu);

  return 0;
}

/* the C entry of secondary cores */

void arm64_boot_secondary_c_routine(void)
{
#ifdef CONFIG_PERCPU_SECTION
  write_sysreg(this_cpu() * PERCPU_OFFSET, tpidr_el1);
#  ifdef CONFIG_BUILD_PROTECTED
  write_sysreg(this_cpu() * USERSPACE->us_offset_percpu, tpidrro_el0);
#  endif
#endif

#ifdef CONFIG_ARCH_HAVE_MPU
  arm64_mpu_init(false);
#endif

#ifdef CONFIG_ARCH_HAVE_MMU
  arm64_mmu_init(false);
#endif

#undef g_nx_initstate
  while (OSINIT_OS_INITIALIZING());

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)up_perf_getfreq());
#endif

#if defined(CONFIG_BMP)
  nx_start();
#elif defined(CONFIG_SMP)

  arm64_smp_init_top();
#endif
}
