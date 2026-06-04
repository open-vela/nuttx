/****************************************************************************
 * arch/arm/src/t113/t113_cpuboot.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/sched.h>
#include <nuttx/sched_note.h>

#include <arch/irq.h>
#include <arch/barriers.h>
#include <arch/arm_a_r/cp15.h>

#include "arm_internal.h"
#include "scu.h"
#include "init/init.h"
#include "smp.h"
#include "gic.h"
#include "sched/sched.h"
#include "hardware/t113_cpucfg.h"
#include "hardware/t113_clk.h"
#include "t113_boot.h"

/* AMP slave never resets/releases another CPU.  Master is the sole owner
 * of the C0 CPUCFG block (T113_C0_RST_CTRL / T113_C0_CTRL_REG0 /
 * T113_CPU_SOFT_ENT(n)) - slave touching those registers can only fence
 * the master image into reset.
 *
 * The body below splits in two: the SMP-specific hooks (up_cpu_start /
 * arm_cpu_boot) only exist under !CONFIG_UP, while t113_release_cpu1 is
 * needed by the UP-mode rptun cpu1-master too (CONFIG_NCPUS=1 in AMP).
 */

#ifndef CONFIG_T113_RPTUN_SLAVE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_CNTFRQ  T113_TIMER_FREQUENCY
#define GIC_IRQ_SEC_PHY_TIMER 29

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint8_t _vector_start[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void t113_set_cpu_boot_entry(int cpu, uint32_t entry)
{
  putreg32(entry, T113_CPU_SOFT_ENT(cpu));
  up_udelay(100);
  arm_dsb(15);
  arm_isb();
}

static void t113_enable_cpu(int cpu)
{
  uint32_t val;

  /* Assert core reset */

  val = getreg32(T113_C0_RST_CTRL);
  val &= ~(1 << cpu);
  putreg32(val, T113_C0_RST_CTRL);
  up_udelay(100);

  /* L1RSTDISABLE hold low */

  val = getreg32(T113_C0_CTRL_REG0);
  val &= ~(1 << cpu);
  putreg32(val, T113_C0_CTRL_REG0);
  up_udelay(200);

  /* Deassert core reset */

  val = getreg32(T113_C0_RST_CTRL);
  val |= (1 << cpu);
  putreg32(val, T113_C0_RST_CTRL);
  up_udelay(100);

  arm_dsb(15);
  arm_isb();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef CONFIG_UP
/****************************************************************************
 * Name: t113_cpu_disable
 ****************************************************************************/

void t113_cpu_disable(int cpu)
{
  uint32_t val;

  val = getreg32(T113_C0_RST_CTRL);
  val &= ~(1 << cpu);
  putreg32(val, T113_C0_RST_CTRL);
}

/****************************************************************************
 * Name: up_cpu_start
 *
 * Description:
 *   Chip-specific override of the arm_a_r weak default.  Called by
 *   nx_smp_start() once per secondary CPU (1..CONFIG_SMP_NCPUS-1) after
 *   CPU0 has finished arm_enable_smp(0) in arm_boot_primary().  Programs
 *   the CPU soft-entry register with the secondary entry point and pulses
 *   the core reset / L1RSTDISABLE sequence to release the target CPU.
 *
 ****************************************************************************/

int up_cpu_start(int cpu)
{
  sinfo("Starting CPU%d\n", cpu);

  DEBUGASSERT(cpu >= 0 && cpu < CONFIG_NCPUS && cpu != up_cpu_index());

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Notify of the start event */

  sched_note_cpu_start(this_task(), cpu);
#endif

  t113_set_cpu_boot_entry(cpu, (uint32_t)__cpu1_start);
  t113_enable_cpu(cpu);

  return OK;
}

/****************************************************************************
 * Name: arm_cpu_boot
 *
 * Description:
 *   Chip-specific secondary-CPU hook invoked by the common
 *   arm_boot_secondary() between arm_enable_smp() and the MM-ready wait.
 *   Runs with IRQs disabled and before up_irqinitialize(), so it must
 *   not touch the GIC distributor or perform work that needs per-CPU
 *   IRQ state.
 *
 *   T113 has no such work: secondary timer/IRQ bring-up happens later
 *   inside nx_start() via up_timer_initialize(), which is BMP-aware
 *   (g_oneshot_lower and g_irqvector are per-CPU under BMP).
 *
 ****************************************************************************/

void arm_cpu_boot(int cpu)
{
  UNUSED(cpu);
}
#endif /* !CONFIG_UP */

/****************************************************************************
 * Name: t113_release_cpu1
 *
 * Description:
 *   Release CPU1 to begin executing at a runtime-supplied entry address.
 *   Used by the core0-NuttX rptun cpu1-master: the rptun loader has already
 *   copied the ELF segments into DRAM and resolved the entry point (ELF
 *   e_entry) into rproc->bootaddr; this writes that entry into CPU1's
 *   soft-entry register and pulses the reset / L1RSTDISABLE sequence.
 *   The entry is the true e_entry, which is NOT equal to RAM_START (the
 *   image base holds the exception vector table; __start sits above it).
 *
 * Input Parameters:
 *   entry - physical entry address (ELF e_entry) for CPU1.
 *
 ****************************************************************************/

void t113_release_cpu1(uintptr_t entry)
{
  syslog(LOG_INFO, "cpu1-master: releasing CPU1 to 0x%08lx\n",
         (unsigned long)entry);

  t113_set_cpu_boot_entry(1, (uint32_t)entry);
  t113_enable_cpu(1);
}

#endif /* !CONFIG_T113_RPTUN_SLAVE */
