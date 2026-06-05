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
 *   Chip-specific override of the armv7-a weak default.  Called by
 *   nx_smp_start() once per secondary CPU (1..CONFIG_SMP_NCPUS-1) after
 *   CPU0 has finished arm_enable_smp(0) in arm_boot().  Programs
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
 *   Secondary-CPU C entry point.  arm_cpuhead.S jumps here ("b
 *   arm_cpu_boot") once CPUn (n != 0) has set up its mode, stack, MMU and
 *   caches.  There is no common arm_boot_secondary() layer on this port, so
 *   this function is the full secondary bring-up and must NOT return:
 *
 *     - configure the FPU for this CPU,
 *     - enable SMP cache coherency (SCU),
 *     - wait until CPU0 has the memory manager ready,
 *     - bring up this CPU's GIC CPU interface (up_irqinitialize ->
 *       arm_gic_initialize -> arm_gic_init_done sets the per-CPU bit that
 *       CPU0's arm_gic_wait_done() spins on),
 *     - start the per-CPU tick (SMP), then enter the IDLE task.
 *
 *   Under BMP each core runs an independent image, so it falls through to
 *   nx_start() with the timer used as its main timer.
 *
 ****************************************************************************/

void arm_cpu_boot(int cpu)
{
  /* Initialize the FPU for this CPU */

  arm_fpuconfig();

  /* Enable SMP cache coherency (SCU) for this CPU */

  arm_enable_smp(cpu);

  /* Wait until CPU0 has finished memory manager initialization before
   * touching anything that may allocate.
   */

  while (!OSINIT_MM_READY())
    {
    }

  /* Bring up this CPU's interrupt controller.  arm_gic_initialize() ends
   * with arm_gic_init_done(), publishing this CPU's bit so CPU0's
   * arm_gic_wait_done() can proceed when it IPIs us.
   */

  up_irqinitialize();

#ifdef CONFIG_BMP
  /* BMP: this core runs its own image with the timer as its main timer. */

  nx_start();
#else
  /* SMP: secondary CPUs need their own tick source, then enter IDLE.
   * T113 uses the per-CPU ARM generic timer, not the common armv7-a
   * arm_timer.c (which this chip does not build), so call the chip hook.
   */

  t113_timer_secondary_init();

  nx_idle_trampoline();
#endif
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
