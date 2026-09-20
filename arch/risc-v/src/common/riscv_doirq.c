/****************************************************************************
 * arch/risc-v/src/common/riscv_doirq.c
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

#include <nuttx/irq.h>
#include <nuttx/addrenv.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/barriers.h>
#include <arch/board/board.h>
#include <sched/sched.h>

#include "riscv_internal.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT

/* This config allows higher priority IRQ to interrupt lower priority ones.
 * It is mainly for single core MCU devices in FLAT or PROTECTED modes.
 */

/* Keep exceptions as non-preemptive and mask local interrupts */

#  define IRQ_DISPATCH(irq, regs)                           \
            {                                               \
              if (irq > RISCV_MAX_EXCEPTION)                \
                {                                           \
                  if (irq < RISCV_IRQ_EXT)                  \
                    {                                       \
                      up_disable_irq(irq);                  \
                    }                                       \
                  up_irq_enable();                          \
                }                                           \
              irq_dispatch(irq, regs);                      \
              if (irq > RISCV_MAX_EXCEPTION)                \
                {                                           \
                  up_irq_save();                            \
                  if (irq < RISCV_IRQ_EXT)                  \
                    {                                       \
                      up_enable_irq(irq);                   \
                    }                                       \
                }                                           \
            }
#else
#  define IRQ_DISPATCH(irq, regs)    irq_dispatch(irq, regs)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uintreg_t *riscv_doirq_top(int irq, uintreg_t *regs)
{
  struct tcb_s **running_task = &g_running_task;
  bool restore_context = false;
  struct tcb_s *tcb = this_task();

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  /* NOTE: In case of ecall, we need to adjust mepc in the context */

  if (irq >= RISCV_IRQ_ECALLU && irq <= RISCV_IRQ_ECALLM)
    {
      regs[REG_EPC] += 4;
      if (regs[REG_A0] != SYS_restore_context)
        {
          (*running_task)->xcp.regs = regs;
        }
      else
        {
          restore_context = true;
        }
    }
  else
    {
      (*running_task)->xcp.regs = regs;
    }

  /* Current regs non-zero indicates that we are processing an interrupt;
   * current_regs is also used to manage interrupt level context switches.
   *
   * An exception may occur while processing an interrupt. In this case,
   * current_regs will be non-NULL.
   */

  up_set_interrupt_context(true);
  UP_DMB();

  nxsched_suspend_scheduler(tcb);

  /* Deliver the IRQ.
   *
   * ESP-HAL peripheral ISRs are compiled against ESP-IDF and do not preserve
   * callee-saved registers (they stash the IRQ number in s1 and clobber
   * s0-s5). Save s0-s5 across the dispatch so the g_running_tasks base,
   * restore_context, and the g_readytorun base survive and the comparisons
   * below do not fault. */

  {
    uint32_t saved[6];

    __asm__ volatile(
      "sw s0,  0(%0)\n"
      "sw s1,  4(%0)\n"
      "sw s2,  8(%0)\n"
      "sw s3, 12(%0)\n"
      "sw s4, 16(%0)\n"
      "sw s5, 20(%0)\n"
      : : "r"(saved) : "memory"
    );

    IRQ_DISPATCH(irq, regs);

    __asm__ volatile(
      "lw s0,  0(%0)\n"
      "lw s1,  4(%0)\n"
      "lw s2,  8(%0)\n"
      "lw s3, 12(%0)\n"
      "lw s4, 16(%0)\n"
      "lw s5, 20(%0)\n"
      : : "r"(saved) : "memory"
    );
  }

  tcb = this_task();

  /* Check for a context switch. */

  if (*running_task != tcb || restore_context)
    {
#ifdef CONFIG_ARCH_ADDRENV
      /* Make sure that the address environment for the previously
       * running task is closed down gracefully (data caches dump,
       * MMU flushed) and set up the address environment for the new
       * thread at the head of the ready-to-run list.
       */

      addrenv_switch(tcb);
      tcb = this_task();
#endif

      /* Record the new "running" task when context switch occurred.
       * g_running_tasks[] is only used by assertion logic for reporting
       * crashes.
       */

      *running_task = tcb;
    }

  nxsched_resume_scheduler(tcb);

  /* Set irq flag */

  up_set_interrupt_context(false);

#endif
  board_autoled_off(LED_INIRQ);

  regs = tcb->xcp.regs;

  /* (*running_task)->xcp.regs is about to become invalid
   * and will be marked as NULL to avoid misusage.
   */

  (*running_task)->xcp.regs = NULL;
  return regs;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uintreg_t *riscv_doirq(int irq, uintreg_t *regs)
{
  if (up_interrupt_context())
    {
      IRQ_DISPATCH(irq, regs);
      return regs;
    }

  return riscv_doirq_top(irq, regs);
}
