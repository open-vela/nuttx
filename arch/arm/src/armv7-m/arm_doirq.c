/****************************************************************************
 * arch/arm/src/armv7-m/arm_doirq.c
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
#include <arch/board/board.h>
#include <sched/sched.h>

#include "arm_internal.h"
#include "exc_return.h"
#include "nvic.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void exception_direct(void)
{
  int irq;

  up_addrenv_enter_kernel();
  irq = getipsr();
  arm_ack_irq(irq);
  irq_dispatch(irq, NULL);

  if (g_running_task != this_task())
    {
      up_trigger_irq(NVIC_IRQ_PENDSV, 0);
    }

  up_addrenv_leave_kernel();
}

uint32_t *arm_doirq(int irq, uint32_t *regs)
{
  struct tcb_s **running_task;
  struct tcb_s *tcb;

  up_addrenv_enter_kernel();
  running_task = &g_running_task;
  tcb = *running_task;

  /* This judgment proves that (*running_task)->xcp.regs
   * is invalid, and we can safely overwrite it.
   */

  if (tcb != NULL)
    {
      tcb->xcp.regs = regs;
    }

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  /* Acknowledge the interrupt */

  arm_ack_irq(irq);

  if (irq == NVIC_IRQ_PENDSV)
    {
#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
      /* Dispatch the PendSV interrupt */

      irq_dispatch(irq, regs);
#endif
#ifndef CONFIG_DISABLE_SIGNALS
      if (tcb && tcb->sigdeliver && !tcb->xcp.saved_regs)
        {
          /* Pendsv able to access running tcb with no critical section */

          up_schedule_sigaction(tcb);
        }
#endif

      up_irq_save();
    }
  else
    {
      irq_dispatch(irq, regs);
    }

  tcb = this_task();

  if (tcb != *running_task)
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

  regs = tcb->xcp.regs;
#endif

  board_autoled_off(LED_INIRQ);

  /* (*running_task)->xcp.regs is about to become invalid
   * and will be marked as NULL to avoid misusage.
   */

  (*running_task)->xcp.regs = NULL;
  up_addrenv_leave_kernel();
  return regs;
}
