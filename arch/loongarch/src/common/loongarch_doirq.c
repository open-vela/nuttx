/****************************************************************************
 * arch/loongarch/src/common/loongarch_doirq.c
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
#include <syscall.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/barriers.h>
#include <arch/board/board.h>
#include <sched/sched.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_DISPATCH(irq, regs)    irq_dispatch(irq, regs)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uintreg_t *loongarch_doirq_top(int irq, uintreg_t *regs)
{
  struct tcb_s **running_task = &g_running_tasks[this_cpu()];
  bool restore_context = false;
  struct tcb_s *tcb = this_task();

  board_autoled_on(LED_INIRQ);
#ifdef CONFIG_SUPPRESS_INTERRUPTS
  PANIC();
#else

  if (irq == LOONGARCH_IRQ_SYS)
    {
      if (regs[REG_R4] != SYS_restore_context)
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

  up_set_interrupt_context(true);
  UP_DMB();

  IRQ_DISPATCH(irq, regs);
  tcb = this_task();

  if (*running_task != tcb || restore_context)
    {
      *running_task = tcb;
    }

  up_set_interrupt_context(false);

#endif
  board_autoled_off(LED_INIRQ);

  regs = tcb->xcp.regs;
  (*running_task)->xcp.regs = NULL;
  return regs;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uintreg_t *loongarch_doirq(int irq, uintreg_t *regs)
{
  if (up_interrupt_context())
    {
      IRQ_DISPATCH(irq, regs);
      return regs;
    }

  return loongarch_doirq_top(irq, regs);
}

uintreg_t *loongarch_dispatch_irq(int irq, uintreg_t *regs)
{
  return loongarch_doirq(irq, regs);
}
