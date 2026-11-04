/****************************************************************************
 * arch/loongarch/src/common/loongarch_swint.c
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <syscall.h>

#include <arch/irq.h>
#include <nuttx/addrenv.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#ifdef CONFIG_LIB_SYSCALL
#  include <syscall.h>
#endif

#include "sched/sched.h"
#include "signal/signal.h"
#include "loongarch_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef uintptr_t (*syscall_t)(unsigned int, ...);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_LIB_SYSCALL

uintptr_t dispatch_syscall(unsigned int nbr, uintptr_t parm1,
                           uintptr_t parm2, uintptr_t parm3,
                           uintptr_t parm4, uintptr_t parm5,
                           uintptr_t parm6, void *context)
{
  struct tcb_s *rtcb         = this_task();

  atomic_or(&rtcb->flags, TCB_FLAG_SYSCALL);

  register long a0 asm("$a0") = (long)(nbr);
  register long a1 asm("$a1") = (long)(parm1);
  register long a2 asm("$a2") = (long)(parm2);
  register long a3 asm("$a3") = (long)(parm3);
  register long a4 asm("$a4") = (long)(parm4);
  register long a5 asm("$a5") = (long)(parm5);
  register long a6 asm("$a6") = (long)(parm6);
  syscall_t do_syscall;
  uintptr_t ret;

  if (a0 > SYS_maxsyscall)
    {
      return -ENOSYS;
    }

  rtcb->xcp.sregs = context;

  a0 -= CONFIG_SYS_RESERVED;

  do_syscall = (syscall_t)g_stublookup[a0];

  ret = do_syscall(a0, a1, a2, a3, a4, a5, a6);

  atomic_and(&rtcb->flags, ~TCB_FLAG_SYSCALL);

  nxsig_unmask_pendingsignal();

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int loongarch_swint(int irq, void *context, void *arg)
{
  uintreg_t *regs = (uintreg_t *)context;
  struct tcb_s *tcb = this_task();
  int cpu = this_cpu();

  regs[REG_EPC] += 4;

#ifdef CONFIG_DEBUG_SYSCALL_INFO
  svcinfo("Entry: regs: %p cmd: %d\n", regs, regs[REG_R4]);
  up_dump_register(regs);
#endif

  switch (regs[REG_R4])
    {
      case SYS_restore_context:
        {
          loongarch_restorecontext(tcb);
          break_critical_section();
        }
        break;

      case SYS_switch_context:
        {
          loongarch_savecontext(g_running_tasks[cpu]);
          loongarch_restorecontext(tcb);
          break_critical_section();
        }
        break;

#ifndef CONFIG_BUILD_FLAT
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb   = this_task();

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn  = regs[REG_EPC];

#if defined(CONFIG_BUILD_PROTECTED)
          regs[REG_EPC]        = (uintptr_t)USERSPACE->signal_handler;
#else
          regs[REG_EPC]        = (uintptr_t)ARCH_DATA_RESERVE->ar_sigtramp;
#endif
          regs[REG_INT_CTX]   &= ~(PLV_MASK << CSR_CRMD_PLV_SHIFT);

          regs[REG_R4]         = regs[REG_R5];
          regs[REG_R5]         = regs[REG_R6];
          regs[REG_R6]         = regs[REG_R7];
          regs[REG_R7]         = regs[REG_R8];
        }
        break;
#endif

#ifndef CONFIG_BUILD_FLAT
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb   = this_task();

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);
          regs[REG_EPC]        = rtcb->xcp.sigreturn;
          regs[REG_INT_CTX]   |= (PLV_KERN << CSR_CRMD_PLV_SHIFT);

          rtcb->xcp.sigreturn  = 0;
        }
        break;
#endif

      case SYS_assert_handler:
        {
          _assert((const char *)regs[REG_R5], (int)regs[REG_R6],
                  (const char *)regs[REG_R7], (void *)running_regs(), false);
        }
        break;

      default:
        DEBUGPANIC();
        break;
    }

  return OK;
}
