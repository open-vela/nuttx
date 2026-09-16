/****************************************************************************
 * arch/loongarch/src/common/loongarch_initialstate.c
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
#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/tls.h>
#include <nuttx/kmalloc.h>
#include <arch/irq.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
  struct xcptcontext *xcp = &tcb->xcp;
  uintptr_t regval;
  uintptr_t topstack;

  memset(xcp, 0, sizeof(struct xcptcontext));

  if (tcb->pid == IDLE_PROCESS_ID)
    {
      tcb->stack_alloc_ptr = (void *)g_cpux_idlestack(this_cpu());
      tcb->stack_base_ptr  = tcb->stack_alloc_ptr;
      tcb->adj_stack_size  = SMP_IDLE_STACK_SIZE;

#ifdef CONFIG_STACK_COLORATION
      loongarch_stack_color(tcb->stack_alloc_ptr, 0);
#endif

      loongarch_set_idleintctx();
      return;
    }

  topstack = (uintptr_t)tcb->stack_base_ptr + tcb->adj_stack_size;

  xcp->regs = (uintreg_t *)(topstack - XCPTCONTEXT_SIZE);
  memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

  xcp->regs[REG_SP]      = topstack;

  xcp->regs[REG_EPC]     = (uintptr_t)tcb->start;

#ifdef CONFIG_SCHED_THREAD_LOCAL
  xcp->regs[REG_R13]     = (uintptr_t)tcb->stack_alloc_ptr +
                                     sizeof(struct tls_info_s);
#endif

  regval = loongarch_get_newintctx();
  xcp->regs[REG_INT_CTX] = regval;

#ifndef CONFIG_BUILD_FLAT
  tcb->xcp.initregs = tcb->xcp.regs;
#endif
}
