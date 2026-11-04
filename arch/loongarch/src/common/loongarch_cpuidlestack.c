/****************************************************************************
 * arch/loongarch/src/common/loongarch_cpuidlestack.c
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
#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/sched.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STACK_ISALIGNED(a)   ((uintptr_t)(a) & ~SMP_STACK_MASK)

uintptr_t g_idle_topstack = (uintptr_t)_ebss +
                                       SMP_STACK_SIZE * CONFIG_SMP_NCPUS;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SMP
int up_cpu_idlestack(int cpu, struct tcb_s *tcb, size_t stack_size)
{
  uintptr_t stack_alloc;

  DEBUGASSERT(cpu > 0 && cpu < CONFIG_SMP_NCPUS && tcb != NULL &&
              stack_size <= SMP_IDLE_STACK_SIZE);

  stack_alloc = (uintptr_t)g_cpux_idlestack(cpu);
  DEBUGASSERT(stack_alloc != 0 && STACK_ISALIGNED(stack_alloc));

  tcb->adj_stack_size  = SMP_IDLE_STACK_SIZE;
  tcb->stack_alloc_ptr = (void *)stack_alloc;
  tcb->stack_base_ptr  = tcb->stack_alloc_ptr;

  return OK;
}
#endif
