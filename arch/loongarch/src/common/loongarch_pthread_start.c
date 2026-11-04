/****************************************************************************
 * arch/loongarch/src/common/loongarch_pthread_start.c
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
#include <pthread.h>
#include <nuttx/arch.h>
#include <assert.h>

#include <arch/syscall.h>

#include "loongarch_internal.h"

#include "sched/sched.h"

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__) && \
    !defined(CONFIG_DISABLE_PTHREAD)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_pthread_start(pthread_trampoline_t startup,
                      pthread_startroutine_t entrypt, pthread_addr_t arg)
{
  struct tcb_s *rtcb = this_task();
  uintptr_t sp;

  sp = (uintptr_t)rtcb->stack_base_ptr + rtcb->adj_stack_size;

  loongarch_jump_to_user((uintptr_t)startup, (uintreg_t)entrypt,
                         (uintreg_t)arg, 0, sp, rtcb->xcp.initregs);
}

#endif
