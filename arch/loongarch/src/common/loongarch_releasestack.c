/****************************************************************************
 * arch/loongarch/src/common/loongarch_releasestack.c
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

#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_release_stack(struct tcb_s *dtcb, int ttype)
{
  if (dtcb->stack_alloc_ptr &&
      (atomic_read(&dtcb->flags) & TCB_FLAG_FREE_STACK))
    {
      group_delayfree(dtcb->group, dtcb->stack_alloc_ptr);
    }

  atomic_and(&dtcb->flags, ~TCB_FLAG_FREE_STACK);
  dtcb->stack_alloc_ptr = NULL;
  dtcb->stack_base_ptr = NULL;
  dtcb->adj_stack_size = 0;
}
