/****************************************************************************
 * mm/kmm_heap/kmm_free.c
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

#include <assert.h>
#include <debug.h>

#include <stdint.h>
#include <nuttx/mm/mm.h>

/* [V764] ledger hook, see esp32s3_allocateheap.c */
void vox3_kmm_note(uintptr_t ra, size_t size, int delta);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_free
 *
 * Description:
 *   Returns a chunk of kernel memory to the list of free nodes, merging
 *   with adjacent free chunks if possible.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

/* [V764] Mirror of the allocation-side hook: without this a task whose
 * stack is freed on exit stays on the ledger forever and the dump would point
 * at the wrong owner.  kmm_free() has no size argument, so the ledger is told
 * only the amount to give back - which is all vox3_kmm_note() needs. */

/* [V813] Declared here so the guard below can reach it. */
bool v813_tcb_is_static(FAR const void *p);
void v813_tcb_static_free(FAR void *p);

void kmm_free(FAR void *mem)
{
  /* [V813] A pool block must never reach the heap allocator.
   *
   * kmm_free() is the single release point for kernel-heap memory, and a TCB is
   * released from more than one place (a failed task init, a failed exec, and
   * the task-exit path).  Guarding here covers all of them at once; guarding
   * each call site would work until a fourth is added. */
  if (v813_tcb_is_static(mem))
    {
      v813_tcb_static_free(mem);
      return;
    }

  vox3_kmm_note((uintptr_t)__builtin_return_address(0), 4096, -1);
  mm_free(KNR_HEAP, mem);
}

/****************************************************************************
 * Name: kmm_delayfree
 *
 * Description:
 *   Add mem to delaylist, mem will be freed delay a while.
 *
 * Input Parameters:
 *   mem: addr of mem to free delay.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void kmm_delayfree(FAR void *mem)
{
  mm_delayfree(KNR_HEAP, mem);
}
