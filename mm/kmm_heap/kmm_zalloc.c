/****************************************************************************
 * mm/kmm_heap/kmm_zalloc.c
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

#include <nuttx/mm/mm.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

/* [V764] ledger hook, see esp32s3_allocateheap.c */
void vox3_kmm_note(uintptr_t ra, size_t size, int delta);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_zalloc
 *
 * Description:
 *   Allocate and zero memory from the kernel heap.
 *
 * Input Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Returned Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ****************************************************************************/

FAR void *kmm_zalloc(size_t size)
{
  FAR void *p;

  /* [V764] attribute big internal-heap blocks to their caller
   * before handing them on; see vox3_kmm_note() for why only
   * allocations >= 4 KiB are recorded. */
  p = mm_zalloc(KNR_HEAP, size);

  /* [V794] failure only.  The [V764] ledger records just a caller address
   * for allocations >= 4 KiB, so a FAILED request of any size left no trace
   * at all -- which is why a boot could report 120 KB free and still fail to
   * get a TCB with no explanation.  Print the size next to the live ledger. */
  if (p == NULL)
    {
      struct mallinfo mi = kmm_mallinfo();

      /* [V797] record the caller too.  The size alone never identified the
       * requester: a 1700-byte failure appeared 3656 times in one capture and
       * nothing in the log said who was asking.  The return address is the
       * return INTO the caller of kmm_zalloc, which is what addr2line can
       * resolve.  (The V764 ledger stores the same value, but only for
       * allocations >= 4 KiB and only on success, so failures never reached
       * it.) */
      syslog(LOG_ERR,
             "[V797] kmm_zalloc FAILED requested=%u arena=%u "
             "fordblks=%u mxordblk=%u uordblks=%u ra=0x%08x\n",
             (unsigned)size, (unsigned)mi.arena, (unsigned)mi.fordblks,
             (unsigned)mi.mxordblk, (unsigned)mi.uordblks,
             (unsigned)(uintptr_t)__builtin_return_address(0));
    }
  else
    {
      vox3_kmm_note((uintptr_t)__builtin_return_address(0), size, 1);
    }

  return p;
}
