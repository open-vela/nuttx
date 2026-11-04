/****************************************************************************
 * mm/kmm_heap/kmm_malloc.c
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

/* [V794] Failure-time accounting for the kernel heap.
 *
 * On a black-screen boot the ledger reports 120 KB free and a 120 KB
 * largest block, yet nxthread_create() cannot get a TCB.  That cannot both
 * be true, so print the request size next to the live ledger AT THE POINT
 * OF FAILURE instead of inferring it from a dump taken elsewhere.
 *
 * Installed in BOTH kmm_malloc() and kmm_zalloc() because they do not call
 * each other -- kmm_zalloc() goes straight to mm_zalloc(KNR_HEAP, size),
 * so a hook in kmm_malloc() would never see the TCB request.
 */
#include <malloc.h>
#include <syslog.h>

/* [V808] Same address as kmm_mallinfo(); recorded because it was suspected for
 * a long time to be a different pool.  It is not:
 *
 *   struct mallinfo kmm_mallinfo(void) { return mm_mallinfo(KNR_HEAP); }
 *
 * So the arena printed here and the arena printed by the V764 ledger are the
 * same pool, and the "mallinfo reports one pool while kmm_zalloc uses another"
 * theory -- which sat in the notes through a dozen revisions -- does not hold.
 * Both report KNR_HEAP; the numbers are comparable. */
static void __attribute__((noinline)) v794_note_failure(size_t size,
                                                       uintptr_t ra)
{
  struct mallinfo mi = mm_mallinfo(KNR_HEAP);

  /* [V808] Rate limit.
   *
   * A failed allocation often repeats, sometimes thousands of times per boot
   * for one caller, and each one printed a full line.  That turned the kmsg ring
   * into a wall of near-identical text, which cost real cycles and buried every
   * other message -- including the lines that actually distinguish one cause
   * from another.
   *
   * So: report the first few, then only when the picture changes (a different
   * caller, a different request size), then keep a count and print it with a
   * periodic digest.  The total is never lost, only the repetition. */
  static uint32_t s_count;
  static uint32_t s_last_ra;
  static size_t   s_last_size;
  static uint32_t s_suppressed;

  uint32_t count = ++s_count;

  if (count <= 5 || ra != s_last_ra || size != s_last_size)
    {
      s_suppressed = 0;
      s_last_ra    = ra;
      s_last_size  = size;

      syslog(LOG_ERR,
             "[V794] kmm_malloc FAILED requested=%u arena=%u fordblks=%u "
             "mxordblk=%u uordblks=%u ra=0x%08x\n",
             (unsigned)size, (unsigned)mi.arena, (unsigned)mi.fordblks,
             (unsigned)mi.mxordblk, (unsigned)mi.uordblks, (unsigned)ra);
    }
  else if (++s_suppressed >= 200)
    {
      s_suppressed = 0;
      syslog(LOG_ERR,
             "[V808] kmm_malloc FAILED x%u total (latest requested=%u "
             "ra=0x%08x; same site, suppressed interim lines)\n",
             (unsigned)count, (unsigned)size, (unsigned)ra);
    }
}

/* [V764] ledger hook, see esp32s3_allocateheap.c */
struct v764_acct_s;
void vox3_kmm_note(uintptr_t ra, size_t size, int delta);
void vox3_intram_account(uint32_t *arena, uint32_t *used,
                         uint32_t *free, uint32_t *largest);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kmm_malloc
 *
 * Description:
 *   Allocate memory from the kernel heap.
 *
 * Input Parameters:
 *   size - Size (in bytes) of the memory region to be allocated.
 *
 * Returned Value:
 *   The address of the allocated memory (NULL on failure to allocate)
 *
 ****************************************************************************/

FAR void *kmm_malloc(size_t size)
{
  FAR void *p;
  uintptr_t caller;

  /* [V764] attribute big internal-heap blocks to their caller
   * before handing them on; see vox3_kmm_note() for why only
   * allocations >= 4 KiB are recorded. */
  p = mm_malloc(KNR_HEAP, size);

  /* [V797] The failure hook used to sit ABOVE the mm_malloc() call, testing
   * `p == NULL` before p had been assigned.  So it read an uninitialised stack
   * value: the log fired at random and its request sizes could not be trusted.
   * That is why the V794 numbers never matched what the caller had asked for.
   * It has to come after the allocation, and it now records the caller, because
   * a 1700-byte request failed 3656 times in one capture with no way to tell
   * who was making it. */
  if (p == NULL)
    {
      caller = (uintptr_t)__builtin_return_address(0);
      v794_note_failure(size, caller);
    }

  if (p != NULL)
    {
      vox3_kmm_note((uintptr_t)__builtin_return_address(0),
                   size, 1);
    }

  return p;
}
