/****************************************************************************
 * mm/mm_heap/mm_foreach.c
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

#include <assert.h>
#include <debug.h>
#include <stdbool.h>
#ifdef CONFIG_MM_RECORD_STACK
#  include <execinfo.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/mm/kasan.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mutex.h>
#include <sched.h>

#include "mm_heap/mm.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_MM_RECORD_STACK
/* Dump the allocation backtrace recorded in a node, to identify who
 * allocated a heap node with smashed metadata. */

static void mm_foreach_dump_node(FAR struct mm_heap_s *heap,
                                 FAR struct mm_allocnode_s *node)
{
  int r;

  merr("MM CORRUPT: node=%p size=%zu preceding=%u\n",
       node, MM_SIZEOF_NODE(node), (unsigned int)node->preceding);

  merr("MM CORRUPT: heap=%p regions:", heap);
  for (r = 0; r < heap->mm_nregions; r++)
    {
      merr(" [%p..%p)", heap->mm_heapstart[r], heap->mm_heapend[r]);
    }
  merr("\n");

  /* Raw bytes around the smashed header: identifies the writer's
   * data fingerprint (pixels / JSON / pointers). */

  FAR volatile uint8_t *pre = (FAR volatile uint8_t *)node - 32;
  FAR volatile uint8_t *post = (FAR volatile uint8_t *)node;
  int i;

  for (i = 0; i < 32; i++)
    {
      merr("%02x", pre[i]);
    }

  merr(" | ");

  for (i = 0; i < 48; i++)
    {
      merr("%02x", post[i]);
    }

  merr("\n");
}

/* A free-list neighbor must be inside one of the heap regions or inside
 * the heap struct itself (bucket list heads). Anything else is corrupted
 * metadata; report instead of faulting on the deref. */

static bool mm_foreach_nearby(FAR struct mm_heap_s *heap, FAR void *p)
{
  int r;

  if (p == NULL)
    {
      return true; /* legal list tail */
    }

  if ((uintptr_t)p & 3)
    {
      return false;
    }

  if (p >= (FAR void *)heap && p < (FAR void *)(heap + 1))
    {
      return true;
    }

#if CONFIG_MM_REGIONS > 1
  for (r = 0; r < heap->mm_nregions; r++)
#else
  for (r = 0; r < 1; r++)
#endif
    {
      if (p >= heap->mm_heapstart[r] &&
          p < heap->mm_heapend[r])
        {
          return true;
        }
    }

  return false;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_foreach
 *
 * Description:
 *   Visit each node to run handler in heap.
 *
 ****************************************************************************/

void mm_foreach(FAR struct mm_heap_s *heap, mm_node_handler_t handler,
                FAR void *arg)
{
  FAR struct mm_allocnode_s *node;
  FAR struct mm_allocnode_s *prev;
  size_t nodesize;
  bool bypass;
  bool locked = false;
#if CONFIG_MM_REGIONS > 1
  int region;
#else
#  define region 0
#endif

  DEBUGASSERT(handler);

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < heap->mm_nregions; region++)
#endif
    {
      prev = NULL;

      /* Visit each node in the region
       * Retake the mutex for each region to reduce latencies
       */

#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
      if (up_interrupt_context())
        {
#ifdef CONFIG_SMP
          return;
#else
          if (nxrmutex_is_locked(&heap->mm_lock))
            {
              return;
            }
#endif
        }
      else
        {
          DEBUGASSERT(!sched_idletask());
          DEBUGVERIFY(nxrmutex_lock(&heap->mm_lock));
          locked = true;
        }
#else
      /* User space in kernel build cannot check interrupt context */

      DEBUGVERIFY(nxrmutex_lock(&heap->mm_lock));
      locked = true;
#endif

      bypass = kasan_bypass(true);

      for (node = heap->mm_heapstart[region];
           node < heap->mm_heapend[region];
           node = (FAR struct mm_allocnode_s *)((FAR char *)node + nodesize))
        {
          nodesize = MM_SIZEOF_NODE(node);
          DEBUGASSERT(nodesize >= MM_SIZEOF_ALLOCNODE);
          minfo("region=%d node=%p size=%zu preceding=%u (%c %c)\n",
                region, node, nodesize, (unsigned int)node->preceding,
                MM_PREVNODE_IS_FREE(node) ? 'F' : 'A',
                MM_NODE_IS_ALLOC(node) ? 'A' : 'F');

#ifdef CONFIG_MM_RECORD_STACK
          /* A smashed size field makes the walk escape the region and
           * fault on the next node header. Report the current node
           * (its allocation record names the overflowing buffer's
           * owner) and stop instead of faulting. */

          if (nodesize < MM_SIZEOF_ALLOCNODE ||
              (uintptr_t)node + nodesize > (uintptr_t)heap->mm_heapend[region])
            {
              merr("MM CORRUPT: walk escapes region=%d node=%p size=%zu\n",
                   region, node, nodesize);
              mm_foreach_dump_node(heap, node);
              kasan_bypass(bypass);
              if (locked)
                {
                  DEBUGVERIFY(nxrmutex_unlock(&heap->mm_lock));
                }
              return;
            }

          if (!MM_NODE_IS_ALLOC(node))
            {
              FAR struct mm_freenode_s *fnode = (FAR void *)node;

              if (!mm_foreach_nearby(heap, fnode->blink) ||
                  (fnode->flink != NULL &&
                   !mm_foreach_nearby(heap, fnode->flink)))
                {
                  merr("MM CORRUPT: free node %p blink=%p flink=%p\n",
                       node, fnode->blink, fnode->flink);
                  mm_foreach_dump_node(heap, node);
                  kasan_bypass(bypass);
                  if (locked)
                    {
                      DEBUGVERIFY(nxrmutex_unlock(&heap->mm_lock));
                    }
                  return;
                }
            }
#endif

          handler(node, arg);

          DEBUGASSERT(MM_PREVNODE_IS_ALLOC(node) ||
                      MM_SIZEOF_NODE(prev) == node->preceding);
          prev = node;
        }

      minfo("region=%d node=%p heapend=%p\n",
            region, node, heap->mm_heapend[region]);
      DEBUGASSERT(node == heap->mm_heapend[region]);
      handler(node, arg);

      kasan_bypass(bypass);
      if (locked)
        {
          DEBUGVERIFY(nxrmutex_unlock(&heap->mm_lock));
        }
    }
#undef region
}
