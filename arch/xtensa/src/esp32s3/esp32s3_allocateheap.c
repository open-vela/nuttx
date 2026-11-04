/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_allocateheap.c
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
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <malloc.h>
#include <syslog.h>

#include <debug.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/mm/mm.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/userspace.h>
#include <arch/board/board.h>
/* [V636] board_memorymap.h is only generated for protected/kernel builds; the
 * FLAT kernel-heap path this board uses takes everything it needs from
 * memory_layout.h and the ROM layout. */
#if defined(CONFIG_MM_KERNEL_HEAP) && defined(CONFIG_BUILD_PROTECTED)
#include <arch/board/board_memorymap.h>
#endif

#include <arch/esp32s3/memory_layout.h>
#include "xtensa.h"
#include "hardware/esp32s3_rom_layout.h"
#ifdef CONFIG_ESP32S3_SPIRAM
#  include "esp32s3_spiram.h"
#  include "esp32s3_himem.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
#  if defined(CONFIG_ESP32S3_SPIRAM)
#    define MM_USER_HEAP_EXTRAM
#  else
#    define MM_USER_HEAP_IRAM
#  endif

#  define MM_ADDREGION kmm_addregion
#else
#  define MM_ADDREGION umm_addregion
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel and
 *   userspace heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, userspace heap.
 *
 *   If a protected kernel space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  uintptr_t ubase;
  uintptr_t utop;
  size_t    usize;

#ifdef CONFIG_MM_KERNEL_HEAP
#  ifdef CONFIG_BUILD_PROTECTED
  ubase = USERSPACE->us_dataend;

  /* Align the heap top address to 256 bytes to match the PMS split address
   * requirement.
   */

  utop  = ALIGN_DOWN(ets_rom_layout_p->dram0_rtos_reserved_start, 256);

#  elif defined(CONFIG_BUILD_FLAT)
#    ifdef MM_USER_HEAP_EXTRAM
  ubase = (uintptr_t)esp_spiram_allocable_vaddr_start();
  utop  = (uintptr_t)(esp_spiram_allocable_vaddr_end() -
                      esp_himem_reserved_area_size());
#    elif defined(MM_USER_HEAP_IRAM)
  ubase = (uintptr_t)_sheap + XTENSA_IMEM_REGION_SIZE;
  utop  = (uintptr_t)HEAP_REGION1_END;
#    endif /* MM_USER_HEAP_EXTRAM */

#  endif /* CONFIG_BUILD_PROTECTED */

#else /* !CONFIG_MM_KERNEL_HEAP */

  /* Skip internal heap region if CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP is
   * enabled.
   */

  ubase = (uintptr_t)(_sheap) + XTENSA_IMEM_REGION_SIZE;
  utop  = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#endif /* CONFIG_MM_KERNEL_HEAP */

  usize = utop - ubase;

  /* [V212] Internal DRAM is the only DMA-capable memory on this part and it
   * is shared with .bss; print the negotiation so a runaway static
   * allocation is immediately visible instead of showing up as a board that
   * boots to a blank console. */
  
  syslog(LOG_ERR,
         "[V636] heap split: app(malloc)=%s, kernel(kmm_malloc)=internal DRAM\n",
#ifdef MM_USER_HEAP_EXTRAM
         "PSRAM"
#else
         "internal DRAM (shared with the radio - see esp32s3_wifi_adapter.c)"
#endif
         );

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        ubase, utop, usize);

  DEBUGASSERT(utop > ubase);

  board_autoled_on(LED_HEAPALLOCATE);

  /* Return the userspace heap settings */

  *heap_start = (void *)ubase;
  *heap_size  = usize;
}

/****************************************************************************
 * Name: up_allocate_kheap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_PROTECTED=y) with both kernel and
 *   userspace heaps (CONFIG_MM_KERNEL_HEAP=y), this function allocates
 *   (and protects) the kernel space heap.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KERNEL_HEAP
void up_allocate_kheap(void **heap_start, size_t *heap_size)
{
  uintptr_t kbase;
  uintptr_t ktop;
  size_t    ksize;

#ifdef CONFIG_BUILD_PROTECTED
  /* These values come from the linker scripts (kernel-space.ld and
   * protected_memory.ld).
   * Check boards/xtensa/esp32s3.
   */

  kbase = (uintptr_t)_sheap;
  ktop  = KDRAM_END;
#elif defined(CONFIG_BUILD_FLAT)
#  ifdef MM_USER_HEAP_IRAM
  /* Skip internal heap region if CONFIG_XTENSA_IMEM_USE_SEPARATE_HEAP is
   * enabled.
   */

  kbase = (uintptr_t)HEAP_REGION2_START;
  ktop  = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#  else
  kbase = (uintptr_t)_sheap + XTENSA_IMEM_REGION_SIZE;
  ktop  = (uintptr_t)ets_rom_layout_p->dram0_rtos_reserved_start;
#  endif /* MM_USER_HEAP_IRAM */

#endif

  ksize = ktop - kbase;

  /* [V636] This is the resident of internal DRAM now: everything the WiFi
   * blob allocates through kmm_malloc() lands here, and nothing else competes
   * with it for DMA-capable memory. */

  syslog(LOG_ERR,
         "[V636] kernel heap (WiFi/DMA): %" PRIxPTR "..%" PRIxPTR
         " size=%zu\n", kbase, ktop, ksize);

  minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
        kbase, ktop, ksize);

  DEBUGASSERT(ktop > kbase);

  board_autoled_on(LED_HEAPALLOCATE);

  *heap_start = (void *)kbase;
  *heap_size  = ksize;
}
#endif /* CONFIG_BUILD_PROTECTED && CONFIG_MM_KERNEL_HEAP */

/****************************************************************************
 * Name: xtensa_add_region
 *
 * Description:
 *   RAM may be added in non-contiguous chunks. This routine adds all chunks
 *   that may be used for heap.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void xtensa_add_region(void)
{
  void  *start;
  void  *end;
  size_t size = 0;
  int availregions = 1;
  int nregions = CONFIG_MM_REGIONS - 1;

#if defined(CONFIG_ESP32S3_SPIRAM_COMMON_HEAP) && !defined(MM_USER_HEAP_EXTRAM)
  availregions++;
#endif

  if (nregions < availregions)
    {
      mwarn("Some memory regions are left unused!\n");
      mwarn("Increase CONFIG_MM_REGIONS to add them to the heap\n");
    }

#if defined(CONFIG_ESP32S3_SPIRAM_COMMON_HEAP) && !defined(MM_USER_HEAP_EXTRAM)
  start = (void *)esp_spiram_allocable_vaddr_start();
  end = (void *)(esp_spiram_allocable_vaddr_end() -
                 esp_himem_reserved_area_size());
  size  = (size_t)(end - start);
#endif

  if (size)
    {
      DEBUGASSERT(end > start);
      MM_ADDREGION(start, size);
      minfo("Heap: start=%" PRIxPTR " end=%" PRIxPTR " size=%zu\n",
            (uintptr_t)start, (uintptr_t)end, size);
    }
}
#endif


/****************************************************************************
 * [V633] Internal-DRAM pressure probe
 *
 * The WiFi blob allocates its TX and management buffers from the internal,
 * DMA-capable part of the system heap.  That is region 0, i.e.
 * [_sheap, ets_rom_layout_p->dram0_rtos_reserved_start).  mallinfo() only
 * reports the sum over all regions (internal + PSRAM), so probe region 0
 * directly: the allocator walks the region list in order, and ESP32-S3 maps
 * internal DRAM at 0x3FC00000..0x3FCFFFFF while PSRAM lives far below it.
 * A malloc() whose address lands in the internal window therefore proves
 * region 0 can still satisfy that size.  Scanning downwards and stopping at
 * the first internal hit yields a lower bound for "largest free internal
 * block".
 *
 * This exists because a single 8 KB task stack created before association is
 * enough to swing the radio from 6/6 to 0/6 (v629 vs v630), and because the
 * radio's own TX path starts failing with ESP_ERR_NO_MEM once this region is
 * exhausted.
 *
 ****************************************************************************/

/****************************************************************************
 * [V764] Who is eating the internal-DRAM kernel heap?
 *
 * vox3_intram_maxblock() above answers "can the WiFi blob still get a DMA
 * buffer", which is the symptom.  It cannot answer whether the region is
 * genuinely exhausted or merely fragmented, and it certainly cannot name the
 * holder.  On the real cold power-on capture both readings look identical:
 *
 *   [V636] kernel heap (WiFi/DMA): 3fcbd8b0..3fceee34 size=202116
 *   [V58]  probe gw:80 = -12  probe srv:2301 = -12      <- socket() ENOMEM
 *   [V633] t=6  intram_max=0
 *   [V757] fbmem=0x3c22f588 len=115200 -> PSRAM         <- NOT the framebuffer
 *   [agent] Failed to create agent_loop task
 *
 * ...while the EN reset of the very same image sits at intram_max=131072.  So
 * the panel's own framebuffer is no longer the culprit (v757 moved it to
 * PSRAM) and something else is holding ~200 KiB of internal DRAM.  Guessing
 * between "task stacks", "I2S bounce buffers" and "WiFi blob TX rings" is
 * exactly the kind of guess rule 3 exists to punish, so v764 measures.
 *
 * Two instruments, both allocation-path only and both read-only:
 *
 *   1. vox3_intram_account() - kmm_mallinfo() reports the KERNEL heap alone,
 *      with no PSRAM mixed in.  uordblks close to arena means the bytes are
 *      really gone (look for the holder); a healthy arena with a tiny
 *      mxordblk means fragmentation (look for the pinhole instead).  Those
 *      two failures need opposite fixes, so the distinction is the whole
 *      point of this hunk.
 *
 *   2. vox3_kmm_note() - a caller-attributed ledger for allocations of
 *      V764_LEDGER_MIN bytes or more.  8 slots is enough: in this tree only
 *      task stacks, the I2S bounce buffers and the WiFi blob's own TX rings
 *      are anywhere near 4 KiB, and mallinfo shows none of them by owner.
 *      The slot stores __builtin_return_address(0), which addr2line resolves
 *      against nuttx/nuttx without needing frame pointers.
 *
 * The ledger is deliberately tiny and allocation-free: it stores into a
 * static array, so instrumenting the allocator cannot itself perturb the
 * allocator.
 ****************************************************************************/

#define V764_LEDGER_MIN   4096u
#define V764_LEDGER_SLOTS 8

struct v764_ledger_s
{
  uintptr_t ra;    /* return address of the caller, 0 = empty slot */
  size_t    bytes; /* currently outstanding bytes from this caller */
  unsigned  count; /* number of live allocations from this caller */
};

static struct v764_ledger_s g_v764_ledger[V764_LEDGER_SLOTS];

/* [V764] Called from kmm_malloc()/kmm_zalloc() for big internal-heap blocks.
 * delta < 0 on free.  Lock-free by design: the allocator is called from every
 * context including interrupts, and a syslog() here would recurse. */

void vox3_kmm_note(uintptr_t ra, size_t size, int delta)
{
  struct v764_ledger_s *slot;
  int i;

  if (size < V764_LEDGER_MIN)
    {
      return;
    }

  for (i = 0; i < V764_LEDGER_SLOTS; i++)
    {
      slot = &g_v764_ledger[i];

      if (slot->ra == ra || slot->ra == 0)
        {
          if (delta > 0)
            {
              slot->bytes += size;
              slot->count++;
            }
          else
            {
              size_t back = slot->bytes > (size_t)delta ? slot->bytes -
                                                          (size_t)(-delta) : 0;

              slot->bytes = back;
              if (slot->count > 0)
                {
                  slot->count--;
                }
            }

          slot->ra = (slot->bytes == 0 && slot->count == 0) ? 0 : ra;
          return;
        }
    }

  /* All slots taken.  Deliberately silent: this runs in the allocator and
   * printing here would be a recursion hazard.  The dump below reports how
   * many slots were in use so a lost block is visible as "8/8". */

  g_v764_ledger[V764_LEDGER_SLOTS - 1].bytes += (size_t)delta;
}

/* [V764] Kernel-heap-only accounting, for the V633 heartbeat. */

void vox3_intram_account(uint32_t *out_arena, uint32_t *out_used,
                         uint32_t *out_free, uint32_t *out_largest)
{
  struct mallinfo mi = kmm_mallinfo();

  *out_arena   = (uint32_t)mi.arena;
  *out_used    = (uint32_t)mi.uordblks;
  *out_free    = (uint32_t)mi.fordblks;
  *out_largest = (uint32_t)mi.mxordblk;
}

/* [V764] One-shot dump of the ledger.  addr2line -e nuttx/nuttx <ra> turns
 * these into function names offline, so the caller identity survives the
 * syslog truncation. */

void vox3_kmm_dump(void)
{
  struct mallinfo mi = kmm_mallinfo();
  int used = 0;
  int i;

  for (i = 0; i < V764_LEDGER_SLOTS; i++)
    {
      if (g_v764_ledger[i].ra != 0)
        {
          used++;
        }
    }

  syslog(LOG_ERR,
         "[V764] kheap arena=%u used=%u free=%u largest=%u "
         "nfree=%u nalloc=%u ledger=%d/%d\n",
         (unsigned int)mi.arena, (unsigned int)mi.uordblks,
         (unsigned int)mi.fordblks, (unsigned int)mi.mxordblk,
         (unsigned int)mi.ordblks, (unsigned int)mi.aordblks,
         used, V764_LEDGER_SLOTS);

  for (i = 0; i < V764_LEDGER_SLOTS; i++)
    {
      if (g_v764_ledger[i].ra != 0)
        {
          syslog(LOG_ERR, "[V764] ledger[%d] ra=0x%08x live=%u n=%u\n",
                 i, (unsigned int)g_v764_ledger[i].ra,
                 (unsigned int)g_v764_ledger[i].bytes,
                 g_v764_ledger[i].count);
        }
    }
}

size_t vox3_intram_maxblock(size_t *out_total_free)
{
  size_t n;
  size_t best = 0;

  /* [V636] With CONFIG_MM_KERNEL_HEAP=y kmm_malloc() targets the kernel heap,
   * which up_allocate_kheap() places in internal DRAM - the same heap the WiFi
   * blob's os_adapter allocators use.  Plain malloc() would report the PSRAM
   * application heap and always look healthy, so probe with kmm_*. */

  /* [V807] Read the largest free block instead of probing for it.
   *
   * The probe allocated for real: kmm_malloc(131072), then 65536, then 32768...
   * until one succeeded.  The kernel heap is about 185 KiB, so the first five
   * or six attempts always fail -- and each of those is a genuine call into the
   * allocator, made from the V633 heartbeat, by a thread that has no business
   * allocating while it is measuring.
   *
   * The damage was not the wasted cycles.  One boot produced hundreds of real
   * ENOMEMs, each attributed by return address to whatever happened to call in,
   * and those numbers were then read as evidence about that caller.  They were
   * measuring this loop.  Two rounds of investigation were spent chasing a
   * "WiFi is fragmenting the heap" conclusion that came from these probe's own
   * failures.
   *
   * The rule recorded during the audio work applies verbatim: an evidence probe
   * must not change the state it observes.  mallinfo() already carries
   * mxordblk -- the largest single free block -- which is exactly the number the
   * probe was trying to discover by allocating.
   *
   * Worth stating plainly, because it changes how earlier numbers read: when
   * this probe reported 131072 on a healthy boot, that was the probe succeeding
   * at a 128 KiB allocation, not the heap holding 128 KiB free.  Those are
   * unrelated quantities. */
  {
    struct mallinfo mi = kmm_mallinfo();

    if (out_total_free != NULL)
      {
        *out_total_free = (size_t)mi.fordblks;
      }

    return (size_t)mi.mxordblk;
  }
}

