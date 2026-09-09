/****************************************************************************
 * arch/arm/src/bk7258/bk7258_psram.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/mm/mm.h>
#include <nuttx/mutex.h>
#include <errno.h>
#include <stdio.h>
#include "bk7258_psram.h"
#include "hardware/bk7258_psram.h"

/* Only the vendor AP heap is owned here. CP heap, media slabs and linker
 * sections remain untouched; ordinary malloc continues to use SRAM.
 */

static mutex_t g_lock = NXMUTEX_INITIALIZER;
static struct mm_heap_s *g_heap;
static unsigned int g_allocations;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int bk7258_psram_initialize(void)
{
  nxmutex_lock(&g_lock);
  if (g_heap == NULL)
    {
      volatile uint32_t *first = (void *)BK7258_AP_PSRAM_HEAP_BASE;
      volatile uint32_t *last = (void *)(BK7258_AP_PSRAM_HEAP_BASE +
                                       BK7258_AP_PSRAM_HEAP_SIZE - 4);
      uint32_t saved_first = *first;
      uint32_t saved_last = *last;
      *first = 0x72583501;
      *last = 0x35ac7258;
      __asm__ volatile("dmb sy" ::: "memory");
      int valid = *first == 0x72583501 && *last == 0x35ac7258;
      *first = saved_first;
      *last = saved_last;
      __asm__ volatile("dmb sy" ::: "memory");
      if (!valid)
        {
          nxmutex_unlock(&g_lock);
          return -EIO;
        }

      g_heap = mm_initialize("ap-psram", (void *)BK7258_AP_PSRAM_HEAP_BASE,
                             BK7258_AP_PSRAM_HEAP_SIZE);
    }

  int ret = g_heap == NULL ? -ENOMEM : 0;
  nxmutex_unlock(&g_lock);
  return ret;
}

int bk7258_psram_shutdown(void)
{
  int ret = 0;
  nxmutex_lock(&g_lock);
  if (g_allocations != 0) ret = -EBUSY;
  else if (g_heap != NULL)
    {
      mm_uninitialize(g_heap);
      g_heap = NULL;
    }

  nxmutex_unlock(&g_lock);
  return ret;
}

void bk7258_psram_power_lost(void)
{
  nxmutex_lock(&g_lock);
  g_heap = NULL;
  g_allocations = 0;
  nxmutex_unlock(&g_lock);
}

void *bk7258_psram_malloc(size_t size)
{
  void *ptr = NULL;
  nxmutex_lock(&g_lock);
  if (g_heap != NULL && size != 0)
    {
      ptr = mm_malloc(g_heap, size);
      if (ptr != NULL) g_allocations++;
    }

  nxmutex_unlock(&g_lock);
  return ptr;
}

void bk7258_psram_free(void *ptr)
{
  nxmutex_lock(&g_lock);
  if (g_heap != NULL && ptr != NULL)
    {
      mm_free(g_heap, ptr);
      g_allocations--;
    }

  nxmutex_unlock(&g_lock);
}

void *bk7258_psram_realloc(void *ptr, size_t size)
{
  if (ptr == NULL) return bk7258_psram_malloc(size);
  if (size == 0)
    {
      bk7258_psram_free(ptr);
      return NULL;
    }

  void *result = NULL;
  uintptr_t address = (uintptr_t)ptr;
  nxmutex_lock(&g_lock);
  if (g_heap != NULL && address >= BK7258_AP_PSRAM_HEAP_BASE &&
      address < BK7258_AP_PSRAM_HEAP_BASE + BK7258_AP_PSRAM_HEAP_SIZE)
    result = mm_realloc(g_heap, ptr, size);
  nxmutex_unlock(&g_lock);
  return result;
}

uint32_t bk7258_psram_heap_used(void)
{
  nxmutex_lock(&g_lock);
  uint32_t used = g_heap == NULL ? 0 : mm_mallinfo(g_heap).uordblks;
  nxmutex_unlock(&g_lock);
  return used;
}

void bk7258_psram_dump(void)
{
  printf("PSRAM: AP heap used=%lu bytes\n",
         (unsigned long)bk7258_psram_heap_used());
}
