/****************************************************************************
 * arch/arm/src/bk7258/bk7258_allocateheap.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>

#include "arm_internal.h"

extern uint8_t __heap_start[];
extern uint8_t __heap_end[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  DEBUGASSERT(&__heap_end[0] > &__heap_start[0]);
  *heap_start = __heap_start;
  *heap_size = (uintptr_t)__heap_end - (uintptr_t)__heap_start;
}

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
}
#endif
