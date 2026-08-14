/****************************************************************************
 * arch/arm/src/bk7258/bk7258_allocateheap.c
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
#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The heap extends from the top of the idle stack to the end of the RAM
 * data region.  CONFIG_RAM_START/SIZE are set in defconfig
 * (0x28010000 / 336KB for the CPU0 app RAM window).
 */

#define BK7258_RAM_END (CONFIG_RAM_START + CONFIG_RAM_SIZE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = (void *)g_idle_topstack;
  *heap_size  = BK7258_RAM_END - g_idle_topstack;
}
