/****************************************************************************
 * arch/xtensa/src/t113/t113_allocateheap.c
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
#include <nuttx/arch.h>
#include <nuttx/userspace.h>

#include "chip_memory.h"
#include "xtensa.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  /* The heap runs from the end of .bss up to the firmware-region limit
   * (T113_DDR_FW_LIMIT == the rptun carveout base), minus 64 KB.
   * The heap must not reach into the rptun carveout, which holds the
   * rpmsg vrings/buffer pool.
   */

  *heap_start = (void *)_ebss;
  *heap_size  = (size_t)(T113_DDR_FW_LIMIT
                       - (uintptr_t)_ebss
                       - 0x10000);   /* leave 64 KB headroom for stack */
}
