/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_boardinit.c
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
#include <nuttx/board.h>
#include <nuttx/mm/mm.h>
#include <stdint.h>

#include "frdm-mcxn947.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SRAMX is a separate 96 KB SRAM bank, disjoint from the 416 KB main SRAM
 * at 0x20000000 (a large address gap sits between them, so CONFIG_RAM_SIZE
 * must NOT be grown past 416 KB).  SRAMX is unused by the link map; add its
 * non-ECC alias at 0x14000000 as a second heap region.
 */

#define SRAMX_HEAP_BASE  ((void *)0x14000000)
#define SRAMX_HEAP_SIZE  (96 * 1024)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_addregion
 *
 * Description:
 *   Called by the kernel heap init when CONFIG_MM_REGIONS > 1 to add the
 *   non-contiguous SRAMX bank to the heap.
 *
 ****************************************************************************/

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
  /* Flat build: the system heap is the user (umm) heap. */

  umm_addregion(SRAMX_HEAP_BASE, SRAMX_HEAP_SIZE);
}
#endif

/****************************************************************************
 * Name: nxxx_boardinitialize
 *
 * Description:
 *   All architectures must provide the following entry point.  This
 *   entry point is called in the initialization phase -- after
 *   imx_memory_initialize and after all memory has been configured and
 *   mapped but before any devices have been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxxx_boardinitialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board initialization */

  n94x_bringup();
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
