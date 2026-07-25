/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_start.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>
#include <nuttx/board.h>

#include "arm_internal.h"
#include "chip.h"
#include "rk3588m0_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Memory map (M0 view; see hardware/rk3588m0_memorymap.h):
 *
 *   0x00000000 - vector table, .text, .rodata, .data, .bss, idle stack, heap
 *   0x00200000 - end of the region (mcu_reserved is 2MB)
 *
 * Unlike a typical microcontroller port there is no FLASH/SRAM split here: the
 * whole image lives in the 0x00000000 window, which the hardware remaps onto
 * the DDR carveout given by CODE_START (mcu_reserved, physical 0x07a00000).
 * That region is writable - the bare-metal M0 demo stored into it and Linux
 * read the result back - so .data is already in place at its final address and
 * needs no relocation.  The linker script therefore sets _eronly == _sdata.
 */

#define IDLE_STACK ((uint32_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

const uintptr_t g_idle_topstack = IDLE_STACK;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: showprogress
 *
 * Description:
 *   Print a character on the shared UART to show boot status.  Available very
 *   early because the port is already configured by its owner and this only
 *   polls THRE and writes THR.
 *
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) rk3588m0_lowputc((int)c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the reset entry point.  u-boot programs the M0's code window to
 *   point at the reserved DDR region and then releases the core from reset;
 *   the M0 fetches the vector table from address 0 (ARMv6-M has no VTOR) and
 *   lands here.
 *
 ****************************************************************************/

void __start(void)
{
  uint32_t *dest;

  /* The shared console needs no setup, but keep the hook for symmetry with
   * other ports.
   */

  rk3588m0_lowsetup();
  showprogress('A');

  /* Clear .bss.  Done inline rather than via memset so there is no dependency
   * on the state of global variables.
   */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  showprogress('B');

  /* No .data relocation: code and data share one writable region, so the
   * linker placed .data at its final address already.
   */

  /* Perform early serial initialization */

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif
  showprogress('C');

  /* Initialize onboard resources */

  rk3588m0_boardinitialize();
  showprogress('D');

  /* Then start NuttX */

  showprogress('\r');
  showprogress('\n');
  nx_start();

  /* Shouldn't get here */

  for (; ; );
}
