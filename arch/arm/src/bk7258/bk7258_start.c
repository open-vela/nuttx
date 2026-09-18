/****************************************************************************
 * arch/arm/src/bk7258/bk7258_start.c
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

#include <nuttx/init.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nvic.h"

#include "chip.h"
#include "hardware/bk7258_memorymap.h"
#include "bk7258_lowputc.h"
#include "bk7258_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Emit single-character start-up milestones to the console only when
 * CONFIG_DEBUG_FEATURES is set; a no-op otherwise.  Useful for early
 * bring-up to locate where start-up stalls.
 */

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) arm_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_idle_topstack: top of the idle thread stack = end of .bss (_ebss) plus
 * the idle stack size.  The heap starts right after it (see
 * bk7258_allocateheap.c).
 */

const uintptr_t g_idle_topstack =
  (uintptr_t)_ebss + CONFIG_IDLETHREAD_STACKSIZE;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Reset entry point (naked).  Entered by a jump from the bootloader --
 *   this is a "jump", not a true reset, so like ARMINO Reset_Handler_Cpu0
 *   it must explicitly:
 *     1) reload SP from vector[0] (make sure the stack is correct);
 *     2) clear MSPLIM (the Armv8-M stack-limit register; a stale value left
 *        by the bootloader would trigger a stack-limit violation on the
 *        first push -> UsageFault -> no output at all).
 *   It then branches into the C start-up body bk7258_cstart().
 *
 ****************************************************************************/

void __start(void) naked_function;
void __start(void)
{
  /* Ordering is critical (verified by SWD single-step): before setting SP
   * we must first
   *   (1) clear MSPLIM -- the bootloader leaves a high stack-limit value
   *       and our SP (0x28011c68) is below it; setting SP before clearing
   *       the limit means any exception push in between hits SP < MSPLIM
   *       -> STKOF; and
   *   (2) enable the FPU -- an exception push does an FPU lazy save
   *       (FPCCR.LSPEN); with the FPU off this raises NOCP.
   * Together (measured CFSR=0x00180000: NOCP+STKOF) these cause a double
   * fault -> LOCKUP, so clear MSPLIM and enable the FPU first, then set SP
   * and VTOR.  (Also: never write DTCM 0x20000000; on a CPU0 cold boot it
   * is not enabled and any access faults.)  Steps (3) set SP = vector[0]
   * and (4) point VTOR at our vector table so early exceptions are handled
   * by this image.
   */

  __asm__ __volatile__
  (
    "  cpsid i\n"                 /* Disable interrupts */
    "  movs  r0, #0\n"
    "  msr   MSPLIM, r0\n"        /* (1) clear the stack limit first */
    "  ldr   r0, =0xE000ED88\n"   /* (2) enable FPU: SCB->CPACR */
    "  ldr   r1, [r0]\n"
    "  ldr   r2, =0x00f00000\n"   /* CP10/CP11 = full access (bits 23:20) */
    "  orr   r1, r1, r2\n"
    "  str   r1, [r0]\n"
    "  dsb\n"
    "  isb\n"
    "  ldr   r0, =_vectors\n"     /* (3) set SP = vector[0] */
    "  ldr   r1, [r0]\n"
    "  mov   sp, r1\n"
    "  ldr   r0, =_vectors\n"     /* (4) point VTOR at our vectors */
    "  ldr   r1, =0xE000ED08\n"   /* SCB->VTOR */
    "  str   r0, [r1]\n"
    "  b     bk7258_cstart\n"     /* Enter the C start-up body */
  );
}

/****************************************************************************
 * Name: bk7258_wdt_disable
 *
 * Description:
 *   Disable the AON WDT and the normal WDT.  The bootloader enables a
 *   watchdog (~100ms); if NuttX does not disable/feed it early it will be
 *   reset -> boot loop.  The sequence is from ARMINO wdt_hal_close()
 *   (SOC_AON_WDT_REG_BASE=0x44000600, SOC_WDT_REG_BASE=0x44800000).
 *
 ****************************************************************************/

static void bk7258_wdt_disable(void)
{
  uint32_t v;

  /* AON WDT: write key (0x5A/0xA5) << 16, period = 0 */

  putreg32(0x5a0000, BK7258_AON_WDT_BASE);
  putreg32(0xa50000, BK7258_AON_WDT_BASE);

  /* Normal WDT: global_ctrl (off 0x08) bit1 = 1, then ctrl (off 0x10)
   * write key, period = 0.
   */

  v = getreg32(BK7258_WDT_BASE + 0x08);
  putreg32(v | (1u << 1), BK7258_WDT_BASE + 0x08);
  putreg32(0x5a0000, BK7258_WDT_BASE + 0x10);
  putreg32(0xa50000, BK7258_WDT_BASE + 0x10);
}

/****************************************************************************
 * Name: bk7258_cstart
 *
 * Description:
 *   C start-up body: set up the console -> clear .bss -> copy .data ->
 *   board initialization -> nx_start.
 *
 ****************************************************************************/

void bk7258_cstart(void) noreturn_function;
void bk7258_cstart(void)
{
  const uint32_t *src;
  uint32_t *dest;

  /* Initialize the console UART (UART0 = on-board CH340) */

  bk7258_lowsetup();
  showprogress('A');

  /* Disable the bootloader watchdog, otherwise we get reset into a boot
   * loop.
   */

  bk7258_wdt_disable();
  showprogress('B');

  /* Clear .bss */

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; dest++)
    {
      *dest = 0;
    }

  showprogress('C');

  /* Move the initialized data from its load address (_eronly) to RAM */

  for (src = (const uint32_t *)_eronly, dest = (uint32_t *)_sdata;
       dest < (uint32_t *)_edata;
       dest++, src++)
    {
      *dest = *src;
    }

  showprogress('D');

#ifdef CONFIG_ARCH_FPU
  arm_fpuconfig();
#endif

  showprogress('E');

  /* Perform board-specific initialization */

  arm_boardinitialize();
  showprogress('F');

  showprogress('\r');
  showprogress('\n');

  nx_start();

  /* Shouldn't get here */

  for (; ; )
    {
    }
}
