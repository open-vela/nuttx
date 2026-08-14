/****************************************************************************
 * arch/arm/src/bk7258/bk7258_timerisr.c
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

/* Porting reference: arch/arm/src/at32/at32_timerisr.c (classic SysTick
 * tick) and arch/arm/src/mps/mps_timer.c (arch_timer lower-half).
 *
 * 2026-08-14 bug fix — "sleep hangs forever, clock never advances":
 *
 *   The first version of this file did only:
 *
 *     putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
 *     up_timer_set_lowerhalf(systick_initialize(true,
 *                                               BOARD_SYSTICK_CLOCK, -1));
 *
 *   That looks complete but produced *zero* timer interrupts, because
 *   include/nuttx/timers/arch_timer.h defines
 *
 *     #ifdef CONFIG_TIMER_ARCH
 *     void up_timer_set_lowerhalf(struct timer_lowerhalf_s *lower);
 *     #else
 *     #  define up_timer_set_lowerhalf(lower)
 *     #endif
 *
 *   and this board does not enable CONFIG_TIMER / CONFIG_TIMER_ARCH.  The
 *   empty macro swallows its own argument, so the systick_initialize()
 *   call was silently removed by the preprocessor: SysTick was never
 *   enabled, no handler was ever attached, and g_system_timer never
 *   incremented.  The system still booted and NSH still worked (the
 *   console is interrupt driven), which is why this went unnoticed —
 *   it only showed up as sleep() hanging and as 0 ms deltas in the PSRAM
 *   throughput measurements.
 *
 *   Fix: drive the tick directly from the SysTick exception unless the
 *   arch_timer lower-half is actually built in.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/timers/arch_timer.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "systick.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CLK_TCK (include/time.h) is the desired number of ticks per second and
 * follows CONFIG_USEC_PER_TICK (10000us -> 100Hz here).
 *
 * SysTick is clocked from the core clock (CLKSOURCE=1).
 */

#define SYSTICK_RELOAD ((BOARD_SYSTICK_CLOCK / CLK_TCK) - 1)

/* The RELOAD field is 24 bits wide — catch a bad BOARD_SYSTICK_CLOCK at
 * compile time instead of silently truncating the reload value.
 */

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the 24-bit RELOAD register
#endif

#if SYSTICK_RELOAD < 1
#  error SYSTICK_RELOAD too small — check BOARD_SYSTICK_CLOCK / CLK_TCK
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifndef CONFIG_TIMER_ARCH

/****************************************************************************
 * Name: bk7258_timerisr
 *
 * Description:
 *   SysTick interrupt handler: advances the system timer.
 *
 ****************************************************************************/

static int bk7258_timerisr(int irq, uint32_t *regs, void *arg)
{
  UNUSED(irq);
  UNUSED(regs);
  UNUSED(arg);

  nxsched_process_timer();
  return 0;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   hardware (SysTick).
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
#ifdef CONFIG_TIMER_ARCH
  /* Tickless / arch_timer path: hand SysTick to the timer lower-half,
   * which takes care of enabling it.
   */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
  up_timer_set_lowerhalf(systick_initialize(true, BOARD_SYSTICK_CLOCK, -1));
#else
  uint32_t regval;

  /* Set the SysTick exception to the default interrupt priority */

  regval  = getreg32(NVIC_SYSH12_15_PRIORITY);
  regval &= ~NVIC_SYSH_PRIORITY_PR15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << NVIC_SYSH_PRIORITY_PR15_SHIFT);
  putreg32(regval, NVIC_SYSH12_15_PRIORITY);

  /* Program the reload value and restart the counter from zero */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);
  putreg32(0, NVIC_SYSTICK_CURRENT);

  /* Attach the SysTick handler */

  irq_attach(BK7258_IRQ_SYSTICK, (xcpt_t)bk7258_timerisr, NULL);

  /* Core clock source, interrupt enabled, counter enabled */

  putreg32(NVIC_SYSTICK_CTRL_CLKSOURCE | NVIC_SYSTICK_CTRL_TICKINT |
           NVIC_SYSTICK_CTRL_ENABLE, NVIC_SYSTICK_CTRL);

  up_enable_irq(BK7258_IRQ_SYSTICK);
#endif
}
