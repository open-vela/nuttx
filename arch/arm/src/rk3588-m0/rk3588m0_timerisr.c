/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_timerisr.c
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
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "nvic.h"
#include "clock/clock.h"
#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The PMU_M0 core clock feeds SysTick.  SysTick itself is present on this
 * core: RK3588 TRM exposes an "mcu_stcalib" GRF register ("Systick timer
 * counter", with the STCALIB[25]/[24] calibration bits) and the MCU status
 * bits describe the core as possibly waiting on an "internal SysTick".
 *
 * BOARD_MCU_FREQUENCY comes from the board.  On EVB7 the M0 runs at 200MHz:
 * FCLK_PMU_CM0_CORE is fed from hclk_pmu_cm0_root, and PMU_CLKSEL_CON1
 * (0xFD7F0304) reads 0x00000400, i.e. bits[11:10]=0b01 selecting
 * clk_pmu1_200m_src (the mux tops out at 400MHz).
 */

#define SYSTICK_CLOCK  BOARD_MCU_FREQUENCY

/* CLK_TCK ticks per second (see include/time.h), default 100 = 10ms.
 * At 200MHz and 100Hz the reload is 1,999,999, which fits the 24-bit field.
 */

#define SYSTICK_RELOAD ((SYSTICK_CLOCK / CLK_TCK) - 1)

#if SYSTICK_RELOAD > 0x00ffffff
#  error SYSTICK_RELOAD exceeds the range of the RELOAD register
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  rk3588m0_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int rk3588m0_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  uint32_t regval;

  /* Set the SysTick interrupt to the default priority */

  regval  = getreg32(ARMV6M_SYSCON_SHPR3);
  regval &= ~SYSCON_SHPR3_PRI_15_MASK;
  regval |= (NVIC_SYSH_PRIORITY_DEFAULT << SYSCON_SHPR3_PRI_15_SHIFT);
  putreg32(regval, ARMV6M_SYSCON_SHPR3);

  /* Configure SysTick to interrupt at the requested rate */

  putreg32(SYSTICK_RELOAD, ARMV6M_SYSTICK_RVR);

  /* Start counting from a known value rather than whatever is left over from
   * earlier firmware: the current-value register is not reset by writing RVR.
   */

  putreg32(0, ARMV6M_SYSTICK_CVR);

  /* Attach the timer interrupt vector */

  irq_attach(RK3588M0_IRQ_SYSTICK, (xcpt_t)rk3588m0_timerisr, NULL);

  /* Enable SysTick, clocked by the core clock, with interrupts */

  putreg32((SYSTICK_CSR_CLKSOURCE | SYSTICK_CSR_TICKINT |
            SYSTICK_CSR_ENABLE),
           ARMV6M_SYSTICK_CSR);

  /* And enable the timer interrupt */

  up_enable_irq(RK3588M0_IRQ_SYSTICK);
}
