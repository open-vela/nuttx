/****************************************************************************
 * arch/arm/src/bk7258/bk7258_timer.c
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

#if !defined(CONFIG_TIMER_ARCH) || !defined(CONFIG_ARM_SYSTICK)
#  error "BK7258 system timer requires TIMER_ARCH and ARM_SYSTICK"
#endif

#include <time.h>

#include <nuttx/timers/arch_timer.h>
#include <nuttx/timers/timer.h>

#include "arm_internal.h"
#include "nvic.h"
#include "systick.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* BK7258 CPU core (ARMv8-M Star/M33F) runs at 480 MHz.  The SysTick
 * peripheral is clocked by the core clock; the tick period is derived
 * from CONFIG_USEC_PER_TICK through CLK_TCK.
 */

#define BK7258_SYSTICK_CLOCK    (480 * 1000 * 1000)
#define SYSTICK_RELOAD          ((BK7258_SYSTICK_CLOCK / CLK_TCK) - 1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   hardware.  The BK7258 uses the standard ARMv8-M SysTick as the NuttX
 *   system timer.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  /* Set reload register (preload, avoids a zero reload glitch) */

  putreg32(SYSTICK_RELOAD, NVIC_SYSTICK_RELOAD);

  /* Register the SysTick lower-half timer driver */

  up_timer_set_lowerhalf(systick_initialize(true, BK7258_SYSTICK_CLOCK, -1));
}
