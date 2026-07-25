/****************************************************************************
 * boards/arm/rk3588-m0/evb7-m0/src/rk3588m0_boardinitialize.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3588m0_boardinitialize
 *
 * Description:
 *   All RK3588-M0 architectures must provide this entry point, called very
 *   early in the initialization sequence - after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ****************************************************************************/

void rk3588m0_boardinitialize(void)
{
  /* Publish the magic word as early as possible so the Linux side can tell
   * "the core started and its stores land in DDR" apart from "the core never
   * ran", exactly as the bare-metal bring-up firmware did. The counter is
   * bumped later, once the scheduler and SysTick are running, so the pair
   * distinguishes a core that started from one that is actually scheduling.
   */

  putreg32(BOARD_HEARTBEAT_MAGIC, BOARD_HEARTBEAT_MAGIC_ADDR);
  putreg32(0, BOARD_HEARTBEAT_COUNT_ADDR);
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().  board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase may
 *   be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  syslog(LOG_INFO, "[M0] NuttX on RK3588 PMU Cortex-M0\n");
}
#endif
