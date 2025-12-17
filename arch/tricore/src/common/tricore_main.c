/****************************************************************************
 * arch/tricore/src/common/tricore_main.c
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

#include <tricore_internal.h>
#include <nuttx/init.h>

#include "Ifx_Types.h"
#include "IfxCpu.h"

#ifdef CONFIG_ARCH_CHIP_AURIX_TC4XX
#  include "IfxWtu.h"
#else
#  include "IfxScuWdt.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void core_main(void)
{
  static IfxCpu_syncEvent g_sync_event = 0;

  /* !!WATCHDOG0 AND SAFETY WATCHDOG ARE DISABLED HERE!!
   * Enable the watchdogs and service them periodically if it is required
   */

  /* Wait for CPU sync event */

  IfxCpu_emitEvent(&g_sync_event);
  IfxCpu_waitEvent(&g_sync_event, 1);

#ifdef USE_EARLYSERIALINIT
  tricore_earlyserialinit();
#endif

#ifdef CONFIG_BUILD_PROTECTED
  tricore_userspace();
#endif

#ifdef CONFIG_ARCH_USE_MPU
  tricore_mpuinit();
#endif

#ifdef CONFIG_TRICOREHT
  __mtcr(CPU_TRCCFG, __mfcr(CPU_TRCCFG) | (1U << 15));
#endif

  nx_start();

  while (1);
}

void weak_function tricore_apu_init(void)
{
}

void weak_function tricore_csrm_apu_init(void)
{
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void core0_main(void)
{
  /* All WDTs except WDTCPU0 and system watchdog WDTSYS are in
   * disabled mode after Boot-FW execution. Disable the watchdog
   * to ensure the normal startup of the system.
   */

#if defined(CONFIG_ARCH_CHIP_AURIX_TC3XX)
  IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
  IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
#elif defined(CONFIG_ARCH_CHIP_AURIX_TC4XX)
  IfxWtu_disableCpuWatchdog(IfxWtu_getCpuWatchdogPassword());
  IfxWtu_disableSystemWatchdog(IfxWtu_getSystemWatchdogPassword());
#endif

  /* Before start we need configure access regs, only core0 can
   * do this. This is done in tricore_apu_init() function.
   */

  tricore_apu_init();

  core_main();
}

void core1_main(void)
{
  core_main();
}

void core2_main(void)
{
  core_main();
}

void core3_main(void)
{
  core_main();
}

void core4_main(void)
{
  core_main();
}

void core5_main(void)
{
  core_main();
}

void core6_main(void)
{
  tricore_csrm_apu_init();
  core_main();
}
