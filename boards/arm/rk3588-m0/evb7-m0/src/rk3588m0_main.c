/****************************************************************************
 * boards/arm/rk3588-m0/evb7-m0/src/rk3588m0_main.c
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
#include <unistd.h>
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#ifdef CONFIG_RPTUN
#  include "rk3588m0_rptun.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: m0_main
 *
 * Description:
 *   Initial task for the first-milestone bring-up of NuttX on the RK3588 PMU
 *   Cortex-M0.  There is no shell yet: the shared UART2 is only borrowed for
 *   output, so a console would mean fighting Linux's fiq-debugger for input.
 *   An nsh over rpmsg is the plan instead, reusing what already works on the
 *   cpu_l3 side.
 *
 *   What this proves, and how to observe it from Linux:
 *
 *     - the image boots and the scheduler runs: the syslog lines below appear
 *       on the shared console (interleaved with Linux output, as expected);
 *     - SysTick actually fires: sleep() only returns if the tick is running, so
 *       a counter that keeps advancing means timer + scheduler are alive.
 *
 *       busybox devmem 0x07a00800 32   -> 0x414D5030 ("AMP0"), set at boot
 *       busybox devmem 0x07a00804 32   -> increments once per second
 *
 *   That is deliberately the same contract the bare-metal firmware used, so the
 *   existing checks keep working - only now the counter advancing also proves
 *   the tick, because it is driven by sleep() rather than a delay loop.
 *
 ****************************************************************************/

int m0_main(int argc, char *argv[])
{
  uint32_t count = 0;
  uint32_t readback;

  syslog(LOG_INFO, "[M0] NuttX up on PMU Cortex-M0, %luHz core clock\n",
         (unsigned long)BOARD_MCU_FREQUENCY);

  /* Prove the DDR window before anything relies on it: store a marker through
   * the 0x60000000 view and read it back through the code window, which covers
   * the same physical bytes. Agreement means soc_con11 was programmed as
   * intended (from the FIT exsram_start property) and that the store is visible
   * without cache maintenance - the two properties rpmsg vrings will depend on.
   */

  putreg32(BOARD_SHMEM_MAGIC, BOARD_SHMEM_BASE);
  readback = getreg32(BOARD_SHMEM_CODE_VIEW);

  syslog(LOG_INFO, "[M0] shmem window: wrote 0x%08lx, code view reads "
                   "0x%08lx -> %s\n",
         (unsigned long)BOARD_SHMEM_MAGIC, (unsigned long)readback,
         readback == BOARD_SHMEM_MAGIC ? "OK" : "MISMATCH");

  /* Bring up rpmsg from here rather than board_late_initialize: that hook runs
   * on the idle thread's small stack in the flat build, and OpenAMP needs far
   * more than that.
   */

#ifdef CONFIG_RPTUN
  {
    int ret = rk3588m0_rptun_init("rpmsg", "linux");

    syslog(LOG_INFO, "[M0] rptun init %s (%d)\n",
           ret >= 0 ? "ok" : "FAILED", ret);
  }
#endif

  for (; ; )
    {
      /* sleep() returns only if the SysTick tick is being delivered, so an
       * advancing counter is evidence of the timer, not just of the core.
       */

      sleep(1);

      putreg32(++count, BOARD_HEARTBEAT_COUNT_ADDR);

      /* Mirror the counter into the shared window so Linux can watch it move at
       * physical 0x07b00004 as well - the same liveness proof, but travelling
       * through the DDR window the vrings will use.
       */

      putreg32(count, BOARD_SHMEM_BASE + 4);

      /* Keep the shared console quiet: print rarely, since Linux and the cpu_l3
       * NuttX share this UART.
       */

      if ((count % 30) == 0)
        {
          syslog(LOG_INFO, "[M0] alive, %lu s\n", (unsigned long)count);
        }
    }

  return 0;
}
