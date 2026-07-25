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
#include <fcntl.h>
#include <sys/mount.h>
#include <sched.h>
#include <unistd.h>
#include <debug.h>

#include <nuttx/kthread.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#ifdef CONFIG_RPTUN
#  include "rk3588m0_rptun.h"
#endif

#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

#ifdef CONFIG_SYSTEM_NSH
int nsh_main(int argc, char *argv[]);
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int m0_heartbeat(int argc, char *argv[]);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: m0_main
 *
 * Description:
 *   Initial task on the RK3588 PMU Cortex-M0. It checks the shared-memory
 *   window, brings up the rpmsg link to Linux, and then becomes the shell.
 *
 *   The shell talks over rpmsg rather than the UART: the physical UART2 is only
 *   borrowed for output, so taking it for input would mean fighting Linux's
 *   fiq-debugger for the port. This mirrors what already works on cpu_l3.
 *
 *   Startup order matters. Both rpmsg bring-up steps run here rather than in
 *   board_late_initialize, which executes on the idle thread's stack in the flat
 *   build - OpenAMP overflows it, and the symptom is misleading: the boot
 *   progress characters still appear because they bypass syslog, but no syslog
 *   line is ever printed.
 *
 ****************************************************************************/

int m0_main(int argc, char *argv[])
{
  uint32_t readback;
  int ret;

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

#ifdef CONFIG_RPMSG_UART
  /* Register the virtual serial port before the tunnel comes up, so the
   * device-created callback can arm its endpoint as soon as rpmsg is ready.
   *
   * isconsole = false, as on cpu_l3: asking for /dev/console fails with EEXIST
   * because the arch serial layer already owns it, and uart_rpmsg unwinds by
   * unregistering its rpmsg callback - which silently prevents the channel from
   * ever being announced. nsh is bound to this port through CONFIG_NSH_ALTCONDEV
   * instead.
   */

  {
    ret = uart_rpmsg_init("linux", "m0", 4096, false);

    syslog(LOG_INFO, "[M0] uart_rpmsg init %s (%d)\n",
           ret >= 0 ? "ok" : "FAILED", ret);
  }
#endif

#ifdef CONFIG_RPTUN
  {
    ret = rk3588m0_rptun_init("rpmsg", "linux");

    syslog(LOG_INFO, "[M0] rptun init %s (%d)\n",
           ret >= 0 ? "ok" : "FAILED", ret);
  }
#endif

#ifdef CONFIG_SYSTEM_NSH
  /* Run the heartbeat in the background and hand this task over to nsh, which
   * takes its stdio from the rpmsg port via CONFIG_NSH_ALTCONDEV.
   */

  if (kthread_create("m0_hb", SCHED_PRIORITY_DEFAULT, 2048,
                     m0_heartbeat, NULL) < 0)
    {
      syslog(LOG_ERR, "[M0] failed to start heartbeat\n");
    }

  /* Mount procfs, which several nsh commands read through rather than through a
   * syscall: free needs /proc/meminfo and ps needs /proc/<pid>. Nothing in
   * nshlib mounts it - CONFIG_NSH_PROC_MOUNTPOINT only says where to look - and
   * this board has no nsh arch-init hook, so it has to happen here.
   */

  ret = mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "[M0] mount procfs failed (%d)\n", ret);
    }

  /* Populate stdin/stdout/stderr with the rpmsg port before handing over to
   * nsh.
   *
   * This is not merely tidy - nsh does not work without it. There is no
   * /dev/console on this core (CONFIG_DEV_CONSOLE is unset, as UART2 is
   * write-only through syslog and is shared with Linux), so descriptors 0-2 are
   * still free when this task starts. nshlib's alternate-console path assumes
   * the opposite: for stderr and then stdout it opens the device, dup2()s it
   * into place, and closes the descriptor the open returned. With 0-2 free the
   * stdout open itself returns 1, so the dup2 is a no-op and the close then
   * shuts descriptor 1 - nsh ends up running with no stdout at all. Input keeps
   * working, so the shell looks alive from the rpmsg side while every prompt and
   * every command result is dropped by write() with EBADF, never reaching the
   * serial driver.
   *
   * Opening the port here first pushes nsh's own opens up to descriptor 3 and
   * above, where its dup2/close sequence behaves as intended.
   */

  {
    int fd = open(CONFIG_NSH_ALTSTDIN, O_RDWR);

    syslog(LOG_INFO, "[M0] open %s -> %d\n", CONFIG_NSH_ALTSTDIN, fd);

    if (fd < 0)
      {
        syslog(LOG_ERR, "[M0] no console device - nsh would be mute\n");
      }
    else
      {
        if (fd != 0)
          {
            dup2(fd, 0);
          }

        dup2(fd, 1);
        dup2(fd, 2);

        if (fd > 2)
          {
            close(fd);
          }
      }
  }

  ret = nsh_main(argc, argv);

  /* Reaching here means nsh gave up; without this the task would just vanish */

  syslog(LOG_ERR, "[M0] nsh_main returned %d\n", ret);
  return ret;
#else
  return m0_heartbeat(argc, argv);
#endif
}

/****************************************************************************
 * Name: m0_heartbeat
 *
 * Description:
 *   Liveness loop. Driven by sleep() rather than a delay loop, so an advancing
 *   counter is evidence that the tick is really being delivered and not merely
 *   that the core is spinning.
 *
 *   Observable from Linux:
 *     busybox devmem 0x07ae0000 32   -> 0x414D5030 ("AMP0"), set at boot
 *     busybox devmem 0x07ae0004 32   -> increments once per second
 *
 ****************************************************************************/

static int m0_heartbeat(int argc, char *argv[])
{
  uint32_t count = 0;

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
