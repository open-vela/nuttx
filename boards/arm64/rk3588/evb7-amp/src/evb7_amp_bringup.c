/****************************************************************************
 * boards/arm64/rk3588/evb7-amp/src/evb7_amp_bringup.c
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
#include <sys/types.h>
#include <syslog.h>
#include <stdint.h>
#include "evb7_amp.h"
#ifdef CONFIG_RPTUN
#  include "evb7_amp_shm.h"
#endif

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

#ifdef CONFIG_RPTUN
#  include <rk3588_rptun.h>
#endif

#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Direct polled write to the SHARED UART2 (0xfeb50000) for the few [AMP]
 * bring-up status lines. Does NOT go through the NuttX serial driver or any
 * IRQ. Borrow-only: poll LSR.THRE + write THR, never reconfigure.
 */

#define UART2_THR (*(volatile uint32_t *)0xfeb50000UL)
#define UART2_LSR (*(volatile uint32_t *)0xfeb50014UL)
#define UART_LSR_THRE (1u << 5)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void amp_putc(char c)
{
  unsigned int spin = 0;
  while (!(UART2_LSR & UART_LSR_THRE) && ++spin < 200000)
    {
    }

  UART2_THR = (uint32_t)c;
}

static void amp_puts(const char *s)
{
  while (*s)
    {
      if (*s == '\n')
        {
          amp_putc('\r');
        }

      amp_putc(*s++);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
/****************************************************************************
 * Name: rpmsg_serialinit
 *
 * Description:
 *   drivers_initialize() calls this unconditionally when CONFIG_RPMSG_UART is
 *   set, but that phase is too early: the rptun rpmsg device is not up yet and
 *   the uart_rpmsg device-created callback would never fire. So the real
 *   uart_rpmsg_init() is done in evb7_amp_bringup() (board_late_initialize,
 *   same phase where rk3588_rptun_init creates the device). This stub only
 *   satisfies the unconditional call.
 *
 ****************************************************************************/

void rpmsg_serialinit(void)
{
}
#endif

/****************************************************************************
 * Name: evb7_amp_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int evb7_amp_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

  amp_puts("[AMP] hello from NuttX on cpu_l3 (RK3588 EVB7 V11 AMP)\n");
  amp_puts("[AMP] build=nsh-over-rpmsg v9-doorbell\n");

#ifdef CONFIG_RPMSG_UART
  /* Register the rpmsg virtual UART (nsh-over-rpmsg). Registered BEFORE
   * rk3588_rptun_init so its device-created callback fires when the rpmsg
   * device comes up, creating the "rpmsg-ttyproxy" endpoint.
   *
   *   - cpuname "linux" MUST match the rptun remote name below.
   *   - devname "proxy" -> /dev/ttyproxy + endpoint "rpmsg-ttyproxy"; the
   *     Linux side binds it via the rpmsg_nsh_tty driver.
   *   - isconsole=false: the arch serial layer already owns /dev/console, so
   *     claiming it would fail -EEXIST and tear down this driver's rpmsg
   *     callback. nsh is bound to /dev/ttyproxy via CONFIG_NSH_ALTCONDEV.
   */

  uart_rpmsg_init("linux", "proxy", 4096, false);
#endif

#ifdef CONFIG_RPTUN
  /* Bring up the OpenAMP/rptun tunnel to the Linux master over mailbox0.
   * shmemname "rpmsg", remote (master) cpuname "linux".
   */

  ret = rk3588_rptun_init("rpmsg", "linux");
  if (ret < 0)
    {
      amp_puts("[AMP] rptun init failed\n");
    }
  else
    {
      amp_puts("[AMP] rptun init ok\n");
    }

#ifdef CONFIG_INPUT_TOUCHSCREEN
  /* /dev/input0 before the endpoint that feeds it, so no forwarded event can
   * arrive with nowhere to go. The driver guards against that anyway, but the
   * ordering makes the guard a backstop rather than the mechanism.
   */

  evb7_amp_touch_init();
#endif

  /* Shared frame area towards Linux. Registered after the tunnel exists, and
   * with the same remote name, so its device-created callback fires for the
   * same rpmsg device the console channel uses.
   */

  evb7_amp_shm_init("linux");

#ifdef CONFIG_VIDEO_FB
  /* /dev/fb0 over the shared area. After the control block exists, since the
   * framebuffer's flush path publishes through it.
   */

  evb7_amp_fb_init();
#endif
#endif

  UNUSED(ret);
  return OK;
}
