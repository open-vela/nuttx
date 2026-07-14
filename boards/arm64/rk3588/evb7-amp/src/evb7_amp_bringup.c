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
#include <nuttx/kthread.h>
#include <sys/types.h>
#include <syslog.h>
#include <unistd.h>
#include <stdint.h>
#include "evb7_amp.h"

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

#ifdef CONFIG_RPTUN
#  include <rk3588_rptun.h>
#  include <nuttx/rpmsg/rpmsg.h>
#  include <openamp/open_amp.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Direct polled write to the SHARED UART2 (0xfeb50000) for the few [AMP]
 * bring-up / rpmsg status lines. Does NOT go through the NuttX serial driver
 * or any IRQ. Borrow-only: poll LSR.THRE + write THR, never reconfigure.
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

#ifdef CONFIG_RPTUN

/* Minimal rpmsg echo server. On rpmsg-device-ready it creates a named
 * endpoint "rpmsg-echo", which sends a name-service announcement to Linux.
 * Linux (rpmsg host) then creates the channel; any bytes it sends are echoed
 * straight back. This is the first end-to-end rpmsg round-trip test.
 */

/* Name matches Linux rockchip_rpmsg_test id_table ("rpmsg-ap3-ch0") so the
 * rockchip test driver binds our channel and runs a ping-pong both ways.
 */

#define RPMSG_ECHO_NAME "rpmsg-ap3-ch0"

/* Internal OpenAMP helper: re-send a name-service announcement for an existing
 * endpoint (each call re-kicks the mailbox doorbell). Declared here because
 * it lives in rpmsg_internal.h.
 */

extern int rpmsg_send_ns_message(FAR struct rpmsg_endpoint *ept,
                                 unsigned long flags);

static struct rpmsg_endpoint     g_echo_ept;
static struct rpmsg_device      *g_echo_rdev;

static int echo_ept_cb(FAR struct rpmsg_endpoint *ept, FAR void *data,
                       size_t len, uint32_t src, FAR void *priv)
{
  /* Echo the payload straight back to the sender. Use the NON-BLOCKING send:
   * dropping an echo under momentary TX-buffer pressure just self-throttles
   * the ping-pong instead of stalling this RX context. The endpoint is set to
   * RPMSG_PRIO_RT (see echo_announce_thread) so rpmsg_virtio dispatches this
   * callback inline on the poll thread, never consuming an RX work item.
   */

  rpmsg_trysend(ept, data, len);
  return 0;
}

static void echo_ns_unbind(FAR struct rpmsg_endpoint *ept)
{
  amp_puts("[AMP] rpmsg echo unbind\n");
  rpmsg_destroy_ept(ept);
}

/* Deferred announce. The rpmsg device is created very early on the NuttX side
 * (we pin DRIVER_OK), but Linux does not probe its mailbox / fill the vrings /
 * enable the B2A interrupt until ~3.4s into its boot. Announcing before that
 * loses the single NS doorbell (proven: Linux mailbox IRQ count stayed 0) and
 * risks reading a stale vring. So wait until Linux is up, THEN create the
 * endpoint (which sends the NS announcement with a real buffer + a doorbell
 * Linux will actually receive).
 */

static int echo_announce_thread(int argc, char *argv[])
{
  int ret;
  int i;

  sleep(6);

  /* Process RX inline (synchronously in the dispatcher) instead of deferring
   * to the rpmsg work queue. Under the rockchip test's tight echo flood the
   * deferred path exhausts the fixed RX work pool (DEBUGASSERT in rpmsg.c);
   * RT priority makes rpmsg_virtio_rx_dispatch call our callback directly, so
   * no work items are consumed.
   *
   * IMPORTANT: set the priority BEFORE rpmsg_create_ept (as the mainline
   * rpmsg_ping / rpmsg socket code does). rpmsg_register_endpoint only keeps a
   * non-default priority that is already set on the endpoint; and create_ept
   * sends the NS announcement inline, so the endpoint must already be RT when
   * Linux learns our address.
   */

  rpmsg_set_priority(&g_echo_ept, RPMSG_PRIO_RT);

  ret = rpmsg_create_ept(&g_echo_ept, g_echo_rdev, RPMSG_ECHO_NAME,
                         RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
                         echo_ept_cb, echo_ns_unbind);
  if (ret < 0)
    {
      amp_puts("[AMP] rpmsg echo announce failed\n");
      return 0;
    }

  amp_puts("[AMP] rpmsg echo announced\n");

  /* Re-announce once per second. A single mailbox doorbell did not reliably
   * set Linux's B2A status (only the repeated manual kick ever did), so keep
   * re-kicking via fresh NS announcements until Linux binds the channel
   * (dest_addr learned) or we give up.
   */

  for (i = 0; i < 30 && g_echo_ept.dest_addr == RPMSG_ADDR_ANY; i++)
    {
      sleep(1);
      rpmsg_send_ns_message(&g_echo_ept, RPMSG_NS_CREATE);
      amp_puts("[AMP] rpmsg re-announce\n");
    }

  return 0;
}

static void echo_device_created(FAR struct rpmsg_device *rdev, FAR void *priv)
{
  amp_puts("[AMP] rpmsg device ready: ");
  amp_puts(rpmsg_get_cpuname(rdev));
  amp_puts("\n");

  g_echo_rdev = rdev;
  kthread_create("echo_ann", 100, 4096, echo_announce_thread, NULL);
}

#endif /* CONFIG_RPTUN */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#ifdef CONFIG_RPTUN
  /* Register the rpmsg echo server BEFORE rptun init so its device-created
   * callback fires as soon as the rpmsg device comes up.
   */

  rpmsg_register_callback(NULL, echo_device_created, NULL, NULL, NULL);

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
#endif

  UNUSED(ret);
  return OK;
}
