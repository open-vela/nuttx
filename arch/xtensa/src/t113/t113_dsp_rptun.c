/****************************************************************************
 * arch/xtensa/src/t113/t113_dsp_rptun.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <debug.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include <nuttx/rptun/rptun.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <sched.h>

#include "t113_msgbox.h"
#include "t113_rsctable.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The DSP is the slave/remote; the master (AP) names this core "dsp".  The
 * rptun device node on the DSP is /dev/rptun/ap (named after the peer).
 */

#define T113_DSP_RPTUN_CPUNAME  "ap"

/* RX/TX ring-buffer size for the rpmsg-tty serial channel (bytes each).
 * These buffers are allocated from the kernel heap, not from the rpmsg
 * vring buffer pool.
 */

#define T113_RPMSG_UART_BUFSIZE  4096

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_dsp_rptun_dev_s
{
  struct rptun_dev_s rptun;
  rptun_callback_t   callback;
  void              *arg;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *t113_rptun_get_cpuname(struct rptun_dev_s *dev);
static struct resource_table *
                   t113_rptun_get_resource(struct rptun_dev_s *dev);
static bool t113_rptun_is_autostart(struct rptun_dev_s *dev);
static bool t113_rptun_is_master(struct rptun_dev_s *dev);
static int  t113_rptun_start(struct rptun_dev_s *dev);
static int  t113_rptun_stop(struct rptun_dev_s *dev);
static int  t113_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int  t113_rptun_register_callback(struct rptun_dev_s *dev,
                                         rptun_callback_t callback,
                                         void *arg);
static void t113_rptun_msgbox_rx(void *arg, uint32_t word);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_t113_dsp_rptun_ops =
{
  .get_cpuname       = t113_rptun_get_cpuname,
  .get_firmware      = NULL,    /* slave never loads an ELF */
  .get_addrenv       = NULL,
  .get_resource      = t113_rptun_get_resource,
  .is_autostart      = t113_rptun_is_autostart,
  .is_master         = t113_rptun_is_master,
  .start             = t113_rptun_start,
  .stop              = t113_rptun_stop,
  .notify            = t113_rptun_notify,
  .register_callback = t113_rptun_register_callback,
};

static struct t113_dsp_rptun_dev_s g_t113_dsp_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *t113_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return T113_DSP_RPTUN_CPUNAME;
}

static struct resource_table *
t113_rptun_get_resource(struct rptun_dev_s *dev)
{
  UNUSED(dev);

  /* The table lives in our own .resource_table section (shared DDR); the
   * master already populated/patched the vring DAs when it loaded the ELF.
   */

  return t113_rsctable_get();
}

static bool t113_rptun_is_autostart(struct rptun_dev_s *dev)
{
  UNUSED(dev);

  /* The slave comes up with the DSP and waits for the master's kicks. */

  return true;
}

static bool t113_rptun_is_master(struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return false;
}

static int t113_rptun_start(struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return 0;   /* nothing to release on the slave */
}

static int t113_rptun_stop(struct rptun_dev_s *dev)
{
  UNUSED(dev);
  return 0;
}

static int t113_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  UNUSED(dev);

  /* Kick the AP: vring notifyid is the doorbell payload. */

  t113_msgbox_send(vqid);
  return 0;
}

static int t113_rptun_register_callback(struct rptun_dev_s *dev,
                                        rptun_callback_t callback,
                                        void *arg)
{
  struct t113_dsp_rptun_dev_s *priv =
    (struct t113_dsp_rptun_dev_s *)dev;

  priv->callback = callback;
  priv->arg      = arg;

  t113_msgbox_attach(callback ? t113_rptun_msgbox_rx : NULL, priv);
  return 0;
}

static void t113_rptun_msgbox_rx(void *arg, uint32_t word)
{
  struct t113_dsp_rptun_dev_s *priv = (struct t113_dsp_rptun_dev_s *)arg;

  if (priv->callback != NULL)
    {
      priv->callback(priv->arg, word);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_dsp_rptun_init
 *
 * Description:
 *   Register the DSP-side rptun slave (/dev/rptun/ap) and, since
 *   is_autostart is true, start the remoteproc slave so it waits for the
 *   AP master's vring kicks over the MSGBOX doorbell.  Must run after the
 *   scheduler and IRQs are up (board late init).
 *
 ****************************************************************************/

int t113_dsp_rptun_init(void)
{
  struct t113_dsp_rptun_dev_s *dev = &g_t113_dsp_rptun_dev;

  t113_msgbox_init();

  dev->rptun.ops = &g_t113_dsp_rptun_ops;
  return rptun_initialize(&dev->rptun);
}

#ifdef CONFIG_RPMSG_UART

/****************************************************************************
 * Name: t113_dsp_rpmsg_nsh
 *
 * Description:
 *   Secondary NSH session bound to the rpmsg-uart channel.  /dev/ttyDSP was
 *   registered with isconsole=false (the DSP keeps its native UART2 console
 *   in parallel -- dual console), so the serial framework left the termios
 *   flags at 0: no CR/LF translation and no echo, which makes the channel
 *   unusable from `cu` (ladder-indented prompts, Enter never accepted).
 *   Force the canonical-console flags here, then redirect stdio and run NSH
 *   so the AP reaches the DSP shell with `cu -l /dev/ttyDSP`.
 *
 ****************************************************************************/

extern int nsh_consolemain(int argc, FAR char *argv[]);

static int t113_dsp_rpmsg_nsh(int argc, FAR char *argv[])
{
  struct termios tio;
  int fd = -1;
  int i;

  /* Open /dev/ttyDSP, which rpmsg_serialinit() registered just before
   * spawning this task.  In the normal path it is already present, so the
   * first open() succeeds; the bounded retry only covers a transient
   * registration race.  Opening before the AP master has bound the
   * "rpmsg-ttyDSP" endpoint is fine -- the first read simply blocks until
   * traffic arrives, which is what a login shell wants.  Give up after ~5 s
   * rather than spin forever and leak this task slot.
   */

  for (i = 0; i < 50; i++)
    {
      fd = open("/dev/ttyDSP", O_RDWR);
      if (fd >= 0)
        {
          break;
        }

      usleep(100000);
    }

  if (fd < 0)
    {
      serr("ERROR: open /dev/ttyDSP failed: %d\n", errno);
      return EXIT_FAILURE;
    }

  if (tcgetattr(fd, &tio) == 0)
    {
      tio.c_iflag |= ICRNL;
      tio.c_oflag |= OPOST | ONLCR;
      tio.c_lflag |= ECHO | ICANON | ISIG;
      tcsetattr(fd, TCSANOW, &tio);
    }

  dup2(fd, 0);
  dup2(fd, 1);
  dup2(fd, 2);
  if (fd > 2)
    {
      close(fd);
    }

  return nsh_consolemain(0, NULL);
}

/****************************************************************************
 * Name: rpmsg_serialinit
 *
 * Description:
 *   Called by drivers_initialize() when CONFIG_RPMSG_UART is set.  The DSP
 *   registers /dev/ttyDSP (devname "DSP", peer "ap") with isconsole=false so
 *   its native UART2 console stays live, and spawns a second NSH session on
 *   that channel so the AP can `cu -l /dev/ttyDSP` into the DSP shell.
 *
 ****************************************************************************/

void rpmsg_serialinit(void)
{
  int pid;

  uart_rpmsg_init("ap", "DSP", T113_RPMSG_UART_BUFSIZE, false);

  pid = task_create("dsp_rpmsg_nsh", CONFIG_INIT_PRIORITY,
                    CONFIG_INIT_STACKSIZE, t113_dsp_rpmsg_nsh, NULL);
  if (pid < 0)
    {
      serr("ERROR: spawn dsp_rpmsg_nsh failed: %d\n", errno);
    }
}

#endif /* CONFIG_RPMSG_UART */
