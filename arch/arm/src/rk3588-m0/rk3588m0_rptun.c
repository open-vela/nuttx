/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_rptun.c
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
 * rpmsg transport for the RK3588 PMU Cortex-M0.
 *
 * This core is the virtio DEVICE; Linux is the master. The doorbell is
 * mailbox0, shared with the cpu_l3 link but on its own channels: this core
 * writes B2A channel 1 (Linux "rpmsg-rx") and reads A2B channel 2 (Linux
 * "rpmsg-tx"), while channels 0 and 3 stay with cpu_l3. mailbox1 would have
 * been tidier but its registers are not accessible - see the note in
 * hardware/rk3588m0_memorymap.h.
 *
 * RX is polled rather than interrupt-driven, deliberately, for now. Reaching
 * this core with a SoC interrupt means programming INTMUX (TRM 9.3.3), whose
 * outputs occupy IRQ 16-23 of the 32 NVIC lines. The cpu_l3 link was brought up
 * on polling first and only later moved to interrupts, and the same order keeps
 * the moving parts down here: prove the transport, then optimise the wakeup.
 *
 * A note on addresses: the resource table holds this core's window view, while
 * Linux fills descriptors with physical addresses. rk3588m0_addrenv.c rebases
 * between the two, so nothing here has to translate by hand.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/signal.h>
#include <stdbool.h>

#include "arm_internal.h"
#include "chip.h"
#include "rk3588m0_rptun.h"
#include "rk3588m0_rsctable.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MBOX_BASE       RK3588M0_MAILBOX0_BASE
#define TX_CHAN         RK3588M0_MBOX_TX_CHAN
#define RX_CHAN         RK3588M0_MBOX_RX_CHAN

/* RX poll cadence. The cpu_l3 link ran happily at 5ms, and rpmsg latency here
 * is bounded by this figure.
 */

#define RPTUN_POLL_MS   5

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rk3588m0_rptun_shmem_s
{
  struct rptun_rsc_s            rsc;
};

struct rk3588m0_rptun_dev_s
{
  struct rptun_dev_s            rptun;
  rptun_callback_t              callback;
  void                         *arg;
  struct rk3588m0_rptun_shmem_s *shmem;
  char                          cpuname[RPMSG_NAME_SIZE + 1];
  char                          shmemname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *rk3588m0_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *rk3588m0_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s *
rk3588m0_rptun_get_addrenv(struct rptun_dev_s *dev);
static struct resource_table *
rk3588m0_rptun_get_resource(struct rptun_dev_s *dev);
static bool rk3588m0_rptun_is_autostart(struct rptun_dev_s *dev);
static bool rk3588m0_rptun_is_master(struct rptun_dev_s *dev);
static int rk3588m0_rptun_start(struct rptun_dev_s *dev);
static int rk3588m0_rptun_stop(struct rptun_dev_s *dev);
static int rk3588m0_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int rk3588m0_rptun_register_callback(struct rptun_dev_s *dev,
                                            rptun_callback_t callback,
                                            void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_rk3588m0_rptun_ops =
{
  .get_cpuname       = rk3588m0_rptun_get_cpuname,
  .get_firmware      = rk3588m0_rptun_get_firmware,
  .get_addrenv       = rk3588m0_rptun_get_addrenv,
  .get_resource      = rk3588m0_rptun_get_resource,
  .is_autostart      = rk3588m0_rptun_is_autostart,
  .is_master         = rk3588m0_rptun_is_master,
  .start             = rk3588m0_rptun_start,
  .stop              = rk3588m0_rptun_stop,
  .notify            = rk3588m0_rptun_notify,
  .register_callback = rk3588m0_rptun_register_callback,
};

static struct rk3588m0_rptun_dev_s g_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *rk3588m0_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct rk3588m0_rptun_dev_s *priv =
      container_of(dev, struct rk3588m0_rptun_dev_s, rptun);

  return priv->cpuname;
}

static const char *rk3588m0_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

static const struct rptun_addrenv_s *
rk3588m0_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

static struct resource_table *
rk3588m0_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct rk3588m0_rptun_dev_s *priv =
      container_of(dev, struct rk3588m0_rptun_dev_s, rptun);

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc.rsc_tbl_hdr;
    }

  /* Hand out the WRITABLE copy, never the const template: OpenAMP writes the
   * allocated notify ids back into the table. Getting this wrong faulted the
   * cpu_l3 port, where .rodata is mapped read-only.
   */

  priv->shmem =
    (struct rk3588m0_rptun_shmem_s *)rk3588m0_copy_rsc_table();

  return &priv->shmem->rsc.rsc_tbl_hdr;
}

static bool rk3588m0_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

static bool rk3588m0_rptun_is_master(struct rptun_dev_s *dev)
{
  return false;   /* Linux is the master/host */
}

static int rk3588m0_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

static int rk3588m0_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

/****************************************************************************
 * Name: rk3588m0_rptun_notify
 *
 * Description:
 *   Ring Linux. Both registers matter and so does their order: writing CMD is
 *   what latches the doorbell, while DAT carries the handshake magic Linux
 *   checks before accepting the kick. Writing CMD alone does nothing, which
 *   cost a debugging round on the cpu_l3 link.
 *
 ****************************************************************************/

static int rk3588m0_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  putreg32(RK3588M0_RPMSG_MBOX_MAGIC, MBOX_BASE + RK3588M0_MBOX_B2A_DAT(TX_CHAN));
  putreg32(RK3588M0_RPMSG_LINK_ID,    MBOX_BASE + RK3588M0_MBOX_B2A_CMD(TX_CHAN));

  return 0;
}

static int rk3588m0_rptun_register_callback(struct rptun_dev_s *dev,
                                            rptun_callback_t callback,
                                            void *arg)
{
  struct rk3588m0_rptun_dev_s *priv =
      container_of(dev, struct rk3588m0_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

/****************************************************************************
 * Name: rk3588m0_rptun_rx_poll
 *
 * Description:
 *   Watch the mailbox for a kick from Linux. Every pending A2B channel is
 *   drained and acknowledged defensively, then OpenAMP is told to look at the
 *   rings.
 *
 ****************************************************************************/

static int rk3588m0_rptun_rx_poll(int argc, char *argv[])
{
  struct rk3588m0_rptun_dev_s *dev = &g_rptun_dev;

  for (; ; )
    {
      uint32_t status;

      /* Keep our RX channel enabled. A2B_INTEN is shared with the cpu_l3 link,
       * which read-modify-writes it from its own keepalive thread; if that
       * happens to straddle our own update, our bit is lost. Re-asserting it
       * here is idempotent and costs one register write per poll.
       */

      putreg32(getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_INTEN) | (1u << RX_CHAN),
               MBOX_BASE + RK3588M0_MBOX_A2B_INTEN);

      status = getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_STATUS);

      if (status != 0u)
        {
          unsigned int ch;

          for (ch = 0; ch < 4; ch++)
            {
              if (status & (1u << ch))
                {
                  /* Reading CMD/DAT and writing the bit back clears it */

                  getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_CMD(ch));
                  getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_DAT(ch));
                  putreg32(1u << ch, MBOX_BASE + RK3588M0_MBOX_A2B_STATUS);
                }
            }

          if (dev->callback != NULL)
            {
              dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
            }
        }

      nxsig_usleep(RPTUN_POLL_MS * 1000);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3588m0_rptun_init
 ****************************************************************************/

int rk3588m0_rptun_init(const char *shmemname, const char *cpuname)
{
  struct rk3588m0_rptun_dev_s *dev = &g_rptun_dev;
  int ret;

  /* Clear stale vring state. The rings live in reserved DDR that survives a
   * soft reset, and this core pins VIRTIO_CONFIG_STATUS_DRIVER_OK and starts
   * consuming long before Linux initialises the rings. Without this it reads a
   * stale avail->idx and replays orphaned descriptors from the previous run,
   * which on the cpu_l3 link underflowed the OpenAMP held-buffer counter and
   * exhausted the RX work pool. Safe on a cold boot: Linux re-initialises its
   * rings much later.
   */

  memset((void *)RK3588M0_VRING0, 0, RK3588M0_VRING_SIZE);
  memset((void *)RK3588M0_VRING1, 0, RK3588M0_VRING_SIZE);

  /* Let Linux's kicks raise the mailbox status bit we poll for */

  putreg32(getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_INTEN) | (1u << RX_CHAN),
           MBOX_BASE + RK3588M0_MBOX_A2B_INTEN);

  dev->rptun.ops = &g_rk3588m0_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      rpmsgerr("ERROR: rptun_initialize failed %d\n", ret);
      return ret;
    }

  ret = kthread_create("m0_rptun_rx", CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE, rk3588m0_rptun_rx_poll, NULL);
  if (ret < 0)
    {
      rpmsgerr("ERROR: rx poll thread failed %d\n", ret);
      return ret;
    }

  return 0;
}
