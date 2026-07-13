/****************************************************************************
 * arch/arm64/src/rk3588/rk3588_rptun.c
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

#include <debug.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/nuttx.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kthread.h>
#include <nuttx/signal.h>
#include <nuttx/rptun/rptun.h>

#include "arm64_internal.h"
#include "arm64_arch.h"
#include "rk3588_rptun.h"
#include "rk3588_rsctable.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* rockchip mailbox0 (V1 regs). cpu_l3 = B side, Linux = A side.
 * TX (cpu_l3 -> Linux): write B2A_DAT then B2A_CMD on channel 0.
 * RX (Linux -> cpu_l3): A2B interrupt (amp-irqs routes SPI 100 to cpu_l3).
 */

#define MBOX0_BASE            0xfec60000ul
#define MBOX_A2B_INTEN        0x00
#define MBOX_A2B_STATUS       0x04
#define MBOX_A2B_CMD(x)       (0x08 + (x) * 8)
#define MBOX_A2B_DAT(x)       (0x0c + (x) * 8)
#define MBOX_B2A_CMD(x)       (0x30 + (x) * 8)
#define MBOX_B2A_DAT(x)       (0x34 + (x) * 8)

#define RPMSG_MBOX_MAGIC      0x524d5347u  /* "RMSG", from rockchip_rpmsg.h */
#define RPMSG_LINK_ID         0x03u        /* rockchip,link-id (m=0, r=3) */

/* rk3588-amp.dtsi: mboxes = <&mailbox0 0 &mailbox0 3>, names rpmsg-rx,rpmsg-tx.
 * TX cpu_l3->Linux uses B2A channel 0 (Linux's "rpmsg-rx" = mailbox0 ch0).
 * RX Linux->cpu_l3 arrives on A2B channel 3 (Linux's "rpmsg-tx" = mailbox0 ch3)
 * and the amp routes that A2B interrupt (SPI 100) to cpu_l3.
 */

#define RPMSG_TX_CHAN         0            /* B2A: cpu_l3 -> Linux (rpmsg-rx) */
#define RPMSG_RX_CHAN         3            /* A2B: Linux -> cpu_l3 (rpmsg-tx) */

/* A2B mailbox interrupt to cpu_l3: GIC SPI 100 -> NuttX irq 100 + 32 */

#define RK3588_MBOX_A2B_IRQ   (100 + 32)

#define getreg32b(a)          (*(volatile uint32_t *)(a))
#define putreg32b(v, a)       (*(volatile uint32_t *)(a) = (v))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rk3588_rptun_shmem_s
{
  struct rptun_rsc_s          rsc;
};

struct rk3588_rptun_dev_s
{
  struct rptun_dev_s          rptun;
  rptun_callback_t            callback;
  void                       *arg;
  struct rk3588_rptun_shmem_s *shmem;
  char                        cpuname[RPMSG_NAME_SIZE + 1];
  char                        shmemname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *rk3588_rptun_get_cpuname(struct rptun_dev_s *dev);
static const char *rk3588_rptun_get_firmware(struct rptun_dev_s *dev);
static const struct rptun_addrenv_s *
rk3588_rptun_get_addrenv(struct rptun_dev_s *dev);
static struct resource_table *
rk3588_rptun_get_resource(struct rptun_dev_s *dev);
static bool rk3588_rptun_is_autostart(struct rptun_dev_s *dev);
static bool rk3588_rptun_is_master(struct rptun_dev_s *dev);
static int rk3588_rptun_start(struct rptun_dev_s *dev);
static int rk3588_rptun_stop(struct rptun_dev_s *dev);
static int rk3588_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int rk3588_rptun_register_callback(struct rptun_dev_s *dev,
                                          rptun_callback_t callback,
                                          void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rptun_ops_s g_rk3588_rptun_ops =
{
  .get_cpuname       = rk3588_rptun_get_cpuname,
  .get_firmware      = rk3588_rptun_get_firmware,
  .get_addrenv       = rk3588_rptun_get_addrenv,
  .get_resource      = rk3588_rptun_get_resource,
  .is_autostart      = rk3588_rptun_is_autostart,
  .is_master         = rk3588_rptun_is_master,
  .start             = rk3588_rptun_start,
  .stop              = rk3588_rptun_stop,
  .notify            = rk3588_rptun_notify,
  .register_callback = rk3588_rptun_register_callback,
};

static struct rk3588_rptun_dev_s g_rptun_dev;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *rk3588_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct rk3588_rptun_dev_s *priv =
      container_of(dev, struct rk3588_rptun_dev_s, rptun);

  return priv->cpuname;
}

static const char *rk3588_rptun_get_firmware(struct rptun_dev_s *dev)
{
  return NULL;
}

static const struct rptun_addrenv_s *
rk3588_rptun_get_addrenv(struct rptun_dev_s *dev)
{
  return NULL;
}

static struct resource_table *
rk3588_rptun_get_resource(struct rptun_dev_s *dev)
{
  struct rk3588_rptun_dev_s *priv =
      container_of(dev, struct rk3588_rptun_dev_s, rptun);

  if (priv->shmem != NULL)
    {
      return &priv->shmem->rsc.rsc_tbl_hdr;
    }

  /* NuttX is the remote; publish our resource table. Linux uses fixed vring
   * addresses from its dts, so this table mainly configures our own OpenAMP.
   * IMPORTANT: return the WRITABLE copy (in the AMP_RPMSG carveout), not the
   * const template in .rodata -- OpenAMP writes notify ids back into it.
   */

  priv->shmem = (struct rk3588_rptun_shmem_s *)rk3588_copy_rsc_table();

  return &priv->shmem->rsc.rsc_tbl_hdr;
}

static bool rk3588_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

static bool rk3588_rptun_is_master(struct rptun_dev_s *dev)
{
  return false;   /* Linux is the master/host */
}

static int rk3588_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

static int rk3588_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

/* Ring Linux: write B2A_DAT (magic) then B2A_CMD (link_id) on channel 0.
 * (Verified: Linux rk_rpmsg_rx_callback accepts this and kicks vring0.)
 */

static int rk3588_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  putreg32b(RPMSG_MBOX_MAGIC, MBOX0_BASE + MBOX_B2A_DAT(RPMSG_TX_CHAN));
  putreg32b(RPMSG_LINK_ID,    MBOX0_BASE + MBOX_B2A_CMD(RPMSG_TX_CHAN));

  return 0;
}

static int rk3588_rptun_register_callback(struct rptun_dev_s *dev,
                                          rptun_callback_t callback,
                                          void *arg)
{
  struct rk3588_rptun_dev_s *priv =
      container_of(dev, struct rk3588_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;

  return 0;
}

/* A2B mailbox interrupt: Linux kicked us. Read/clear status, notify OpenAMP. */

static int rk3588_rptun_isr(int irq, void *context, void *arg)
{
  struct rk3588_rptun_dev_s *dev = &g_rptun_dev;
  uint32_t status = getreg32b(MBOX0_BASE + MBOX_A2B_STATUS);

  /* Handle every pending A2B channel (Linux uses ch3 for rpmsg-tx, but stay
   * robust to any channel). Clear each serviced bit and notify OpenAMP.
   */

  if (status != 0u)
    {
      unsigned int ch;

      for (ch = 0; ch < 4; ch++)
        {
          if (status & (1u << ch))
            {
              (void)getreg32b(MBOX0_BASE + MBOX_A2B_CMD(ch));
              (void)getreg32b(MBOX0_BASE + MBOX_A2B_DAT(ch));
              putreg32b(1u << ch, MBOX0_BASE + MBOX_A2B_STATUS);
            }
        }

      if (dev->callback != NULL)
        {
          dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
        }
    }

  return OK;
}

/* A2B RX poll fallback. The GIC AMP routing of the mailbox A2B SPI (100) to
 * cpu_l3 does not deliver the interrupt here (Linux devmem shows A2B_STATUS
 * stuck with the pending bit set and no ISR), likely due to GIC group/security
 * config that needs BL31 cooperation. Since A2B_STATUS is set reliably by
 * Linux, poll it and drive the same path the ISR would.
 */

/* A2B RX via polling.
 *
 * The mailbox A2B interrupt (SPI 100) cannot be delivered to NuttX here: Linux
 * devmem shows GICD_IROUTER(100)=0x500 (routed to cpu5, not our cpu_l3, and it
 * reverts to 0x500 even after we write 0x300) and GICD_IGROUPR bit for SPI 100
 * = 0 (Group0/secure). rockchip's AMP interrupt delivery to a remote core is
 * arranged by BL31 (EL3) with specific GIC group/security state that our
 * NS-EL1 NuttX cannot reprogram. So poll A2B_STATUS instead -- cpu_l3 is
 * dedicated to NuttX, so a 5ms register poll costs effectively nothing.
 */

static int rk3588_rptun_rx_poll(int argc, char *argv[])
{
  struct rk3588_rptun_dev_s *dev = &g_rptun_dev;

  for (; ; )
    {
      uint32_t status = getreg32b(MBOX0_BASE + MBOX_A2B_STATUS);

      if (status != 0u)
        {
          unsigned int ch;

          for (ch = 0; ch < 4; ch++)
            {
              if (status & (1u << ch))
                {
                  (void)getreg32b(MBOX0_BASE + MBOX_A2B_CMD(ch));
                  (void)getreg32b(MBOX0_BASE + MBOX_A2B_DAT(ch));
                  putreg32b(1u << ch, MBOX0_BASE + MBOX_A2B_STATUS);
                }
            }

          if (dev->callback != NULL)
            {
              dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
            }
        }

      nxsig_usleep(5000);
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rk3588_rptun_init(const char *shmemname, const char *cpuname)
{
  struct rk3588_rptun_dev_s *dev = &g_rptun_dev;
  int ret;

  /* Wipe stale vring state left over in the reserved DDR from a previous
   * NuttX run. Layout (see rk3588_rsctable.c): vring0 @0x07c00000 and
   * vring1 @0x07c08000, each occupying a 0x8000 slot; the resource table
   * lives at 0x07c0f000 (inside vring1's slot, past the actual ring).
   *
   * On a cold AMP boot NuttX comes up and pins VIRTIO_CONFIG_STATUS_DRIVER_OK
   * long before Linux initialises the vrings, so the device side would read a
   * stale avail->idx and replay dozens of orphaned descriptors (observed: the
   * same name-service buffer dispatched 20+ times), which underflows the
   * OpenAMP buffer held-counter (assert in rpmsg_virtio.c) and exhausts the RX
   * work pool (assert in rpmsg.c). Clearing the ring's desc/avail/used area
   * makes both sides start from idx 0. Safe on a cold boot because Linux
   * re-initialises its vrings ~15s later, well after this runs.
   */

  memset((void *)0x07c00000ul, 0, 0x8000);         /* vring0 full slot     */
  memset((void *)0x07c08000ul, 0, 0x0f000 - 0x8000); /* vring1 up to rsctbl */

  /* Enable the A2B RX interrupt (Linux -> cpu_l3 doorbell). Linux sends on
   * channel 3 (rpmsg-tx); also enable channel 0 defensively.
   */

  putreg32b(getreg32b(MBOX0_BASE + MBOX_A2B_INTEN) |
            (1u << RPMSG_RX_CHAN) | (1u << RPMSG_TX_CHAN),
            MBOX0_BASE + MBOX_A2B_INTEN);

  ret = irq_attach(RK3588_MBOX_A2B_IRQ, rk3588_rptun_isr, dev);
  if (ret < 0)
    {
      rpmsgerr("ERROR: irq_attach failed %d\n", ret);
      return ret;
    }

  /* NOTE: the actual GIC enable + routing is deferred to
   * rk3588_rptun_irq_setup() (spawned below) because Linux's later GIC
   * distributor init would otherwise wipe an early enable.
   */

  dev->rptun.ops = &g_rk3588_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      rpmsgerr("ERROR: rptun_initialize failed %d\n", ret);
      return ret;
    }

  /* Start the A2B RX poll thread (mailbox interrupt cannot reach us; see
   * rk3588_rptun_rx_poll comment).
   */

  kthread_create("rptun_rx", 200, 2048, rk3588_rptun_rx_poll, NULL);

  return ret;
}
