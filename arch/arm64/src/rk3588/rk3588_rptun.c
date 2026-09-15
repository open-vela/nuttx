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
#include <nuttx/semaphore.h>
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

/* A2B mailbox RX interrupt for cpu_l3.
 *
 * IMPORTANT: rockchip's rk3588-amp.dtsi amp-irqs list uses GIC INTIDs directly
 * (verified: its UART5 entry is 368 = GIC_SPI 336 + 32). Its MAILBOX entry is
 * 100, i.e. GIC INTID 100 = SPI 68 = mailbox0 A2B channel 3 (the Linux-side
 * B2A ch0-3 are SPI 61-64; the A2B ch0-3 follow as SPI 65-68). Linux's
 * gic_dist_init() routes INTID 100 to cpu_l3 (aff 0x300) via that amp-irqs
 * entry. So the RX doorbell INTID is 100.
 *
 * INTID 100 is a Non-secure Group-1 interrupt and IS delivered to NS-EL1 on
 * cpu_l3 (verified on target). It is NOT a GIC group/secure problem. The only
 * catch is that Linux's gic_dist_init() (~15s) disables all SPIs and clears
 * A2B_INTEN, and never re-enables our amp irq; NuttX must re-arm it (see
 * rk3588_rptun_irq_keepalive). With that, RX is fully interrupt-driven.
 */

#define RK3588_MBOX_A2B_IRQ   100

/* GICv3 distributor (RK3588) — used to re-point our RX INTID's IROUTER back to
 * cpu_l3 after up_enable_irq(). NuttX's generic arm64_gic_irq_enable() writes
 * IROUTER = up_cpu_index() (0 in this UP build => cpu0/Linux), which would
 * steal the mailbox IRQ away from us; we rewrite it to cpu_l3's affinity.
 */

#define RK3588_GICD_BASE       0xfe600000ul
#define RK3588_GICD_IROUTER(n) (RK3588_GICD_BASE + 0x6000ul + (n) * 8ul)
#define RK3588_CPU_L3_AFF      0x300ul   /* MPIDR aff: cluster3, cpu0 = cpu_l3 */
#define putreg64b(v, a)        (*(volatile uint64_t *)(a) = (v))

#define getreg32b(a)          (*(volatile uint32_t *)(a))
#define putreg32b(v, a)       (*(volatile uint32_t *)(a) = (v))

/* GICD_ISENABLER for the RX INTID. INTID 100 -> ISENABLER3 (covers 96..127),
 * bit 4. Used by the keepalive thread to re-arm the interrupt after Linux's
 * gic_dist_init disables all SPIs.
 */

#define RK3588_GICD_ISENABLER(n) (RK3588_GICD_BASE + 0x100ul + ((n) / 32) * 4ul)
#define RK3588_GICD_ISENABLER_BIT(n) (1u << ((n) % 32))

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
  sem_t                       rxsem;
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

/* Re-arm the mailbox RX interrupt: (re-)enable the GIC INTID, keep it routed
 * to cpu_l3, and (re-)enable the mailbox A2B RX channel interrupt. Idempotent
 * and cheap. Used by the bounded startup keepalive (to survive Linux's
 * gic_dist_init) and opportunistically from the TX path (to cover a later
 * Linux GIC re-init such as suspend/resume, whenever we have TX traffic).
 */

static void rk3588_rptun_irq_rearm(void)
{
  putreg32b(RK3588_GICD_ISENABLER_BIT(RK3588_MBOX_A2B_IRQ),
            RK3588_GICD_ISENABLER(RK3588_MBOX_A2B_IRQ));
  putreg64b(RK3588_CPU_L3_AFF, RK3588_GICD_IROUTER(RK3588_MBOX_A2B_IRQ));
  putreg32b(getreg32b(MBOX0_BASE + MBOX_A2B_INTEN) | (1u << RPMSG_RX_CHAN),
            MBOX0_BASE + MBOX_A2B_INTEN);
}

/* Ring Linux: write B2A_DAT (magic) then B2A_CMD (link_id) on channel 0.
 * (Verified: Linux rk_rpmsg_rx_callback accepts this and kicks vring0.)
 */

static int rk3588_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  /* TODO(suspend/resume): opportunistic RX-interrupt backstop.
   *
   * Uncommenting the rk3588_rptun_irq_rearm() below re-arms our RX interrupt
   * on every TX, so it survives a LATER Linux GIC re-init (Linux disables all
   * SPIs and clears mailbox A2B_INTEN whenever it re-runs gic_dist_init).
   *
   * Left DISABLED on purpose: this board has NO Linux suspend/resume or CPU
   * hotplug scenario, and Linux's ONE-TIME gic_dist_init at boot (~15s) is
   * already fully covered by the bounded startup keepalive
   * (rk3588_rptun_irq_keepalive). So the extra per-TX register writes are
   * unnecessary today.
   *
   * Re-enable ONLY IF Linux S3 suspend/resume (or CPU hotplug that re-inits
   * the GIC) is introduced later -- otherwise the RX interrupt would go dead
   * after the first resume with no one to re-arm it.
   */

  /* rk3588_rptun_irq_rearm(); */

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

/* Acknowledge a doorbell on our channel, if one is pending. Returns true when
 * there was something to acknowledge.
 *
 * Only our own channel is touched. The four A2B channels of mailbox0 share one
 * status register, and channels 1 and 2 carry the PMU M0 link's traffic:
 * clearing those would consume a doorbell meant for the M0, whose handler would
 * then find nothing pending and never tell its OpenAMP to look at the rings.
 * (An earlier version cleared all four "defensively", which is exactly how that
 * theft happened.)
 */

static bool rk3588_rptun_ack(void)
{
  if ((getreg32b(MBOX0_BASE + MBOX_A2B_STATUS) & (1u << RPMSG_RX_CHAN)) == 0u)
    {
      return false;
    }

  /* Reading CMD and DAT and writing the bit back is what clears it. The
   * doorbell is level-triggered, so this has to happen before the handler
   * returns or the same interrupt is taken again immediately.
   */

  (void)getreg32b(MBOX0_BASE + MBOX_A2B_CMD(RPMSG_RX_CHAN));
  (void)getreg32b(MBOX0_BASE + MBOX_A2B_DAT(RPMSG_RX_CHAN));
  putreg32b(1u << RPMSG_RX_CHAN, MBOX0_BASE + MBOX_A2B_STATUS);

  return true;
}

/* A2B mailbox interrupt: Linux kicked us. Linux sends on A2B ch3 (rpmsg-tx =
 * INTID 100).
 *
 * The handler only acknowledges the doorbell and wakes the receive thread. It
 * deliberately does not call the rptun callback, even though there is a
 * dedicated 4KB interrupt stack here: that callback is the entry to the whole
 * receive path - rptun_callback, remoteproc_get_notification,
 * virtqueue_notification, rpmsg_virtio_rx_callback - which walks the vrings and
 * dispatches to every endpoint. Running that with interrupts disabled makes the
 * worst-case interrupt latency a function of how much traffic Linux queued, and
 * it puts an unbounded call chain on a stack sized for handlers.
 */

static int rk3588_rptun_isr(int irq, void *context, void *arg)
{
  struct rk3588_rptun_dev_s *dev = &g_rptun_dev;

  if (rk3588_rptun_ack())
    {
      nxsem_post(&dev->rxsem);
    }

  return OK;
}

/* Receive thread: hands OpenAMP the rings once the handler signals a doorbell.
 *
 * Runs with CONFIG_RPTUN_STACKSIZE rather than on the interrupt stack, and at
 * thread priority, so ring processing cannot extend interrupt latency.
 */

static int rk3588_rptun_rx(int argc, char *argv[])
{
  struct rk3588_rptun_dev_s *dev = &g_rptun_dev;

  for (; ; )
    {
      nxsem_wait_uninterruptible(&dev->rxsem);

      if (dev->callback != NULL)
        {
          dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
        }
    }

  return 0;
}

/* RX interrupt keepalive thread.
 *
 * The mailbox A2B RX interrupt (INTID 100) IS a Non-secure Group-1 interrupt
 * and is delivered fine to NS-EL1 on cpu_l3 -- proven on target: with this
 * keepalive active, rpmsg RX runs entirely from rk3588_rptun_isr (rx_count
 * climbed to 10000 with the poll no longer collecting RX, and a devmem poke of
 * any A2B channel triggers the ISR).
 *
 * The reason interrupt RX did NOT work before was NOT a GIC group/secure
 * problem: Linux's gic_dist_init() (~15s) resets the (shared) GICD -- it
 * DISABLES all SPIs (GICD_ICENABLER) and its rockchip-amp handling only
 * restores IROUTER, never re-enables; Linux also clears the mailbox A2B
 * interrupt-enable. So the SPI ended up disabled with A2B_INTEN cleared, and
 * NuttX (having enabled it once at boot) never re-armed it.
 *
 * Fix (pure NS side, no BL31/OP-TEE change): re-arm the RX path. Linux only
 * disables it ONCE, during gic_dist_init (~15s), so this thread re-arms every
 * 100ms for the first ~30s (to reliably survive that event) and then EXITS --
 * steady state needs no polling. A later Linux GIC re-init (suspend/resume) is
 * covered opportunistically by the TX-path re-arm in rk3588_rptun_notify().
 */

#define RK3588_RPTUN_KEEPALIVE_MS      100
#define RK3588_RPTUN_KEEPALIVE_ROUNDS  300   /* 300 * 100ms = 30s */

static int rk3588_rptun_irq_keepalive(int argc, char *argv[])
{
  struct rk3588_rptun_dev_s *dev = &g_rptun_dev;
  int i;

  for (i = 0; i < RK3588_RPTUN_KEEPALIVE_ROUNDS; i++)
    {
      rk3588_rptun_irq_rearm();

      /* Pick up a doorbell that arrived while the interrupt was disarmed.
       * Re-arming alone does not recover it: the mailbox status bit stays set
       * but the interrupt has already been missed, so without this check the
       * kick is lost for good. That window is real - the same one on the M0
       * link accounted for 93 doorbells during startup - and it is exactly the
       * window this thread exists to cover.
       */

      if (rk3588_rptun_ack())
        {
          nxsem_post(&dev->rxsem);
        }

      nxsig_usleep(RK3588_RPTUN_KEEPALIVE_MS * 1000);
    }

  return 0;   /* startup window done; steady state needs no re-arm */
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

  /* Enable the mailbox A2B RX interrupt (Linux -> cpu_l3 doorbell) and wire up
   * the ISR. Linux sends rpmsg-tx on A2B channel 3 => INTID 100. Enable that
   * channel's mailbox interrupt, attach + enable the GIC INTID, and route it
   * to cpu_l3. (A keepalive thread below re-arms all of this after Linux's GIC
   * reset; see rk3588_rptun_irq_keepalive.)
   */

  putreg32b(getreg32b(MBOX0_BASE + MBOX_A2B_INTEN) | (1u << RPMSG_RX_CHAN),
            MBOX0_BASE + MBOX_A2B_INTEN);

  /* Signalling semaphore between the handler and the receive thread. Priority
   * inheritance has to be off: the thread only ever waits and never posts, so
   * with inheritance enabled it stays a holder forever and gets its priority
   * boosted by anything else that touches the semaphore.
   */

  nxsem_init(&dev->rxsem, 0, 0);
  nxsem_set_protocol(&dev->rxsem, SEM_PRIO_NONE);

  ret = irq_attach(RK3588_MBOX_A2B_IRQ, rk3588_rptun_isr, dev);
  if (ret < 0)
    {
      rpmsgerr("ERROR: irq_attach(%d) failed %d\n", RK3588_MBOX_A2B_IRQ, ret);
      return ret;
    }

  up_enable_irq(RK3588_MBOX_A2B_IRQ);
  putreg64b(RK3588_CPU_L3_AFF, RK3588_GICD_IROUTER(RK3588_MBOX_A2B_IRQ));

  dev->rptun.ops = &g_rk3588_rptun_ops;
  strncpy(dev->cpuname, cpuname, RPMSG_NAME_SIZE);
  strncpy(dev->shmemname, shmemname, RPMSG_NAME_SIZE);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      rpmsgerr("ERROR: rptun_initialize failed %d\n", ret);
      return ret;
    }

  /* The thread that actually processes received rings; the handler only signals
   * it. Started after rptun_initialize so the callback is registered by the
   * time a doorbell can wake it.
   */

  ret = kthread_create("rptun_rx", CONFIG_RPTUN_PRIORITY,
                       CONFIG_RPTUN_STACKSIZE, rk3588_rptun_rx, NULL);
  if (ret < 0)
    {
      rpmsgerr("ERROR: rx thread failed %d\n", ret);
      return ret;
    }

  /* Start the bounded RX interrupt keepalive thread: re-arms INTID 100 /
   * IROUTER / A2B_INTEN for the first ~30s to survive Linux's one-time GIC
   * reset (~15s), then exits. RX itself is interrupt-driven (rk3588_rptun_isr);
   * the TX path (rk3588_rptun_notify) re-arms opportunistically thereafter.
   */

  kthread_create("rptun_ka", 200, 2048, rk3588_rptun_irq_keepalive, NULL);

  return OK;
}
