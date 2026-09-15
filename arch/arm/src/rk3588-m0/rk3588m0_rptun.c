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

#include <assert.h>
#include <string.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kthread.h>
#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun.h>
#include <nuttx/semaphore.h>
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

/* The SoC interrupt this core's RX doorbell arrives on, and the NVIC line the
 * INTMUX delivers it to. A2B channel 2 is irq_mailbox0_bb2, source id 99, which
 * falls in the 64-127 block and therefore emerges on external line 17.
 */

#define MBOX_RX_INTID   RK3588_MBOX0_BB_INTID(RX_CHAN)
#define MBOX_RX_IRQ     (RK3588M0_IRQ_EXTINT + \
                         RK3588M0_INTMUX_EXTINT_OF(MBOX_RX_INTID))

/* Pin the two numbers the routing above derives, because getting either wrong
 * fails silently: no interrupt is ever delivered, and the re-arm wakeup below
 * quietly carries the link at its own cadence instead. Values for A2B channel
 * 2, from TRM Table 1-3 (irq_mailbox0_bb2 = id 99) and Table 9-7 (ids 64-127
 * emerge on external line 17, i.e. vector 33).
 */

static_assert(MBOX_RX_INTID == 99, "mailbox0 A2B ch2 is SoC interrupt id 99");
static_assert(MBOX_RX_IRQ == 33, "SoC id 99 arrives on NVIC vector 33");
static_assert(RK3588M0_INTMUX_GROUP_OF(MBOX_RX_INTID) == 12, "id 99 -> group 12");
static_assert(RK3588M0_INTMUX_BIT_OF(MBOX_RX_INTID) == (1u << 3), "id 99 -> bit 3");

/* How often the receive thread re-arms the doorbell when nothing has arrived.
 *
 * This is not a poll cadence - reception is interrupt-driven - but the enables
 * it re-asserts are not ours alone. A2B_INTEN is one register shared with the
 * cpu_l3 link, which read-modify-writes it from its own keepalive and from
 * every notify; if either straddles our update, our bit is lost and the
 * doorbell goes quiet with no way to notice. Re-asserting costs two register
 * writes per interval and bounds the outage instead.
 *
 * The same wakeup doubles as a safety net: it re-checks the mailbox status, so
 * a kick that somehow failed to raise an interrupt still gets serviced within
 * one interval rather than stalling the link.
 */

#define RPTUN_REARM_MS  100

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
  sem_t                         rxsem;
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
 * Name: rk3588m0_rptun_rearm
 *
 * Description:
 *   Re-assert both enables the doorbell depends on: our channel's bit in the
 *   mailbox A2B interrupt enable, and our source's bit in the INTMUX. Both
 *   writes are idempotent.
 *
 *   The mailbox half is the one that actually needs repeating, because that
 *   register is shared with the cpu_l3 link. The INTMUX half is private to this
 *   core and is only included so that a single call restores the whole path.
 *
 ****************************************************************************/

static void rk3588m0_rptun_rearm(void)
{
  putreg32(getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_INTEN) | (1u << RX_CHAN),
           MBOX_BASE + RK3588M0_MBOX_A2B_INTEN);

  putreg32(getreg32(RK3588M0_INTMUX_BASE_OF(MBOX_RX_INTID) +
                    RK3588M0_INTMUX_ENABLE(
                        RK3588M0_INTMUX_GROUP_OF(MBOX_RX_INTID))) |
           RK3588M0_INTMUX_BIT_OF(MBOX_RX_INTID),
           RK3588M0_INTMUX_BASE_OF(MBOX_RX_INTID) +
           RK3588M0_INTMUX_ENABLE(
               RK3588M0_INTMUX_GROUP_OF(MBOX_RX_INTID)));
}

/****************************************************************************
 * Name: rk3588m0_rptun_ack
 *
 * Description:
 *   Acknowledge a doorbell on our channel, if one is pending. Returns true
 *   when there was something to acknowledge.
 *
 *   Only our own channel is touched. The four A2B channels of mailbox0 share
 *   one status register, and channels 0 and 3 carry the cpu_l3 link's traffic:
 *   clearing those would consume a doorbell meant for the other core, whose
 *   handler would then find nothing pending and never tell its OpenAMP to look
 *   at the rings.
 *
 ****************************************************************************/

static bool rk3588m0_rptun_ack(void)
{
  if ((getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_STATUS) & (1u << RX_CHAN)) == 0u)
    {
      return false;
    }

  /* Reading CMD and DAT and writing the bit back is what clears it. The
   * doorbell is level-triggered, so this has to happen before the handler
   * returns or the same interrupt is taken again immediately.
   */

  getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_CMD(RX_CHAN));
  getreg32(MBOX_BASE + RK3588M0_MBOX_A2B_DAT(RX_CHAN));
  putreg32(1u << RX_CHAN, MBOX_BASE + RK3588M0_MBOX_A2B_STATUS);

  return true;
}

/****************************************************************************
 * Name: rk3588m0_rptun_isr
 *
 * Description:
 *   Mailbox doorbell handler: Linux has put something in the rings.
 *
 *   The handler only acknowledges the doorbell and wakes the receive thread; it
 *   does not call into OpenAMP itself. That differs from the cpu_l3 port, which
 *   can afford to, and the reason is the stack: this core is built with
 *   CONFIG_ARCH_INTERRUPTSTACK=0, so a handler runs on the stack of whichever
 *   thread it interrupted - possibly the 2KB heartbeat thread - while walking
 *   the vrings needs far more than that. Handing the work to a thread with a
 *   known 16KB stack keeps it bounded, and also keeps a slow core from doing
 *   ring processing with interrupts disabled.
 *
 ****************************************************************************/

static int rk3588m0_rptun_isr(int irq, void *context, void *arg)
{
  struct rk3588m0_rptun_dev_s *dev = arg;

  if (rk3588m0_rptun_ack())
    {
      nxsem_post(&dev->rxsem);
    }

  return OK;
}

/****************************************************************************
 * Name: rk3588m0_rptun_rx
 *
 * Description:
 *   Receive thread. Waits for the handler's signal and hands OpenAMP the
 *   rings. The wait is bounded so the same loop also re-arms the doorbell and
 *   re-checks the mailbox; see RPTUN_REARM_MS for why both are needed.
 *
 ****************************************************************************/

static int rk3588m0_rptun_rx(int argc, char *argv[])
{
  struct rk3588m0_rptun_dev_s *dev = &g_rptun_dev;

  for (; ; )
    {
      bool kicked;

      kicked = nxsem_tickwait(&dev->rxsem, MSEC2TICK(RPTUN_REARM_MS)) >= 0;

      rk3588m0_rptun_rearm();

      /* A pending bit here means a kick that produced no interrupt - either it
       * arrived while the enable was clobbered, or it landed before the handler
       * was attached. Service it the same way.
       */

      if (rk3588m0_rptun_ack())
        {
          kicked = true;
        }

      if (kicked && dev->callback != NULL)
        {
          dev->callback(dev->arg, RPTUN_NOTIFY_ALL);
        }
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

  /* Wire up the doorbell before anything can ring it.
   *
   * Three enables stand between a Linux kick and this core's handler, and all
   * three have to be set: the mailbox has to raise its interrupt for our
   * channel, the INTMUX has to pass that source through to an NVIC line, and
   * the NVIC line has to be enabled. rk3588m0_rptun_rearm() does the first two;
   * the source lands on MBOX_RX_IRQ, computed from the interrupt id rather than
   * hard-coded, so the channel choice in the memory map stays the only place
   * that decides it.
   */

  /* Signalling semaphore: the handler only ever posts and the receive thread
   * only ever waits, so priority inheritance has to be turned off. Otherwise
   * the thread becomes a permanent holder - it never posts - and gets its
   * priority boosted by anything else that touches the semaphore.
   */

  nxsem_init(&dev->rxsem, 0, 0);
  nxsem_set_protocol(&dev->rxsem, SEM_PRIO_NONE);

  ret = irq_attach(MBOX_RX_IRQ, rk3588m0_rptun_isr, dev);
  if (ret < 0)
    {
      rpmsgerr("ERROR: irq_attach(%d) failed %d\n", MBOX_RX_IRQ, ret);
      return ret;
    }

  rk3588m0_rptun_rearm();
  up_enable_irq(MBOX_RX_IRQ);

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
                       CONFIG_RPTUN_STACKSIZE, rk3588m0_rptun_rx, NULL);
  if (ret < 0)
    {
      rpmsgerr("ERROR: rx thread failed %d\n", ret);
      return ret;
    }

  return 0;
}
