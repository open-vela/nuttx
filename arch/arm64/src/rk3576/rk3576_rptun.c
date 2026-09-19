/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_rptun.c
 *
 * RK3576 AMP-slave rptun backend.
 *
 * Peer: Linux drivers/rpmsg/rockchip_rpmsg_softirq.c (master).  Transport
 * is two reserved GIC SPIs, no mailbox:
 *   - RX (INTID 172, GIC_SPI 140): Linux irq_retrigger()s it; the Linux
 *     device tree (amp-irqs) routes it to this core.  One interrupt serves
 *     both virtqueues, so the handler kicks the whole vdev.
 *   - TX (INTID 173, GIC_SPI 141): we set it pending in GICD_ISPENDR; the
 *     GIC delivers it to a Linux core whose threaded handler runs
 *     vring_interrupt() on its rvq.
 *
 * Shared memory is Normal Non-cacheable on both sides (see the RK3576 MMU
 * map and the Linux no-map reserved regions), so no cache maintenance.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/rptun/rptun.h>
#include <openamp/open_amp.h>

#include <arch/chip/chip.h>

#include "arm64_internal.h"
#include "rk3576_rptun.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define GICD_ISPENDR(n)  (CONFIG_GICD_BASE + 0x200 + ((n) / 32) * 4)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rk3576_rptun_dev_s
{
  struct rptun_dev_s rptun;
  rptun_callback_t   callback;
  void              *arg;
  char               cpuname[RPMSG_NAME_SIZE + 1];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static const char *rk3576_rptun_get_cpuname(struct rptun_dev_s *dev);
static struct resource_table *
rk3576_rptun_get_resource(struct rptun_dev_s *dev);
static bool rk3576_rptun_is_autostart(struct rptun_dev_s *dev);
static bool rk3576_rptun_is_master(struct rptun_dev_s *dev);
static int  rk3576_rptun_start(struct rptun_dev_s *dev);
static int  rk3576_rptun_stop(struct rptun_dev_s *dev);
static int  rk3576_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid);
static int  rk3576_rptun_register_callback(struct rptun_dev_s *dev,
                                           rptun_callback_t callback,
                                           void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern struct rptun_rsc_s g_rk3576_rsc_table;

static struct rk3576_rptun_dev_s g_rptun_dev;

static const struct rptun_ops_s g_rk3576_rptun_ops =
{
  .get_cpuname       = rk3576_rptun_get_cpuname,
  .get_resource      = rk3576_rptun_get_resource,
  .is_autostart      = rk3576_rptun_is_autostart,
  .is_master         = rk3576_rptun_is_master,
  .start             = rk3576_rptun_start,
  .stop              = rk3576_rptun_stop,
  .notify            = rk3576_rptun_notify,
  .register_callback = rk3576_rptun_register_callback,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static const char *rk3576_rptun_get_cpuname(struct rptun_dev_s *dev)
{
  struct rk3576_rptun_dev_s *priv =
      container_of(dev, struct rk3576_rptun_dev_s, rptun);

  return priv->cpuname;
}

static struct resource_table *
rk3576_rptun_get_resource(struct rptun_dev_s *dev)
{
  /* Static table in DRAM; the Linux master uses fixed addresses and never
   * reads it, so we simply hand back our compiled-in copy.
   */

  return (struct resource_table *)&g_rk3576_rsc_table;
}

static bool rk3576_rptun_is_autostart(struct rptun_dev_s *dev)
{
  return true;
}

static bool rk3576_rptun_is_master(struct rptun_dev_s *dev)
{
  return false;                 /* REMOTE: Linux is the virtio master */
}

static int rk3576_rptun_start(struct rptun_dev_s *dev)
{
  return 0;
}

static int rk3576_rptun_stop(struct rptun_dev_s *dev)
{
  return 0;
}

static int rk3576_rptun_notify(struct rptun_dev_s *dev, uint32_t vqid)
{
  /* Kick Linux: make our TX SPI pending.  GICD_ISPENDR is write-1-to-set,
   * so this is atomic against Linux touching other bits in the same word.
   */

  __asm__ volatile("dsb sy" ::: "memory");
  putreg32(1u << (RK3576_IRQ_RPMSG_TX % 32),
           GICD_ISPENDR(RK3576_IRQ_RPMSG_TX));
  return 0;
}

static int rk3576_rptun_register_callback(struct rptun_dev_s *dev,
                                          rptun_callback_t callback,
                                          void *arg)
{
  struct rk3576_rptun_dev_s *priv =
      container_of(dev, struct rk3576_rptun_dev_s, rptun);

  priv->callback = callback;
  priv->arg      = arg;
  return 0;
}

/****************************************************************************
 * Name: rk3576_rptun_isr
 *
 * Description:
 *   Linux -> us kick handler (INTID 172).  A single interrupt serves the
 *   whole vdev, so notify rptun for all virtqueues.
 *
 ****************************************************************************/

static int rk3576_rptun_isr(int irq, void *context, void *arg)
{
  struct rk3576_rptun_dev_s *priv = arg;

  if (priv->callback != NULL)
    {
      priv->callback(priv->arg, RPTUN_NOTIFY_ALL);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int rk3576_rptun_init(const char *shmemname, const char *cpuname)
{
  struct rk3576_rptun_dev_s *dev = &g_rptun_dev;
  int ret;

  dev->rptun.ops = &g_rk3576_rptun_ops;
  strlcpy(dev->cpuname, cpuname, sizeof(dev->cpuname));

  /* Attach and enable the RX kick.  up_enable_irq() writes GICD_ISENABLER
   * (write-1-to-set), and up_prioritize_irq() sets our priority; routing to
   * this core is done by the board (rk3576_amp_irq_route) and re-applied by
   * Linux from its amp-irqs table.
   */

  ret = irq_attach(RK3576_IRQ_RPMSG_RX, rk3576_rptun_isr, dev);
  if (ret < 0)
    {
      return ret;
    }

  up_prioritize_irq(RK3576_IRQ_RPMSG_RX, RK3576_AMP_IRQ_PRIO_VAL);
  up_enable_irq(RK3576_IRQ_RPMSG_RX);

  ret = rptun_initialize(&dev->rptun);
  if (ret < 0)
    {
      irq_detach(RK3576_IRQ_RPMSG_RX);
    }

  return ret;
}
