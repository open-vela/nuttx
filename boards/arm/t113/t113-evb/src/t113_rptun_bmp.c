/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_rptun_bmp.c
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

/* BMP NuttX-to-NuttX rptun: a single image runs on both A7 cores and decides
 * its role at run time by this_cpu().  core0 is master (populates the
 * in-image resource table, then registers with rptun_bmp); core1 is slave
 * (registers and spin-waits on the table's ver field).
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/nuttx.h>
#include <nuttx/rptun/rptun_bmp.h>
#include <nuttx/serial/uart_rpmsg.h>

#include "arm_internal.h"
#include "gic.h"
#include "t113-evb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SGI assignment:
 *   core0 -> core1 : SGI15  (master irq_trigger / slave irq_event)
 *   core1 -> core0 : SGI14  (slave  irq_trigger / master irq_event)
 *
 * Per-core banked SGI: master sends to cpuset {1}, slave to {0}.
 */

#define T113_RSC_TABLE_SIZE  0x10000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_rptun_rsc_s
{
  struct rptun_rsc_s rsc;
  uint8_t padding[T113_RSC_TABLE_SIZE - sizeof(struct rptun_rsc_s)];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* BMP: rsc_table is an in-image global, both CPUs see the same address by
 * virtue of running the same image.
 */

static struct t113_rptun_rsc_s g_rptun_rsc;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rptun_setup_shmem
 *
 * Description:
 *   Populate the shared virtio resource table.  Copied from the qemu
 *   armv8-r reference board; rptun_bmp has no public helper for this.
 *   The ver field is written last behind UP_DMB() so the slave's spin
 *   barrier observes a consistent table.
 *
 ****************************************************************************/

static void rptun_setup_shmem(struct rptun_rsc_s *rsc,
                              const char *host_cpuname,
                              const char *remote_cpuname)
{
  memset(rsc, 0, sizeof(struct rptun_rsc_s));
  rsc->offset[0]                = offsetof(struct rptun_rsc_s,
                                           rpmsg_vdev);
  rsc->rpmsg_vdev.type          = RSC_VDEV;
  rsc->rpmsg_vdev.id            = VIRTIO_ID_RPMSG;
  rsc->rpmsg_vdev.dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                                | 1 << VIRTIO_RPMSG_F_ACK
                                | 1 << VIRTIO_RPMSG_F_BUFSZ
                                | 1 << VIRTIO_RPMSG_F_CPUNAME;
  rsc->rpmsg_vdev.num_of_vrings = 2;
  rsc->rpmsg_vdev.notifyid      = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vdev.config_len    = sizeof(struct fw_rsc_config);
  rsc->rpmsg_vdev.reserved[0]   = VIRTIO_DEV_DRIVER;
  rsc->rpmsg_vdev.reserved[1]   = 0;
  rsc->rpmsg_vring0.align       = 8;
  rsc->rpmsg_vring0.num         = 8;
  rsc->rpmsg_vring0.notifyid    = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vring0.da          = 0;
  rsc->rpmsg_vring1.align       = 8;
  rsc->rpmsg_vring1.num         = 8;
  rsc->rpmsg_vring1.notifyid    = RSC_NOTIFY_ID_ANY;
  rsc->rpmsg_vring1.da          = 0;
  rsc->config.r2h_buf_size      = 0x200;
  rsc->config.h2r_buf_size      = 0x200;
  strlcpy((char *)rsc->config.host_cpuname, host_cpuname,
          VIRTIO_RPMSG_CPUNAME_SIZE);
  strlcpy((char *)rsc->config.remote_cpuname, remote_cpuname,
          VIRTIO_RPMSG_CPUNAME_SIZE);

  rsc->offset[1]                = offsetof(struct rptun_rsc_s,
                                           carveout);
  rsc->carveout.type            = RSC_CARVEOUT;
  rsc->carveout.da              = (metal_phys_addr_t)rsc +
                                  ALIGN_UP(sizeof(struct rptun_rsc_s), 8);
  rsc->carveout.pa              = FW_RSC_U32_ADDR_ANY;
  rsc->carveout.len             = 0x200 * 8 * 2 + 0x1000;
  memcpy(rsc->carveout.name, "vdev0buffer", 11);

  rsc->rsc_tbl_hdr.ver          = 1;
  rsc->rsc_tbl_hdr.num          = 2;
  UP_DMB();
}

/****************************************************************************
 * Name: t113_rptun_is_master
 *
 * Description:
 *   Resolve the rptun role for BMP: a single image runs on both cores and
 *   decides the role at run time by physical core id (core0 is master).
 *
 ****************************************************************************/

static bool t113_rptun_is_master(void)
{
  return this_cpu() == 0;
}

/****************************************************************************
 * Name: t113_rptun_register
 *
 * Description:
 *   BMP NuttX-to-NuttX registration: core0 (master) populates the in-image
 *   resource table then registers with rptun_bmp; core1 (slave) registers
 *   and spin-waits on the ver field.  SGI doorbell directions are symmetric:
 *   master receives on SGI14 / triggers SGI15 (peer core1); slave receives
 *   on SGI15 / triggers SGI14 (peer core0).
 *
 ****************************************************************************/

static int t113_rptun_register(FAR struct rptun_rsc_s *rsc, bool master)
{
  cpu_set_t cpuset;

  CPU_ZERO(&cpuset);

  if (master)
    {
      rptun_setup_shmem(rsc, "core0", "core1");
      CPU_SET(1, &cpuset);
      return rptun_bmp_init("core1", true, rsc,
                            GIC_IRQ_SGI14, GIC_IRQ_SGI15, cpuset);
    }

  CPU_SET(0, &cpuset);
  return rptun_bmp_init("core0", false, rsc,
                        GIC_IRQ_SGI15, GIC_IRQ_SGI14, cpuset);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_rptun_init
 *
 * Description:
 *   BMP bring-up: a single image runs on both cores.  core0 is master and
 *   initializes the in-image shared resource table (g_rptun_rsc) before
 *   registering; core1 is slave and spin-waits inside rptun_bmp_init().
 *
 ****************************************************************************/

int t113_rptun_init(void)
{
  return t113_rptun_register(&g_rptun_rsc.rsc, t113_rptun_is_master());
}

/****************************************************************************
 * Name: rpmsg_serialinit
 *
 * Description:
 *   Called by drivers_initialize() under CONFIG_RPMSG_UART.  The master side
 *   registers /dev/ttyCORE1 and the peer attaches it; the role is resolved
 *   at run time via t113_rptun_is_master().
 *
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  if (t113_rptun_is_master())
    {
      /* BMP master exposes /dev/ttyCORE1 as a regular tty. */

      uart_rpmsg_init("core1", "CORE1", 256, false);
    }
  else
    {
      /* BMP slave (core1) migrates its shell onto the rpmsg console. */

      uart_rpmsg_init("core0", "CORE1", 256, true);
    }
}
#endif
