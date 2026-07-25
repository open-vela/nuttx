/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_rsctable.c
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

#include <string.h>

#include <nuttx/nuttx.h>

#include "chip.h"
#include "rk3588m0_rsctable.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The layout must agree with the Linux side, which takes its vring addresses
 * from the device tree and never reads this table:
 *
 *   vring0  Linux -> this core
 *   vring1  this core -> Linux
 *
 * Sizes come from include/linux/rpmsg/rockchip_rpmsg.h (RPMSG_BUF_COUNT = 64,
 * RPMSG_VRING_ALIGN = 0x1000, RPMSG_VRING_SIZE = 0x8000).
 *
 * The addresses here are this core's WINDOW view (RK3588M0_EXSRAM_BASE and up),
 * not physical: OpenAMP dereferences them directly. Linux addresses the same
 * bytes physically through its own reserved-memory nodes, and
 * rk3588m0_addrenv.c rebases between the two views for anything that travels
 * inside the vrings.
 */

#define NUM_VRINGS              0x02
#define RL_BUFFER_COUNT         RK3588M0_RPMSG_BUF_COUNT
#define VRING_ALIGN             RK3588M0_VRING_ALIGN
#define VDEV0_VRING_BASE        RK3588M0_VRING0
#define VRING_SIZE              RK3588M0_VRING_SIZE
#define RSC_BUFFER_BASE         RK3588M0_RPMSG_POOL
#define RSC_BUFFER_SIZE         RK3588M0_RPMSG_POOL_SIZE
#define RESOURCE_TABLE_BASE     RK3588M0_RSC_TABLE
#define NO_RESOURCE_ENTRIES     (2)
#define RSC_TABLE_VERSION       (1)

/* Feature bits, and why exactly these (all learned on the cpu_l3 link):
 *
 *   F_NS (bit 0)      - name service announcement. Byte-identical to mainline
 *                       Linux virtio_rpmsg_bus, which is the only feature the
 *                       rockchip host offers.
 *   F_CPUNAME (bit 3) - hard-required by openvela's rpmsg_virtio.c, but purely
 *                       local: it only makes this side read cpu names out of
 *                       our own config space, so it never goes on the wire.
 *
 * F_ACK / F_BUFSZ / F_BUFADDR / F_PRIORITY are deliberately left off: they
 * change on-wire behaviour and mainline rockchip rpmsg does not implement them.
 *
 * VIRTIO_RING_F_MUST_NOTIFY (bit 30) is added because OpenAMP otherwise
 * suppresses the doorbell whenever the peer's avail->flags carries
 * VRING_AVAIL_F_NO_INTERRUPT. The rockchip host does not use that suppression -
 * it expects the remote to always kick the mailbox - and with the flag off the
 * mailbox interrupt count on Linux stayed at zero and no announcement ever
 * arrived. It is a local flag and also stays off the wire.
 */

#define RSC_VDEV_MUST_NOTIFY    (1u << 30)
#define RSC_VDEV_DFEATURES      ((1u << 0) | (1u << 3) | RSC_VDEV_MUST_NOTIFY)

/* Declare the virtio driver (Linux) ready up front. The rockchip host uses
 * fixed device-tree vrings and never writes VIRTIO_CONFIG_STATUS_DRIVER_OK into
 * our table, so rptun_create_device() would spin on -EAGAIN forever waiting for
 * it. Linux has its vrings up and reports itself online early in boot, so
 * assuming readiness here is safe.
 */

#define RSC_VDEV_STATUS_DRIVER_OK (0x04)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#if defined(__GNUC__)
__attribute__ ((section(".resource_table")))
#else
#  error Compiler not supported!
#endif
const struct rptun_rsc_s g_rk3588m0_rsc_table =
{
  .rsc_tbl_hdr =
  {
    RSC_TABLE_VERSION,
    NO_RESOURCE_ENTRIES,
    {
      0, 0
    }
  },
  .offset =
  {
    offsetof(struct rptun_rsc_s, rpmsg_vdev),
    offsetof(struct rptun_rsc_s, carveout)
  },
  .log_trace =
  {
    RSC_TRACE, 0, 0
  },
  .rpmsg_vdev =
  {
    RSC_VDEV,                     /* type                        */
    7,                            /* id (VIRTIO_ID_RPMSG)        */
    2,                            /* notifyid                    */
    RSC_VDEV_DFEATURES,           /* dfeatures                   */
    RSC_VDEV_DFEATURES,           /* gfeatures: pinned, see above */
    sizeof(struct fw_rsc_config), /* config_len                  */
    RSC_VDEV_STATUS_DRIVER_OK,    /* status                      */
    NUM_VRINGS,                   /* num_of_vrings               */
    {
      0, 0                        /* reserved                    */
    }
  },
  .rpmsg_vring0 =
  {
    VDEV0_VRING_BASE,
    VRING_ALIGN,
    RL_BUFFER_COUNT,
    0,
    0
  },
  .rpmsg_vring1 =
  {
    VDEV0_VRING_BASE + VRING_SIZE,
    VRING_ALIGN,
    RL_BUFFER_COUNT,
    1,
    0
  },
  .config =
  {
    .host_cpuname   = "linux",    /* the peer (Linux master)     */
    .remote_cpuname = "pmu_m0"    /* this core                   */
  },
  .carveout =
  {
    RSC_CARVEOUT,
    RSC_BUFFER_BASE,
    RSC_BUFFER_BASE,
    RSC_BUFFER_SIZE,
    0,
    0,
    "rpmsg_shm"
  }
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3588m0_copy_rsc_table
 *
 * Description:
 *   Copy the const template into writable shared memory and return that
 *   pointer. OpenAMP writes allocated notify ids back into the table, so the
 *   copy handed to rptun must not be the .rodata original.
 *
 ****************************************************************************/

void *rk3588m0_copy_rsc_table(void)
{
  memcpy((void *)RESOURCE_TABLE_BASE, (void *)&g_rk3588m0_rsc_table,
         sizeof(g_rk3588m0_rsc_table));

  return (void *)RESOURCE_TABLE_BASE;
}
