/****************************************************************************
 * arch/arm64/src/rk3588/rk3588_rsctable.c
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

#include "rk3588_rsctable.h"
#include <string.h>
#include <nuttx/nuttx.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Layout MUST match Linux rockchip rpmsg (drivers + rk3588-amp.dtsi):
 *   vring0 @ 0x07c00000 (Linux->NuttX), vring1 @ 0x07c08000 (NuttX->Linux)
 *   RPMSG_BUF_COUNT = 64, RPMSG_VRING_ALIGN = 0x1000, RPMSG_VRING_SIZE = 0x8000
 *   rpmsg buffer pool in rpmsg-dma@0x08000000 (2MB, no-map)
 */

#define NUM_VRINGS               0x02
#define RL_BUFFER_COUNT          64            /* RPMSG_BUF_COUNT */
#define VRING_ALIGN              0x1000        /* RPMSG_VRING_ALIGN */
#define VDEV0_VRING_BASE         0x07c00000    /* rpmsg_reserved */
#define VRING_SIZE               0x8000        /* RPMSG_VRING_SIZE */
#define RSC_BUFFER_BASE          0x08000000    /* rpmsg-dma_reserved */
#define RSC_BUFFER_SIZE          0x00200000    /* 2MB */
#define RESOURCE_TABLE_BASE      RK3588_RSC_TABLE_BASE  /* writable copy dst */

#define NO_RESOURCE_ENTRIES      (2)
#define RK3588_RSC_TABLE_VERSION (1)

/* virtio rpmsg feature bits (openamp/rpmsg_virtio.h):
 *   F_NS=0, F_ACK=1, F_BUFSZ=2, F_CPUNAME=3, F_BUFADDR=4, F_PRIORITY=5.
 *
 * We advertise ONLY F_NS + F_CPUNAME:
 *   - F_NS      : name-service announcement, identical wire format to mainline
 *                 Linux virtio_rpmsg_bus (rpmsg_hdr / rpmsg_ns_msg match).
 *   - F_CPUNAME : required by openvela's drivers/rpmsg/rpmsg_virtio.c
 *                 (DEBUGASSERT). It only makes the NuttX side read the cpu
 *                 names from OUR local config space, so it stays off the wire.
 * We deliberately DO NOT set F_ACK/F_BUFSZ/F_BUFADDR/F_PRIORITY -- those change
 * the on-wire behaviour and would break interop with mainline rockchip rpmsg,
 * which only offers F_NS.
 *
 * We ADD VIRTIO_RING_F_MUST_NOTIFY (bit 30): by default OpenAMP's
 * vq_ring_must_notify() suppresses the doorbell when the peer's vring
 * avail->flags has VRING_AVAIL_F_NO_INTERRUPT set. rockchip Linux does not use
 * that virtio suppression -- it relies on the remote ALWAYS kicking the
 * mailbox (proven: with this off, the mailbox RX IRQ count on Linux stayed 0
 * and the NS never arrived). MUST_NOTIFY forces an unconditional kick. It is a
 * local OpenAMP flag and never goes on the wire.
 */

#define RSC_VDEV_MUST_NOTIFY     (1u << 30)  /* VIRTIO_RING_F_MUST_NOTIFY */
#define RSC_VDEV_DFEATURES       ((1u << 0) | (1u << 3) | RSC_VDEV_MUST_NOTIFY)
                                             /* F_NS | F_CPUNAME | MUST_NOTIFY */

/* Pre-declare the virtio driver (Linux master) as ready. The rockchip Linux
 * rpmsg host uses fixed dts vring addresses and does NOT read/write our
 * resource table, so it never sets VIRTIO_CONFIG_STATUS_DRIVER_OK (0x04) in
 * our vdev status. Without this, rptun_create_device() on the NuttX (DEVICE)
 * side spins on -EAGAIN forever. Linux brings its vrings up and is "online"
 * from early boot, so it is safe to assume the driver is ready here.
 */

#define RSC_VDEV_STATUS_DRIVER_OK (0x04)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Place resource table in a special ELF section */

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__ ((section(".resource_table")))
#else
#error Compiler not supported!
#endif
const struct rptun_rsc_s g_rk3588_rsc_table =
{
    .rsc_tbl_hdr =
    {
        RK3588_RSC_TABLE_VERSION,
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
        RSC_VDEV,                   /* type */
        7,                          /* id (VIRTIO_ID_RPMSG) */
        2,                          /* notifyid */
        RSC_VDEV_DFEATURES,         /* dfeatures: F_NS | F_CPUNAME */
        RSC_VDEV_DFEATURES,         /* gfeatures: pre-negotiated (Linux, the
                                     * DRIVER, never writes our table, so the
                                     * DEVICE-role feature = dfeatures & gfeatures
                                     * would be 0; pin gfeatures = dfeatures) */
        sizeof(struct fw_rsc_config), /* config_len */
        RSC_VDEV_STATUS_DRIVER_OK,  /* status: driver (Linux) assumed ready */
        NUM_VRINGS,                 /* num_of_vrings */
        {
            0, 0                    /* reserved */
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
        .host_cpuname   = "linux",  /* peer (Linux master) name */
        .remote_cpuname = "cpu3"    /* our name (NuttX on cpu_l3) */
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

void *rk3588_copy_rsc_table(void)
{
  memcpy((void *)RESOURCE_TABLE_BASE, (void *)&g_rk3588_rsc_table,
          sizeof(g_rk3588_rsc_table));

  return (void *)RESOURCE_TABLE_BASE;
}
