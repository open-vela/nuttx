/****************************************************************************
 * arch/xtensa/src/t113/t113_rsctable.c
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

#include <stddef.h>

#include <nuttx/rptun/rptun.h>
#include <openamp/remoteproc.h>
#include <openamp/virtio.h>

#include "chip_memory.h"
#include "t113_rsctable.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Two vrings, sun8iw20 vendor geometry (matches the AP master and any
 * Linux/vendor-RTOS counterpart): 256 descriptors, 128-byte align, 512-byte
 * buffers.  The vrings and the rpmsg buffer pool live in the shared DDR
 * window (DSP-view 0x37900000 = AP-view 0x47900000) so both cores see the
 * same physical RAM -- critical so the AP observes the slave setting
 * VIRTIO_CONFIG_STATUS_DRIVER_OK.
 */

#define NUM_VRINGS          2
#define VRING_NUM           256
#define VRING_ALIGN         128
#define RPMSG_BUF_SIZE      512

#define VRING0_ID           1            /* RX (master -> remote) */
#define VRING1_ID           2            /* TX (remote -> master) */

#define RSC_VDEV_FEATURE_NS      (1 << 0) /* name-service announcement */
#define RSC_VDEV_FEATURE_CPUNAME (1 << 3) /* VIRTIO_RPMSG_F_CPUNAME: host /
                                           * remote cpu names in config space */

/* Shared-window carveout for the vrings + rpmsg buffer pool.
 *
 * CRITICAL: the carveout must NOT overlap the resource-table metadata.  The
 * framework's rptun_init_carveout() builds an mm_heap at the carveout base;
 * if that base overlapped the table (linker puts .resource_table at
 * 0x37900000), the heap node header would overwrite it.
 *
 * So the carveout is a dedicated, non-overlapping slice high in the 5 MB
 * window, past the DSP firmware image.  The vring resource DAs are left 0 so
 * the framework relocates both vrings sequentially from the carveout base
 * (rptun_update_vring_da), then uses the remainder as the buffer pool.  The
 * carveout is NOT part of the ELF image (no section storage): it is plain
 * shared DDR that physically exists; the master maps it via address-env.
 *
 * Window map (DSP-view = AP-view - 0x10000000):
 *   0x37900000  resource-table metadata (.resource_table, in ELF, ~264 B)
 *   0x37904400  trace ring (see linker script)
 *   0x37920000  DSP firmware text/data/bss (in ELF, ~120 KB)
 *   0x37D00000  carveout: vring0, vring1, buffer pool (this slice, ~273 KB)
 *   0x37E00000  end of 5 MB window
 */

#define CARVEOUT_DA         T113_DDR_CARVEOUT_BASE
#define CARVEOUT_LEN        T113_DDR_CARVEOUT_SIZE  /* 1 MB: 2 vrings + pool */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* The master (AP) reads this table out of the DSP ELF's .resource_table
 * section (its rptun get_resource returns NULL); the DSP slave's
 * get_resource returns this table.  Both cores run the same ELF, so the
 * table is identical by construction.
 */

locate_data(".resource_table") used_data
const struct rptun_rsc_s g_t113_rsc_table =
{
  .rsc_tbl_hdr =
  {
    .ver      = 1,
    .num      = 2,           /* RSC_VDEV + RSC_CARVEOUT */
    .reserved =
    {
      0, 0
    },
  },

  .offset =
  {
    offsetof(struct rptun_rsc_s, rpmsg_vdev),
    offsetof(struct rptun_rsc_s, carveout),
    0,
  },

  .log_trace =
  {
    .type = RSC_TRACE, .da = 0, .len = 0,
  },

  .rpmsg_vdev =
  {
    .type          = RSC_VDEV,
    .id            = VIRTIO_ID_RPMSG,
    .notifyid      = 0,
    .dfeatures     = RSC_VDEV_FEATURE_NS | RSC_VDEV_FEATURE_CPUNAME,
    .gfeatures     = 0,
    .config_len    = sizeof(struct fw_rsc_config),
    .status        = 0,
    .num_of_vrings = NUM_VRINGS,
    .reserved      =
    {
      0, 0
    },
  },

  /* da = 0: the framework relocates each vring into the carveout in turn. */

  .rpmsg_vring0 =
  {
    .da = 0, .align = VRING_ALIGN, .num = VRING_NUM,
    .notifyid = VRING0_ID, .reserved = 0,
  },

  .rpmsg_vring1 =
  {
    .da = 0, .align = VRING_ALIGN, .num = VRING_NUM,
    .notifyid = VRING1_ID, .reserved = 0,
  },

  .config =
  {
    .h2r_buf_size   = RPMSG_BUF_SIZE,
    .r2h_buf_size   = RPMSG_BUF_SIZE,
    .host_cpuname   = "ap",
    .remote_cpuname = "dsp",
  },

  /* Carveout: a dedicated slice high in the window, clear of the table and
   * the firmware image.  pa is ADDR_ANY so the master resolves it through
   * its address-env (da 0x37D00000 -> pa 0x47D00000).
   */

  .carveout =
  {
    .type  = RSC_CARVEOUT,
    .da    = CARVEOUT_DA,
    .pa    = FW_RSC_U32_ADDR_ANY,
    .len   = CARVEOUT_LEN,
    .flags = 0,
    .name  = "vdev0buffer",
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct resource_table *t113_rsctable_get(void)
{
  return (struct resource_table *)&g_t113_rsc_table;
}
