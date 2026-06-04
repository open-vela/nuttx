/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_rptun_slave.c
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

/* core1 NuttX as an ELF-loaded rptun slave (master is Linux remoteproc or
 * core0-NuttX).
 *
 * The master loads this ELF into the carveout, parses the static
 * .resource_table section below, places the vrings at the fixed
 * device-addresses the table declares, and releases core1.  This file
 * carries that static table (ver=1, valid the instant core1 starts) and
 * wires the rptun slave onto a GIC-SGI doorbell via rptun_bmp -- which is a
 * generic "GIC-SGI doorbell + resource_table holder" backend.  Because the
 * table is already ver=1, rptun_bmp_get_resource() returns at once rather
 * than spinning for a peer to publish it.
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

#include <openamp/remoteproc.h>
#include <openamp/virtio.h>
#include <openamp/rpmsg_virtio.h>

#include "arm_internal.h"
#include "mmu.h"
#include "gic.h"
#include "hardware/t113_memorymap.h"
#include "t113-evb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_RSC_TABLE_SIZE  0x10000

/* Shared vring + buffer window inside the master's carveout.  Both cores
 * must see the same physical DRAM here, so it lives in the no-map carveout
 * (CONFIG_T113_RPTUN_CORE1_CARVEOUT_BASE) in the high-DDR SHM aggregation
 * zone, separate from the image.  The window is exactly one 1 MB MMU section
 * so the slave can remap just this region Normal Non-Cacheable without
 * disturbing its own cacheable image.  Layout:
 *
 *   base + 0x000000  vring0  (RX, master -> remote)   vring_size(256,128)
 *   base + 0x002000  vring1  (TX, remote -> master)   vring_size(256,128)
 *   base + 0x004000  rpmsg buffer pool ("vdev0buffer")
 *   base + 0x100000  end of window
 *
 * The vring device-addresses are PINNED (non-zero) so OpenAMP leaves them as
 * declared rather than relocating, and the master pre-registers matching
 * carveouts at the same addresses; otherwise the two sides would compute
 * different vring locations and never meet.
 */

#define SHMEM_BASE        CONFIG_T113_RPTUN_CORE1_CARVEOUT_BASE
#define SHMEM_SIZE        T113_CPU1_CARVEOUT_SIZE   /* single source: memorymap.h */

#define VRING_NUM         256
#define VRING_ALIGN       128
#define RPMSG_BUF_SIZE    512

#define VRING0_DA         (SHMEM_BASE + 0x00000000)   /* RX (master -> us) */
#define VRING1_DA         (SHMEM_BASE + 0x00002000)   /* TX (us -> master) */
#define BUFPOOL_DA        (SHMEM_BASE + 0x00004000)
#define BUFPOOL_LEN       (SHMEM_SIZE - 0x00004000 - 0x00001000)

#define VRING0_NOTIFYID   1
#define VRING1_NOTIFYID   2

/* Life-sign cell: the carveout's top 4 KB.
 * It lives inside the Normal-Non-Cacheable remap, so a write reaches DRAM
 * immediately and the master / a JLink AHB-AP read observes it without
 * any cache maintenance.  Two markers prove core1 progress:
 *   [0] = LIFESIGN_MAGIC  -> core1 booted NuttX and reached slave init
 *   [1] = LIFESIGN_RPTUN  -> rptun_bmp_init() returned (slave attached)
 */

#define LIFESIGN_DA       (SHMEM_BASE + 0x000ff000)
#define LIFESIGN_MAGIC    0x5a510001u
#define LIFESIGN_RPTUN    0x5a51600du

/* GIC SGI doorbell (matches the t113-evb SGI convention):
 *   core0 -> core1 : SGI15  (we receive)
 *   core1 -> core0 : SGI14  (we trigger)
 */

#define DOORBELL_EVENT    GIC_IRQ_SGI15            /* inbound: master kicks us */
#define DOORBELL_TRIGGER  GIC_IRQ_SGI14            /* outbound: we kick master */

/* Negotiated rpmsg feature set: the intersection of the NuttX rpmsg device
 * (offers NS|ACK|BUFSZ|CPUNAME|BUFADDR|PRIORITY) and the Vela Linux
 * virtio_rpmsg driver (supports NS|ACK|BUFSZ|CPUNAME).
 */

#define RSC_FEATURES \
  ((1u << VIRTIO_RPMSG_F_NS)    | (1u << VIRTIO_RPMSG_F_ACK) | \
   (1u << VIRTIO_RPMSG_F_BUFSZ) | (1u << VIRTIO_RPMSG_F_CPUNAME))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_rptun_rsc_s
{
  struct rptun_rsc_s rsc;
  uint8_t padding[T113_RSC_TABLE_SIZE - sizeof(struct rptun_rsc_s)];
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Static resource table in its own ELF section.  The master (Linux
 * remoteproc or core0-NuttX) finds it by section name and parses the vring
 * geometry out of it; our own get_resource (via rptun_bmp) returns the same
 * table.  ver=1 from the start means rptun_bmp_get_resource() never spins.
 *
 * NOT const: rptun_bmp's reboot notifier writes rsc_tbl_hdr.ver = 0 on
 * shutdown, so the table must live in a writable section.
 */

locate_data(".resource_table") used_data
struct rptun_rsc_s g_cpu1_rsc_table =
{
  .rsc_tbl_hdr =
  {
    .ver      = 1,
    .num      = 2,           /* RSC_VDEV + RSC_CARVEOUT */
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
    .dfeatures     = RSC_FEATURES,
    .gfeatures     = 0,
    .config_len    = sizeof(struct fw_rsc_config),
    .status        = 0,
    .num_of_vrings = 2,
  },

  /* Pinned vring DAs (non-zero): the framework keeps them as declared and
   * the master places its vrings at the same physical addresses.
   */

  .rpmsg_vring0 =
  {
    .da = VRING0_DA, .align = VRING_ALIGN, .num = VRING_NUM,
    .notifyid = VRING0_NOTIFYID, .reserved = 0,
  },

  .rpmsg_vring1 =
  {
    .da = VRING1_DA, .align = VRING_ALIGN, .num = VRING_NUM,
    .notifyid = VRING1_NOTIFYID, .reserved = 0,
  },

  .config =
  {
    .h2r_buf_size   = RPMSG_BUF_SIZE,
    .r2h_buf_size   = RPMSG_BUF_SIZE,
    .host_cpuname   = "core0",
    .remote_cpuname = "core1",
  },

  .carveout =
  {
    .type  = RSC_CARVEOUT,
    .da    = BUFPOOL_DA,
    .pa    = BUFPOOL_DA,
    .len   = BUFPOOL_LEN,
    .flags = 0,
    .name  = "vdev0buffer",
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_rptun_remap_shmem_nc
 *
 * Description:
 *   Remap a shared-memory region as Normal Non-Cacheable on the calling
 *   CPU before any cross-image read or write.  The DDR section that
 *   covers this address range is mapped as Normal/Cacheable by
 *   t113_setupmappings(), but neither CPU enables ACTLR.SMP / SCU in
 *   these splits, so cacheable writes from one core are not visible to
 *   the other.
 *
 *   This MUST be Normal memory, not Device or Strongly-Ordered: ARMv7-A
 *   Device/SO enforce strict natural alignment on every load/store, and
 *   OpenAMP / virtqueue code paths do incidental unaligned word accesses
 *   on the rpmsg buffer payload (e.g. memcpy through a header into a
 *   non-aligned offset) that abort on Device.  Normal Non-Cacheable
 *   keeps unaligned access legal while still bypassing the dcache, so
 *   no cache maintenance is needed on either side of the rpmsg ring.
 *
 *   The encoding is TEX=0b001, C=0, B=0 (with TRE=0, the SCTLR default).
 *
 *   Must be called once per CPU during board bringup, BEFORE the rptun
 *   driver touches the rsc_table.
 *
 ****************************************************************************/

#define T113_RPROC_SHMEM_FLAGS \
  (PMD_TYPE_SECT | PMD_SECT_AP_RW1 | (1 << PMD_SECT_TEX_SHIFT) | \
   PMD_SECT_S | PMD_SECT_DOM(0) | PMD_SECT_XN)

static void t113_rptun_remap_shmem_nc(uintptr_t base, size_t size)
{
  uintptr_t paddr = base;
  uintptr_t end   = base + size;

  for (; paddr < end; paddr += SECTION_SIZE)
    {
      /* DDR is identity-mapped (PA == VA). */

      mmu_l1_setentry(paddr, paddr, T113_RPROC_SHMEM_FLAGS);
    }

  /* Any line currently in dcache for the prior Cacheable mapping must
   * be invalidated so subsequent accesses go to physical memory.  After
   * this the region is non-cacheable and will not repopulate.
   */

  up_invalidate_dcache(base, base + size);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_rptun_init
 *
 * Description:
 *   core1 NuttX slave bring-up: ELF-loaded by a master (Linux remoteproc
 *   or core0-NuttX).  Remap the shared window non-cacheable, drop
 *   the life-sign markers, and hand the static ver=1 resource table to
 *   rptun_bmp on the GIC-SGI doorbell; the slave attaches at once and waits
 *   for the master's kicks on SGI15.
 *
 ****************************************************************************/

int t113_rptun_init(void)
{
  volatile uint32_t *lifesign = (volatile uint32_t *)(uintptr_t)LIFESIGN_DA;
  cpu_set_t cpuset;
  int ret;

  CPU_ZERO(&cpuset);

  t113_rptun_remap_shmem_nc(SHMEM_BASE, SHMEM_SIZE);

  /* First life-sign: core1 booted NuttX and reached slave init.  The cell
   * is in the just-remapped Non-Cacheable window, so this store reaches DRAM
   * at once -- readable by the master / a JLink AHB-AP probe with no cache
   * maintenance.  Clear marker [1] too: the carveout is no-map DRAM that is
   * not zeroed across a core1 re-deploy, so a stale RPTUN marker from a
   * prior boot must not read "attached" before rptun_bmp_init() returns.
   */

  lifesign[0] = LIFESIGN_MAGIC;
  lifesign[1] = 0;
  arm_dsb(15);

  CPU_SET(0, &cpuset);                  /* the peer (master) runs on core0 */

  ret = rptun_bmp_init("core0", false, &g_cpu1_rsc_table,
                       DOORBELL_EVENT, DOORBELL_TRIGGER, cpuset);

  /* Second life-sign: rptun slave fully attached (init returned). */

  lifesign[1] = LIFESIGN_RPTUN;
  arm_dsb(15);

  return ret;
}

/****************************************************************************
 * Name: rpmsg_serialinit
 *
 * Description:
 *   Called by drivers_initialize() under CONFIG_RPMSG_UART.  This slave
 *   binds an rpmsg-tty channel "CORE1" to the master peer (named "core0" in
 *   the static resource table), so the master's /dev/ttyCORE1 reaches this
 *   shell.  The physical UART2 is left to the DSP.
 *
 *   isconsole MUST be false here: drivers_initialize() runs this long before
 *   t113_rptun_init() attaches the vrings, so the rpmsg endpoint is not yet
 *   ready.  Registering it as /dev/console (isconsole=true) would route this
 *   image's early syslog through a not-ready endpoint and block boot when
 *   the xmit buffer fills -- which also stalls the master waiting for our NS
 *   announce.  Early core1 log goes to RAMLOG instead and is read back over
 *   /dev/ttyCORE1 once attached.
 *
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init("core0", "CORE1", 256, false);
}
#endif
