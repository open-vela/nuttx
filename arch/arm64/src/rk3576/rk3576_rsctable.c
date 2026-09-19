/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_rsctable.c
 *
 * Static OpenAMP resource table for the RK3576 AMP slave.
 *
 * The Linux master (drivers/rpmsg/rockchip_rpmsg_softirq.c) uses fixed
 * physical vring addresses and never exchanges a resource table, so we
 * describe the exact same geometry here and expose it through rptun's
 * get_resource op.  Everything must match the Linux side bit-for-bit:
 *
 *   vring0  0x47800000   (Linux rvq / our tvq)
 *   vring1  0x47808000   (Linux svq / our rvq), stride RPMSG_VRING_SIZE
 *   64 descriptors, 0x1000 align, 512B buffers
 *   features: name-service announcement only (no CPUNAME private bit —
 *             Linux virtio_rpmsg_bus does not understand it)
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/rptun/rptun.h>
#include <openamp/open_amp.h>

#include "rk3576_rptun.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NUM_VRINGS              0x02
#define VRING_COUNT             64          /* RPMSG_BUF_COUNT (Linux)   */
#define VRING_ALIGN             0x1000
#define VDEV0_VRING_BASE        RK3576_AMP_VRING0_BASE
#define VRING_SIZE              0x8000      /* RPMSG_VRING_SIZE (Linux)  */

#define NO_RESOURCE_ENTRIES     (2)

/* Device features.  NuttX's rpmsg_virtio DEBUGASSERTs that CPUNAME is
 * present (it reads the local/peer cpu names from the config space below);
 * NS is the actual name-service announcement bit that Linux understands.
 * CPUNAME is read locally and never crosses the wire, so it is invisible to
 * the Linux master.
 */

#define RSC_VDEV_FEATURES       ((1U << VIRTIO_RPMSG_F_NS) | \
                                 (1U << VIRTIO_RPMSG_F_CPUNAME))
#define RK3576_RSC_TABLE_VERSION (1)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* NOT const: OpenAMP's resource-table parser writes back into this table
 * (vdev status, vring notifyids, etc.) during rproc init, so it must live
 * in writable memory.  Placing it in .rodata causes a permission fault
 * (ESR EC=0x25 DFSC=0x0f) inside handle_vdev_rsc().
 */

struct rptun_rsc_s g_rk3576_rsc_table =
{
  .rsc_tbl_hdr =
  {
    RK3576_RSC_TABLE_VERSION,
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
    RSC_VDEV,
    VIRTIO_ID_RPMSG,        /* 7 */
    2,                      /* notifyid */
    RSC_VDEV_FEATURES,      /* dfeatures: NS + CPUNAME */

    /* gfeatures: normally written by the driver after negotiation, but the
     * Linux master never touches this table, so pre-set it equal to
     * dfeatures.  openamp uses (dfeatures & gfeatures) as the active set.
     */

    RSC_VDEV_FEATURES,

    /* config_len: size of the fw_rsc_config that immediately follows the
     * vrings in this table.  openamp locates the virtio config space there;
     * with config_len=0 the DEVICE side reads an empty cpuname and the
     * uart_rpmsg binding (strcmp against "linux") silently fails.
     */

    sizeof(struct fw_rsc_config),

    /* status: pre-set DRIVER_OK.  The standard OpenAMP handshake has the
     * master write DRIVER_OK into a *shared* resource table for the remote
     * (device) side to poll, but our master is Linux rockchip_rpmsg_softirq
     * which never touches a resource table (it uses fixed vring addresses).
     * So we assert master-ready ourselves; the Linux side has already
     * initialised both vrings by the time this table is parsed.
     */

    VIRTIO_CONFIG_STATUS_DRIVER_OK,
    NUM_VRINGS,
    {
      0, 0
    }
  },

  .rpmsg_vring0 =
  {
    VDEV0_VRING_BASE,
    VRING_ALIGN,
    VRING_COUNT,
    0,                      /* notifyid 0 */
    0
  },

  .rpmsg_vring1 =
  {
    VDEV0_VRING_BASE + VRING_SIZE,
    VRING_ALIGN,
    VRING_COUNT,
    1,                      /* notifyid 1 */
    0
  },

  .config =
  {
    0, 0,                   /* h2r/r2h buf size (no BUFSZ feature) */
    0, 0,                   /* h2r/r2h buf addr (no BUFADDR feature) */
    "linux",                /* host_cpuname: the Linux master */
    "nuttx",                /* remote_cpuname: us (cpu3) */
    0,                      /* priority (no PRIORITY feature) */
    {0, 0, 0},              /* reserved1 */
    {0, 0, 0, 0, 0}         /* reserved2 */
  },

  /* rpmsg buffer pool (Linux allocates from its rpmsg-dma reserved region;
   * on our side it is the fixed carve-out at RK3576_AMP_BUFPOOL_BASE).
   */

  .carveout =
  {
    RSC_CARVEOUT,
    RK3576_AMP_BUFPOOL_BASE,
    RK3576_AMP_BUFPOOL_BASE,
    RK3576_AMP_BUFPOOL_SIZE,
    0, 0,
    "rpmsg_shm"
  }
};
