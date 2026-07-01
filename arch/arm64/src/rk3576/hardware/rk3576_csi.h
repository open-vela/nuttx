/***************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_csi.h
 *
 * MIPI CSI-2 Receiver register definitions for RK3576
 *
 ***************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CSI_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_CSI_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* CSI controller count */

#define RK3576_CSI_COUNT           2

/* CSI register offsets */

#define CSI_MACIF_CTL              0x0000  /* MAC interface control */
#define CSI_MACIF_CMD              0x0004  /* MAC interface command */
#define CSI_CLKAdminController     0x0008  /* Clock auto control */
#define CSI_DPHY_CTL               0x000c  /* DPHY control */
#define CSI_DPHY_CLK_LANE          0x0010  /* Clock lane setting */
#define CSI_DPHY_DATA0_LANE        0x0014  /* Data lane 0 setting */
#define CSI_DPHY_DATA1_LANE        0x0018  /* Data lane 1 setting */
#define CSI_DPHY_DATA2_LANE        0x001c  /* Data lane 2 setting */
#define CSI_DPHY_DATA3_LANE        0x0020  /* Data lane 3 setting */
#define CSI_DPHY_TIMING            0x0024  /* DPHY timing */
#define CSI_INT_ENABLE             0x0030  /* Interrupt enable */
#define CSI_INT_CLEAR              0x0034  /* Interrupt clear */
#define CSI_INT_STATUS             0x0038  /* Interrupt status */
#define CSI_RAW_WR_INT_STATUS      0x003c  /* Raw write interrupt status */
#define CSI_FRAME_NUM              0x0040  /* Frame number */
#define CSI_LINE_CNTR              0x0044  /* Line counter */
#define CSI_DATA_IDS_1             0x0048  /* Data type identification 1 */
#define CSI_DATA_IDS_2             0x004c  /* Data type identification 2 */
#define CSI_ERR_INT_STATUS         0x0050  /* Error interrupt status */
#define CSI_ERR_INT_ENABLE         0x0054  /* Error interrupt enable */
#define CSI_MASK_INT_STATUS        0x0058  /* Mask interrupt status */
#define CSI_ST_DATA_END            0x005c  /* Short packet data end */
#define CSI_SPARSE_STREAM          0x0060  /* Sparse stream */
#define CSI_IMAGE_SIZE             0x0064  /* Image size */
#define CSI_LINE_GAP               0x0068  /* Line gap */

/* CSI MACIF_CTL bits */

#define CSI_MACIF_CTL_ENABLE       (1 << 0)
#define CSI_MACIF_CTL_MODE_SHIFT   4
#define CSI_MACIF_CTL_MODE_MASK    (0x3 << 4)
#define CSI_MACIF_CTL_MODE_VIDEO   (0 << 4)
#define CSI_MACIF_CTL_MODE_CMD     (1 << 4)
#define CSI_MACIF_CTL_SHUTDWN      (1 << 6)
#define CSI_MACIF_CTL_DMA_EN       (1 << 8)
#define CSI_MACIF_CTL_FORMAT_SHIFT 12
#define CSI_MACIF_CTL_FORMAT_MASK  (0xf << 12)

/* CSI_MACIF_CMD bits */

#define CSI_MACIF_CMD_DMA_START    (1 << 0)
#define CSI_MACIF_CMD_DMA_STOP     (1 << 1)

/* CSI_DPHY_CTL bits */

#define CSI_DPHY_CTL_PD            (1 << 0)  /* Power down */
#define CSI_DPHY_CTL_RST           (1 << 1)  /* Reset */
#define CSI_DPHY_CTL_ENABLE        (1 << 2)  /* Enable */
#define CSI_DPHY_CTL_TEST_MODE     (1 << 8)  /* Test mode */
#define CSI_DPHY_CTL_TEST_CLK      (1 << 9)
#define CSI_DPHY_CTL_TEST_CLR      (1 << 10)
#define CSI_DPHY_CTL_TEST_EN       (1 << 11)
#define CSI_DPHY_CTL_LANE_EN_SHIFT 16
#define CSI_DPHY_CTL_LANE_EN_MASK  (0xf << 16)

/* CSI_INT_STATUS / CSI_INT_ENABLE bits */

#define CSI_INT_FRAME_START        (1 << 0)
#define CSI_INT_FRAME_END          (1 << 1)
#define CSI_INT_LINE_START         (1 << 2)
#define CSI_INT_LINE_END           (1 << 3)
#define CSI_INT_OVERFLOW           (1 << 4)
#define CSI_INT_FRAME_DROP         (1 << 5)
#define CSI_INT_ECC_ERROR          (1 << 6)
#define CSI_INT_CRC_ERROR          (1 << 7)

/* CSI_ERR_INT_STATUS bits */

#define CSI_ERR_PHY                (1 << 0)
#define CSI_ERR_PKT_HDR            (1 << 1)
#define CSI_ERR_ECC                (1 << 2)
#define CSI_ERR_CRC                (1 << 3)
#define CSI_ERR_OVERFLOW           (1 << 4)

/* CSI_DATA_IDS_1 bits — data type identification */

#define CSI_DATA_IDS_DT_SHIFT(n)   ((n) * 8)
#define CSI_DATA_IDS_DT_MASK(n)    (0x3f << CSI_DATA_IDS_DT_SHIFT(n))

/* CSI data types */

#define CSI_DT_YUV422_8BIT         0x1e
#define CSI_DT_YUV422_10BIT        0x20
#define CSI_DT_RGB565              0x22
#define CSI_DT_RGB888              0x24
#define CSI_DT_RAW6                0x28
#define CSI_DT_RAW7                0x29
#define CSI_DT_RAW8                0x2a
#define CSI_DT_RAW10               0x2b
#define CSI_DT_RAW12               0x2c
#define CSI_DT_RAW14               0x2d
#define CSI_DT_CUSTOM              0x30

/* CSI image formats */

#define CSI_FMT_YUV422_8BIT        0
#define CSI_FMT_RGB888             1
#define CSI_FMT_RAW8               2
#define CSI_FMT_RAW10              3
#define CSI_FMT_RAW12              4

/* Timeouts */

#define CSI_TIMEOUT_MS             100

#ifndef __ASSEMBLY__

extern const uint32_t g_csi_base[RK3576_CSI_COUNT];

#endif /* __ASSEMBLY__ */

#endif
