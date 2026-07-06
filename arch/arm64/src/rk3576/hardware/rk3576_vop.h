/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_vop.h
 *
 * Video Output Processor (VOP) register definitions for RK3576
 * Supports 2 VOP controllers, each with up to 4 window layers
 ****************************************************************************/

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_VOP_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_VOP_H

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/* VOP controller count */

#define RK3576_VOP_COUNT          2

/* VOP global registers */

#define VOP_REG_SYS_CTRL         0x0000
#define VOP_REG_DSP_CTRL0        0x0004
#define VOP_REG_DSP_CTRL1        0x0008
#define VOP_REG_DSP_BG           0x000c
#define VOP_REG_INTEN            0x0010
#define VOP_REG_INTSTS           0x0014
#define VOP_REG_WIN0_CTRL        0x0020
#define VOP_REG_WIN0_YRGB_MST   0x0024
#define VOP_REG_WIN0_CBR_MST    0x0028
#define VOP_REG_WIN0_VIR         0x002c
#define VOP_REG_WIN0_ACT         0x0030
#define VOP_REG_WIN0_DSP_POS    0x0034
#define VOP_REG_WIN0_SCL_FACTOR  0x0038
#define VOP_REG_WIN0_ALPHA_CTRL  0x003c
#define VOP_REG_SYS_CTRL1       0x0040
#define VOP_REG_LINE_FLAG        0x0050
#define VOP_REG_WIN0_SCL_YRGB   0x0060
#define VOP_REG_WIN0_SCL_CBR    0x0064
#define VOP_REG_VERSION          0x00fc

/* Window layer register offsets (each window has 0x80 stride) */

#define VOP_WIN_STRIDE           0x0080

/* Window register offsets (relative to window base) */

#define VOP_WIN_CTRL             0x0000
#define VOP_WIN_YRGB_MST         0x0004
#define VOP_WIN_CBR_MST          0x0008
#define VOP_WIN_VIR              0x000c
#define VOP_WIN_ACT              0x0010
#define VOP_WIN_DSP_POS          0x0014
#define VOP_WIN_SCL_FACTOR       0x0018
#define VOP_WIN_ALPHA_CTRL       0x001c
#define VOP_WIN_SCL_YRGB         0x0040
#define VOP_WIN_SCL_CBR          0x0044

/* VOP_SYS_CTRL bits */

#define VOP_SYS_CTRL_EN          (1 << 0)
#define VOP_SYS_CTRL_DMA_EN      (1 << 1)

/* VOP_DSP_CTRL0 bits */

#define VOP_DSP_CTRL0_DCLK_EN    (1 << 0)
#define VOP_DSP_CTRL0_DSP_OUT_EN (1 << 1)
#define VOP_DSP_CTRL0_DSP_DATA_MODE_SHIFT 6
#define VOP_DSP_CTRL0_DSP_DATA_MODE_MASK  (0x3 << 6)
#define VOP_DSP_CTRL0_DSP_BG_MODE (1 << 15)

/* VOP_DSP_CTRL0 data mode values */

#define VOP_DSP_DATA_MODE_VIDEO   0
#define VOP_DSP_DATA_MODE_COMMAND 1

/* VOP_DSP_CTRL1 bits */

#define VOP_DSP_CTRL1_WIN0_ALPHA_MODE_SHIFT 0
#define VOP_DSP_CTRL1_WIN0_ALPHA_MODE_MASK  (0x3 << 0)
#define VOP_DSP_CTRL1_WIN1_ALPHA_MODE_SHIFT 2
#define VOP_DSP_CTRL1_WIN1_ALPHA_MODE_MASK  (0x3 << 2)
#define VOP_DSP_CTRL1_MIX_COLOR_SHIFT 8
#define VOP_DSP_CTRL1_MIX_COLOR_MASK  (0x3 << 8)

/* VOP_DSP_BG bits (background color, 0xAARRGGBB) */

#define VOP_DSP_BG_ALPHA_SHIFT   24
#define VOP_DSP_BG_RED_SHIFT     16
#define VOP_DSP_BG_GREEN_SHIFT   8
#define VOP_DSP_BG_BLUE_SHIFT    0

/* VOP_INTEN bits */

#define VOP_INTEN_VSYNC_EN       (1 << 0)
#define VOP_INTEN_LINE_FLAG_EN   (1 << 1)
#define VOP_INTEN_WIN0_EMPTY_EN  (1 << 4)
#define VOP_INTEN_WIN0_IP_EN     (1 << 5)
#define VOP_INTEN_WIN1_EMPTY_EN  (1 << 6)
#define VOP_INTEN_DMA_FINISH_EN  (1 << 8)
#define VOP_INTENPostBack_EN     (1 << 9)
#define VOP_INTEN_FENCE_EN       (1 << 10)

/* VOP_INTSTS bits */

#define VOP_INTSTS_VSYNC         (1 << 0)
#define VOP_INTSTS_LINE_FLAG     (1 << 1)
#define VOP_INTSTS_WIN0_EMPTY    (1 << 4)
#define VOP_INTSTS_WIN0_IP       (1 << 5)
#define VOP_INTSTS_WIN1_EMPTY    (1 << 6)
#define VOP_INTSTS_DMA_FINISH    (1 << 8)
#define VOP_INTSTSPostBack       (1 << 9)
#define VOP_INTSTS_FENCE         (1 << 10)

/* VOP_WINx_CTRL bits */

#define VOP_WIN_CTRL_EN          (1 << 0)
#define VOP_WIN_CTRL_FORMAT_SHIFT 1
#define VOP_WIN_CTRL_FORMAT_MASK (0x7 << 1)
#define VOP_WIN_CTRL_FMT_ARGB8888  (0 << 1)
#define VOP_WIN_CTRL_FMT_RGB888    (1 << 1)
#define VOP_WIN_CTRL_FMT_RGB565    (2 << 1)
#define VOP_WIN_CTRL_FMT_ARGB1555  (3 << 1)
#define VOP_WIN_CTRL_FMT_YUYV      (4 << 1)
#define VOP_WIN_CTRL_FMT_YUV420    (5 << 1)
#define VOP_WIN_CTRL_FMT_YUV422    (6 << 1)

#define VOP_WIN_CTRL_MIRROR_SHIFT 4
#define VOP_WIN_CTRL_MIRROR_MASK  (0x3 << 4)
#define VOP_WIN_CTRL_MIRROR_NONE  (0 << 4)
#define VOP_WIN_CTRL_MIRROR_H     (1 << 4)
#define VOP_WIN_CTRL_MIRROR_V     (2 << 4)
#define VOP_WIN_CTRL_MIRROR_HV    (3 << 4)

#define VOP_WIN_CTRL_INTERLACE   (1 << 8)
#define VOP_WIN_CTRL_RB_SWAP     (1 << 10)
#define VOP_WIN_CTRL_ALPHA_SWAP  (1 << 11)

#define VOP_WIN_CTRL_YUV_MODE_SHIFT 12
#define VOP_WIN_CTRL_YUV_MODE_MASK  (0x3 << 12)
#define VOP_WIN_CTRL_YUV_MODE_16   (0 << 12)
#define VOP_WIN_CTRL_YUV_MODE_24   (1 << 12)
#define VOP_WIN_CTRL_YUV_MODE_30   (2 << 12)

/* VOP_WINx_ALPHA_CTRL bits */

#define VOP_WIN_ALPHA_CTRL_EN    (1 << 0)
#define VOP_WIN_ALPHA_CTRL_MODE_SHIFT 1
#define VOP_WIN_ALPHA_CTRL_MODE_MASK  (0x3 << 1)
#define VOP_WIN_ALPHA_MODE_PIXEL  (0 << 1)  /* Per-pixel alpha */
#define VOP_WIN_ALPHA_MODE_PLANE  (1 << 1)  /* Plane alpha */
#define VOP_WIN_ALPHA_MODE_COMBO  (2 << 1)  /* Combination */

#define VOP_WIN_ALPHA_CTRL_FACTOR_SHIFT 4
#define VOP_WIN_ALPHA_CTRL_FACTOR_MASK  (0xff << 4)

/* VOP format definitions */

#define VOP_FMT_ARGB8888         0
#define VOP_FMT_RGB888           1
#define VOP_FMT_RGB565           2
#define VOP_FMT_ARGB1555         3
#define VOP_FMT_YUYV             4
#define VOP_FMT_YUV420           5
#define VOP_FMT_YUV422           6

/* VOP max layers per controller */

#define VOP_MAX_WINDOWS          4

/* VOP scaling factor (1.0 = 0x10000) */

#define VOP_SCL_SCALE_1X         0x00010000
#define VOP_SCL_SCALE_2X         0x00020000

#ifndef __ASSEMBLY__

extern const uint32_t g_vop_base[RK3576_VOP_COUNT];

#endif /* __ASSEMBLY__ */

#endif
