/****************************************************************************
 * arch/arm/src/t113/hardware/t113_tcon.h
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

/* T113-S3 TCON-LCD0 + display_if_top register layout.
 *
 * TCON-LCD0  : T113_TCON_LCD0_BASE   = 0x05461000
 * display_if_top (LCDTOP) : T113_DISPLAY_TOP_BASE = 0x05460000
 *
 * Only the subset of registers used by the MIPI DSI bring-up path is
 * declared here.  TCON1, CEU, gamma, CMAP, sync, fill and CPU-mode trigger
 * registers are intentionally omitted.
 *
 * Source: T113-S3 User Manual v1.1 section 5.5 (TCON), cross-checked against
 * vendor lowlevel_v2x register layout for hardware fact-finding only.
 */

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_TCON_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_TCON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "t113_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets - display_if_top (relative to T113_DISPLAY_TOP_BASE) */

#define T113_LCDTOP_TV_SETUP_OFFSET     0x0000  /* TV setup (unused) */
#define T113_LCDTOP_DSI_SRC_SEL_OFFSET  0x0004  /* DSI source select */
#define T113_LCDTOP_VDPO_SRC_SEL_OFFSET 0x0008  /* VDPO source select (unused) */
#define T113_LCDTOP_TCON_DE_PERH_OFFSET 0x0018  /* TCON <-> DE interconnect */
#define T113_LCDTOP_TCON_CLK_GATE_OFFSET 0x0020 /* TCON clock gating (CCU-managed) */

/* Register offsets - TCON-LCD0 (relative to T113_TCON_LCD0_BASE) */

/* Global control / interrupt */

#define T113_TCON_GCTL_OFFSET           0x0000  /* Global control */
#define T113_TCON_GINT0_OFFSET          0x0004  /* IRQ flag [15:0] + IRQ enable [31:16] */
#define T113_TCON_GINT1_OFFSET          0x0008  /* TCON0/1 video-line IRQ counters */

/* TCON-LCD0 timing block */

#define T113_TCON_TCON0_CTL_OFFSET      0x0040  /* TCON0 main control */
#define T113_TCON_TCON0_DCLK_OFFSET     0x0044  /* TCON0 dot-clock divider / enable */
#define T113_TCON_TCON0_BASIC0_OFFSET   0x0048  /* Active size: x[27:16], y[11:0] */
#define T113_TCON_TCON0_BASIC1_OFFSET   0x004c  /* Horizontal: ht[28:16], hbp[11:0] */
#define T113_TCON_TCON0_BASIC2_OFFSET   0x0050  /* Vertical:   vt[28:16], vbp[11:0] */
#define T113_TCON_TCON0_BASIC3_OFFSET   0x0054  /* Sync widths: hspw, vspw */
#define T113_TCON_TCON0_HV_CTL_OFFSET   0x0058  /* HV-interface control */
#define T113_TCON_TCON0_CPU_CTL_OFFSET  0x0060  /* CPU-mode (DSI / 8080) main control */
#define T113_TCON_TCON0_IO_POL_OFFSET   0x0088  /* IO polarity (DCLK / HSYNC / VSYNC / DE) */
#define T113_TCON_TCON0_IO_TRI_OFFSET   0x008c  /* IO tri-state control */
#define T113_TCON_TCON0_CPU_TRI0_OFFSET 0x0160  /* CPU-mode trigger 0 (block size / space) */
#define T113_TCON_TCON0_CPU_TRI1_OFFSET 0x0164  /* CPU-mode trigger 1 (block num) */
#define T113_TCON_TCON0_CPU_TRI2_OFFSET 0x0168  /* CPU-mode trigger 2 (sync / start) */
#define T113_TCON_VOLUME_CTL_OFFSET     0x01f0  /* Safe-period FIFO threshold + mode */

/* -----------------------------------------------------------------------
 * Bit-field macros - display_if_top
 * -----------------------------------------------------------------------
 */

/* T113_LCDTOP_DSI_SRC_SEL - DSI source select (single-bit per host) */

#define T113_LCDTOP_DSI0_SRC_SEL        (1 << 0)   /* 0 = TCON-LCD0, 1 = TCON-LCD1 */
#define T113_LCDTOP_DSI1_SRC_SEL        (1 << 4)   /* 0 = TCON-LCD0, 1 = TCON-LCD1 */

/* -----------------------------------------------------------------------
 * Bit-field macros - TCON-LCD0 global control
 * -----------------------------------------------------------------------
 */

/* T113_TCON_GCTL - global control */

#define T113_TCON_GCTL_IO_MAP_SEL       (1 << 0)   /* 0 = TCON0, 1 = TCON1 on IO pads */
#define T113_TCON_GCTL_PAD_SEL          (1 << 1)   /* Pad mux select */
#define T113_TCON_GCTL_GAMMA_EN         (1u << 30) /* Gamma table enable */
#define T113_TCON_GCTL_TCON_EN          (1u << 31) /* Module enable */

/* T113_TCON_GINT0 - IRQ flag [15:0] + IRQ enable [31:16] */

#define T113_TCON_GINT0_IRQ_FLAG_SHIFT  0
#define T113_TCON_GINT0_IRQ_FLAG_MASK   (0xffff << 0)
#define T113_TCON_GINT0_IRQ_EN_SHIFT    16
#define T113_TCON_GINT0_IRQ_EN_MASK     (0xffffu << 16)

/* T113_TCON_GINT1 - line-count interrupts */

#define T113_TCON_GINT1_TCON1_LINE_SHIFT 0
#define T113_TCON_GINT1_TCON1_LINE_MASK  (0xfff << 0)
#define T113_TCON_GINT1_TCON0_LINE_SHIFT 16
#define T113_TCON_GINT1_TCON0_LINE_MASK  (0xfff << 16)

/* -----------------------------------------------------------------------
 * Bit-field macros - TCON-LCD0 channel
 * -----------------------------------------------------------------------
 */

/* T113_TCON_TCON0_CTL - main control */

#define T113_TCON_CTL_SRC_SEL_SHIFT     0
#define T113_TCON_CTL_SRC_SEL_MASK      (0x7 << 0) /* 0=DE0, 1=DE1, 4=blue color bar */
#define T113_TCON_CTL_START_DELAY_SHIFT 4
#define T113_TCON_CTL_START_DELAY_MASK  (0x1f << 4)
#define T113_TCON_CTL_INTERLACE_EN      (1 << 20)
#define T113_TCON_CTL_FIFO1_RST         (1 << 21)
#define T113_TCON_CTL_RB_SWAP           (1 << 23)
#define T113_TCON_CTL_IF_SHIFT          24
#define T113_TCON_CTL_IF_MASK           (0x3 << 24) /* 0=HV, 1=CPU, 2=LVDS, 3=DSI */
#define T113_TCON_CTL_WORK_MODE         (1 << 28)
#define T113_TCON_CTL_TCON0_EN          (1u << 31)

/* tcon0_if encodings */

#define T113_TCON_CTL_IF_HV             0
#define T113_TCON_CTL_IF_CPU            1
#define T113_TCON_CTL_IF_LVDS           2
#define T113_TCON_CTL_IF_DSI            3

/* T113_TCON_TCON0_DCLK - dot-clock divider / enable */

#define T113_TCON_DCLK_DIV_SHIFT        0
#define T113_TCON_DCLK_DIV_MASK         (0x7f << 0)  /* Divider [6:0] */
#define T113_TCON_DCLK_EN_SHIFT         28
#define T113_TCON_DCLK_EN_MASK          (0xfu << 28) /* Per-output dot-clock enable nibble */

/* T113_TCON_TCON0_BASIC0 - active picture size
 * (programmed value = pixels - 1)
 */

#define T113_TCON_BASIC0_Y_SHIFT        0
#define T113_TCON_BASIC0_Y_MASK         (0xfff << 0)  /* vactive - 1 */
#define T113_TCON_BASIC0_X_SHIFT        16
#define T113_TCON_BASIC0_X_MASK         (0xfff << 16) /* hactive - 1 */

/* T113_TCON_TCON0_BASIC1 - horizontal timing */

#define T113_TCON_BASIC1_HBP_SHIFT      0
#define T113_TCON_BASIC1_HBP_MASK       (0xfff << 0)   /* hbp + hsync - 1 */
#define T113_TCON_BASIC1_HT_SHIFT       16
#define T113_TCON_BASIC1_HT_MASK        (0x1fff << 16) /* htotal - 1 */

/* T113_TCON_TCON0_BASIC2 - vertical timing */

#define T113_TCON_BASIC2_VBP_SHIFT      0
#define T113_TCON_BASIC2_VBP_MASK       (0xfff << 0)   /* vbp + vsync - 1 */
#define T113_TCON_BASIC2_VT_SHIFT       16
#define T113_TCON_BASIC2_VT_MASK        (0x1fff << 16) /* 2 * vtotal */

/* T113_TCON_TCON0_BASIC3 - sync pulse widths */

#define T113_TCON_BASIC3_VSPW_SHIFT     0
#define T113_TCON_BASIC3_VSPW_MASK      (0x3ff << 0)   /* vsync width - 1 */
#define T113_TCON_BASIC3_HSPW_SHIFT     16
#define T113_TCON_BASIC3_HSPW_MASK      (0x3ff << 16)  /* hsync width - 1 */

/* T113_TCON_TCON0_HV_CTL - HV-interface control */

#define T113_TCON_HV_CCIR_CSC_DIS       (1 << 19)
#define T113_TCON_HV_SYUV_FDLY_SHIFT    20
#define T113_TCON_HV_SYUV_FDLY_MASK     (0x3 << 20)
#define T113_TCON_HV_SYUV_SEQ_SHIFT     22
#define T113_TCON_HV_SYUV_SEQ_MASK      (0x3 << 22)
#define T113_TCON_HV_SRGB_SEQ_SHIFT     24
#define T113_TCON_HV_SRGB_SEQ_MASK      (0xf << 24)
#define T113_TCON_HV_MODE_SHIFT         28
#define T113_TCON_HV_MODE_MASK          (0xfu << 28)

/* T113_TCON_TCON0_IO_POL - IO polarity */

#define T113_TCON_IOPOL_DATA_INV_SHIFT  0
#define T113_TCON_IOPOL_DATA_INV_MASK   (0xffffff << 0)
#define T113_TCON_IOPOL_SYNC_INV_SHIFT  24
#define T113_TCON_IOPOL_SYNC_INV_MASK   (0x3 << 24)   /* HSYNC/VSYNC active polarity */
#define T113_TCON_IOPOL_CLK_INV         (1 << 26)     /* Pixel-clock inversion */
#define T113_TCON_IOPOL_DE_INV          (1 << 27)     /* DE polarity */
#define T113_TCON_IOPOL_DCLK_SEL_SHIFT  28
#define T113_TCON_IOPOL_DCLK_SEL_MASK   (0x7 << 28)
#define T113_TCON_IOPOL_IO_OUTPUT_SEL   (1u << 31)

/* T113_TCON_TCON0_IO_TRI - IO tri-state */

#define T113_TCON_IOTRI_DATA_TRI_SHIFT  0
#define T113_TCON_IOTRI_DATA_TRI_MASK   (0xffffff << 0)
#define T113_TCON_IOTRI_IO0_TRI         (1 << 24)
#define T113_TCON_IOTRI_IO1_TRI         (1 << 25)
#define T113_TCON_IOTRI_IO2_TRI         (1 << 26)
#define T113_TCON_IOTRI_IO3_TRI         (1 << 27)
#define T113_TCON_IOTRI_RGB_ENDIAN      (1 << 28)

/* T113_TCON_TCON0_CPU_CTL - CPU-mode main control */

#define T113_TCON_CPU_CTL_TRIGGER_EN        (1u << 0)
#define T113_TCON_CPU_CTL_TRIGGER_START     (1u << 1)
#define T113_TCON_CPU_CTL_TRIGGER_FIFO_EN   (1u << 2)
#define T113_TCON_CPU_CTL_MODE_SHIFT        28
#define T113_TCON_CPU_CTL_MODE_MASK         (0xfu << T113_TCON_CPU_CTL_MODE_SHIFT)
#define T113_TCON_CPU_CTL_MODE_DSI          1u  /* cpu_mode = MODE_DSI */

/* T113_TCON_TCON0_CPU_TRI0 - block size / block space */

#define T113_TCON_CPU_TRI0_BLK_SIZE_SHIFT   0
#define T113_TCON_CPU_TRI0_BLK_SIZE_MASK    (0xfffu << 0)   /* hactive - 1 */
#define T113_TCON_CPU_TRI0_BLK_SPACE_SHIFT  16
#define T113_TCON_CPU_TRI0_BLK_SPACE_MASK   (0xfffu << 16)  /* line idle slots */

/* T113_TCON_TCON0_CPU_TRI1 - block num */

#define T113_TCON_CPU_TRI1_BLK_NUM_SHIFT    0
#define T113_TCON_CPU_TRI1_BLK_NUM_MASK     (0xffffu << 0)  /* vactive - 1 */

/* T113_TCON_TCON0_CPU_TRI2 - start delay + start set */

#define T113_TCON_CPU_TRI2_START_SET_SHIFT  0
#define T113_TCON_CPU_TRI2_START_SET_MASK   (0x1fffu << 0)
#define T113_TCON_CPU_TRI2_SYNC_MODE_SHIFT  13
#define T113_TCON_CPU_TRI2_SYNC_MODE_MASK   (0x3u << 13)
#define T113_TCON_CPU_TRI2_TRANS_MODE       (1u << 15)
#define T113_TCON_CPU_TRI2_START_DELAY_SHIFT 16
#define T113_TCON_CPU_TRI2_START_DELAY_MASK (0xffffu << 16)

/* T113_TCON_VOLUME_CTL - safe-period (FIFO under-run guard) */

#define T113_TCON_VOLUME_MODE_SHIFT         0
#define T113_TCON_VOLUME_MODE_MASK          (0x3u << 0)
#define T113_TCON_VOLUME_FIFO_NUM_SHIFT     16
#define T113_TCON_VOLUME_FIFO_NUM_MASK      (0x1fffu << 16)

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline uint32_t t113_tcon_getreg(uint32_t off)
{
  return getreg32(T113_TCON_LCD0_BASE + off);
}

static inline void t113_tcon_putreg(uint32_t off, uint32_t val)
{
  putreg32(val, T113_TCON_LCD0_BASE + off);
}

static inline uint32_t t113_lcdtop_getreg(uint32_t off)
{
  return getreg32(T113_DISPLAY_TOP_BASE + off);
}

static inline void t113_lcdtop_putreg(uint32_t off, uint32_t val)
{
  putreg32(val, T113_DISPLAY_TOP_BASE + off);
}

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_TCON_H */
