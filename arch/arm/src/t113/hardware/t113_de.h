/****************************************************************************
 * arch/arm/src/t113/hardware/t113_de.h
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

/* T113-S3 Display Engine 2.0 register layout - primary-plane bring-up
 * subset.
 *
 * Only the registers needed to bring up a single UI overlay (mixer global,
 * blender global + channel-0 input attributes, UI overlay channel 1 layer 0)
 * are declared here.  VI plane, scaler/VSU/GSU, CSC, ASE/FCC/FCE, secondary
 * mixer and color-key/CSC matrix are intentionally omitted.
 *
 * Hardware facts (offsets, bit positions and field shapes) were determined
 * from the T113-S3 User Manual v1.1 and cross-checked against the vendor
 * register layout for hardware fact-finding only.  No vendor source code,
 * type names or comments are reproduced here.
 *
 * Sub-module layout relative to T113_DE_BASE (= 0x05000000):
 *
 *   GLB (mixer global)              0x05100000  - sub-block size  ~0x40
 *   BLD (alpha blender, "APB")      0x05101000  - sub-block size  ~0x100
 *   OVL ch0 (VI plane)              0x05102000  - not used in bring-up
 *   OVL ch1 (UI plane 0, primary)   0x05103000  - bring-up target
 *   OVL ch2/ch3 (UI planes 1,2)     0x05104000/5000 - not used in bring-up
 */

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_DE_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_DE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "t113_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* -----------------------------------------------------------------------
 * Sub-module bases
 * -----------------------------------------------------------------------
 */

#define T113_DE_TOP_BASE    (T113_DE_BASE + 0x00000000)  /* DE module (per-block clock/reset/mux) */
#define T113_DE_GLB_BASE    (T113_DE_BASE + 0x00100000)
#define T113_DE_BLD_BASE    (T113_DE_BASE + 0x00101000)
#define T113_DE_UI0_BASE    (T113_DE_BASE + 0x00103000)  /* OVL ch1, UI plane 0 */

/* -----------------------------------------------------------------------
 * DE TOP register offsets (relative to T113_DE_TOP_BASE)
 *
 * The DE module exposes its own clock/reset/divider/mux registers separate
 * from the SoC-level CCU gates.  After enabling the bus from CCU side via
 * T113_CLK_DE_BUS / T113_CLK_DE, software must still assert the per-block
 * AHB gate at +0x04 and de-assert the AHB reset at +0x08 inside the DE
 * module before any GLB / BLD / UI register write will land - readbacks
 * from those blocks return 0x00040800 (bus-stuck pattern) until then.
 * -----------------------------------------------------------------------
 */

#define T113_DE_TOP_SCLK_GATE_OFFSET     0x0000  /* DE core module clock gate */
#define T113_DE_TOP_HCLK_GATE_OFFSET     0x0004  /* DE core AHB clock gate */
#define T113_DE_TOP_AHB_RESET_OFFSET     0x0008  /* DE core AHB reset (write 1 to release) */
#define T113_DE_TOP_DIV_OFFSET           0x000c  /* Per-core M divider */
#define T113_DE_TOP_MUX_OFFSET           0x0010  /* DE0/DE1 <-> TCON0/TCON1 mux */

/* Bit positions inside the DE TOP module-enable / gate / reset registers.
 * CORE0 is the DE0 mixer used by this driver; CORE1 / WB exist on bigger
 * SoCs and are intentionally not exposed.
 */

#define T113_DE_TOP_CORE0                (1u << 0)  /* DE0 module enable / gate / reset */

/* -----------------------------------------------------------------------
 * GLB register offsets (relative to T113_DE_GLB_BASE)
 * -----------------------------------------------------------------------
 */

#define T113_DE_GLB_CTL_OFFSET     0x0000  /* Global control (rt_en etc.) */
#define T113_DE_GLB_STATUS_OFFSET  0x0004  /* Status / IRQ flags (read-only) */
#define T113_DE_GLB_DBUFF_OFFSET   0x0008  /* Double-buffer ready trigger */
#define T113_DE_GLB_SIZE_OFFSET    0x000c  /* Mixer output size (see convention) */

/* -----------------------------------------------------------------------
 * BLD register offsets (relative to T113_DE_BLD_BASE)
 *
 * Per-channel input attributes form an array of 4 pipes at the head of the
 * block, but the array starts at offset 0x04 because offset 0x00 is the
 * blender-wide fcolor master enable.  Each pipe slot is 0x10 bytes:
 *   +0x00 : pipe FCOLOR (constant fill)
 *   +0x04 : pipe INSIZE (size-1 convention, see notes)
 *   +0x08 : pipe OFFSET (x/y origin in mixer space)
 *   +0x0c : reserved
 * For single UI bring-up only pipe 0 is programmed.
 * -----------------------------------------------------------------------
 */

#define T113_DE_BLD_FCOLOR_CTL_OFFSET    0x0000  /* Per-pipe fcolor & enable master */
#define T113_DE_BLD_PIPE_FCOLOR_OFFSET(n)  (0x0004 + (n) * 0x10)
#define T113_DE_BLD_PIPE_INSIZE_OFFSET(n)  (0x0008 + (n) * 0x10)
#define T113_DE_BLD_PIPE_OFFSET_OFFSET(n)  (0x000c + (n) * 0x10)

/* Global blender registers */

#define T113_DE_BLD_ROUTE_CTL_OFFSET     0x0080  /* Channel-to-pipe routing (4 bits per ch) */
#define T113_DE_BLD_PREMULT_CTL_OFFSET   0x0084  /* Per-pipe premultiplied-alpha mode */
#define T113_DE_BLD_BKCOLOR_OFFSET       0x0088  /* Background color (no plane) */
#define T113_DE_BLD_OUTPUT_SIZE_OFFSET   0x008c  /* Blender output size (size-1 convention) */

/* Per-pipe blend mode */

#define T113_DE_BLD_CTL_OFFSET(n)        (0x0090 + (n) * 0x04)

#define T113_DE_BLD_CK_CTL_OFFSET        0x00b0  /* Colorkey master control */
#define T113_DE_BLD_OUT_OFFSET           0x00fc  /* Output formatter / interlace / premul */

/* Offsets 0x00b4..0x00f8 carry colorkey config and the output CSC matrix.
 * They are reserved for bring-up and intentionally not declared here.
 */

/* -----------------------------------------------------------------------
 * UI overlay channel register offsets
 *
 * Each UI channel hosts an array of 4 layers (stride 0x20, 8 dwords each)
 * starting at offset 0.  Bring-up uses layer 0 only.  After the layer
 * array (0x00..0x7f) the global haddr/size registers begin at 0x80.
 * -----------------------------------------------------------------------
 */

#define T113_DE_UI_LAY_STRIDE        0x20

/* Per-layer offsets (offset within the UI channel base, for layer 0): */

#define T113_DE_UI_LAY_ATTR_OFFSET       0x0000  /* Layer attribute */
#define T113_DE_UI_LAY_SIZE_OFFSET       0x0004  /* Layer size (size-1 convention) */
#define T113_DE_UI_LAY_COOR_OFFSET       0x0008  /* Layer x/y origin in mixer space */
#define T113_DE_UI_LAY_PITCH_OFFSET      0x000c  /* Layer pitch in bytes */
#define T113_DE_UI_LAY_TOP_LADDR_OFFSET  0x0010  /* Top buffer low 32 bits */
#define T113_DE_UI_LAY_BOT_LADDR_OFFSET  0x0014  /* Bottom buffer low 32 bits (interlace) */
#define T113_DE_UI_LAY_FCOLOR_OFFSET     0x0018  /* Constant fill color */

/* Post-array overlay-global registers (offsets within the UI channel
 * base):
 */

#define T113_DE_UI_TOP_HADDR_OFFSET   0x0080  /* High 8 bits of TOP physaddr (per layer) */
#define T113_DE_UI_BOT_HADDR_OFFSET   0x0084  /* High 8 bits of BOT physaddr (per layer) */
#define T113_DE_UI_OVL_SIZE_OFFSET    0x0088  /* Overlay aperture size (size-1 convention) */

/* -----------------------------------------------------------------------
 * Bit-field macros - GLB
 * -----------------------------------------------------------------------
 */

/* T113_DE_GLB_CTL - global control */

#define T113_DE_GLB_CTL_RT_EN          (1u << 0)   /* Real-time pipeline enable */

/* T113_DE_GLB_DBUFF - double-buffer ready */

#define T113_DE_GLB_DBUFF_RDY          (1u << 0)   /* Latch shadow regs on next frame */

/* -----------------------------------------------------------------------
 * Size-register field convention
 *
 * For T113_DE_GLB_SIZE, T113_DE_BLD_OUTPUT_SIZE, T113_DE_UI_LAY_SIZE and
 * T113_DE_UI_OVL_SIZE the register is laid out as
 *
 *     bits[12: 0]  = (width  - 1)
 *     bits[28:16]  = (height - 1)
 *
 * i.e. the "size minus one" convention is used in ALL four size registers.
 * Programming the raw width/height (without the -1) yields a one-pixel
 * over-run that the mixer will clamp silently - hardware verified.
 * -----------------------------------------------------------------------
 */

#define T113_DE_SIZE_WIDTH_SHIFT       0
#define T113_DE_SIZE_WIDTH_MASK        (0x1fffu << T113_DE_SIZE_WIDTH_SHIFT)
#define T113_DE_SIZE_HEIGHT_SHIFT      16
#define T113_DE_SIZE_HEIGHT_MASK       (0x1fffu << T113_DE_SIZE_HEIGHT_SHIFT)

/* Helper: build a size register value from raw width/height (auto -1). */

#define T113_DE_SIZE_PACK(w, h) \
  ((((w) ? ((w) - 1) : 0) & 0x1fffu) | \
   ((((h) ? ((h) - 1) : 0) & 0x1fffu) << 16))

/* -----------------------------------------------------------------------
 * Bit-field macros - UI layer attribute
 * -----------------------------------------------------------------------
 */

/* T113_DE_UI_LAY_ATTR - layer attribute (per layer) */

#define T113_DE_UI_LAY_ATTR_LAY_EN         (1u << 0)
#define T113_DE_UI_LAY_ATTR_ALPMOD_SHIFT   1
#define T113_DE_UI_LAY_ATTR_ALPMOD_MASK    (0x3u << T113_DE_UI_LAY_ATTR_ALPMOD_SHIFT)
#define T113_DE_UI_LAY_ATTR_ACCESS_SW      (1u << 3)
#define T113_DE_UI_LAY_ATTR_FCOLOR_EN      (1u << 4)
#define T113_DE_UI_LAY_ATTR_FMT_SHIFT      8
#define T113_DE_UI_LAY_ATTR_FMT_MASK       (0x1fu << T113_DE_UI_LAY_ATTR_FMT_SHIFT)
#define T113_DE_UI_LAY_ATTR_ALPCTL_SHIFT   16
#define T113_DE_UI_LAY_ATTR_ALPCTL_MASK    (0x3u << T113_DE_UI_LAY_ATTR_ALPCTL_SHIFT)
#define T113_DE_UI_LAY_ATTR_TOP_DOWN       (1u << 23)
#define T113_DE_UI_LAY_ATTR_ALPHA_SHIFT    24
#define T113_DE_UI_LAY_ATTR_ALPHA_MASK     (0xffu << T113_DE_UI_LAY_ATTR_ALPHA_SHIFT)

/* T113_DE_UI_LAY_COOR - layer origin */

#define T113_DE_UI_LAY_COOR_X_SHIFT        0
#define T113_DE_UI_LAY_COOR_X_MASK         (0xffffu << 0)
#define T113_DE_UI_LAY_COOR_Y_SHIFT        16
#define T113_DE_UI_LAY_COOR_Y_MASK         (0xffffu << 16)

/* -----------------------------------------------------------------------
 * UI layer pixel format constants  (lay_fmt field, 5 bits)
 * -----------------------------------------------------------------------
 */

#define T113_DE_UI_FMT_ARGB8888   0x00  /* MSB:A-R-G-B:LSB, 32 bpp */
#define T113_DE_UI_FMT_XRGB8888   0x04  /*       X-R-G-B,    32 bpp */
#define T113_DE_UI_FMT_RGB888     0x0a  /* 24 bpp, packed R-G-B     */
#define T113_DE_UI_FMT_RGB565     0x08  /* 16 bpp                   */

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* Sub-module register accessors.  T113 has a 1-to-1 device/virtual mapping
 * (no MMU in the boot path), so these reduce to plain volatile pointer
 * dereferences.
 */

static inline uint32_t t113_de_top_getreg(uint32_t off)
{
  return *(volatile uint32_t *)(T113_DE_TOP_BASE + off);
}

static inline void t113_de_top_putreg(uint32_t off, uint32_t val)
{
  *(volatile uint32_t *)(T113_DE_TOP_BASE + off) = val;
}

static inline uint32_t t113_de_glb_getreg(uint32_t off)
{
  return *(volatile uint32_t *)(T113_DE_GLB_BASE + off);
}

static inline void t113_de_glb_putreg(uint32_t off, uint32_t val)
{
  *(volatile uint32_t *)(T113_DE_GLB_BASE + off) = val;
}

static inline uint32_t t113_de_bld_getreg(uint32_t off)
{
  return *(volatile uint32_t *)(T113_DE_BLD_BASE + off);
}

static inline void t113_de_bld_putreg(uint32_t off, uint32_t val)
{
  *(volatile uint32_t *)(T113_DE_BLD_BASE + off) = val;
}

static inline uint32_t t113_de_ui0_getreg(uint32_t off)
{
  return *(volatile uint32_t *)(T113_DE_UI0_BASE + off);
}

static inline void t113_de_ui0_putreg(uint32_t off, uint32_t val)
{
  *(volatile uint32_t *)(T113_DE_UI0_BASE + off) = val;
}

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_DE_H */
