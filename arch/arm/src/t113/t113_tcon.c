/****************************************************************************
 * arch/arm/src/t113/t113_tcon.c
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

/* T113-S3 TCON-LCD0 timing controller driver.
 *
 * Programs the timing block that sits between the Display Engine (DE) and
 * the downstream display interface.  For the X4B MIPI-DSI bring-up the data
 * path is:
 *
 *   DE0 -> TCON-LCD0 (HV parallel pixel stream) -> display_if_top
 *          -> DSI host -> panel
 *
 * The DSI host serialises the parallel HV stream, so the tcon0_if field is
 * programmed to 0 (HV) even when the API caller selects DSI video mode.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include "arm_internal.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_tcon.h"
#include "t113_clk.h"
#include "t113_tcon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CCU TCON-LCD0 module (functional) clock register.  The CCU base symbol is
 * exported by hardware/t113_ccu.h; the gating helper inside t113_clk handles
 * bit31 (gate), but the source-mux / divider fields are programmed here so
 * the pixel-clock plan stays local to this driver.
 */

#define T113_CCU_TCON_LCD0_CLK_REG  (T113_CCU_BASE + 0x0b60)

/* Bit layout of CCU TCON-LCD0 module clock register (0x0b60).
 *
 *   [31]    gate (handled by t113_clk_enable())
 *   [26:24] CLK_SRC_SEL: 0 = HOSC (24 MHz), 1 = PLL_VIDEO0(1x),
 *                        2 = PLL_VIDEO0(2x), 3 = PLL_VIDEO1(1x),
 *                        4 = PLL_VIDEO1(2x)
 *   [23:8]  reserved
 *   [7:0]   FACTOR_M (M divider, output = src / (M+1))
 */

#define TCON_CLK_SRC_SHIFT          24
#define TCON_CLK_SRC_MASK           (0x7u << TCON_CLK_SRC_SHIFT)
#define TCON_CLK_SRC_HOSC           (0x0u << TCON_CLK_SRC_SHIFT)
#define TCON_CLK_M_SHIFT            0
#define TCON_CLK_M_MASK             (0xffu << TCON_CLK_M_SHIFT)

/* Module clock rate produced by tcon_module_clk_set().  HOSC is 24 MHz; with
 * M = 0 the TCON sees the full 24 MHz reference.  The per-channel dot clock
 * divider in TCON0_DCLK fans this down to the panel pixel rate.
 *
 * TODO: switch to PLL_VIDEO0 / PLL_PERI source for accurate 31 MHz pixel
 * clock once the basic DSI path is verified end-to-end.
 */

#define T113_TCON_MODULE_CLK_HZ     24000000u

/* LCDTOP "tcon_clk_gate" register layout (offset 0x0020).  Per the T113-S3
 * user manual / vendor headers the field map is:
 *
 *   [0]  vdpo0_clk_gate
 *   [1]  vdpo1_clk_gate
 *   [15:2]  reserved
 *   [16] dsi0 clock gate (TCON-LCD0 -> DSI0)
 *   [17] dsi1 clock gate
 *   ...
 */

#define T113_LCDTOP_DSI0_CLK_GATE   (1u << 16)

/* DSI bridge constants (vendor de_dsi.c).  The DSI host's pixel pipeline
 * runs at byte_clk = bitrate/8; the TCON dot clock pre-divides the byte
 * clock by tcon_div before driving the DSI host's pixel input.
 */

#define T113_TCON_DSI_TCON_DIV      4u
#define T113_TCON_DSI_VIDEO_HEAD    40u  /* HSA + HSE + HFP head bytes */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tcon_module_clk_set
 *
 * Description:
 *   Configure the TCON-LCD0 module/functional clock to HOSC / 1 = 24 MHz.
 *   The gate bit is left to t113_clk_enable(T113_CLK_TCON_LCD0).
 *
 ****************************************************************************/

static void tcon_module_clk_set(void)
{
  uint32_t reg = getreg32(T113_CCU_TCON_LCD0_CLK_REG);

  reg &= ~(TCON_CLK_SRC_MASK | TCON_CLK_M_MASK);
  reg |= TCON_CLK_SRC_HOSC;          /* parent = HOSC (24 MHz) */
  reg |= (0u << TCON_CLK_M_SHIFT);   /* M = 0 -> divide by 1 */
  putreg32(reg, T113_CCU_TCON_LCD0_CLK_REG);
}

/****************************************************************************
 * Name: tcon_program_timing
 *
 * Description:
 *   Program the TCON-LCD0 channel-0 timing block (BASIC0..3 / HV_CTL /
 *   IO_POL / IO_TRI) for the requested mode.  TCON itself is left disabled
 *   on return; the caller flips the enable bits as the last step.
 *
 ****************************************************************************/

static void tcon_program_timing(const struct t113_tcon_timing_s *timing,
                                uint32_t dclk_div)
{
  uint32_t hactive = timing->hactive;
  uint32_t hbp     = timing->hbp;
  uint32_t hfp     = timing->hfp;
  uint32_t hsync   = timing->hsync;
  uint32_t vactive = timing->vactive;
  uint32_t vbp     = timing->vbp;
  uint32_t vfp     = timing->vfp;
  uint32_t vsync   = timing->vsync;
  uint32_t htotal  = hactive + hbp + hfp + hsync;
  uint32_t vtotal  = vactive + vbp + vfp + vsync;
  uint32_t reg;
  int32_t  start_delay;
  uint32_t if_field;

  /* TCON0_CTL: source = DE0, interface depends on panel type.  For DSI
   * panels the TCON output goes through the DSI host as a parallel CPU
   * bus (IF=CPU=1), NOT raw HV.  Vendor BL dump confirms TCON0_CTL[25:24]
   * = 01 for the GC9503CV DSI panel.  Hardcoding IF=HV starved the DSI
   * host of pixel data - DPHY state looked perfect but no bytes flowed.
   */

  if (timing->if_type == T113_TCON_IF_DSI)
    {
      if_field = T113_TCON_CTL_IF_CPU;
    }
  else
    {
      if_field = T113_TCON_CTL_IF_HV;
    }

  reg = (if_field << T113_TCON_CTL_IF_SHIFT) & T113_TCON_CTL_IF_MASK;
  t113_tcon_putreg(T113_TCON_TCON0_CTL_OFFSET, reg);

  /* TCON0_DCLK: per-channel dot-clock divider + 4-bit output enable nibble.
   * Pixel rate at the panel = module_clk / dclk_div.
   */

  reg = ((dclk_div << T113_TCON_DCLK_DIV_SHIFT) & T113_TCON_DCLK_DIV_MASK) |
        T113_TCON_DCLK_EN_MASK;
  t113_tcon_putreg(T113_TCON_TCON0_DCLK_OFFSET, reg);

  /* BASIC0: active picture size minus 1 */

  reg = (((hactive - 1) << T113_TCON_BASIC0_X_SHIFT) &
         T113_TCON_BASIC0_X_MASK) |
        (((vactive - 1) << T113_TCON_BASIC0_Y_SHIFT) &
         T113_TCON_BASIC0_Y_MASK);
  t113_tcon_putreg(T113_TCON_TCON0_BASIC0_OFFSET, reg);

  /* BASIC1/2/3 + HV_CTL + IO_POL + IO_TRI are for raw HV interfaces.
   * Vendor BL dump confirms all zero for DSI bridge - h/v timing comes
   * from CPU_TRIG registers (0x1160/0x1164) instead.
   */

  if (timing->if_type != T113_TCON_IF_DSI)
    {
      reg = (((htotal - 1) << T113_TCON_BASIC1_HT_SHIFT) &
             T113_TCON_BASIC1_HT_MASK) |
            ((((hbp + hsync) - 1) << T113_TCON_BASIC1_HBP_SHIFT) &
             T113_TCON_BASIC1_HBP_MASK);
      t113_tcon_putreg(T113_TCON_TCON0_BASIC1_OFFSET, reg);

      reg = (((vtotal * 2u) << T113_TCON_BASIC2_VT_SHIFT) &
             T113_TCON_BASIC2_VT_MASK) |
            ((((vbp + vsync) - 1) << T113_TCON_BASIC2_VBP_SHIFT) &
             T113_TCON_BASIC2_VBP_MASK);
      t113_tcon_putreg(T113_TCON_TCON0_BASIC2_OFFSET, reg);

      reg = (((hsync - 1) << T113_TCON_BASIC3_HSPW_SHIFT) &
             T113_TCON_BASIC3_HSPW_MASK) |
            (((vsync - 1) << T113_TCON_BASIC3_VSPW_SHIFT) &
             T113_TCON_BASIC3_VSPW_MASK);
      t113_tcon_putreg(T113_TCON_TCON0_BASIC3_OFFSET, reg);

      t113_tcon_putreg(T113_TCON_TCON0_HV_CTL_OFFSET,
                       T113_TCON_HV_CCIR_CSC_DIS);
    }

  /* IO_POL: leave default polarities (all zero); panel-specific tuning is
   * deferred to a later phase.
   */

  t113_tcon_putreg(T113_TCON_TCON0_IO_POL_OFFSET, 0);

  /* IO_TRI: TCON-LCD0 outputs are routed internally to the DSI host, the
   * physical RGB pads are not driven externally.  Tri-state every output
   * bit so the pads stay high-Z.
   */

  t113_tcon_putreg(T113_TCON_TCON0_IO_TRI_OFFSET,
                   T113_TCON_IOTRI_DATA_TRI_MASK |
                   T113_TCON_IOTRI_IO0_TRI |
                   T113_TCON_IOTRI_IO1_TRI |
                   T113_TCON_IOTRI_IO2_TRI |
                   T113_TCON_IOTRI_IO3_TRI);

  /* start_delay = vtotal - vactive - 8, clamped to [0, 31].  Vendor BL
   * dump shows TCON0_CTL[8:4]=0 (start_delay=0) for the GC9503CV DSI
   * panel - the typical 10..31 range is for parallel RGB; DSI host
   * absorbs latency itself.
   */

  start_delay = (int32_t)vtotal - (int32_t)vactive - 8;
  if (start_delay < 0)
    {
      start_delay = 0;
    }
  else if (start_delay > 31)
    {
      start_delay = 31;
    }

  /* For DSI bridge mode the vendor BL dump shows START_DELAY=0; the DSI
   * host introduces its own pipeline latency.  For raw HV interfaces the
   * computed value is correct.
   */

  if (timing->if_type == T113_TCON_IF_DSI)
    {
      start_delay = 0;
    }

  reg = t113_tcon_getreg(T113_TCON_TCON0_CTL_OFFSET);
  reg &= ~T113_TCON_CTL_START_DELAY_MASK;
  reg |= ((uint32_t)start_delay << T113_TCON_CTL_START_DELAY_SHIFT) &
         T113_TCON_CTL_START_DELAY_MASK;
  t113_tcon_putreg(T113_TCON_TCON0_CTL_OFFSET, reg);
}

/****************************************************************************
 * Name: tcon_route_to_dsi0
 *
 * Description:
 *   Wire TCON-LCD0 to DSI host 0 inside display_if_top:
 *
 *     - DSI0_SRC_SEL = 0  (DSI0 takes its pixel stream from TCON-LCD0)
 *     - tcon_clk_gate.dsi0 = 1  (allow TCON dot clock through to DSI host)
 *
 ****************************************************************************/

static void tcon_route_to_dsi0(void)
{
  uint32_t reg;

  reg = t113_lcdtop_getreg(T113_LCDTOP_DSI_SRC_SEL_OFFSET);
  reg &= ~T113_LCDTOP_DSI0_SRC_SEL;
  t113_lcdtop_putreg(T113_LCDTOP_DSI_SRC_SEL_OFFSET, reg);

  reg = t113_lcdtop_getreg(T113_LCDTOP_TCON_CLK_GATE_OFFSET);
  reg |= T113_LCDTOP_DSI0_CLK_GATE;
  t113_lcdtop_putreg(T113_LCDTOP_TCON_CLK_GATE_OFFSET, reg);
}

/****************************************************************************
 * Name: tcon_program_cpu_dsi
 *
 * Description:
 *   Program the TCON CPU-mode trigger registers used by the DSI bridge path.
 *   Mirrors vendor lowlevel_v2x tcon0_cfg_mode_tri() for video DSI:
 *
 *     CPU_TRI0 = block_size  (hactive - 1)
 *              + block_space (htotal * dsi_bpp / (tcon_div * lanes)
 *                              - hactive - 40)
 *     CPU_TRI1 = block_num   (vactive - 1)
 *     CPU_TRI2 = trans_start_set = 10, sync_mode = 0, trans_mode = 0,
 *                start_delay = (vt - vy - 9) * ht * de_clk
 *                              / dclk_freq / 8
 *     VOLUME   = safe_period_mode = 3,
 *                safe_period_fifo_num = dclk_freq * 15
 *     CPU_CTL  = trigger_en | trigger_fifo_en | (cpu_mode = MODE_DSI)
 *
 *   On the DSI bridge path TCON_CTL.IF is left at HV (0 = parallel pixel
 *   to DSI host), but tcon0_if = 1 (CPU) is required by the vendor
 *   reference for the DSI command-mode-style packetisation that v40 DSI
 *   silicon uses; the CPU_CTL.cpu_mode field then selects MODE_DSI.
 *
 ****************************************************************************/

static void tcon_program_cpu_dsi(const struct t113_tcon_timing_s *t)
{
  uint32_t htotal = (uint32_t)t->hactive + t->hbp + t->hfp + t->hsync;
  uint32_t vtotal = (uint32_t)t->vactive + t->vbp + t->vfp + t->vsync;
  uint32_t bpp    = 24u;        /* RGB888 - only format we accept */
  uint32_t lanes  = (t->lanes != 0) ? t->lanes : 2u;
  uint32_t de_mhz = T113_TCON_MODULE_CLK_HZ / 1000000u;
  uint32_t dclk_mhz = t->pixel_clk_hz / 1000000u;
  uint32_t blk_space;
  uint32_t start_delay;
  uint32_t fifo_num;
  uint32_t reg;

  if (dclk_mhz == 0)
    {
      dclk_mhz = 1;
    }

  /* WARNING: this is the VENDOR FORMULA implementation, not the BL dump.
   * EVB4 vendor BL dump for the same X4B 480x800 @ 2-lane panel shows:
   *   CPU_TRI0 block_space = 0x18    (formula computes a different value)
   *   CPU_TRI2 start_delay = 0x6720  (formula computes a different value)
   * Vendor BL likely came from different timing or includes hand-tuned
   * margins not captured by tcon0_cfg_mode_tri().  We use the formula
   * because it is the documented derivation and reproduces correctly
   * across panel variants; if the panel fails to lock, shows scan tear,
   * or blanks intermittently, override these two values with the BL magic
   * numbers above as the first probe.
   */

  blk_space = htotal * bpp /
              (T113_TCON_DSI_TCON_DIV * lanes) -
              t->hactive - T113_TCON_DSI_VIDEO_HEAD;

  /* start_delay: formula computes from timing; EVB4 BL dump = 0x6720. */

  start_delay = ((uint32_t)vtotal - t->vactive - 9u) * htotal * de_mhz /
                dclk_mhz / 8u;

  fifo_num = dclk_mhz * 15u;

  /* CPU_TRI0: block_size = hactive-1, block_space = vendor formula. */

  reg = (((uint32_t)t->hactive - 1u) << T113_TCON_CPU_TRI0_BLK_SIZE_SHIFT) &
        T113_TCON_CPU_TRI0_BLK_SIZE_MASK;
  reg |= (blk_space << T113_TCON_CPU_TRI0_BLK_SPACE_SHIFT) &
         T113_TCON_CPU_TRI0_BLK_SPACE_MASK;
  t113_tcon_putreg(T113_TCON_TCON0_CPU_TRI0_OFFSET, reg);

  /* CPU_TRI1: block_num = vactive-1. */

  reg = (((uint32_t)t->vactive - 1u) << T113_TCON_CPU_TRI1_BLK_NUM_SHIFT) &
        T113_TCON_CPU_TRI1_BLK_NUM_MASK;
  t113_tcon_putreg(T113_TCON_TCON0_CPU_TRI1_OFFSET, reg);

  /* CPU_TRI2: video-mode start_set = 10, sync/trans modes = 0, start_delay
   * computed by vendor formula above.
   */

  reg = (10u << T113_TCON_CPU_TRI2_START_SET_SHIFT) &
        T113_TCON_CPU_TRI2_START_SET_MASK;
  reg |= (start_delay << T113_TCON_CPU_TRI2_START_DELAY_SHIFT) &
         T113_TCON_CPU_TRI2_START_DELAY_MASK;
  t113_tcon_putreg(T113_TCON_TCON0_CPU_TRI2_OFFSET, reg);

  /* TCON-wide safe-period FIFO threshold (vendor:
   *   safe_period_mode = 3 (always-on),
   *   safe_period_fifo_num = dclk_mhz * 15)
   */

  reg = (3u << T113_TCON_VOLUME_MODE_SHIFT) & T113_TCON_VOLUME_MODE_MASK;
  reg |= (fifo_num << T113_TCON_VOLUME_FIFO_NUM_SHIFT) &
         T113_TCON_VOLUME_FIFO_NUM_MASK;
  t113_tcon_putreg(T113_TCON_VOLUME_CTL_OFFSET, reg);

  /* CPU_CTL: cpu_mode = MODE_DSI, trigger_fifo_en + trigger_en. */

  reg = T113_TCON_CPU_CTL_TRIGGER_EN | T113_TCON_CPU_CTL_TRIGGER_FIFO_EN;
  reg |= (T113_TCON_CPU_CTL_MODE_DSI << T113_TCON_CPU_CTL_MODE_SHIFT) &
         T113_TCON_CPU_CTL_MODE_MASK;
  t113_tcon_putreg(T113_TCON_TCON0_CPU_CTL_OFFSET, reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_tcon_initialize
 *
 * Description:
 *   See t113_tcon.h.
 *
 ****************************************************************************/

int t113_tcon_initialize(const struct t113_tcon_timing_s *timing)
{
  uint32_t dclk_div;
  uint32_t reg;

  /* ---- argument validation -------------------------------------------- */

  if (timing == NULL)
    {
      lcderr("timing is NULL\n");
      return -EINVAL;
    }

  if (timing->if_type != T113_TCON_IF_HV &&
      timing->if_type != T113_TCON_IF_DSI)
    {
      lcderr("unsupported if_type %u\n", timing->if_type);
      return -ENOTSUP;
    }

  if (timing->format != T113_TCON_FMT_RGB888)
    {
      lcderr("unsupported format %u\n", timing->format);
      return -ENOTSUP;
    }

  if (timing->hactive == 0 || timing->vactive == 0 ||
      timing->pixel_clk_hz == 0)
    {
      lcderr("zero hactive/vactive/pixel_clk\n");
      return -EINVAL;
    }

  lcdinfo("init %ux%u, if=%u, pixel_clk=%lu Hz\n",
          timing->hactive, timing->vactive, timing->if_type,
          (unsigned long)timing->pixel_clk_hz);

  /* ---- clock + reset --------------------------------------------------- */

  t113_clk_enable(T113_CLK_TCON_LCD0_BUS);
  t113_clk_reset_deassert(T113_RST_TCON_LCD0);
  t113_clk_enable(T113_CLK_TCON_LCD0);
  tcon_module_clk_set();

  /* Pixel clock = module_clk / dclk_div.  Round to nearest integer divider,
   * clamp to 1 (the divider field is 7 bits so 1..127 are legal).  At first
   * bring-up the panel runs at module_clk_hz (24 MHz) when target ~= source.
   */

  dclk_div = T113_TCON_MODULE_CLK_HZ / timing->pixel_clk_hz;
  if (dclk_div == 0)
    {
      dclk_div = 1;
    }
  else if (dclk_div > 0x7f)
    {
      dclk_div = 0x7f;
    }

  /* ---- disable TCON before reprogramming ------------------------------ */

  reg = t113_tcon_getreg(T113_TCON_GCTL_OFFSET);
  reg &= ~T113_TCON_GCTL_TCON_EN;
  t113_tcon_putreg(T113_TCON_GCTL_OFFSET, reg);

  reg = t113_tcon_getreg(T113_TCON_TCON0_CTL_OFFSET);
  reg &= ~T113_TCON_CTL_TCON0_EN;
  t113_tcon_putreg(T113_TCON_TCON0_CTL_OFFSET, reg);

  /* Clear all IRQ enables and pending flags. */

  t113_tcon_putreg(T113_TCON_GINT0_OFFSET, 0);

  /* ---- programme timing ------------------------------------------------ */

  tcon_program_timing(timing, dclk_div);

  /* DSI bridge: program CPU-mode trigger registers + safe-period FIFO and
   * route TCON-LCD0 to DSI host 0 inside display_if_top.  See
   * tcon_program_cpu_dsi() for the field-by-field rationale.
   */

  if (timing->if_type == T113_TCON_IF_DSI)
    {
      tcon_program_cpu_dsi(timing);
      tcon_route_to_dsi0();
    }

  /* ---- enable global + channel ---------------------------------------- */

  reg = t113_tcon_getreg(T113_TCON_GCTL_OFFSET);
  reg |= T113_TCON_GCTL_TCON_EN | T113_TCON_GCTL_PAD_SEL;
  t113_tcon_putreg(T113_TCON_GCTL_OFFSET, reg);

  reg = t113_tcon_getreg(T113_TCON_TCON0_CTL_OFFSET);
  reg |= T113_TCON_CTL_TCON0_EN;
  t113_tcon_putreg(T113_TCON_TCON0_CTL_OFFSET, reg);

  /* TODO: register IRQ 122 for vsync counting once a consumer needs it. */

  lcdinfo("ready, dclk_div=%lu\n", (unsigned long)dclk_div);
  return 0;
}
