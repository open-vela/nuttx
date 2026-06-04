/****************************************************************************
 * arch/arm/src/t113/t113_de.c
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

/* T113-S3 Display Engine 2.0 mixer driver.
 *
 * Programs a single primary UI plane on overlay channel 1 / blender pipe 0
 * and exposes a NuttX framebuffer vtable backed by a kmm_memalign-allocated
 * scan-out buffer.  fb_register() is left to the board glue layer; this
 * driver only owns the mixer hardware and the framebuffer memory.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>

#include "arm_internal.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_de.h"
#include "t113_clk.h"
#include "t113_de.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CCU DE-functional-clock register.  The gate bit (bit 31) is owned by
 * t113_clk_enable(T113_CLK_DE); only the source-mux and M-divider fields are
 * programmed locally so the DE clock plan stays in this driver.
 */

#define T113_CCU_DE_CLK_REG         (T113_CCU_BASE + 0x0600)

/* Bit layout of CCU DE clock register (0x0600).
 *
 *   [31]    gate (handled by t113_clk_enable())
 *   [26:24] CLK_SRC_SEL: 0 = PLL_PERIPH0(2x), 1 = PLL_VIDEO0(4x),
 *                        2 = PLL_VIDEO1(4x),  3 = PLL_AUDIO1/2
 *   [4:0]   FACTOR_M (M divider, output = src / (M+1))
 */

#define T113_DE_CLK_M_SHIFT         0
#define T113_DE_CLK_M_MASK          (0x1fu << T113_DE_CLK_M_SHIFT)
#define T113_DE_CLK_SRC_SHIFT       24
#define T113_DE_CLK_SRC_MASK        (0x7u  << T113_DE_CLK_SRC_SHIFT)
#define T113_DE_CLK_SRC_PERIPH0_2X  0u

/* Pixel-format -> bytes-per-pixel helper for the two formats this driver
 * advertises (32-bit ARGB and 24-bit packed RGB).
 */

#define T113_DE_BPP_TO_BYTES(b)     ((b) >> 3)

/* Framebuffer alignment.  64 bytes covers the worst-case ARM Cortex-A7
 * cache-line size and matches the alignment SoC DMA engines on this family
 * expect for linear scan-out buffers.
 */

#define T113_DE_FB_ALIGN            64

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Framebuffer state.  All zero before t113_de_initialize() succeeds; once
 * g_de_fb is non-NULL the driver is treated as initialised and a second
 * call returns success without touching the hardware again.
 */

static uint8_t  *g_de_fb;
static size_t    g_de_fb_size;
static uint16_t  g_de_fb_w;
static uint16_t  g_de_fb_h;
static uint16_t  g_de_fb_stride;
static uint8_t   g_de_fb_bpp;
static uint8_t   g_de_fb_fmt;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int t113_de_fb_getvideoinfo(struct fb_vtable_s *vtable,
                                   struct fb_videoinfo_s *vinfo);
static int t113_de_fb_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                   struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int t113_de_fb_updatearea(struct fb_vtable_s *vtable,
                                 const struct fb_area_s *area);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct fb_vtable_s g_de_fb_vtable =
{
  .getvideoinfo = t113_de_fb_getvideoinfo,
  .getplaneinfo = t113_de_fb_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea   = t113_de_fb_updatearea,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: de_module_clk_set
 *
 * Description:
 *   Configure the DE functional clock to PLL_PERIPH0(2x) / 3 (~200 MHz).
 *   The gate bit is left to t113_clk_enable(T113_CLK_DE).
 *
 ****************************************************************************/

static void de_module_clk_set(void)
{
  uint32_t reg = getreg32(T113_CCU_DE_CLK_REG);

  reg &= ~(T113_DE_CLK_M_MASK | T113_DE_CLK_SRC_MASK);
  reg |= (T113_DE_CLK_SRC_PERIPH0_2X << T113_DE_CLK_SRC_SHIFT);
  reg |= (2u << T113_DE_CLK_M_SHIFT);   /* M+1 = 3 -> ~200 MHz */
  putreg32(reg, T113_CCU_DE_CLK_REG);
}

/****************************************************************************
 * Name: de_top_module_enable
 *
 * Description:
 *   Bring the DE0 mixer block out of reset and enable its per-block AHB
 *   gate.  Mirrors the vendor lowlevel_v2x __de_clk_enable() flow: write the
 *   AHB reset register with the CORE0 bit set (release reset), then assert
 *   the per-block AHB clock gate.  Only after both operations land do GLB /
 *   BLD / UI register reads return real values - without them every read
 *   from the mixer comes back as the bus-stuck pattern 0x00040800 and writes
 *   are silently dropped.  The SoC-level CCU enables (T113_CLK_DE_BUS /
 *   T113_CLK_DE) opens the AHB master path TO the DE module; this helper
 *   opens the gate INSIDE the DE module.
 *
 ****************************************************************************/

static void de_top_module_enable(void)
{
  uint32_t reg;

  reg = t113_de_top_getreg(T113_DE_TOP_AHB_RESET_OFFSET);
  reg |= T113_DE_TOP_CORE0;
  t113_de_top_putreg(T113_DE_TOP_AHB_RESET_OFFSET, reg);

  reg = t113_de_top_getreg(T113_DE_TOP_HCLK_GATE_OFFSET);
  reg |= T113_DE_TOP_CORE0;
  t113_de_top_putreg(T113_DE_TOP_HCLK_GATE_OFFSET, reg);
}

/****************************************************************************
 * Name: de_init_glb
 *
 * Description:
 *   Programme the mixer GLB block: enable the real-time pipeline and set
 *   the mixer output canvas size.
 *
 ****************************************************************************/

static void de_init_glb(uint16_t w, uint16_t h)
{
  t113_de_glb_putreg(T113_DE_GLB_CTL_OFFSET, T113_DE_GLB_CTL_RT_EN);
  t113_de_glb_putreg(T113_DE_GLB_SIZE_OFFSET, T113_DE_SIZE_PACK(w, h));
}

/****************************************************************************
 * Name: de_init_bld
 *
 * Description:
 *   Programme the alpha blender for a single pipe-0 input fed by UI ch1
 *   covering the full canvas.  The other three pipes are left disabled and
 *   the output formatter is configured for progressive, non-premultiplied
 *   RGB without colour-space conversion.
 *
 ****************************************************************************/

static void de_init_bld(uint16_t w, uint16_t h)
{
  uint32_t size = T113_DE_SIZE_PACK(w, h);

  /* Per-pipe master enable: pipe 0 input enable + fcolor source select. */

  t113_de_bld_putreg(T113_DE_BLD_FCOLOR_CTL_OFFSET, 0x00000101u);

  /* Pipe-0 input attributes. */

  t113_de_bld_putreg(T113_DE_BLD_PIPE_FCOLOR_OFFSET(0), 0xff000000u);
  t113_de_bld_putreg(T113_DE_BLD_PIPE_INSIZE_OFFSET(0), size);
  t113_de_bld_putreg(T113_DE_BLD_PIPE_OFFSET_OFFSET(0), 0u);

  /* Channel-to-pipe routing: pipe 0 fed by channel 1 (UI ch1).  4 bits per
   * pipe; channel encoding 0 = VI ch0, 1 = UI ch1, 2 = UI ch2, 3 = UI ch3.
   */

  t113_de_bld_putreg(T113_DE_BLD_ROUTE_CTL_OFFSET, 0x00000001u);

  t113_de_bld_putreg(T113_DE_BLD_PREMULT_CTL_OFFSET, 0u);
  t113_de_bld_putreg(T113_DE_BLD_BKCOLOR_OFFSET,    0xff000000u);
  t113_de_bld_putreg(T113_DE_BLD_OUTPUT_SIZE_OFFSET, size);

  /* SRC_OVER blend mode for pipe 0 (src=SRC_ALPHA, dst=ONE_MINUS_SRC_ALPHA
   * for both colour and alpha channels).
   */

  t113_de_bld_putreg(T113_DE_BLD_CTL_OFFSET(0), 0x03010301u);

  /* Output formatter: progressive, no premultiply, no CSC. */

  t113_de_bld_putreg(T113_DE_BLD_OUT_OFFSET, 0u);
}

/****************************************************************************
 * Name: de_init_ui_layer
 *
 * Description:
 *   Programme UI overlay channel 1 layer 0 to scan out the framebuffer at
 *   (0, 0) covering the full canvas.  T113 boots without an MMU so the
 *   physical and virtual framebuffer addresses coincide; the high 8 bits
 *   of the layer-0 top buffer pointer are zero because all DDR sits in the
 *   low 4 GiB.
 *
 ****************************************************************************/

static void de_init_ui_layer(uint16_t w, uint16_t h, uint8_t bpp,
                             uint16_t stride, uint8_t fmt,
                             uintptr_t fb_addr)
{
  const uint32_t lay = 0;                 /* layer 0 within UI ch1 */
  uint32_t lay_off  = lay * T113_DE_UI_LAY_STRIDE;
  uint32_t size     = T113_DE_SIZE_PACK(w, h);
  uint32_t attr;

  UNUSED(bpp);

  t113_de_ui0_putreg(T113_DE_UI_LAY_PITCH_OFFSET + lay_off, stride);
  t113_de_ui0_putreg(T113_DE_UI_LAY_TOP_LADDR_OFFSET + lay_off,
                     (uint32_t)fb_addr);
  t113_de_ui0_putreg(T113_DE_UI_TOP_HADDR_OFFSET, 0u);
  t113_de_ui0_putreg(T113_DE_UI_LAY_SIZE_OFFSET + lay_off, size);
  t113_de_ui0_putreg(T113_DE_UI_LAY_COOR_OFFSET + lay_off, 0u);
  t113_de_ui0_putreg(T113_DE_UI_LAY_FCOLOR_OFFSET + lay_off, 0xff000000u);
  t113_de_ui0_putreg(T113_DE_UI_OVL_SIZE_OFFSET, size);

  attr = T113_DE_UI_LAY_ATTR_LAY_EN |
         (((uint32_t)fmt << T113_DE_UI_LAY_ATTR_FMT_SHIFT) &
          T113_DE_UI_LAY_ATTR_FMT_MASK) |
         (((uint32_t)0xff << T113_DE_UI_LAY_ATTR_ALPHA_SHIFT) &
          T113_DE_UI_LAY_ATTR_ALPHA_MASK);
  t113_de_ui0_putreg(T113_DE_UI_LAY_ATTR_OFFSET + lay_off, attr);
}

/****************************************************************************
 * Name: t113_de_fb_getvideoinfo
 *
 * Description:
 *   Framebuffer vtable hook: report the active video format and resolution.
 *
 ****************************************************************************/

static int t113_de_fb_getvideoinfo(struct fb_vtable_s *vtable,
                                   struct fb_videoinfo_s *vinfo)
{
  if (vtable == NULL || vinfo == NULL)
    {
      return -EINVAL;
    }

  if (g_de_fb == NULL)
    {
      return -ENODEV;
    }

  vinfo->fmt     = g_de_fb_fmt;
  vinfo->xres    = g_de_fb_w;
  vinfo->yres    = g_de_fb_h;
  vinfo->nplanes = 1;
  return 0;
}

/****************************************************************************
 * Name: t113_de_fb_getplaneinfo
 *
 * Description:
 *   Framebuffer vtable hook: hand back the scan-out buffer descriptor for
 *   plane 0.  Only one plane is exposed.
 *
 ****************************************************************************/

static int t113_de_fb_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                                   struct fb_planeinfo_s *pinfo)
{
  if (vtable == NULL || pinfo == NULL)
    {
      return -EINVAL;
    }

  if (planeno != 0)
    {
      return -EINVAL;
    }

  if (g_de_fb == NULL)
    {
      return -ENODEV;
    }

  pinfo->fbmem   = g_de_fb;
  pinfo->fblen   = g_de_fb_size;
  pinfo->stride  = g_de_fb_stride;
  pinfo->display = 0;
  pinfo->bpp     = g_de_fb_bpp;
  return 0;
}

#ifdef CONFIG_FB_UPDATE
/****************************************************************************
 * Name: t113_de_fb_updatearea
 *
 * Description:
 *   Framebuffer vtable hook for partial updates.  No-op on the T113 mixer:
 *   the hardware scans the framebuffer continuously, so writes from the
 *   client are reflected on the panel without any flush from the driver.
 *
 ****************************************************************************/

static int t113_de_fb_updatearea(struct fb_vtable_s *vtable,
                                 const struct fb_area_s *area)
{
  UNUSED(vtable);

  /* Flush dcache for the updated FB region so DE sees coherent data via
   * AXI.  Without this, CPU writes via cached mapping sit in L1 while DE
   * reads stale DRAM and panel shows old/blank pixels.
   */

  if (g_de_fb != NULL && area != NULL)
    {
      uintptr_t start = (uintptr_t)g_de_fb +
                        ((uintptr_t)area->y * g_de_fb_stride) +
                        ((uintptr_t)area->x *
                         (g_de_fb_bpp >> 3));
      size_t bytes = (size_t)area->h * g_de_fb_stride;
      up_clean_dcache(start, start + bytes);
    }
  else if (g_de_fb != NULL)
    {
      up_clean_dcache((uintptr_t)g_de_fb,
                      (uintptr_t)g_de_fb + g_de_fb_size);
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_de_initialize
 *
 * Description:
 *   See t113_de.h.
 *
 ****************************************************************************/

int t113_de_initialize(const struct t113_de_config_s *cfg)
{
  uint8_t  fmt;
  uint16_t stride;
  size_t   fb_size;
  uint8_t  *fb;

  /* ---- argument validation --------------------------------------------- */

  if (cfg == NULL)
    {
      lcderr("cfg is NULL\n");
      return -EINVAL;
    }

  if (cfg->width == 0 || cfg->width > 2047 ||
      cfg->height == 0 || cfg->height > 2047)
    {
      lcderr("invalid resolution %ux%u\n", cfg->width, cfg->height);
      return -EINVAL;
    }

  if (cfg->bpp == 32)
    {
      fmt = T113_DE_UI_FMT_ARGB8888;
    }
  else if (cfg->bpp == 24)
    {
      fmt = T113_DE_UI_FMT_RGB888;
    }
  else
    {
      lcderr("unsupported bpp %u\n", cfg->bpp);
      return -EINVAL;
    }

  /* ---- idempotency: single-shot bring-up ------------------------------- */

  if (g_de_fb != NULL)
    {
      lcdinfo("already initialised\n");
      return 0;
    }

  lcdinfo("init %ux%u, bpp=%u\n", cfg->width, cfg->height, cfg->bpp);

  /* ---- framebuffer allocation ------------------------------------------ */

  stride  = (uint16_t)(cfg->width * T113_DE_BPP_TO_BYTES(cfg->bpp));
  fb_size = (size_t)stride * cfg->height;

  fb = kmm_memalign(T113_DE_FB_ALIGN, fb_size);
  if (fb == NULL)
    {
      lcderr("framebuffer alloc failed (%zu bytes)\n", fb_size);
      return -ENOMEM;
    }

  if (cfg->bpp == 32)
    {
      uint32_t *p = (uint32_t *)fb;
      size_t pixels = fb_size >> 2;
      while (pixels--)
        {
          *p++ = 0xff000000u;
        }
    }
  else
    {
      memset(fb, 0, fb_size);
    }

  /* Flush dcache for the entire FB so DE reads coherent data via AXI on
   * the very first scan-out.  The FB sits in cached DDR; without an
   * explicit clean, the CPU-side init writes may still be in L1 when DE
   * starts fetching pixels.
   */

  up_clean_dcache((uintptr_t)fb, (uintptr_t)fb + fb_size);

  /* ---- clock + reset --------------------------------------------------- */

  de_module_clk_set();
  t113_clk_enable(T113_CLK_DE_BUS);
  t113_clk_reset_deassert(T113_RST_DE);
  t113_clk_enable(T113_CLK_DE);

  /* DPSS_TOP gates the display sub-system bus interconnect that hosts
   * DISP_IF_TOP (0x05460000) - the routing block between TCON and DSI.
   * Without it, DISP_IF_TOP writes silently drop (readback all zero) and
   * TCON pixel data never reaches the DSI host.  Distinct from
   * T113_CLK_DE / T113_RST_DE.
   */

  t113_clk_enable(T113_CLK_DPSS_TOP);
  t113_clk_reset_deassert(T113_RST_DPSS_TOP);

  /* DE block has its own AHB reset + clock gate INSIDE the module register
   * window (separate from the CCU-side gates above).  Without this the
   * mixer sub-blocks (GLB / BLD / UI) are bus-stuck and writes drop.
   */

  de_top_module_enable();

  /* ---- mixer programming ----------------------------------------------- */

  de_init_glb(cfg->width, cfg->height);
  de_init_bld(cfg->width, cfg->height);
  de_init_ui_layer(cfg->width, cfg->height, cfg->bpp, stride, fmt,
                   (uintptr_t)fb);

  /* Latch the shadow registers; the HW clears DBUFF_RDY on the next vsync
   * after applying the new configuration.
   */

  t113_de_glb_putreg(T113_DE_GLB_DBUFF_OFFSET, T113_DE_GLB_DBUFF_RDY);

  /* ---- publish state for vtable callbacks ------------------------------ */

  g_de_fb_w      = cfg->width;
  g_de_fb_h      = cfg->height;
  g_de_fb_bpp    = cfg->bpp;
  g_de_fb_stride = stride;
  g_de_fb_size   = fb_size;
  g_de_fb_fmt    = (cfg->bpp == 32) ? FB_FMT_RGB32 : FB_FMT_RGB24;
  g_de_fb        = fb;

  lcdinfo("ready, fb=%p stride=%u\n", fb, (unsigned)stride);
  return 0;
}

/****************************************************************************
 * Name: t113_de_get_fb_vtable
 *
 * Description:
 *   See t113_de.h.
 *
 ****************************************************************************/

struct fb_vtable_s *t113_de_get_fb_vtable(void)
{
  return (g_de_fb != NULL) ? &g_de_fb_vtable : NULL;
}

/****************************************************************************
 * Name: t113_de_get_fb_buffer
 *
 * Description:
 *   See t113_de.h.
 *
 ****************************************************************************/

uint8_t *t113_de_get_fb_buffer(void)
{
  return g_de_fb;
}
