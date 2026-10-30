/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_dc.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
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

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/video/fb.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_gpio.h"

#if defined(CONFIG_LS2K0300_DC)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DC (Display Controller) register offsets for dvo0, relative to the DC
 * register base.  Ported from U-Boot drivers/video/loongson_fb.c, which is
 * the only documented source for this block -- the SoC manual chapter 8
 * (p.79) gives the base address and the supported colour depths but no
 * register bit definitions at all.
 */

#define LS_DC_FB_CONF          0x1240
#define LS_DC_FB_ADDR0         0x1260
#define LS_DC_FB_STRIDE        0x1280
#define LS_DC_FB_ORIG          0x1300
#define LS_DC_DITHER_CONF      0x1360
#define LS_DC_DITHER_TABLE_LOW 0x1380
#define LS_DC_DITHER_TABLE_HIGH 0x13a0
#define LS_DC_PAN_CONF         0x13c0
#define LS_DC_PAN_TIMING       0x13e0
#define LS_DC_HDISPLAY         0x1400
#define LS_DC_HSYNC            0x1420
#define LS_DC_VDISPLAY         0x1480
#define LS_DC_VSYNC            0x14a0
#define LS_DC_FB_ADDR1         0x1580
#define LS_DC_FB_ADDR0_HI      0x15a0
#define LS_DC_FB_ADDR1_HI      0x15c0

#define LS_FB_CONF_RESET       (1 << 20)
#define LS_FB_CONF_OUTPUT_EN   (1 << 8)
#define LS_PIXFMT_RGB888       0x4
#define LS_PAN_CONF_VAL        0x80001111
#define LS_HSYNC_VSYNC_POL     0x40000000

/* Panel: ALIENTEK ATK-MD0430R 4.3" 800x480.
 *
 * To retarget another panel change these nine numbers and let
 * ls_dc_cal_freq() re-solve the pixel PLL; nothing else needs touching.
 *
 * Source of truth: U-Boot drivers/video/loongson_common.c "ATK-MD0430R-
 * 800x480" and the dts "atk_md0430r" node -- the two agree field by
 * field, including vsync_len = 0.  Note the U-Boot dts tree currently
 * #if 0's this entry in favour of a 7" 1024x600 panel; that tree is out of
 * date and is not the panel on this board.
 *
 * 27 MHz / 928 / 525 = 55.4 Hz rather than 60 Hz.  Kept at the vendor
 * table value on purpose: the panel is DE-only (hsync_active = vsync_active
 * = pixelclk_active = 0) and tolerates it, and "fixing" the clock would
 * mix a second variable into any display debugging.
 */

#define LS_DC_XRES             800
#define LS_DC_YRES             480
#define LS_DC_PIXCLK_KHZ       27000
#define LS_DC_HFP              48
#define LS_DC_HBP              40
#define LS_DC_HSW              40
#define LS_DC_VFP              13
#define LS_DC_VBP              32
#define LS_DC_VSW              0

/* Pixel format: RGB888, 4 bytes per pixel (U-Boot writes stride as
 * hactive * 4 and its logo blitter writes B,G,R,0 -- little-endian
 * 0x00RRGGBB).  FB_FMT_RGB32 in NuttX uses the same layout.
 */

#define LS_DC_FB_BYTESPP       4
#define LS_DC_FB_STRIDE_BYTES  (LS_DC_XRES * LS_DC_FB_BYTESPP)
#define LS_DC_FB_SIZE          (LS_DC_FB_STRIDE_BYTES * LS_DC_YRES)

/* Bytes to wipe before the first draw.  U-Boot painted its logo while the
 * panel was in a different mode (1024x600 at stride 4096), so its pixels
 * sit further into this buffer than our 800x480 frame covers.  Clearing
 * well past the visible frame makes sure none of it shows through.
 */

#define LS_DC_FB_WIPE_BYTES    0x400000

/* Framebuffer physical address.
 *
 * Lives in DRAM bank0 (phys 0x0..0x10000000), entirely outside the 8 MiB
 * of NuttX RAM at phys 0x98000000 (bank1), so the DC can never scan over
 * OS data.  0x0D000000..0x0F000000 is the DVO framebuffer reserve named in
 * U-Boot board/loongson/common/platform.c; U-Boot lands its own
 * framebuffer at 0x0DC00000 (carved down from ram_top = 0x0EC00000, 1 MiB
 * aligned, 16 MiB).  Nothing in the tree hardcodes that address, so it is
 * fixed here.
 */

#define LS_DC_FB_PA            0x0dc00000

/* CPU access window for the framebuffer.
 *
 * U-Boot writes through the *cached* window and never flushes -- its cache
 * flush is compiled out on LoongArch (drivers/video/video-uclass.c
 * video_sync is #if defined(CONFIG_ARM)).  NuttX on LoongArch has no cache
 * maintenance at all: up_clean_dcache() expands to nothing when
 * CONFIG_ARCH_DCACHE is off and arch/loongarch implements neither, so a
 * cached write path cannot be made coherent by software here.
 *
 * Default to the uncached window, which is coherent by construction.  Set
 * to 0 to reproduce U-Boot's cached path for A/B comparison; the boot
 * probe reports whether the two aliases agree on this hardware.
 */

#define LS_DC_FB_UNCACHED      1

/* Backlight enable: GPIO43, driven high.
 *
 * Taken from the board schematic.  The 40-pin LCD FPC has no backlight
 * line at all (board manual table 3-3 lists only the RGB bus, syncs,
 * touch and LCD_RESETn/GPIO38) -- the enable is a board-level GPIO.
 * Earlier guesses of GPIO84/88/89 were wrong: GPIO88 is a user LED and
 * GPIO84/89 are unused.  GPIO43 sits in the GPIO0-63 bank, which resets
 * to *input*, so an untouched enable pin floats and the backlight only
 * glimmers -- which is exactly the "extremely dim" symptom seen before
 * the real pin was known.
 *
 * The circuit wants a steady level: PWM on this pin turns the backlight
 * OFF.  GPIO0 must NOT be touched -- per SoC manual table 1-1 it is
 * lcd_clk.
 */

#define LS_DC_BKL_PIN          43

/* LCD data pinmux (U-Boot dtsi "dc_pins"): GPIO0-27 must be MAIN_FUNC to
 * carry CLK/VSYNC/HSYNC/DE plus the 24 RGB data lines.  2 bits per pin.
 *
 * NOTE: U-Boot expresses this as value 0x00ffffff with mask 0x00ffffff,
 * i.e. it only rewrites the low 24 bits and LEAVES bits 24-31 (GPIO28-31)
 * at their previous setting.  This driver blind-writes the whole register
 * with putreg32(), so the high byte must explicitly carry 0b11 (MAIN_FUNC)
 * for GPIO28-31: those pins are GMAC0_RX_CTL/RX0/RX1/RX2 per SoC manual
 * table 1-1, and writing 0 there demotes them to GPIO mode, disconnects
 * the RGMII receive lines, and the MAC never sees the PHY's in-band link
 * status ("PHY: link still DOWN" + no DHCP + ping fails).
 */

#define LS_PINMUX_GPIO0_15      0x16000490
#define LS_PINMUX_GPIO0_15_VAL  0xffffffff
#define LS_PINMUX_GPIO16_31     0x16000494
#define LS_PINMUX_GPIO16_31_VAL 0xffffffff

/* Backlight pin has to be plain GPIO rather than its alternate function.
 * GPIO43 lives in the GPIO32-47 bank; the mask is computed at runtime so
 * only GPIO43's two bits are cleared and GMAC's GPIO32-39 (which
 * gmac0_pins sets to 0b11) keep their function.
 */

#define LS_PINMUX_GPIO32_47     0x16000498

/* Pixel PLL reference clock.  U-Boot hardcodes 100 MHz here and ignores
 * the rate it reads from the DT "refclk" clock; the dts fixed-clock
 * osc_clk is 100 MHz, so the two agree.
 */

#define LS_DC_REFCLK_KHZ       100000

/* Pixel PLL lock status bit (U-Boot config_pll polls it) */

#define LS_PIXPLL_LOCK_BIT     0x80

/* PLL lock wait timeout: 1 second of 1 us delays.  U-Boot spins forever;
 * a bounded wait keeps a dead PLL from hanging boot. */

#define LS_PIXPLL_LOCK_RETRIES 1000000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ls_dc_pll_cfg
{
  uint32_t l2_div;
  uint32_t l1_loopc;
  uint32_t l1_frefc;
};

/* Framebuffer device.  The vtable must be the first member: the fb core
 * overwrites vtable->priv with its own handle (fb.c fb_register_device),
 * so the driver recovers its state by casting the vtable pointer back.
 */

struct ls2k0300_fb_s
{
  struct fb_vtable_s vtable;
  struct fb_videoinfo_s videoinfo;
  struct fb_planeinfo_s planeinfo;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct ls2k0300_fb_s g_dc_fb;

/* up_fbinitialize() may be reached by more than one code path (fb.h says
 * so explicitly); the hardware only needs programming once.
 */

static bool g_dc_hw_ready;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Framebuffer pointer through the selected CPU access window */

static FAR volatile uint32_t *ls_dc_fb_ptr(void)
{
#if LS_DC_FB_UNCACHED
  return (FAR volatile uint32_t *)PHYS_TO_UNCACHED(LS_DC_FB_PA);
#else
  return (FAR volatile uint32_t *)PHYS_TO_CACHED(LS_DC_FB_PA);
#endif
}

/* Brute-force the pixel PLL (U-Boot cal_freq).
 *
 * Note U-Boot's "min" variable is never updated, so it does not search for
 * the best fit -- it returns the first combination whose error is under
 * 1 MHz.  This walks the same space in the same order to land on the same
 * answer: for 27000 kHz that is pstdiv=50, loopc=54, frefc=4, error 0.
 */

static int ls_dc_cal_freq(uint32_t pixclock_khz,
                          FAR struct ls_dc_pll_cfg *cfg)
{
  uint32_t pstdiv;
  uint32_t frefc;
  uint32_t loopc;
  uint32_t a;
  uint32_t b;
  uint32_t diff;

  for (pstdiv = 1; pstdiv < 128; pstdiv++)
    {
      a = pixclock_khz * pstdiv;
      for (frefc = 4; frefc < 6; frefc++)
        {
          for (loopc = 24; loopc < 161; loopc++)
            {
              if ((loopc < 12 * frefc) || (loopc > 32 * frefc))
                {
                  continue;
                }

              b = LS_DC_REFCLK_KHZ * loopc / frefc;
              diff = (a > b) ? (a - b) : (b - a);
              if (diff < 1000)
                {
                  cfg->l2_div = pstdiv;
                  cfg->l1_loopc = loopc;
                  cfg->l1_frefc = frefc;
                  return OK;
                }
            }
        }
    }

  return -EINVAL;
}

/* LS2K300 pixel PLL programming sequence (U-Boot config_pll, 2K300
 * branch): power down -> program dividers -> enable -> wait lock ->
 * output on.  Register layout: l2_div[30:24], l1_loopc[23:15],
 * l1_frefc[14:8].  (The 2K500 branch packs loopc at bit 16 instead --
 * do not mix them.)
 */

static int ls_dc_config_pll(FAR const struct ls_dc_pll_cfg *cfg)
{
  uint32_t val;
  int i;

  val = (cfg->l2_div << 24) | (cfg->l1_loopc << 15) |
        (cfg->l1_frefc << 8);

  putreg32(0, LS2K0300_PIX0_PLL);
  putreg32(1 << 5, LS2K0300_PIX0_PLL);
  putreg32(val, LS2K0300_PIX0_PLL);
  val |= (1 << 3);
  putreg32(val, LS2K0300_PIX0_PLL);

  for (i = 0; i < LS_PIXPLL_LOCK_RETRIES; i++)
    {
      if (getreg32(LS2K0300_PIX0_PLL) & LS_PIXPLL_LOCK_BIT)
        {
          putreg32(val | 1, LS2K0300_PIX0_PLL);
          return OK;
        }

      up_udelay(1);
    }

  return -ETIMEDOUT;
}

/* Program the DC for the panel timing in the macros above.
 *
 * Register sequence is U-Boot config_fb() for dvo0/RGB888, including the
 * deliberate double write of PAN_CONF and the FB_CONF reset+enable bits
 * being raised together in a single write.
 */

static void ls_dc_config_fb(uint32_t fb_pa)
{
  uintptr_t base = LS2K0300_DC_BASE;
  uint32_t hsync_start = LS_DC_XRES + LS_DC_HFP;
  uint32_t hsync_end = hsync_start + LS_DC_HSW;
  uint32_t htotal = hsync_end + LS_DC_HBP;
  uint32_t vsync_start = LS_DC_YRES + LS_DC_VFP;
  uint32_t vsync_end = vsync_start + LS_DC_VSW;
  uint32_t vtotal = vsync_end + LS_DC_VBP;

  /* Disable output while the registers are being written */

  putreg32(0, base + LS_DC_FB_CONF);
  putreg32(0, base + LS_DC_FB_ADDR0_HI);
  putreg32(0, base + LS_DC_FB_ADDR1_HI);
  putreg32(fb_pa, base + LS_DC_FB_ADDR0);

  /* U-Boot points ADDR1 at the same buffer as ADDR0 -- there is no true
   * double buffering here.
   */

  putreg32(fb_pa, base + LS_DC_FB_ADDR1);
  putreg32(0, base + LS_DC_DITHER_CONF);
  putreg32(0, base + LS_DC_DITHER_TABLE_LOW);
  putreg32(0, base + LS_DC_DITHER_TABLE_HIGH);
  putreg32(0, base + LS_DC_PAN_CONF);
  putreg32(LS_PAN_CONF_VAL, base + LS_DC_PAN_CONF);
  putreg32(0, base + LS_DC_PAN_TIMING);
  putreg32(0, base + LS_DC_FB_ORIG);
  putreg32((htotal << 16) | LS_DC_XRES, base + LS_DC_HDISPLAY);
  putreg32(LS_HSYNC_VSYNC_POL | (hsync_end << 16) | hsync_start,
           base + LS_DC_HSYNC);
  putreg32((vtotal << 16) | LS_DC_YRES, base + LS_DC_VDISPLAY);
  putreg32(LS_HSYNC_VSYNC_POL | (vsync_end << 16) | vsync_start,
           base + LS_DC_VSYNC);
  putreg32(LS_FB_CONF_RESET | LS_FB_CONF_OUTPUT_EN | LS_PIXFMT_RGB888,
           base + LS_DC_FB_CONF);
  putreg32(LS_DC_XRES * LS_DC_FB_BYTESPP, base + LS_DC_FB_STRIDE);
}

/* GPIO0-27 -> MAIN_FUNC: the RGB/HSYNC/VSYNC/DE lines of the LCD.  Done
 * here rather than trusted from the bootloader -- nothing says the boot
 * state survives into the OS.
 */

static void ls_dc_pinmux_setup(void)
{
  putreg32(LS_PINMUX_GPIO0_15_VAL,
           PHYS_TO_UNCACHED(LS_PINMUX_GPIO0_15));
  putreg32(LS_PINMUX_GPIO16_31_VAL,
           PHYS_TO_UNCACHED(LS_PINMUX_GPIO16_31));
}

/* Backlight on.  DIR=0 -> output, OUT=1 -> on (active high).  GPIO
 * direction and output level are byte-per-pin registers.
 */

static void ls_dc_backlight_on(void)
{
  uintptr_t pinmux = PHYS_TO_UNCACHED(LS_PINMUX_GPIO32_47);
  uint32_t val;

  /* Clear only GPIO43's two bits so the pin takes the GPIO function,
   * leaving the rest of the bank alone -- GMAC needs GPIO32-39 at
   * MAIN_FUNC.  Reads of this block are unreliable, so fall back to
   * "everything else MAIN_FUNC" when the read comes back stuck.
   */

  val = getreg32(pinmux);
  if (val == 0 || val == 0xffffffff)
    {
      val = 0xffffffff;
    }

  val &= ~(3u << ((LS_DC_BKL_PIN - 32) * 2));
  putreg32(val, pinmux);

  putreg8(0, LS2K0300_GPIO_BASE + LS2K0300_GPIO_DIR + LS_DC_BKL_PIN);
  putreg8(1, LS2K0300_GPIO_BASE + LS2K0300_GPIO_OUT + LS_DC_BKL_PIN);
}

/* Dump the DC registers and pixel PLL to syslog.  Called once at boot so
 * that a working configuration can be diffed against U-Boot's, and from
 * the "lcd dump" subcommand so it can be re-run without reflashing.
 */

void ls2k0300_dc_dump(FAR const char *tag)
{
  uintptr_t base = LS2K0300_DC_BASE;

  syslog(LOG_INFO,
         "ls2k0300_dc[%s]: pixpll=0x%08lx fb_conf=0x%08lx pan=0x%08lx "
         "pan_tim=0x%08lx orig=0x%08lx\n",
         tag,
         (unsigned long)getreg32(LS2K0300_PIX0_PLL),
         (unsigned long)getreg32(base + LS_DC_FB_CONF),
         (unsigned long)getreg32(base + LS_DC_PAN_CONF),
         (unsigned long)getreg32(base + LS_DC_PAN_TIMING),
         (unsigned long)getreg32(base + LS_DC_FB_ORIG));
  syslog(LOG_INFO,
         "ls2k0300_dc[%s]: hdisp=0x%08lx hsync=0x%08lx vdisp=0x%08lx "
         "vsync=0x%08lx stride=0x%08lx\n",
         tag,
         (unsigned long)getreg32(base + LS_DC_HDISPLAY),
         (unsigned long)getreg32(base + LS_DC_HSYNC),
         (unsigned long)getreg32(base + LS_DC_VDISPLAY),
         (unsigned long)getreg32(base + LS_DC_VSYNC),
         (unsigned long)getreg32(base + LS_DC_FB_STRIDE));
  syslog(LOG_INFO,
         "ls2k0300_dc[%s]: addr0=0x%08lx addr0_hi=0x%08lx addr1=0x%08lx "
         "addr1_hi=0x%08lx dither=0x%08lx\n",
         tag,
         (unsigned long)getreg32(base + LS_DC_FB_ADDR0),
         (unsigned long)getreg32(base + LS_DC_FB_ADDR0_HI),
         (unsigned long)getreg32(base + LS_DC_FB_ADDR1),
         (unsigned long)getreg32(base + LS_DC_FB_ADDR1_HI),
         (unsigned long)getreg32(base + LS_DC_DITHER_CONF));
  syslog(LOG_INFO,
         "ls2k0300_dc[%s]: backlight gpio%d dir=0x%02x out=0x%02x "
         "pinmux32_47=0x%08lx (expect dir=00 out=01, bit22-23 clear)\n",
         tag, LS_DC_BKL_PIN,
         getreg8(LS2K0300_GPIO_BASE + LS2K0300_GPIO_DIR + LS_DC_BKL_PIN),
         getreg8(LS2K0300_GPIO_BASE + LS2K0300_GPIO_OUT + LS_DC_BKL_PIN),
         (unsigned long)getreg32(PHYS_TO_UNCACHED(LS_PINMUX_GPIO32_47)));
}

/* M0 acceptance test card.
 *
 * Every landmark has a known on-screen position, so a misreading moves an
 * edge rather than merely looking odd:
 *
 *   top band y=0..15   horizontal ruler: white x0-99, red x100-149,
 *                      green x150-174, blue x175-199, 1px black/white
 *                      checkerboard x200..XRES-1
 *   cross              4px white vertical bar at x=396..399 (centre) and
 *                      4px white horizontal bar at y=238..241 (centre)
 *   bottom band        same ruler, so a stride error shows twice
 *
 * If the DC reads fewer bytes per pixel than we write, every x landmark
 * scales by (bytes written / bytes read): 4 B/px is as drawn, 3 B/px ends
 * the white block at x=133 and 2 B/px at x=200.  The checkerboard period
 * reveals the sampling pitch.  A wrong stride smears the vertical bar.
 */

static void ls_dc_fill_test_pattern(FAR volatile uint32_t *fb)
{
  int x;
  int y;

  for (y = 0; y < LS_DC_YRES; y++)
    {
      FAR volatile uint32_t *line =
          fb + (y * LS_DC_FB_STRIDE_BYTES) / 4;

      for (x = 0; x < LS_DC_XRES; x++)
        {
          uint32_t pix;

          if ((y < 16) || (y >= LS_DC_YRES - 16))
            {
              if (x < 100)
                {
                  pix = 0x00ffffff;             /* white */
                }
              else if (x < 150)
                {
                  pix = 0x00ff0000;             /* red */
                }
              else if (x < 175)
                {
                  pix = 0x0000ff00;             /* green */
                }
              else if (x < 200)
                {
                  pix = 0x000000ff;             /* blue */
                }
              else
                {
                  pix = (x & 1) ? 0x00ffffff : 0x00000000;
                }
            }
          else if ((x >= 396 && x < 400) ||
                   (y >= 238 && y < 242))
            {
              pix = 0x00ffffff;                 /* crosshair */
            }
          else
            {
              pix = 0x00000000;
            }

          line[x] = pix;
        }
    }
}

/* fb vtable callbacks.  These two are the only mandatory ones -- the fb
 * core calls them with no NULL check (fb.c fb_get_panelinfo), everything
 * else is optional and simply yields -ENOTTY.
 */

static int ls_dc_getvideoinfo(FAR struct fb_vtable_s *vtable,
                              FAR struct fb_videoinfo_s *vinfo)
{
  if (vtable == NULL || vinfo == NULL)
    {
      return -EINVAL;
    }

  memcpy(vinfo, &g_dc_fb.videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

static int ls_dc_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                              FAR struct fb_planeinfo_s *pinfo)
{
  if (vtable == NULL || pinfo == NULL || planeno != 0)
    {
      return -EINVAL;
    }

  memcpy(pinfo, &g_dc_fb.planeinfo, sizeof(struct fb_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* M0 acceptance test card.  Drawn at boot so the panel timing can be
 * checked against the expected register values in the same dump; the
 * application layer draws over it once it opens /dev/fb0.
 */

void ls2k0300_dc_show_test_pattern(void)
{
  ls_dc_fill_test_pattern(ls_dc_fb_ptr());
}

int ls2k0300_dc_initialize(void)
{
  struct ls_dc_pll_cfg cfg;
  int ret;

  if (g_dc_hw_ready)
    {
      return OK;
    }

  ls_dc_pinmux_setup();

  ret = ls_dc_cal_freq(LS_DC_PIXCLK_KHZ, &cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ls2k0300_dc: no PLL solution for %d kHz pixel clock\n",
             LS_DC_PIXCLK_KHZ);
      return ret;
    }

  ret = ls_dc_config_pll(&cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ls2k0300_dc: pixel PLL lock timeout\n");
      return ret;
    }


  ls_dc_config_fb(LS_DC_FB_PA);
  ls_dc_backlight_on();

  /* Take the screen over cleanly: no U-Boot picture left behind. */

  memset((FAR void *)ls_dc_fb_ptr(), 0, LS_DC_FB_WIPE_BYTES);

  /* Published through /dev/fb0.  FB_FMT_RGB32 is 0x00RRGGBB, matching
   * what U-Boot's blitter writes and what ls_dc_fill_test_pattern draws.
   */

  g_dc_fb.videoinfo.fmt     = FB_FMT_RGB32;
  g_dc_fb.videoinfo.xres    = LS_DC_XRES;
  g_dc_fb.videoinfo.yres    = LS_DC_YRES;
  g_dc_fb.videoinfo.nplanes = 1;

  g_dc_fb.planeinfo.fbmem   = (FAR void *)ls_dc_fb_ptr();
  g_dc_fb.planeinfo.fblen   = LS_DC_FB_SIZE;
  g_dc_fb.planeinfo.stride  = LS_DC_FB_STRIDE_BYTES;
  g_dc_fb.planeinfo.bpp     = LS_DC_FB_BYTESPP * 8;
  g_dc_fb.planeinfo.xres_virtual = LS_DC_XRES;
  g_dc_fb.planeinfo.yres_virtual = LS_DC_YRES;

  g_dc_fb.vtable.getvideoinfo = ls_dc_getvideoinfo;
  g_dc_fb.vtable.getplaneinfo = ls_dc_getplaneinfo;

  syslog(LOG_INFO,
         "ls2k0300_dc: %dx%d@%dkHz rgb888 stride=%d fb_pa=0x%08x "
         "fb_cpu=%s\n",
         LS_DC_XRES, LS_DC_YRES, LS_DC_PIXCLK_KHZ, LS_DC_FB_STRIDE_BYTES,
         LS_DC_FB_PA, LS_DC_FB_UNCACHED ? "uncached" : "cached");


  g_dc_hw_ready = true;
  return OK;
}

/* Arch-side hooks for fb_register().  up_fbinitialize() must tolerate
 * being called more than once (fb.h).
 */

int up_fbinitialize(int display)
{
  if (display != 0)
    {
      return -ENODEV;
    }

  return ls2k0300_dc_initialize();
}

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (display != 0 || vplane != 0)
    {
      return NULL;
    }

  return &g_dc_fb.vtable;
}

#endif /* CONFIG_LS2K0300_DC */
