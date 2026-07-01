/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_vop.c
 *
 * Video Output Processor (VOP) driver for RK3576
 *
 * Supports:
 * - 2 VOP controllers (VOP0, VOP1)
 * - Up to 4 window layers per VOP (WIN0-WIN3)
 * - Multiple pixel formats (ARGB8888, RGB888, RGB565, ARGB1555, YUV)
 * - Hardware scaling
 * - Alpha blending (per-pixel and plane alpha)
 * - Mirror/flip
 * - Interrupt handling (vsync, frame done)
 * - Background color configuration
 *
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "arm64_internal.h"
#include "hardware/rk3576_vop.h"
#include "rk3576_vop.h"

#ifdef CONFIG_RK3576_VOP

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#define VOP_LINE_FLAG_MASK       0xffff

const uint32_t g_vop_base[RK3576_VOP_COUNT] =
{
  RK3576_VOP0_ADDR,
  RK3576_VOP1_ADDR,
};

/***************************************************************************
 * Private Types
 ***************************************************************************/

struct rk3576_vop_window_s
{
  bool enabled;
  int format;
  int width;
  int height;
  int stride;
  int x;
  int y;
  uint32_t addr;
  uint32_t cbr_addr;
  int alpha_mode;
  int mirror;
};

struct rk3576_vop_s
{
  bool enabled;
  int width;
  int height;
  struct rk3576_vop_window_s win[VOP_MAX_WINDOWS];
};

/***************************************************************************
 * Private Data
 ***************************************************************************/

static struct rk3576_vop_s g_vop[RK3576_VOP_COUNT];

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static inline uint32_t vop_win_base(int vop, int win)
{
  return g_vop_base[vop] + (win * VOP_WIN_STRIDE);
}

static inline int vop_clamp(int val, int min, int max)
{
  if (val < min)
    {
      return min;
    }

  if (val > max)
    {
      return max;
    }

  return val;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int rk3576_vop_init(int vop)
{
  uint32_t base;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  /* Disable all windows */

  for (int i = 0; i < VOP_MAX_WINDOWS; i++)
    {
      putreg32(0, vop_win_base(vop, i) + VOP_WIN_CTRL);
    }

  /* Disable VOP */

  putreg32(0, base + VOP_REG_SYS_CTRL);
  putreg32(0, base + VOP_REG_DSP_CTRL0);

  /* Clear all interrupts */

  putreg32(0xffffffff, base + VOP_REG_INTSTS);
  putreg32(0, base + VOP_REG_INTEN);

  /* Set default background to black */

  putreg32(0xff000000, base + VOP_REG_DSP_BG);

  /* Clear state */

  memset(&g_vop[vop], 0, sizeof(g_vop[vop]));

  ginfo("VOP%d: initialized\n", vop);
  return OK;
}

int rk3576_vop_enable(int vop)
{
  uint32_t base;
  uint32_t ctrl;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  /* Enable VOP and DMA */

  ctrl = getreg32(base + VOP_REG_SYS_CTRL);
  ctrl |= VOP_SYS_CTRL_EN | VOP_SYS_CTRL_DMA_EN;
  putreg32(ctrl, base + VOP_REG_SYS_CTRL);

  /* Enable clock and output */

  ctrl = getreg32(base + VOP_REG_DSP_CTRL0);
  ctrl |= VOP_DSP_CTRL0_DCLK_EN | VOP_DSP_CTRL0_DSP_OUT_EN;
  putreg32(ctrl, base + VOP_REG_DSP_CTRL0);

  g_vop[vop].enabled = true;

  ginfo("VOP%d: enabled (%dx%d)\n", vop,
        g_vop[vop].width, g_vop[vop].height);
  return OK;
}

int rk3576_vop_disable(int vop)
{
  uint32_t base;
  uint32_t ctrl;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  /* Disable clock and output */

  ctrl = getreg32(base + VOP_REG_DSP_CTRL0);
  ctrl &= ~(VOP_DSP_CTRL0_DCLK_EN | VOP_DSP_CTRL0_DSP_OUT_EN);
  putreg32(ctrl, base + VOP_REG_DSP_CTRL0);

  /* Disable all windows */

  for (int i = 0; i < VOP_MAX_WINDOWS; i++)
    {
      putreg32(0, vop_win_base(vop, i) + VOP_WIN_CTRL);
    }

  /* Disable VOP */

  ctrl = getreg32(base + VOP_REG_SYS_CTRL);
  ctrl &= ~(VOP_SYS_CTRL_EN | VOP_SYS_CTRL_DMA_EN);
  putreg32(ctrl, base + VOP_REG_SYS_CTRL);

  g_vop[vop].enabled = false;

  ginfo("VOP%d: disabled\n", vop);
  return OK;
}

int rk3576_vop_set_resolution(int vop, int w, int h)
{
  uint32_t base;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  g_vop[vop].width = w;
  g_vop[vop].height = h;

  /* Set active display area for WIN0 */

  putreg32(((uint32_t)w << 16) | (uint32_t)h, base + VOP_REG_WIN0_ACT);

  ginfo("VOP%d: resolution %dx%d\n", vop, w, h);
  return OK;
}

int rk3576_vop_set_framebuffer(int vop, uint32_t addr)
{
  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  putreg32(addr, vop_win_base(vop, 0) + VOP_WIN_YRGB_MST);
  g_vop[vop].win[0].addr = addr;
  return OK;
}

int rk3576_vop_set_background(int vop, uint8_t r, uint8_t g, uint8_t b)
{
  uint32_t base;
  uint32_t bg;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  bg = (0xff << VOP_DSP_BG_ALPHA_SHIFT) |
       ((uint32_t)r << VOP_DSP_BG_RED_SHIFT) |
       ((uint32_t)g << VOP_DSP_BG_GREEN_SHIFT) |
       ((uint32_t)b << VOP_DSP_BG_BLUE_SHIFT);

  putreg32(bg, base + VOP_REG_DSP_BG);
  return OK;
}

int rk3576_vop_enable_window(int vop, int win)
{
  uint32_t base;
  uint32_t ctrl;

  if (vop < 0 || vop >= RK3576_VOP_COUNT ||
      win < 0 || win >= VOP_MAX_WINDOWS)
    {
      return -EINVAL;
    }

  base = vop_win_base(vop, win);

  ctrl = getreg32(base + VOP_WIN_CTRL);
  ctrl |= VOP_WIN_CTRL_EN;
  putreg32(ctrl, base + VOP_WIN_CTRL);

  g_vop[vop].win[win].enabled = true;

  ginfo("VOP%d: WIN%d enabled\n", vop, win);
  return OK;
}

int rk3576_vop_disable_window(int vop, int win)
{
  uint32_t base;
  uint32_t ctrl;

  if (vop < 0 || vop >= RK3576_VOP_COUNT ||
      win < 0 || win >= VOP_MAX_WINDOWS)
    {
      return -EINVAL;
    }

  base = vop_win_base(vop, win);

  ctrl = getreg32(base + VOP_WIN_CTRL);
  ctrl &= ~VOP_WIN_CTRL_EN;
  putreg32(ctrl, base + VOP_WIN_CTRL);

  g_vop[vop].win[win].enabled = false;

  ginfo("VOP%d: WIN%d disabled\n", vop, win);
  return OK;
}

int rk3576_vop_set_window(int vop, int win, int x, int y,
                            int w, int h, uint32_t addr, int format)
{
  uint32_t base;
  uint32_t ctrl;
  int bpp;

  if (vop < 0 || vop >= RK3576_VOP_COUNT ||
      win < 0 || win >= VOP_MAX_WINDOWS)
    {
      return -EINVAL;
    }

  base = vop_win_base(vop, win);

  /* Determine bytes per pixel */

  switch (format)
    {
    case VOP_FMT_ARGB8888:
      bpp = 4;
      break;
    case VOP_FMT_RGB888:
      bpp = 3;
      break;
    case VOP_FMT_RGB565:
    case VOP_FMT_ARGB1555:
    case VOP_FMT_YUYV:
      bpp = 2;
      break;
    default:
      bpp = 4;
      break;
    }

  /* Configure format */

  ctrl = getreg32(base + VOP_WIN_CTRL);
  ctrl &= ~VOP_WIN_CTRL_FORMAT_MASK;
  ctrl |= (format << VOP_WIN_CTRL_FORMAT_SHIFT) & VOP_WIN_CTRL_FORMAT_MASK;
  putreg32(ctrl, base + VOP_WIN_CTRL);

  /* Set framebuffer address */

  putreg32(addr, base + VOP_WIN_YRGB_MST);

  /* Set virtual stride (in pixels) */

  putreg32(w, base + VOP_WIN_VIR);

  /* Set active area */

  putreg32(((uint32_t)w << 16) | (uint32_t)h, base + VOP_WIN_ACT);

  /* Set display position */

  putreg32(((uint32_t)x << 16) | (uint32_t)y, base + VOP_WIN_DSP_POS);

  /* Set scaling factor to 1:1 */

  putreg32(VOP_SCL_SCALE_1X | VOP_SCL_SCALE_1X,
           base + VOP_WIN_SCL_FACTOR);

  /* Store state */

  g_vop[vop].win[win].format = format;
  g_vop[vop].win[win].width = w;
  g_vop[vop].win[win].height = h;
  g_vop[vop].win[win].stride = w * bpp;
  g_vop[vop].win[win].x = x;
  g_vop[vop].win[win].y = y;
  g_vop[vop].win[win].addr = addr;

  ginfo("VOP%d: WIN%d %dx%d+%d+%d fmt=%d addr=0x%" PRIx32 "\n",
        vop, win, w, h, x, y, format, addr);
  return OK;
}

int rk3576_vop_set_window_alpha(int vop, int win, int mode, uint8_t alpha)
{
  uint32_t base;
  uint32_t alpha_ctrl;

  if (vop < 0 || vop >= RK3576_VOP_COUNT ||
      win < 0 || win >= VOP_MAX_WINDOWS)
    {
      return -EINVAL;
    }

  base = vop_win_base(vop, win);

  alpha_ctrl = VOP_WIN_ALPHA_CTRL_EN |
               ((mode << VOP_WIN_ALPHA_CTRL_MODE_SHIFT) &
                VOP_WIN_ALPHA_CTRL_MODE_MASK) |
               ((alpha << VOP_WIN_ALPHA_CTRL_FACTOR_SHIFT) &
                VOP_WIN_ALPHA_CTRL_FACTOR_MASK);

  putreg32(alpha_ctrl, base + VOP_WIN_ALPHA_CTRL);

  g_vop[vop].win[win].alpha_mode = mode;

  ginfo("VOP%d: WIN%d alpha mode=%d alpha=%d\n", vop, win, mode, alpha);
  return OK;
}

int rk3576_vop_set_window_mirror(int vop, int win, int mirror)
{
  uint32_t base;
  uint32_t ctrl;

  if (vop < 0 || vop >= RK3576_VOP_COUNT ||
      win < 0 || win >= VOP_MAX_WINDOWS)
    {
      return -EINVAL;
    }

  base = vop_win_base(vop, win);

  ctrl = getreg32(base + VOP_WIN_CTRL);
  ctrl &= ~VOP_WIN_CTRL_MIRROR_MASK;
  ctrl |= (mirror << VOP_WIN_CTRL_MIRROR_SHIFT) & VOP_WIN_CTRL_MIRROR_MASK;
  putreg32(ctrl, base + VOP_WIN_CTRL);

  g_vop[vop].win[win].mirror = mirror;
  return OK;
}

int rk3576_vop_set_scaler(int vop, int win, int src_w, int src_h,
                            int dst_w, int dst_h)
{
  uint32_t base;
  uint32_t scl_factor;

  if (vop < 0 || vop >= RK3576_VOP_COUNT ||
      win < 0 || win >= VOP_MAX_WINDOWS)
    {
      return -EINVAL;
    }

  if (src_w <= 0 || src_h <= 0 || dst_w <= 0 || dst_h <= 0)
    {
      return -EINVAL;
    }

  base = vop_win_base(vop, win);

  /* Calculate scaling factor: factor = (src << 16) / dst */

  uint32_t x_factor = ((uint32_t)src_w << 16) / (uint32_t)dst_w;
  uint32_t y_factor = ((uint32_t)src_h << 16) / (uint32_t)dst_h;

  scl_factor = (y_factor << 16) | x_factor;
  putreg32(scl_factor, base + VOP_WIN_SCL_FACTOR);

  ginfo("VOP%d: WIN%d scaler %dx%d->%dx%d (factor=0x%08" PRIx32 ")\n",
        vop, win, src_w, src_h, dst_w, dst_h, scl_factor);
  return OK;
}

int rk3576_vop_enable_irq(int vop, uint32_t mask)
{
  uint32_t base;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  uint32_t inten = getreg32(base + VOP_REG_INTEN);
  inten |= mask;
  putreg32(inten, base + VOP_REG_INTEN);

  return OK;
}

int rk3576_vop_disable_irq(int vop, uint32_t mask)
{
  uint32_t base;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  uint32_t inten = getreg32(base + VOP_REG_INTEN);
  inten &= ~mask;
  putreg32(inten, base + VOP_REG_INTEN);

  return OK;
}

uint32_t rk3576_vop_get_status(int vop)
{
  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return 0;
    }

  return getreg32(g_vop_base[vop] + VOP_REG_INTSTS);
}

int rk3576_vop_clear_status(int vop, uint32_t mask)
{
  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  putreg32(mask, g_vop_base[vop] + VOP_REG_INTSTS);
  return OK;
}

int rk3576_vop_wait_vsync(int vop)
{
  uint32_t base;
  int timeout;

  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  base = g_vop_base[vop];

  /* Clear pending vsync */

  putreg32(VOP_INTSTS_VSYNC, base + VOP_REG_INTSTS);

  /* Wait for next vsync */

  for (timeout = 0; timeout < 50; timeout++)
    {
      if (getreg32(base + VOP_REG_INTSTS) & VOP_INTSTS_VSYNC)
        {
          putreg32(VOP_INTSTS_VSYNC, base + VOP_REG_INTSTS);
          return OK;
        }

      up_udelay(100);
    }

  return -ETIMEDOUT;
}

int rk3576_vop_get_line(int vop)
{
  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return -EINVAL;
    }

  return (int)(getreg32(g_vop_base[vop] + VOP_REG_LINE_FLAG) &
               VOP_LINE_FLAG_MASK);
}

uint32_t rk3576_vop_get_version(int vop)
{
  if (vop < 0 || vop >= RK3576_VOP_COUNT)
    {
      return 0;
    }

  return getreg32(g_vop_base[vop] + VOP_REG_VERSION);
}

int rk3576_vop_get_window_info(int vop, int win,
                                 struct rk3576_vop_window_info_s *info)
{
  if (vop < 0 || vop >= RK3576_VOP_COUNT ||
      win < 0 || win >= VOP_MAX_WINDOWS || info == NULL)
    {
      return -EINVAL;
    }

  const struct rk3576_vop_window_s *w = &g_vop[vop].win[win];

  info->enabled = w->enabled;
  info->format = w->format;
  info->width = w->width;
  info->height = w->height;
  info->x = w->x;
  info->y = w->y;
  info->addr = w->addr;

  return OK;
}

#endif /* CONFIG_RK3576_VOP */
