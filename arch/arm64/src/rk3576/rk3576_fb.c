/***************************************************************************
 * arch/arm64/src/rk3576/rk3576_fb.c
 *
 * NuttX Framebuffer driver for RK3576 VOP (Video Output Processor)
 *
 * Provides /dev/fb0 interface for LVGL and other display clients.
 * Supports ARGB8888 and RGB565 pixel formats.
 *
 ***************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/video/fb.h>

#include "arm64_internal.h"
#include "rk3576_vop.h"
#include "rk3576_fb.h"

#ifdef CONFIG_RK3576_FB

/***************************************************************************
 * Pre-processor Definitions
 ***************************************************************************/

#ifndef CONFIG_RK3576_FB_WIDTH
#  define CONFIG_RK3576_FB_WIDTH   1024
#endif

#ifndef CONFIG_RK3576_FB_HEIGHT
#  define CONFIG_RK3576_FB_HEIGHT  600
#endif

#ifndef CONFIG_RK3576_FB_DEPTH
#  define CONFIG_RK3576_FB_DEPTH   32
#endif

#if CONFIG_RK3576_FB_DEPTH == 32
#  define FB_BPP                   4
#  define FB_PIXEL_FORMAT          VOP_FMT_ARGB8888
#  define FB_FBINFO_FMT            FB_FORMAT_ARGB8888
#elif CONFIG_RK3576_FB_DEPTH == 16
#  define FB_BPP                   2
#  define FB_PIXEL_FORMAT          VOP_FMT_RGB565
#  define FB_FBINFO_FMT            FB_FORMAT_RGB565
#else
#  error "Unsupported framebuffer depth"
#endif

#define FB_STRIDE                 (CONFIG_RK3576_FB_WIDTH * FB_BPP)
#define FB_SIZE                   (FB_STRIDE * CONFIG_RK3576_FB_HEIGHT)

/***************************************************************************
 * Private Types
 ***************************************************************************/

struct rk3576_fb_s
{
  struct fb_vtable_s vtable;  /* NuttX framebuffer interface */
  uint8_t *fbmem;             /* Framebuffer memory */
  uint32_t paddr;             /* Physical address */
  int vop;                    /* VOP index (0 or 1) */
};

/***************************************************************************
 * Private Function Prototypes
 ***************************************************************************/

static int rk3576_fb_open(struct fb_vtable_s *vtable);
static int rk3576_fb_close(struct fb_vtable_s *vtable);
static int rk3576_fb_getvideoinfo(struct fb_vtable_s *vtable,
                                   struct fb_videoinfo_s *vinfo);
static int rk3576_fb_getplaneinfo(struct fb_vtable_s *vtable,
                                   int planeno,
                                   struct fb_planeinfo_s *pinfo);

/***************************************************************************
 * Private Data
 ***************************************************************************/

static struct rk3576_fb_s g_fbdev;

/***************************************************************************
 * Private Functions
 ***************************************************************************/

static int rk3576_fb_open(struct fb_vtable_s *vtable)
{
  UNUSED(vtable);
  return OK;
}

static int rk3576_fb_close(struct fb_vtable_s *vtable)
{
  UNUSED(vtable);
  return OK;
}

static int rk3576_fb_getvideoinfo(struct fb_vtable_s *vtable,
                                   struct fb_videoinfo_s *vinfo)
{
  UNUSED(vtable);

  vinfo->xres = CONFIG_RK3576_FB_WIDTH;
  vinfo->yres = CONFIG_RK3576_FB_HEIGHT;
  vinfo->nplanes = 1;

  return OK;
}

static int rk3576_fb_getplaneinfo(struct fb_vtable_s *vtable,
                                   int planeno,
                                   struct fb_planeinfo_s *pinfo)
{
  struct rk3576_fb_s *fb = (struct rk3576_fb_s *)vtable;

  if (planeno != 0)
    {
      return -EINVAL;
    }

  pinfo->fbmem   = fb->fbmem;
  pinfo->fblen   = FB_SIZE;
  pinfo->bpp     = CONFIG_RK3576_FB_DEPTH;
  pinfo->stride  = FB_STRIDE;
  pinfo->display = 0;

  return OK;
}

/***************************************************************************
 * Public Functions
 ***************************************************************************/

int rk3576_fb_init(int vop)
{
  struct rk3576_fb_s *fb = &g_fbdev;
  int ret;

  memset(fb, 0, sizeof(*fb));
  fb->vop = vop;

  /* Allocate framebuffer memory (uncached for DMA coherency) */

  fb->fbmem = kmm_zalloc(FB_SIZE);
  if (fb->fbmem == NULL)
    {
      _err("FB: Failed to allocate %d bytes\n", FB_SIZE);
      return -ENOMEM;
    }

  /* Verify framebuffer address fits in 32-bit DMA address space */

  if ((uintptr_t)fb->fbmem > 0xFFFFFFFF)
    {
      _err("FB: Framebuffer address exceeds 32-bit DMA range\n");
      kmm_free(fb->fbmem);
      return -ENOMEM;
    }

  fb->paddr = (uint32_t)(uintptr_t)fb->fbmem;

  /* Setup NuttX framebuffer interface */

  fb->vtable.open        = rk3576_fb_open;
  fb->vtable.close       = rk3576_fb_close;
  fb->vtable.getvideoinfo = rk3576_fb_getvideoinfo;
  fb->vtable.getplaneinfo = rk3576_fb_getplaneinfo;

  /* Configure VOP */

  ret = rk3576_vop_init(vop);
  if (ret < 0)
    {
      _err("FB: VOP init failed: %d\n", ret);
      kmm_free(fb->fbmem);
      return ret;
    }

  rk3576_vop_set_resolution(vop, CONFIG_RK3576_FB_WIDTH,
                             CONFIG_RK3576_FB_HEIGHT);
  rk3576_vop_set_framebuffer(vop, fb->paddr);

  /* Enable VOP */

  ret = rk3576_vop_enable(vop);
  if (ret < 0)
    {
      _err("FB: VOP enable failed: %d\n", ret);
      kmm_free(fb->fbmem);
      return ret;
    }

  /* Register NuttX framebuffer device */

  ret = fb_register_device(0, 0, &fb->vtable);
  if (ret < 0)
    {
      _err("FB: Failed to register /dev/fb0: %d\n", ret);
      kmm_free(fb->fbmem);
      return ret;
    }

  ginfo("FB: /dev/fb0 registered (%dx%d, %dbpp)\n",
        CONFIG_RK3576_FB_WIDTH, CONFIG_RK3576_FB_HEIGHT,
        CONFIG_RK3576_FB_DEPTH);

  return OK;
}

uint8_t *rk3576_fb_getmem(void)
{
  return g_fbdev.fbmem;
}

#endif /* CONFIG_RK3576_FB */
