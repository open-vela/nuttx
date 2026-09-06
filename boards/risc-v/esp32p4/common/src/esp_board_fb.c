/****************************************************************************
 * boards/risc-v/esp32p4/common/src/esp_board_fb.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <nuttx/video/fb.h>

#include "esp_dsi_fb.h"

#define FB_XRES       ESP_DSI_HRES
#define FB_YRES       ESP_DSI_VRES
#define FB_BPP        ESP_DSI_FB_BPP
#define FB_STRIDE     (FB_XRES * (FB_BPP / 8))
#define FB_SIZE       ESP_DSI_FB_SIZE

static int esp_fb_getvideoinfo(struct fb_vtable_s *vtable,
                               struct fb_videoinfo_s *vinfo);
static int esp_fb_getplaneinfo(struct fb_vtable_s *vtable,
                               int planeno,
                               struct fb_planeinfo_s *pinfo);

static struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = FB_FMT_RGB24,
  .xres     = FB_XRES,
  .yres     = FB_YRES,
  .nplanes  = 1,
};

static struct fb_planeinfo_s g_planeinfo;

static struct fb_vtable_s g_vtable =
{
  .getvideoinfo = esp_fb_getvideoinfo,
  .getplaneinfo = esp_fb_getplaneinfo,
};

static int esp_fb_getvideoinfo(struct fb_vtable_s *vtable,
                               struct fb_videoinfo_s *vinfo)
{
  if (vtable == NULL || vinfo == NULL)
    {
      return -EINVAL;
    }

  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

static int esp_fb_getplaneinfo(struct fb_vtable_s *vtable,
                               int planeno,
                               struct fb_planeinfo_s *pinfo)
{
  if (vtable == NULL || pinfo == NULL || planeno != 0)
    {
      return -EINVAL;
    }

  memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
  return OK;
}

int board_fb_initialize(void)
{
  int ret;
  uint8_t *fb;

  /* Get framebuffer from DSI driver (already initialized by bringup) */

  fb = esp_dsi_fb_get_buffer();
  if (fb == NULL)
    {
      syslog(LOG_ERR, "ERROR: DSI framebuffer not available\n");
      return -ENOMEM;
    }

  /* Fill plane info */

  g_planeinfo.fbmem        = (void *)fb;
  g_planeinfo.fblen        = FB_SIZE;
  g_planeinfo.stride       = FB_STRIDE;
  g_planeinfo.bpp          = FB_BPP;
  g_planeinfo.display      = 0;
  g_planeinfo.xres_virtual = FB_XRES;
  g_planeinfo.yres_virtual = FB_YRES;
  g_planeinfo.xoffset      = 0;
  g_planeinfo.yoffset      = 0;

  /* Register /dev/fb0 */

  ret = fb_register_device(0, 0, &g_vtable);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register_device failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "Framebuffer registered at /dev/fb0 (%dx%d, %d bpp)\n",
         FB_XRES, FB_YRES, FB_BPP);
  return OK;
}
