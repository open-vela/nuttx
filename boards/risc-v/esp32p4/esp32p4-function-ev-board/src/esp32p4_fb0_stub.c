/****************************************************************************
 * esp32p4_fb0_stub.c - Framebuffer device backed by MIPI-DSI
 *
 * Implements board_fb_initialize() for the ESP32-P4 Function EV Board.
 * Uses the DSI driver's framebuffer and registers a proper fb device.
 * FBIOPAN_DISPLAY triggers a flush of fb content to the DSI bridge.
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/video/fb.h>
#include <nuttx/kmalloc.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include <syslog.h>

#include "esp_mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define FB_XRES       ESP_DSI_HRES
#define FB_YRES       ESP_DSI_VRES
#define FB_BPP        ESP_DSI_FB_BPP
#define FB_STRIDE     (FB_XRES * (FB_BPP / 8))
#define FB_SIZE       ESP_DSI_FB_SIZE

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_fb_getvideoinfo(struct fb_vtable_s *vtable,
                               struct fb_videoinfo_s *vinfo);
static int esp_fb_getplaneinfo(struct fb_vtable_s *vtable,
                               int planeno,
                               struct fb_planeinfo_s *pinfo);
static int esp_fb_pandisplay(struct fb_vtable_s *vtable,
                             struct fb_planeinfo_s *pinfo);

/****************************************************************************
 * Private Data
 ****************************************************************************/

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
  .pandisplay   = esp_fb_pandisplay,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static int esp_fb_pandisplay(struct fb_vtable_s *vtable,
                             struct fb_planeinfo_s *pinfo)
{
  /* Flush the framebuffer to DSI bridge on every pan_display call */

  esp_mipi_dsi_flush_fb();
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int esp_fb0_stub_register(void)
{
  return OK;
}

int board_fb_initialize(void)
{
  uint8_t *fb;

  fb = esp_mipi_dsi_get_fb();
  if (fb == NULL)
    {
      syslog(LOG_ERR, "FB: DSI framebuffer not available\n");
      return -ENOMEM;
    }

  g_planeinfo.fbmem        = (void *)fb;
  g_planeinfo.fblen        = FB_SIZE;
  g_planeinfo.stride       = FB_STRIDE;
  g_planeinfo.bpp          = FB_BPP;
  g_planeinfo.display      = 0;
  g_planeinfo.xres_virtual = FB_XRES;
  g_planeinfo.yres_virtual = FB_YRES * 2;  /* Double buffer for pan */
  g_planeinfo.xoffset      = 0;
  g_planeinfo.yoffset      = 0;

  int ret = fb_register_device(0, 0, &g_vtable);
  if (ret < 0)
    {
      syslog(LOG_ERR, "FB: fb_register_device failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "FB: /dev/fb0 registered (%dx%d %dbpp, buf=%p)\n",
         FB_XRES, FB_YRES, FB_BPP, fb);
  return OK;
}
