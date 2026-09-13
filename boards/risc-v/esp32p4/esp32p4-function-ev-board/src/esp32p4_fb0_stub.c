/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_fb0_stub.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * /dev/fb0 for the ESP32-P4-Function-EV-Board MIPI-DSI display.
 *
 * Registers the NuttX framebuffer character device on top of the
 * chip-level MIPI-DSI driver (esp_mipi_dsi.h). The panel is RGB565
 * 1024x600. When the DSI driver owns two contiguous framebuffers, the
 * scanout buffer (the one the display DMA cycles through) is never
 * handed to userspace: /dev/fb0 exposes the second buffer, and the
 * updatearea hook (FBIO_UPDATE, issued by LVGL's NuttX fbdev after
 * every refresh) copies the dirty rows into the scanout buffer and
 * writes the CPU cache back. That keeps tearing confined to the few
 * milliseconds of row copy instead of the whole rendering pass. With
 * a single-allocated framebuffer the same hook degrades to writing
 * back just the dirty rows of the one buffer.
 *
 * The writeback is not optional: the graphics stack renders through
 * the cached PSRAM alias (0x48xxxxxx) while the display DMA reads
 * physical memory, so nothing reaches the panel until the dirty
 * region is synced. Do not switch this mapping to the non-cached
 * alias: that window (0x88xxxxxx) is not mapped for userspace on
 * this port, writes there are silently lost.
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <string.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/video/fb.h>

#include "esp_cache.h"
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
 * Private Data
 ****************************************************************************/

static struct fb_vtable_s g_vtable;
static struct fb_videoinfo_s g_videoinfo;
static struct fb_planeinfo_s g_planeinfo;

/* Buffer the display DMA cycles through (scanout) and the one exposed
 * to userspace for rendering; they are the same buffer when the DSI
 * driver could only allocate one framebuffer */

static uint8_t *g_fb_scan;
static uint8_t *g_fb_render;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int esp_fb_getvideoinfo(FAR struct fb_vtable_s *vtable,
                               FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(vtable != NULL && vinfo != NULL);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

static int esp_fb_getplaneinfo(FAR struct fb_vtable_s *vtable,
                               int planeno,
                               FAR struct fb_planeinfo_s *pinfo)
{
  DEBUGASSERT(vtable != NULL && pinfo != NULL && planeno == 0);
  memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
  return OK;
}

static int esp_fb_pandisplay(FAR struct fb_vtable_s *vtable,
                             FAR struct fb_planeinfo_s *pinfo)
{
  /* The scanout buffer is the one the DMA reads: write the whole
   * frame back from the CPU cache, then drain the pan-info entry the
   * upper half queued (no vsync interrupt in this driver) */

  esp_cache_msync((FAR void *)g_fb_scan, FB_SIZE,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  fb_remove_paninfo(vtable, FB_NO_OVERLAY);
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int esp_fb_updatearea(FAR struct fb_vtable_s *vtable,
                             FAR const struct fb_area_s *area)
{
  uintptr_t start;
  size_t len;
  esp_err_t ret;
  static int probe_calls;
  int y1;
  int y2;

  if (probe_calls < 3)
    {
      syslog(LOG_INFO,
             "FB: updatearea call %d: area=(%d,%d %dx%d)\n",
             probe_calls, area->x, area->y, area->w, area->h);
    }

  probe_calls++;

  /* Publish the dirty rows: copy them into the scanout buffer (when
   * rendering goes to a separate buffer) and write the CPU cache back
   * so the display DMA sees them. Rows are contiguous in memory, so
   * the range is naturally aligned with the cache line size; ignore
   * the x range and sync whole rows */

  y1 = area->y < 0 ? 0 : area->y;
  y2 = area->y + area->h;
  if (y2 > FB_YRES)
    {
      y2 = FB_YRES;
    }

  if (y1 >= y2 || area->w <= 0)
    {
      return OK;
    }

  start = (uintptr_t)y1 * FB_STRIDE;
  len   = (size_t)(y2 - y1) * FB_STRIDE;

  if (g_fb_render != NULL)
    {
      memcpy(g_fb_scan + start, g_fb_render + start, len);
    }

  ret = esp_cache_msync((FAR void *)(g_fb_scan + start), len,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  if (ret != ESP_OK)
    {
      /* A failed writeback silently strands the rendered pixels in
       * the CPU cache; make that visible instead of losing them
       * quietly */

      syslog(LOG_ERR, "FB: updatearea msync failed: %d\n", ret);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_fb_initialize(void)
{
  int ret;

  g_fb_scan = esp_mipi_dsi_get_fb();
  if (g_fb_scan == NULL)
    {
      syslog(LOG_ERR, "FB: DSI framebuffer not available\n");
      return -ENOMEM;
    }

  /* Render into the second buffer when one exists so updates can be
   * staged without touching the buffer under the DMA's feet */

  g_fb_render = esp_mipi_dsi_get_fb_n(1);

  g_planeinfo.fbmem        = (FAR void *)(g_fb_render != NULL ?
                                          g_fb_render : g_fb_scan);
  g_planeinfo.fblen        = FB_SIZE;
  g_planeinfo.yres_virtual = FB_YRES;
  g_planeinfo.stride       = FB_STRIDE;
  g_planeinfo.bpp          = FB_BPP;
  g_planeinfo.display      = 0;
  g_planeinfo.xres_virtual = FB_XRES;
  g_planeinfo.xoffset      = 0;
  g_planeinfo.yoffset      = 0;

  g_videoinfo.fmt         = FB_FMT_RGB16_565;
  g_videoinfo.xres        = FB_XRES;
  g_videoinfo.yres        = FB_YRES;
  g_videoinfo.nplanes     = 1;

  g_vtable.getvideoinfo = esp_fb_getvideoinfo;
  g_vtable.getplaneinfo = esp_fb_getplaneinfo;
  g_vtable.pandisplay   = esp_fb_pandisplay;
#ifdef CONFIG_FB_UPDATE
  g_vtable.updatearea   = esp_fb_updatearea;
#endif

  ret = fb_register_device(0, 0, &g_vtable);
  if (ret < 0)
    {
      syslog(LOG_ERR, "FB: fb_register_device failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO,
         "FB: /dev/fb0 registered (%dx%d %dbpp, base=%p scan=%p%s, "
         "fblen=%zu)\n",
         FB_XRES, FB_YRES, FB_BPP, g_planeinfo.fbmem, g_fb_scan,
         g_fb_render != NULL ? " render-staged" : " single", 
         (size_t)g_planeinfo.fblen);
  return OK;
}
