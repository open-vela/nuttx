/****************************************************************************
 * esp32p4_fb0_stub.c - Framebuffer device backed by MIPI-DSI
 *
 * Implements board_fb_initialize() for the ESP32-P4 Function EV Board.
 *
 * When the DSI driver owns two contiguous display buffers, fb0 is a
 * double-buffered device: fblen and yres_virtual cover both buffers, so a
 * single mmap() hands userspace both halves, and FBIOPAN_DISPLAY selects
 * which half the display DMA scans out. That is what makes a zero-copy
 * camera preview possible: the capture DMA fills one half while the other
 * is on screen, then a pan flips them.
 *
 * FBIOPAN_DISPLAY also writes the panned-to half back from the CPU cache
 * so a CPU renderer's pixels reach memory before the half goes live.
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
  .fmt      = FB_FMT_RGB16_565,
  .xres     = FB_XRES,
  .yres     = FB_YRES,
  .nplanes  = 1,
};

static struct fb_planeinfo_s g_planeinfo;

/* Number of display buffers fb0 actually exposes: 2 when the DSI driver
 * owns two contiguous buffers, 1 otherwise.
 */

static int g_fb_count = 1;

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
  int index;
  int ret;

  if (pinfo == NULL)
    {
      return -EINVAL;
    }

  /* yoffset selects the half of the virtual framebuffer to display. With
   * two buffers anything at or past the panel height means buffer 1.
   */

  index = (pinfo->yoffset >= FB_YRES) ? 1 : 0;
  if (index >= g_fb_count)
    {
      index = g_fb_count - 1;
    }

  /* Write back the buffer being panned TO, not the one currently on
   * screen: a CPU renderer draws into the back buffer and expects those
   * pixels in memory before the DMA starts reading them. Order matters,
   * writeback first, then switch the front buffer.
   *
   * The zero-copy camera path does not need this at all - the camera DMA
   * wrote the pixels and the CPU never dirtied the lines - so it pays for
   * a 1.23MB walk over clean cache lines on every frame. Accepted for now;
   * removing it needs the caller to say whether it drew with the CPU.
   */

  esp_mipi_dsi_flush_fb_n(index);

  ret = esp_mipi_dsi_set_front_fb(index);

  /* The upper half queues one pan-info entry per FBIOPAN_DISPLAY and
   * expects the driver to consume it on vsync. This driver has no vsync
   * interrupt, so drop the previous entry here; otherwise the queue fills
   * up after the first pan and every later ioctl returns -ENOSPC. Drain it
   * even when the flip failed, or the queue stays wedged.
   */

  fb_remove_paninfo(vtable, FB_NO_OVERLAY);
  return ret;
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

  /* fb0 is double buffered whenever the DSI driver can back it that way.
   *
   * The requirement is contiguity: reporting fblen = FB_SIZE * 2 and
   * yres_virtual = FB_YRES * 2 promises userspace that one mmap covers
   * both buffers, so buffer 1 must sit immediately after buffer 0. The DSI
   * driver takes both from a single allocation to make that true, and
   * esp_mipi_dsi_fb_is_contiguous() reports whether it succeeded. When it
   * did not, fb0 falls back to exposing only the buffer being scanned out,
   * which is the old single-buffer contract.
   *
   * With two buffers exposed, FBIOPAN_DISPLAY picks the half to scan out
   * from pinfo.yoffset, so a producer can fill one half while the other is
   * live and flip between them without ever copying.
   */

  if (esp_mipi_dsi_get_fb_count() == 2 && esp_mipi_dsi_fb_is_contiguous())
    {
      g_fb_count               = 2;
      g_planeinfo.fbmem        = (void *)esp_mipi_dsi_get_fb_n(0);
      g_planeinfo.fblen        = FB_SIZE * 2;
      g_planeinfo.yres_virtual = FB_YRES * 2;
    }
  else
    {
      g_fb_count               = 1;
      g_planeinfo.fbmem        = (void *)fb;
      g_planeinfo.fblen        = FB_SIZE;
      g_planeinfo.yres_virtual = FB_YRES;
    }

  g_planeinfo.stride       = FB_STRIDE;
  g_planeinfo.bpp          = FB_BPP;
  g_planeinfo.display      = 0;
  g_planeinfo.xres_virtual = FB_XRES;
  g_planeinfo.xoffset      = 0;
  g_planeinfo.yoffset      = 0;

  int ret = fb_register_device(0, 0, &g_vtable);
  if (ret < 0)
    {
      syslog(LOG_ERR, "FB: fb_register_device failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO,
         "FB: /dev/fb0 registered (%dx%d %dbpp, %s, base=%p fblen=%zu)\n",
         FB_XRES, FB_YRES, FB_BPP,
         g_fb_count == 2 ? "double buffered" : "single buffered",
         g_planeinfo.fbmem, (size_t)g_planeinfo.fblen);
  return OK;
}
