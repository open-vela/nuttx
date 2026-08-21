/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_display.c
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

/* ESP32-P4-Function-EV-Board framebuffer / MIPI-DSI display glue.
 *
 * Sequence (follows ESP-IDF DPI panel init order):
 *   1. Allocate RGB565 FB in PSRAM (page-aligned)
 *   2. Reset LCD / power on backlight GPIO
 *   3. Get MIPI-DSI host (registered by arch bringup)
 *   4. Configure DPI video-mode timing for EK79007 (800x1280)
 *   5. ILI9881C vendor DCS register init
 *   6. Bind framebuffer → video_start → display_on
 *   7. Enable backlight
 *
 * The scan geometry sent over the DSI bus is 1280 cols x 800 rows
 * (the ILI9881C native orientation).  The EK79007 carrier board
 * rotates the panel 90°, so the visible area and the LVGL
 * framebuffer are 800 wide x 1280 tall.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>
#include <string.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/compiler.h>
#include <nuttx/kmalloc.h>
#include <nuttx/video/fb.h>
#include <nuttx/video/mipi_display.h>
#include <nuttx/video/mipi_dsi.h>

#include <arch/board/board.h>

#include "espressif/esp_mipi_dsi.h"
#include "espressif/esp_gpio.h"

#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR uint16_t *g_funev_fb;
static bool          g_funev_fb_ready;

static struct fb_videoinfo_s g_funev_videoinfo =
{
  .fmt     = FB_FMT_RGB16_565,
  .xres    = FUNEV_FB_WIDTH,
  .yres    = FUNEV_FB_HEIGHT,
  .nplanes = 1,
};

static struct fb_planeinfo_s g_funev_plane =
{
  .display    = 0,
  .bpp        = FUNEV_FB_BPP,
  .xres_virtual = FUNEV_FB_WIDTH,
  .yres_virtual = FUNEV_FB_HEIGHT,
  .stride     = FUNEV_FB_STRIDE,
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int funev_getvideoinfo(FAR struct fb_vtable_s *vtable,
                              FAR struct fb_videoinfo_s *info);
static int funev_getplaneinfo(FAR struct fb_vtable_s *vtable,
                              int planeno,
                              FAR struct fb_planeinfo_s *pinfo);

#ifdef CONFIG_FB_UPDATE
static int funev_updatearea(FAR struct fb_vtable_s *vtable,
                            FAR const struct fb_overlayinfo_s *oinfo);
#endif

static void funev_fb_fill(uint16_t color);

/****************************************************************************
 * Private Data (vtable)
 ****************************************************************************/

static struct fb_vtable_s g_funev_vtable =
{
  .getvideoinfo = funev_getvideoinfo,
  .getplaneinfo = funev_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea   = funev_updatearea,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int funev_getvideoinfo(FAR struct fb_vtable_s *vtable,
                              FAR struct fb_videoinfo_s *info)
{
  DEBUGASSERT(info != NULL);
  memcpy(info, &g_funev_videoinfo, sizeof(*info));
  return OK;
}

static int funev_getplaneinfo(FAR struct fb_vtable_s *vtable,
                              int planeno,
                              FAR struct fb_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo != NULL);
  if (planeno != 0)
    {
      return -ENODEV;
    }

  memcpy(pinfo, &g_funev_plane, sizeof(*pinfo));
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int funev_updatearea(FAR struct fb_vtable_s *vtable,
                            FAR const struct fb_overlayinfo_s *oinfo)
{
  /* Video-mode panel: the DPI hardware scans the whole FB continuously.
   * A partial-area flush is therefore a no-op (or a cache write-back for
   * the touched region when CONFIG_ARCH_DCACHE is enabled).
   */

#ifdef CONFIG_ARCH_DCACHE
  if (g_funev_fb != NULL)
    {
      up_clean_dcache(
          (uintptr_t)((FAR uint8_t *)g_funev_fb + oinfo->offset),
          oinfo->offset + oinfo->blen);
    }
#endif

  return OK;
}
#endif

static void funev_fb_fill(uint16_t color)
{
  size_t i;
  size_t count = FUNEV_FB_WIDTH * FUNEV_FB_HEIGHT;

  if (g_funev_fb == NULL)
    {
      return;
    }

  for (i = 0; i < count; i++)
    {
      g_funev_fb[i] = color;
    }

#ifdef CONFIG_ARCH_DCACHE
  up_clean_dcache((uintptr_t)g_funev_fb,
                  (uintptr_t)g_funev_fb + FUNEV_FB_SIZE);
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: funev_lcd_reset
 *
 * Description:
 *   Toggle the ILI9881C XRES line (active low pulse).
 *
 ****************************************************************************/

int funev_lcd_reset(void)
{
  int ret;

  /* LCD_RST = GPIO_FUNEV_GPIO_LCD_RST, push-pull, initial high */

  ret = esp_configgpio(FUNEV_GPIO_LCD_RST, OUTPUT);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "lcd_reset: GPIO cfg failed: %d\n", ret);
      return ret;
    }

  /* Drive reset low for 20 ms, then release and wait 120 ms for ILI9881C
   * internal power-up (tRT = 10 ms min, panel recommends 120 ms for full
   * AVDD/AVDD rise on the EK79007 module).
   */

  esp_gpiowrite(FUNEV_GPIO_LCD_RST, false);
  up_mdelay(20);
  esp_gpiowrite(FUNEV_GPIO_LCD_RST, true);
  up_mdelay(120);

  return OK;
}

/****************************************************************************
 * Name: funev_lcd_backlight
 *
 * Description:
 *   Enable or disable the backlight boost converter (GPIO active high).
 *
 ****************************************************************************/

int funev_lcd_backlight(bool on)
{
  int ret;

  ret = esp_configgpio(FUNEV_GPIO_LCD_BL_EN, OUTPUT);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "backlight: GPIO cfg failed: %d\n", ret);
      return ret;
    }

  esp_gpiowrite(FUNEV_GPIO_LCD_BL_EN, on);
  return OK;
}

/****************************************************************************
 * Name: funev_mipi_dsi_dpi_config
 *
 * Description:
 *   Fill in the DPI video-mode timing for EK79007 / ILI9881C.
 *
 ****************************************************************************/

void funev_mipi_dsi_dpi_config(FAR struct esp_mipi_dsi_dpi_config_s *cfg)
{
  DEBUGASSERT(cfg != NULL);

  cfg->h_res               = FUNEV_MIPI_DSI_H_SCAN;
  cfg->v_res               = FUNEV_MIPI_DSI_V_SCAN;
  cfg->hsync_pulse_width   = FUNEV_MIPI_DSI_HSYNC_PULSE_WIDTH;
  cfg->hsync_back_porch    = FUNEV_MIPI_DSI_HSYNC_BACK_PORCH;
  cfg->hsync_front_porch   = FUNEV_MIPI_DSI_HSYNC_FRONT_PORCH;
  cfg->vsync_pulse_width   = FUNEV_MIPI_DSI_VSYNC_PULSE_WIDTH;
  cfg->vsync_back_porch    = FUNEV_MIPI_DSI_VSYNC_BACK_PORCH;
  cfg->vsync_front_porch   = FUNEV_MIPI_DSI_VSYNC_FRONT_PORCH;
  cfg->dpi_clock_freq_mhz  = FUNEV_MIPI_DSI_DPI_CLK_MHZ;
  cfg->virtual_channel     = 0;
  cfg->format              = MIPI_DSI_FMT_RGB565;
}

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the framebuffer for /dev/fb0.
 *   Called by the NuttX FB framework when CONFIG_FB is enabled.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  FAR struct mipi_dsi_host *host;
  FAR struct mipi_dsi_device *device;
  struct esp_mipi_dsi_dpi_config_s dpi;
  int ret;

  if (display != 0)
    {
      return -ENODEV;
    }

  if (g_funev_fb_ready)
    {
      return OK;
    }

  /* 1. Allocate RGB565 FB in PSRAM (32-byte aligned for DMA burst) */

  g_funev_fb = kumm_memalign(32, FUNEV_FB_SIZE);
  if (g_funev_fb == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: FB alloc failed (%u bytes; enable PSRAM)\n",
             (unsigned int)FUNEV_FB_SIZE);
      return -ENOMEM;
    }

  memset(g_funev_fb, 0, FUNEV_FB_SIZE);
  g_funev_plane.fbmem = g_funev_fb;

  /* 2. LCD reset (toggle XRES GPIO) */

  funev_lcd_reset();

  /* 3. Get MIPI-DSI host (registered by arch in esp_bringup) */

  host = esp_mipi_dsi_host_get();
  if (host == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: MIPI-DSI host not ready "
             "(enable CONFIG_ESP32P4_FUNCTION_EV_MIPI_DSI)\n");
      ret = -EAGAIN;
      goto errout_fb;
    }

  /* 4. Configure DPI video-mode timing */

  syslog(LOG_INFO, "Configuring DPI for EK79007...\n");
  funev_mipi_dsi_dpi_config(&dpi);
  ret = esp_mipi_dsi_configure_dpi(&dpi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: configure_dpi failed: %d\n", ret);
      goto errout_fb;
    }

  /* 5. ILI9881C vendor DCS init */

  syslog(LOG_INFO, "ILI9881C panel init...\n");
  device = funev_ek79007_initialize(host);
  if (device == NULL)
    {
      syslog(LOG_ERR, "ERROR: ILI9881C init failed\n");
      ret = -EIO;
      goto errout_fb;
    }

  /* 6. Bind FB and start video */

  ret = esp_mipi_dsi_bind_framebuffer(g_funev_fb, FUNEV_FB_SIZE,
                                      FUNEV_FB_WIDTH, FUNEV_FB_HEIGHT,
                                      FUNEV_FB_BPP);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bind_framebuffer failed: %d\n", ret);
      goto errout_fb;
    }

  funev_fb_fill(0xf800);   /* red test pattern */

  ret = mipi_dsi_dcs_exit_sleep_mode(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sleep_out failed: %d\n", ret);
      goto errout_fb;
    }

  up_mdelay(120);   /* tSLPOUT = 120 ms (ILI9881C datasheet) */

  syslog(LOG_INFO, "Starting video...\n");
  ret = esp_mipi_dsi_video_start();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: video_start failed: %d\n", ret);
      goto errout_fb;
    }

  ret = mipi_dsi_dcs_set_display_on(device);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: display_on failed: %d\n", ret);
      goto errout_fb;
    }

  funev_fb_fill(0x07e0);   /* green: display active */

  /* 7. Enable backlight */

  ret = funev_lcd_backlight(true);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WARNING: backlight failed: %d\n", ret);
    }

  g_funev_fb_ready = true;
  syslog(LOG_INFO,
         "/dev/fb0 ready %ux%u RGB565 @ %p\n",
         (unsigned int)FUNEV_FB_WIDTH,
         (unsigned int)FUNEV_FB_HEIGHT,
         g_funev_fb);

  return OK;

errout_fb:
  kumm_free(g_funev_fb);
  g_funev_fb       = NULL;
  g_funev_plane.fbmem = NULL;
  return ret;
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return the FB vtable for the requested display.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (display != 0 || vplane != 0)
    {
      return NULL;
    }

  return &g_funev_vtable;
}

/****************************************************************************
 * Name: up_fbuninitialize
 *
 * Description:
 *   Uninitialize the framebuffer.
 *
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  /* Display stays active until power off; just disable backlight */

  if (display == 0)
    {
      funev_lcd_backlight(false);
      g_funev_fb_ready = false;
    }
}
