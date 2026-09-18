/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_lcd.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <syslog.h>

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/video/fb.h>

#include "espressif/esp_gpio.h"
#include "espressif/esp_mipi_dsi.h"

#if defined(CONFIG_ESPRESSIF_MIPI_DSI) && defined(CONFIG_VIDEO_FB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ESP32-P4-Function-EV-Board 7" 1024x600 EK79007 MIPI-DSI panel */

#define LCD_XRES          1024
#define LCD_YRES          600
#define LCD_BPP           16
#define LCD_STRIDE        (LCD_XRES * LCD_BPP / 8)
#define LCD_FB_SIZE       (LCD_STRIDE * LCD_YRES)

#define LCD_GPIO_RST      27      /* Panel reset, active low */
#define LCD_GPIO_BL       26      /* Backlight enable, active high */

#define LCD_DSI_LANES     2
#define LCD_DSI_RATE_MBPS 1000
#define LCD_DPHY_LDO_CHAN 3       /* LDO_VO3 powers VDD_MIPI_DPHY */
#define LCD_DPHY_LDO_MV   2500

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ek79007_cmd_s
{
  uint8_t cmd;
  uint8_t data;
  uint8_t data_len;   /* 0 = command without parameter */
  uint16_t delay_ms;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp_getvideoinfo(struct fb_vtable_s *vtable,
                            struct fb_videoinfo_s *vinfo);
static int esp_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                            struct fb_planeinfo_s *pinfo);
#ifdef CONFIG_FB_UPDATE
static int esp_updatearea(struct fb_vtable_s *vtable,
                          const struct fb_area_s *area);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* EK79007 vendor init sequence (2-lane pad control + gamma/timing
 * registers + sleep-out), taken from the panel vendor reference.
 */

static const struct ek79007_cmd_s g_ek79007_init[] =
{
  { 0xb2, 0x10, 1, 0 },     /* Pad control: 2-lane MIPI */
  { 0x80, 0x8b, 1, 0 },
  { 0x81, 0x78, 1, 0 },
  { 0x82, 0x84, 1, 0 },
  { 0x83, 0x88, 1, 0 },
  { 0x84, 0xa8, 1, 0 },
  { 0x85, 0xe3, 1, 0 },
  { 0x86, 0x88, 1, 0 },
  { 0x11, 0x00, 0, 120 },   /* Sleep out */
};

static const struct esp_mipi_dsi_video_s g_panel_timing =
{
  .h_size      = LCD_XRES,
  .v_size      = LCD_YRES,
  .hsw         = 10,
  .hbp         = 160,
  .hfp         = 160,
  .vsw         = 1,
  .vbp         = 23,
  .vfp         = 12,
  .dpi_clk_mhz = 52,
};

static struct fb_vtable_s g_fb_vtable =
{
  .getvideoinfo = esp_getvideoinfo,
  .getplaneinfo = esp_getplaneinfo,
#ifdef CONFIG_FB_UPDATE
  .updatearea   = esp_updatearea,
#endif
};

/* Double buffering: LVGL and other clients render into the back buffer
 * (g_fb, exposed via the fb interface) while the DSI DMA continuously
 * scans out the front buffer.  On update only the dirty rows are copied
 * across and cache-synced, which shrinks the tearing window from the
 * whole rendering time down to a few milliseconds of row copy.
 */

static uint8_t *g_fb;        /* Back buffer, exposed to clients */
static uint8_t *g_fb_front;  /* Front buffer, scanned out by the DMA */
static bool g_lcd_initialized;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int esp_getvideoinfo(struct fb_vtable_s *vtable,
                            struct fb_videoinfo_s *vinfo)
{
  if (vinfo == NULL)
    {
      return -EINVAL;
    }

  vinfo->fmt     = FB_FMT_RGB16_565;
  vinfo->xres    = LCD_XRES;
  vinfo->yres    = LCD_YRES;
  vinfo->nplanes = 1;
  return OK;
}

static int esp_getplaneinfo(struct fb_vtable_s *vtable, int planeno,
                            struct fb_planeinfo_s *pinfo)
{
  if (pinfo == NULL || planeno != 0)
    {
      return -EINVAL;
    }

  memset(pinfo, 0, sizeof(*pinfo));
  pinfo->fbmem   = g_fb;
  pinfo->fblen   = LCD_FB_SIZE;
  pinfo->stride  = LCD_STRIDE;
  pinfo->display = 0;
  pinfo->bpp     = LCD_BPP;
  return OK;
}

#ifdef CONFIG_FB_UPDATE
static int esp_updatearea(struct fb_vtable_s *vtable,
                          const struct fb_area_s *area)
{
  /* Write back only the dirty rows (rows are contiguous in memory), so
   * frequent small UI updates don't pay for a whole-framebuffer sync.
   */

  size_t start = (size_t)area->y * LCD_STRIDE;
  size_t len   = (size_t)area->h * LCD_STRIDE;

  if (start >= LCD_FB_SIZE)
    {
      return OK;
    }

  if (start + len > LCD_FB_SIZE)
    {
      len = LCD_FB_SIZE - start;
    }

  memcpy(g_fb_front + start, g_fb + start, len);
  esp_mipi_dsi_flush(g_fb_front + start, len);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Power up and initialize the EK79007 DSI panel, allocate the PSRAM
 *   framebuffer and start the continuous DPI video stream.
 *
 ****************************************************************************/

int up_fbinitialize(int display)
{
  uint8_t pwr = 0;
  uint8_t id1 = 0;
  int rd;
  int ret;
  size_t i;

  if (g_lcd_initialized)
    {
      return OK;
    }

  /* Backlight off during panel bring-up, then hardware reset pulse */

  esp_configgpio(LCD_GPIO_BL, OUTPUT);
  esp_gpiowrite(LCD_GPIO_BL, false);

  esp_configgpio(LCD_GPIO_RST, OUTPUT);
  esp_gpiowrite(LCD_GPIO_RST, false);
  nxsig_usleep(10 * 1000);
  esp_gpiowrite(LCD_GPIO_RST, true);
  nxsig_usleep(20 * 1000);

  ret = esp_mipi_dsi_init(LCD_DSI_LANES, LCD_DSI_RATE_MBPS,
                          LCD_DPHY_LDO_CHAN, LCD_DPHY_LDO_MV);
  if (ret < 0)
    {
      lcderr("ERROR: esp_mipi_dsi_init failed: %d\n", ret);
      return ret;
    }

  /* Panel initialization over low-power DCS commands */

  for (i = 0; i < sizeof(g_ek79007_init) / sizeof(g_ek79007_init[0]); i++)
    {
      const struct ek79007_cmd_s *c = &g_ek79007_init[i];

      ret = esp_mipi_dsi_dcs_write(c->cmd, &c->data, c->data_len);
      if (ret < 0)
        {
          lcderr("ERROR: DCS 0x%02x failed: %d\n", c->cmd, ret);
          return ret;
        }

      if (c->delay_ms > 0)
        {
          nxsig_usleep((useconds_t)c->delay_ms * 1000);
        }
    }

  /* Panel presence check: a DCS read only succeeds if a powered panel
   * responds on the DSI link.  RDDPM (0x0a) returns the power mode and
   * RDID1 (0xda) the manufacturer ID.
   */

  rd = esp_mipi_dsi_dcs_read(0x0a, &pwr, 1);
  if (rd > 0)
    {
      esp_mipi_dsi_dcs_read(0xda, &id1, 1);
      syslog(LOG_INFO,
             "LCD: panel detected on DSI (power 0x%02x, ID1 0x%02x)\n",
             pwr, id1);
    }
  else
    {
      syslog(LOG_ERR,
             "LCD: no response from panel (rd=%d) - check the display "
             "cable and its power supply\n", rd);
    }

  esp_mipi_dsi_dump_status();

  /* Framebuffer lives in PSRAM (via the SPIRAM heap region) */

  g_fb = kmm_memalign(64, LCD_FB_SIZE);
  g_fb_front = kmm_memalign(64, LCD_FB_SIZE);
  if (g_fb == NULL || g_fb_front == NULL)
    {
      lcderr("ERROR: failed to allocate %d bytes framebuffers\n",
             LCD_FB_SIZE);
      kmm_free(g_fb);
      kmm_free(g_fb_front);
      g_fb = NULL;
      g_fb_front = NULL;
      return -ENOMEM;
    }

  memset(g_fb, 0, LCD_FB_SIZE);
  memset(g_fb_front, 0, LCD_FB_SIZE);
  esp_mipi_dsi_flush(g_fb_front, LCD_FB_SIZE);

  ret = esp_mipi_dsi_dpi_start(&g_panel_timing, g_fb_front);
  if (ret < 0)
    {
      lcderr("ERROR: esp_mipi_dsi_dpi_start failed: %d\n", ret);
      kmm_free(g_fb);
      kmm_free(g_fb_front);
      g_fb = NULL;
      g_fb_front = NULL;
      return ret;
    }

  esp_gpiowrite(LCD_GPIO_BL, true);

  g_lcd_initialized = true;
  return OK;
}

/****************************************************************************
 * Name: up_fbgetvplane
 ****************************************************************************/

struct fb_vtable_s *up_fbgetvplane(int display, int vplane)
{
  if (vplane != 0)
    {
      return NULL;
    }

  return &g_fb_vtable;
}

/****************************************************************************
 * Name: up_fbuninitialize
 ****************************************************************************/

void up_fbuninitialize(int display)
{
}

#endif /* CONFIG_ESPRESSIF_MIPI_DSI && CONFIG_VIDEO_FB */
