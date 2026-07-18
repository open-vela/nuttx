/****************************************************************************
 * boards/risc-v/esp32p4/common/src/esp_board_fb.c
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

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/video/fb.h>
#include <nuttx/kmalloc.h>

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Video information for the framebuffer */

static struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = FB_FMT_RGB16_565,
  .xres     = FB_XRES,
  .yres     = FB_YRES,
  .nplanes  = 1,
};

/* Plane information (populated at init time) */

static struct fb_planeinfo_s g_planeinfo;

/* Framebuffer vtable */

static struct fb_vtable_s g_vtable =
{
  .getvideoinfo = esp_fb_getvideoinfo,
  .getplaneinfo = esp_fb_getplaneinfo,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_fb_getvideoinfo
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

/****************************************************************************
 * Name: esp_fb_getplaneinfo
 ****************************************************************************/

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_fb_initialize
 *
 * Description:
 *   Initialize the MIPI-DSI display and register the framebuffer device
 *   at /dev/fb0.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_fb_initialize(void)
{
  int ret;
  uint8_t *fb;

  /* Get framebuffer allocated by DSI driver (DSI should already be
   * initialized by bringup calling esp_mipi_dsi_initialize() directly).
   */

  fb = esp_mipi_dsi_get_fb();
  if (fb == NULL)
    {
      /* DSI not yet initialized, do it now */

      ret = esp_mipi_dsi_initialize();
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: esp_mipi_dsi_initialize failed: %d\n", ret);
          return ret;
        }

      fb = esp_mipi_dsi_get_fb();
      if (fb == NULL)
        {
          syslog(LOG_ERR, "ERROR: No framebuffer from DSI driver\n");
          return -ENOMEM;
        }
    }

  /* Fill framebuffer with blue for testing */

  uint16_t *fb16 = (uint16_t *)fb;
  for (int i = 0; i < FB_XRES * FB_YRES; i++)
    {
      fb16[i] = 0x001F;  /* Blue in RGB565 */
    }

  /* Fill in plane info with real framebuffer */

  g_planeinfo.fbmem       = (void *)fb;
  g_planeinfo.fblen       = FB_SIZE;
  g_planeinfo.stride      = FB_STRIDE;
  g_planeinfo.bpp         = FB_BPP;
  g_planeinfo.display     = 0;
  g_planeinfo.xres_virtual = FB_XRES;
  g_planeinfo.yres_virtual = FB_YRES;
  g_planeinfo.xoffset     = 0;
  g_planeinfo.yoffset     = 0;

  /* Register the framebuffer device */

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
