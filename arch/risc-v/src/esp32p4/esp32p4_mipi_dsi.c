/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_mipi_dsi.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/video/fb.h>

#include "esp_cache.h"
#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "hal/mipi_dsi_host_ll.h"
#include "hal/mipi_dsi_brg_ll.h"
#include "hal/mipi_dsi_phy_ll.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/pmu_struct.h"

#include "esp32p4_mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSI_REF_CLK_HZ           40000000UL /* 40 MHz XTAL */
#define DSI_DPI_SRC_CLK_MHZ      240.0f     /* 240 MHz PLL */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32p4_dsi_dev_s
{
  struct fb_vtable_s          vtable;
  mipi_dsi_hal_context_t      hal;
  struct esp32p4_dsi_config_s config;
  mutex_t                     lock;
  FAR uint8_t                *fb_mem;
  size_t                      fb_size;
  bool                        streaming;
  bool                        initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int esp32p4_fb_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_videoinfo_s *vinfo);
static int esp32p4_fb_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                   int planeno,
                                   FAR struct fb_planeinfo_s *pinfo);
static int esp32p4_fb_open(FAR struct fb_vtable_s *vtable);
static int esp32p4_fb_close(FAR struct fb_vtable_s *vtable);
static int esp32p4_fb_pandisplay(FAR struct fb_vtable_s *vtable,
                                 FAR struct fb_planeinfo_s *pinfo);
static int esp32p4_fb_ioctl(FAR struct fb_vtable_s *vtable,
                            int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32p4_dsi_dev_s g_dsi_dev =
{
  .vtable =
  {
    .getvideoinfo = esp32p4_fb_getvideoinfo,
    .getplaneinfo = esp32p4_fb_getplaneinfo,
    .open         = esp32p4_fb_open,
    .close        = esp32p4_fb_close,
    .pandisplay   = esp32p4_fb_pandisplay,
    .ioctl        = esp32p4_fb_ioctl,
  },
  .lock           = NXMUTEX_INITIALIZER,
  .fb_mem         = NULL,
  .fb_size        = 0,
  .streaming      = false,
  .initialized    = false,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_dsi_enable_clocks
 ****************************************************************************/

static void esp32p4_dsi_enable_clocks(uint32_t div)
{
  /* 1. Enable peripheral system clock and PHY configuration clock */

  HP_SYS_CLKRST.soc_clk_ctrl1.reg_dsi_sys_clk_en = 1;
  HP_SYS_CLKRST.peri_clk_ctrl03.reg_mipi_dsi_dphy_cfg_clk_en = 1;
  HP_SYS_CLKRST.peri_clk_ctrl03.reg_mipi_dsi_dphy_pll_refclk_en = 1;

  /* 2. Configure DPI pixel clock source and divider */

  HP_SYS_CLKRST.peri_clk_ctrl03.reg_mipi_dsi_dpiclk_src_sel = 1; /* PLL_F240M */
  if (div > 0)
    {
      HP_SYS_CLKRST.peri_clk_ctrl03.reg_mipi_dsi_dpiclk_div_num = div - 1;
    }

  HP_SYS_CLKRST.peri_clk_ctrl03.reg_mipi_dsi_dpiclk_en = 1;

  /* 3. Release reset */

  HP_SYS_CLKRST.hp_rst_en0.reg_rst_en_dsi_brg = 0;
}

/****************************************************************************
 * Name: esp32p4_fb_getvideoinfo
 ****************************************************************************/

static int esp32p4_fb_getvideoinfo(FAR struct fb_vtable_s *vtable,
                                   FAR struct fb_videoinfo_s *vinfo)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  if (vinfo == NULL)
    {
      return -EINVAL;
    }

  vinfo->fmt     = (priv->config.color_format == ESP32P4_DSI_COLOR_RGB888) ?
                   FB_FMT_RGB24 : FB_FMT_RGB16_565;
  vinfo->xres    = priv->config.timing.width;
  vinfo->yres    = priv->config.timing.height;
  vinfo->nplanes = 1;

  return OK;
}

/****************************************************************************
 * Name: esp32p4_fb_getplaneinfo
 ****************************************************************************/

static int esp32p4_fb_getplaneinfo(FAR struct fb_vtable_s *vtable,
                                   int planeno,
                                   FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;
  uint8_t bpp;

  if (pinfo == NULL || planeno != 0)
    {
      return -EINVAL;
    }

  bpp = (priv->config.color_format == ESP32P4_DSI_COLOR_RGB888) ? 24 : 16;

  pinfo->fbmem  = priv->fb_mem;
  pinfo->fblen  = priv->fb_size;
  pinfo->stride = priv->config.timing.width * (bpp / 8);
  pinfo->display = 0;
  pinfo->bpp    = bpp;

  return OK;
}

/****************************************************************************
 * Name: esp32p4_fb_open
 ****************************************************************************/

static int esp32p4_fb_open(FAR struct fb_vtable_s *vtable)
{
  return OK;
}

/****************************************************************************
 * Name: esp32p4_fb_close
 ****************************************************************************/

static int esp32p4_fb_close(FAR struct fb_vtable_s *vtable)
{
  return OK;
}

/****************************************************************************
 * Name: esp32p4_fb_pandisplay
 ****************************************************************************/

static int esp32p4_fb_pandisplay(FAR struct fb_vtable_s *vtable,
                                 FAR struct fb_planeinfo_s *pinfo)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  if (pinfo == NULL || pinfo->fbmem == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);
  priv->fb_mem = (FAR uint8_t *)pinfo->fbmem;

  /* Flush cache for the updated framebuffer memory */

  esp_cache_msync((void *)priv->fb_mem, priv->fb_size,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_fb_ioctl
 ****************************************************************************/

static int esp32p4_fb_ioctl(FAR struct fb_vtable_s *vtable,
                            int cmd, unsigned long arg)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;
  int ret = OK;

  switch (cmd)
    {
      case FBIOPAN_DISPLAY:
        ret = esp32p4_fb_pandisplay(vtable,
                                    (FAR struct fb_planeinfo_s *)arg);
        break;

      case FBIOSET_POWER:

        /* Optional power control */

        break;

      default:
        ginfo("Unsupported FB ioctl: 0x%04x\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_mipi_dsi_initialize
 ****************************************************************************/

int esp32p4_mipi_dsi_initialize(
    FAR const struct esp32p4_dsi_config_s *config)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;
  mipi_dsi_hal_config_t hal_cfg;
  uint32_t div;
  lcd_color_format_t color_fmt;

  if (config == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  memcpy(&priv->config, config, sizeof(struct esp32p4_dsi_config_s));

  /* 1. Calculate clock divider for DPI */

  div = (uint32_t)(DSI_DPI_SRC_CLK_MHZ / config->timing.dpi_clk_mhz);
  if (div == 0)
    {
      div = 1;
    }

  esp32p4_dsi_enable_clocks(div);

  /* 2. Initialize HAL context */

  hal_cfg.bus_id             = 0;
  hal_cfg.lane_bit_rate_mbps = config->lane_bit_rate_mbps;
  hal_cfg.num_data_lanes     = config->num_lanes;

  mipi_dsi_hal_init(&priv->hal, &hal_cfg);

  /* 3. Configure D-PHY TX PLL */

  mipi_dsi_hal_configure_phy_pll(&priv->hal, DSI_REF_CLK_HZ,
                                 config->lane_bit_rate_mbps);

  /* 4. Configure horizontal and vertical video timing */

  mipi_dsi_hal_host_dpi_calculate_divider(&priv->hal, DSI_DPI_SRC_CLK_MHZ,
                                          config->timing.dpi_clk_mhz);

  mipi_dsi_hal_host_dpi_set_horizontal_timing(&priv->hal,
                                              config->timing.hsw,
                                              config->timing.hbp,
                                              config->timing.width,
                                              config->timing.hfp);

  mipi_dsi_hal_host_dpi_set_vertical_timing(&priv->hal,
                                            config->timing.vsw,
                                            config->timing.vbp,
                                            config->timing.height,
                                            config->timing.vfp);

  /* 5. Set Color format in Bridge */

  color_fmt = (config->color_format == ESP32P4_DSI_COLOR_RGB888) ?
              LCD_COLOR_FMT_RGB888 : LCD_COLOR_FMT_RGB565;

  mipi_dsi_brg_ll_set_input_color_format(priv->hal.bridge, color_fmt);
  mipi_dsi_brg_ll_set_output_color_format(priv->hal.bridge, color_fmt, 0);

  priv->initialized = true;
  nxmutex_unlock(&priv->lock);

  ginfo("ESP32-P4 MIPI DSI initialized: %d lanes @ %.1f Mbps\n",
        config->num_lanes, config->lane_bit_rate_mbps);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_dsi_write_dcs
 ****************************************************************************/

int esp32p4_mipi_dsi_write_dcs(uint8_t vc, uint8_t cmd,
                               FAR const void *param, uint16_t param_size)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  if (!priv->initialized)
    {
      return -EPERM;
    }

  nxmutex_lock(&priv->lock);
  mipi_dsi_hal_host_gen_write_dcs_command(&priv->hal, vc, cmd, 1,
                                          param, param_size);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_dsi_read_dcs
 ****************************************************************************/

int esp32p4_mipi_dsi_read_dcs(uint8_t vc, uint8_t cmd,
                              FAR void *param, uint16_t param_size)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  if (!priv->initialized || param == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);
  mipi_dsi_hal_host_gen_read_dcs_command(&priv->hal, vc, cmd, 1,
                                         param, param_size);
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_dsi_set_framebuffer
 ****************************************************************************/

int esp32p4_mipi_dsi_set_framebuffer(uintptr_t fb_addr, size_t fb_size)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  nxmutex_lock(&priv->lock);
  priv->fb_mem  = (FAR uint8_t *)fb_addr;
  priv->fb_size = fb_size;
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_dsi_start_video
 ****************************************************************************/

int esp32p4_mipi_dsi_start_video(void)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  if (!priv->initialized)
    {
      return -EPERM;
    }

  nxmutex_lock(&priv->lock);

  /* Enable DSI Host Video Mode & DSI Bridge */

  mipi_dsi_host_ll_enable_video_mode(priv->hal.host, true);
  mipi_dsi_brg_ll_enable(priv->hal.bridge, true);

  priv->streaming = true;
  nxmutex_unlock(&priv->lock);

  ginfo("ESP32-P4 MIPI DSI Video Stream started [OK]\n");
  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_dsi_stop_video
 ****************************************************************************/

int esp32p4_mipi_dsi_stop_video(void)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  if (!priv->initialized)
    {
      return -EPERM;
    }

  nxmutex_lock(&priv->lock);
  mipi_dsi_brg_ll_enable(priv->hal.bridge, false);
  mipi_dsi_host_ll_enable_video_mode(priv->hal.host, false);
  priv->streaming = false;
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_fb_initialize
 ****************************************************************************/

FAR struct fb_vtable_s *esp32p4_fb_initialize(int display)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;
  uint8_t bytes_per_pixel;

  if (display != 0)
    {
      return NULL;
    }

  bytes_per_pixel = (priv->config.color_format ==
                     ESP32P4_DSI_COLOR_RGB888) ? 3 : 2;

  if (priv->fb_mem == NULL && priv->config.timing.width > 0 &&
      priv->config.timing.height > 0)
    {
      priv->fb_size = priv->config.timing.width *
                      priv->config.timing.height * bytes_per_pixel;
      priv->fb_mem  = (FAR uint8_t *)kmm_memalign(64, priv->fb_size);
      if (priv->fb_mem != NULL)
        {
          memset(priv->fb_mem, 0, priv->fb_size);
          esp_cache_msync((void *)priv->fb_mem, priv->fb_size,
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M);
        }
    }

  return &priv->vtable;
}

/****************************************************************************
 * Name: up_fbinitialize
 ****************************************************************************/

int up_fbinitialize(int display)
{
  return (esp32p4_fb_initialize(display) != NULL) ? OK : -ENODEV;
}

/****************************************************************************
 * Name: up_fbgetvplane
 ****************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int display, int plane)
{
  if (display != 0 || plane != 0)
    {
      return NULL;
    }

  return esp32p4_fb_initialize(display);
}

/****************************************************************************
 * Name: up_fbuninitialize
 ****************************************************************************/

void up_fbuninitialize(int display)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;

  if (display == 0 && priv->fb_mem != NULL)
    {
      kmm_free(priv->fb_mem);
      priv->fb_mem = NULL;
      priv->fb_size = 0;
    }
}
