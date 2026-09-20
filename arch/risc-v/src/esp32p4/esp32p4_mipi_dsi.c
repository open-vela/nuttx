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
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
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
#include "hal/dw_gdma_hal.h"
#include "hal/dw_gdma_ll.h"
#include "hal/dw_gdma_types.h"
#include "hal/ldo_ll.h"
#include "hal/clk_gate_ll.h"
#include "esp_private/periph_ctrl.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/pmu_struct.h"
#include "soc/reg_base.h"

#include "esp32p4_mipi_dsi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSI_REF_CLK_HZ           40000000UL /* 40 MHz XTAL */
#define DSI_DPI_SRC_CLK_MHZ      240.0f     /* 240 MHz PLL */
#define DSI_DMA_CHANNEL          3          /* CSI=0, PPA=0..2 */
#define DSI_MIPI_PHY_LDO_UNIT    2          /* LDO channel 3 */
#define DSI_MIPI_PHY_LDO_MV      2500
#define DSI_PHY_READY_TIMEOUT_US 100000
#define DSI_CMD_TIMEOUT_US       100000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32p4_dsi_dev_s
{
  struct fb_vtable_s          vtable;
  mipi_dsi_hal_context_t      hal;
  dw_gdma_hal_context_t       dma_hal;
  struct esp32p4_dsi_config_s config;
  mutex_t                     lock;
  FAR uint8_t                *fb_mem;
  size_t                      fb_size;
  bool                        streaming;
  bool                        dma_initialized;
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
  .dma_initialized = false,
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
  uint8_t dref = 0;
  uint8_t mul = 0;
  bool use_rail_voltage = false;

  /* CSI and DSI share LDO_VO3.  Re-enabling the configured rail is safe and
   * makes display initialization independent from camera initialization.
   */

  ldo_ll_voltage_to_dref_mul(DSI_MIPI_PHY_LDO_UNIT,
                             DSI_MIPI_PHY_LDO_MV,
                             &dref, &mul, &use_rail_voltage);
  ldo_ll_adjust_voltage(DSI_MIPI_PHY_LDO_UNIT, dref, mul,
                        use_rail_voltage);
  ldo_ll_set_owner(DSI_MIPI_PHY_LDO_UNIT, LDO_LL_UNIT_OWNER_SW);
  ldo_ll_enable_ripple_suppression(DSI_MIPI_PHY_LDO_UNIT, true);
  ldo_ll_enable(DSI_MIPI_PHY_LDO_UNIT, true);
  up_udelay(5000);

  PERIPH_RCC_ATOMIC()
    {
      clk_gate_ll_ref_20m_clk_en(true);
      mipi_dsi_ll_enable_bus_clock(0, true);
      mipi_dsi_ll_reset_register(0);
      mipi_dsi_ll_set_phy_config_clock_source(
        0, MIPI_DSI_PHY_CFG_CLK_SRC_PLL_F20M);
      mipi_dsi_ll_enable_phy_config_clock(0, true);
      mipi_dsi_ll_set_phy_pllref_clock_source(
        0, MIPI_DSI_PHY_PLLREF_CLK_SRC_XTAL);
      mipi_dsi_ll_set_phy_pll_ref_clock_div(0, 1);
      mipi_dsi_ll_enable_phy_pllref_clock(0, true);
      mipi_dsi_ll_set_dpi_clock_source(0,
                                      MIPI_DSI_DPI_CLK_SRC_PLL_F240M);
      mipi_dsi_ll_set_dpi_clock_div(0, div);
      mipi_dsi_ll_enable_dpi_clock(0, true);
    }
}

static int esp32p4_dsi_wait_phy_ready(FAR struct esp32p4_dsi_dev_s *priv)
{
  unsigned int elapsed;

  for (elapsed = 0; elapsed < DSI_PHY_READY_TIMEOUT_US; elapsed += 100)
    {
      if (mipi_dsi_phy_ll_is_pll_locked(priv->hal.host) &&
          mipi_dsi_phy_ll_are_lanes_stopped(priv->hal.host,
                                             priv->config.num_lanes))
        {
          return OK;
        }

      up_udelay(100);
    }

  return -ETIMEDOUT;
}

static int esp32p4_dsi_configure_dma(FAR struct esp32p4_dsi_dev_s *priv)
{
  dw_gdma_dev_t *dev;
  uint32_t beats;

  if (priv->fb_mem == NULL || priv->fb_size == 0 ||
      (priv->fb_size % sizeof(uint64_t)) != 0)
    {
      return -EINVAL;
    }

  PERIPH_RCC_ATOMIC()
    {
      dw_gdma_ll_enable_bus_clock(0, true);
    }

  priv->dma_hal.dev = DW_GDMA_LL_GET_HW(0);
  dev = priv->dma_hal.dev;
  beats = priv->fb_size / sizeof(uint64_t);

  dw_gdma_ll_enable_controller(dev, true);
  dw_gdma_ll_channel_enable(dev, DSI_DMA_CHANNEL, false);
  dw_gdma_ll_channel_clear_intr(dev, DSI_DMA_CHANNEL, UINT32_MAX);
  dw_gdma_ll_channel_set_priority(dev, DSI_DMA_CHANNEL, 3);
  dw_gdma_ll_channel_set_trans_flow(dev, DSI_DMA_CHANNEL,
                                    DW_GDMA_ROLE_MEM,
                                    DW_GDMA_ROLE_PERIPH_DSI,
                                    DW_GDMA_FLOW_CTRL_SELF);
  dw_gdma_ll_channel_set_src_handshake_interface(dev, DSI_DMA_CHANNEL,
                                                 DW_GDMA_HANDSHAKE_HW);
  dw_gdma_ll_channel_set_dst_handshake_interface(dev, DSI_DMA_CHANNEL,
                                                 DW_GDMA_HANDSHAKE_HW);
  dw_gdma_ll_channel_set_dst_handshake_periph(dev, DSI_DMA_CHANNEL,
                                              DW_GDMA_ROLE_PERIPH_DSI);
  dw_gdma_ll_channel_set_src_multi_block_type(dev, DSI_DMA_CHANNEL,
                                              DW_GDMA_BLOCK_TRANSFER_RELOAD);
  dw_gdma_ll_channel_set_dst_multi_block_type(dev, DSI_DMA_CHANNEL,
                                              DW_GDMA_BLOCK_TRANSFER_RELOAD);
  dw_gdma_ll_channel_set_src_addr(dev, DSI_DMA_CHANNEL,
                                  (uint32_t)(uintptr_t)priv->fb_mem);
  dw_gdma_ll_channel_set_dst_addr(dev, DSI_DMA_CHANNEL,
                                  MIPI_DSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_trans_block_size(dev, DSI_DMA_CHANNEL, beats);
  dw_gdma_ll_channel_set_src_master_port(dev, DSI_DMA_CHANNEL,
                                         (uintptr_t)priv->fb_mem);
  dw_gdma_ll_channel_set_dst_master_port(dev, DSI_DMA_CHANNEL,
                                         MIPI_DSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_src_trans_width(dev, DSI_DMA_CHANNEL,
                                         DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_dst_trans_width(dev, DSI_DMA_CHANNEL,
                                         DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_src_burst_items(dev, DSI_DMA_CHANNEL,
                                         DW_GDMA_BURST_ITEMS_512);
  dw_gdma_ll_channel_set_dst_burst_items(dev, DSI_DMA_CHANNEL,
                                         DW_GDMA_BURST_ITEMS_256);
  dw_gdma_ll_channel_set_src_burst_mode(dev, DSI_DMA_CHANNEL,
                                        DW_GDMA_BURST_MODE_INCREMENT);
  dw_gdma_ll_channel_set_dst_burst_mode(dev, DSI_DMA_CHANNEL,
                                        DW_GDMA_BURST_MODE_FIXED);
  dw_gdma_ll_channel_set_src_burst_len(dev, DSI_DMA_CHANNEL, 16);
  dw_gdma_ll_channel_set_dst_burst_len(dev, DSI_DMA_CHANNEL, 16);
  dw_gdma_ll_channel_set_block_markers(dev, DSI_DMA_CHANNEL,
                                       false, false, true);
  dw_gdma_ll_channel_enable_intr_propagation(dev, DSI_DMA_CHANNEL,
                                             UINT32_MAX, false);
  priv->dma_initialized = true;

  return OK;
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
  uint32_t bits_per_pixel;
  lcd_color_format_t color_fmt;
  int ret;

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

  /* Keep the bridge register and FIFO clocks running during early bring-up.
   * This also prevents a first-burst stall while the DPI path is empty.
   */

  mipi_dsi_brg_ll_force_enable_reg_clock(priv->hal.bridge, true);
  mipi_dsi_brg_ll_enable_ref_clock(priv->hal.bridge, true);
  priv->hal.bridge->mem_clk_ctrl.dsi_bridge_mem_clk_force_on = 1;
  priv->hal.bridge->mem_clk_ctrl.dsi_mem_clk_force_on = 1;

  /* 3. Configure D-PHY TX PLL */

  mipi_dsi_hal_configure_phy_pll(&priv->hal, DSI_REF_CLK_HZ,
                                 config->lane_bit_rate_mbps);

  ret = esp32p4_dsi_wait_phy_ready(priv);
  if (ret < 0)
    {
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* Command mode is used for EK79007 initialization before video starts. */

  mipi_dsi_host_ll_enable_video_mode(priv->hal.host, false);
  mipi_dsi_host_ll_set_clock_lane_state(
    priv->hal.host, MIPI_DSI_LL_CLOCK_LANE_STATE_AUTO);
  mipi_dsi_phy_ll_set_switch_time(priv->hal.host, 50, 104, 46, 128);
  mipi_dsi_host_ll_enable_rx_crc(priv->hal.host, true);
  mipi_dsi_host_ll_enable_rx_ecc(priv->hal.host, true);
  mipi_dsi_host_ll_enable_tx_eotp(priv->hal.host, true, false);
  mipi_dsi_host_ll_set_timeout_clock_division(
    priv->hal.host, (uint32_t)(config->lane_bit_rate_mbps / 80.0f + 0.5f));
  mipi_dsi_host_ll_set_escape_clock_division(
    priv->hal.host, (uint32_t)(config->lane_bit_rate_mbps / 144.0f + 0.5f));
  mipi_dsi_host_ll_set_timeout_count(priv->hal.host, 0, 0, 0, 0,
                                     0, 0, 0);
  mipi_dsi_phy_ll_set_max_read_time(priv->hal.host, 6000);
  mipi_dsi_phy_ll_set_stop_wait_time(priv->hal.host, 0x3f);

  mipi_dsi_host_ll_enable_te_ack(priv->hal.host, false);
  /* The EK79007 module does not return write acknowledgements during its
   * reset-time initialization.  Requesting ACK leaves the command buffered
   * while lane 0 waits in bus-turnaround direction.
   */

  mipi_dsi_host_ll_enable_cmd_ack(priv->hal.host, false);
  mipi_dsi_host_ll_enable_bta(priv->hal.host, false);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(
    priv->hal.host, 0, MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(
    priv->hal.host, 1, MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(
    priv->hal.host, 2, MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_long_wr_speed_mode(
    priv->hal.host, MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_short_wr_speed_mode(
    priv->hal.host, 0, MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_short_wr_speed_mode(
    priv->hal.host, 1, MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_long_wr_speed_mode(
    priv->hal.host, MIPI_DSI_LL_TRANS_SPEED_LP);

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
  bits_per_pixel = (config->color_format == ESP32P4_DSI_COLOR_RGB888) ?
                   24 : 16;

  mipi_dsi_host_ll_dpi_set_vcid(priv->hal.host, 0);
  mipi_dsi_host_ll_dpi_set_color_coding(priv->hal.host, color_fmt, 0);
  mipi_dsi_host_ll_dpi_set_timing_polarity(priv->hal.host, false, false,
                                            false, false, false);
  mipi_dsi_host_ll_dpi_enable_lp_horizontal_timing(priv->hal.host,
                                                    true, true);
  mipi_dsi_host_ll_dpi_enable_lp_vertical_timing(priv->hal.host,
                                                  true, true, true, true);
  mipi_dsi_host_ll_dpi_enable_lp_command(priv->hal.host, true);
  mipi_dsi_host_ll_dpi_enable_frame_ack(priv->hal.host, true);
  mipi_dsi_host_ll_dpi_set_video_burst_type(
    priv->hal.host, MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);
  mipi_dsi_host_ll_dpi_set_video_packet_pixel_num(priv->hal.host,
                                                   config->timing.width);
  mipi_dsi_host_ll_dpi_set_trunks_num(priv->hal.host, 0);
  mipi_dsi_host_ll_dpi_set_null_packet_size(priv->hal.host, 0);

  mipi_dsi_brg_ll_set_input_color_format(priv->hal.bridge, color_fmt);
  mipi_dsi_brg_ll_set_output_color_format(priv->hal.bridge, color_fmt, 0);
  mipi_dsi_brg_ll_set_num_pixel_bits(priv->hal.bridge,
                                     config->timing.width *
                                     config->timing.height *
                                     bits_per_pixel);
  mipi_dsi_brg_ll_set_underrun_discard_count(priv->hal.bridge,
                                              config->timing.width);
  mipi_dsi_brg_ll_set_flow_controller(priv->hal.bridge,
                                      MIPI_DSI_LL_FLOW_CONTROLLER_DMA);
  mipi_dsi_brg_ll_set_multi_block_number(priv->hal.bridge, 1);
  mipi_dsi_brg_ll_set_burst_len(priv->hal.bridge, 256);
  mipi_dsi_brg_ll_set_empty_threshold(priv->hal.bridge, 1024 - 256);
  mipi_dsi_brg_ll_enable(priv->hal.bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(priv->hal.bridge);

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
  unsigned int elapsed;
  int ret = OK;

  if (!priv->initialized)
    {
      return -EPERM;
    }

  nxmutex_lock(&priv->lock);
  mipi_dsi_hal_host_gen_write_dcs_command(&priv->hal, vc, cmd, 1,
                                          param, param_size);

  /* The HAL queues commands asynchronously.  The panel setup sequence must
   * not switch the host into video mode while a command is still buffered.
   */

  for (elapsed = 0; elapsed < DSI_CMD_TIMEOUT_US; elapsed += 100)
    {
      if (priv->hal.host->cmd_pkt_status.gen_cmd_empty &&
          priv->hal.host->cmd_pkt_status.gen_pld_w_empty &&
          priv->hal.host->cmd_pkt_status.gen_buff_cmd_empty &&
          priv->hal.host->cmd_pkt_status.gen_buff_pld_empty)
        {
          break;
        }

      up_udelay(100);
    }

  if (elapsed >= DSI_CMD_TIMEOUT_US)
    {
      ret = -ETIMEDOUT;
    }

  nxmutex_unlock(&priv->lock);

  return ret;
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
  int ret;

  if (!priv->initialized || fb_addr == 0 || fb_size == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);
  priv->fb_mem  = (FAR uint8_t *)fb_addr;
  priv->fb_size = fb_size;
  esp_cache_msync(priv->fb_mem, priv->fb_size,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  ret = esp32p4_dsi_configure_dma(priv);
  nxmutex_unlock(&priv->lock);

  return ret;
}

/****************************************************************************
 * Name: esp32p4_mipi_dsi_start_video
 ****************************************************************************/

int esp32p4_mipi_dsi_start_video(void)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;
  int ret;

  if (!priv->initialized)
    {
      return -EPERM;
    }

  nxmutex_lock(&priv->lock);

  if (!priv->dma_initialized)
    {
      ret = esp32p4_dsi_configure_dma(priv);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return ret;
        }
    }

  /* Start memory-to-DSI transfer before enabling DPI output. */

  dw_gdma_ll_channel_enable(priv->dma_hal.dev, DSI_DMA_CHANNEL, true);
  mipi_dsi_host_ll_enable_video_mode(priv->hal.host, true);
  mipi_dsi_brg_ll_enable(priv->hal.bridge, true);
  mipi_dsi_brg_ll_enable_dpi_output(priv->hal.bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(priv->hal.bridge);

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
  mipi_dsi_brg_ll_enable_dpi_output(priv->hal.bridge, false);
  mipi_dsi_brg_ll_update_dpi_config(priv->hal.bridge);
  mipi_dsi_brg_ll_enable(priv->hal.bridge, false);
  mipi_dsi_host_ll_enable_video_mode(priv->hal.host, false);
  if (priv->dma_initialized)
    {
      dw_gdma_ll_channel_enable(priv->dma_hal.dev, DSI_DMA_CHANNEL, false);
    }

  priv->streaming = false;
  nxmutex_unlock(&priv->lock);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_dsi_dump
 ****************************************************************************/

void esp32p4_mipi_dsi_dump(void)
{
  FAR struct esp32p4_dsi_dev_s *priv = &g_dsi_dev;
  FAR dw_gdma_dev_t *dma = priv->dma_hal.dev;

  printf("=== ESP32-P4 MIPI DSI ===\n");
  printf("state init=%d stream=%d dma=%d fb=%p bytes=%zu\n",
         priv->initialized, priv->streaming, priv->dma_initialized,
         priv->fb_mem, priv->fb_size);
  if (priv->hal.host != NULL)
    {
      printf("host version=%08" PRIx32 " pwr=%08" PRIx32
             " mode=%08" PRIx32 " vid=%08" PRIx32 "\n",
             priv->hal.host->version.val, priv->hal.host->pwr_up.val,
             priv->hal.host->mode_cfg.val,
             priv->hal.host->vid_mode_cfg.val);
      printf("host phy=%08" PRIx32 " cmd=%08" PRIx32
             " int0=%08" PRIx32 " int1=%08" PRIx32 "\n",
             priv->hal.host->phy_status.val,
             priv->hal.host->cmd_pkt_status.val,
             priv->hal.host->int_st0.val,
             priv->hal.host->int_st1.val);
    }

  if (priv->hal.bridge != NULL)
    {
      printf("brg clk=%08" PRIx32 " en=%08" PRIx32
             " flow=%08" PRIx32 " dpi=%08" PRIx32 "\n",
             priv->hal.bridge->clk_en.val, priv->hal.bridge->en.val,
             priv->hal.bridge->dma_flow_ctrl.val,
             priv->hal.bridge->dpi_misc_config.val);
      printf("brg depth=%08" PRIx32 " raw=%08" PRIx32
             " st=%08" PRIx32 " bits=%08" PRIx32 "\n",
             priv->hal.bridge->fifo_flow_status.val,
             priv->hal.bridge->int_raw.val, priv->hal.bridge->int_st.val,
             priv->hal.bridge->blk_raw_num_cfg.val);
    }

  if (dma != NULL)
    {
      printf("dma chen=%08" PRIx32 " sar=%08" PRIx32
             " dar=%08" PRIx32 " ts=%08" PRIx32 "\n",
             dma->chen0.val, dma->ch[DSI_DMA_CHANNEL].sar0.sar0,
             dma->ch[DSI_DMA_CHANNEL].dar0.dar0,
             (uint32_t)dma->ch[DSI_DMA_CHANNEL].block_ts0.block_ts);
      printf("dma ctl0=%08" PRIx32 " ctl1=%08" PRIx32
             " cfg0=%08" PRIx32 " cfg1=%08" PRIx32
             " istat=%08" PRIx32 "\n",
             dma->ch[DSI_DMA_CHANNEL].ctl0.val,
             dma->ch[DSI_DMA_CHANNEL].ctl1.val,
             dma->ch[DSI_DMA_CHANNEL].cfg0.val,
             dma->ch[DSI_DMA_CHANNEL].cfg1.val,
             dw_gdma_ll_channel_get_intr_status(dma, DSI_DMA_CHANNEL));
    }

  printf("==========================\n");
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
      priv->fb_mem  = (FAR uint8_t *)kumm_memalign(64, priv->fb_size);
      if (priv->fb_mem != NULL)
        {
          memset(priv->fb_mem, 0, priv->fb_size);
          esp_cache_msync((void *)priv->fb_mem, priv->fb_size,
                          ESP_CACHE_MSYNC_FLAG_DIR_C2M);
          if (priv->initialized)
            {
              esp32p4_dsi_configure_dma(priv);
            }
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
