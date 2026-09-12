/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_mipi_csi.c
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

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/crc32.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>

#include "esp32p4_mipi_csi.h"
#include "esp_cache.h"
#include "esp_irq_p4.h"
#include "esp_private/periph_ctrl.h"

#include "hal/mipi_csi_hal.h"
#include "hal/mipi_csi_ll.h"
#include "hal/mipi_csi_periph.h"

#include "hal/dw_gdma_hal.h"
#include "hal/dw_gdma_ll.h"
#include "hal/dw_gdma_types.h"
#include "hal/isp_ll.h"
#include "hal/ldo_ll.h"
#include "hal/clk_gate_ll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32P4_CSI_DEFAULT_AFULL_THRD 960
#define ESP32P4_CSI_CACHE_LINE         64
#define ESP32P4_CSI_CACHE_CHUNK        (32 * 1024)
#define ESP32P4_CSI_DMA_IDLE_SPINS     1000

/* MIPI D-PHY power supply: LDO channel 3 (unit index 2), 2500 mV.
 * The LDO must be enabled before mipi_csi_hal_init or the D-PHY will
 * never enter HS mode (hint=0, bdepth=0 symptom).
 */

#define ESP32P4_CSI_MIPI_PHY_LDO_UNIT   2    /* LDO_ID2UNIT(chan_id=3) = 2 */
#define ESP32P4_CSI_MIPI_PHY_LDO_MV     2500 /* 2.5 V for MIPI D-PHY */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CSI frame buffer descriptor */

struct esp32p4_csi_buffer_s
{
  FAR uint8_t *raw_buf;      /* Base allocated pointer */
  FAR uint8_t *payload;      /* Frame payload pointer */
  size_t       raw_len;      /* Total allocated length with padding/guard */
  size_t       payload_len;  /* Frame payload length in bytes */
  bool         has_guard;    /* Front/rear guard words enabled */
};

/* ESP32-P4 CSI device structure */

struct esp32p4_csi_dev_s
{
  mutex_t                               lock;
  sem_t                                 dma_sem;
  bool                                  initialized;
  bool                                  enabled;
  mipi_csi_hal_context_t                hal;
  struct esp32p4_mipi_csi_config_s      config;

  /* DMA capture state */

  bool                                  dma_initialized;
  bool                                  dma_running;
  uint8_t                               dma_chan;
  int                                   cpuint;
  dw_gdma_hal_context_t                 dma_hal;
  struct esp32p4_csi_capture_config_s   dma_cfg;

  /* Link list descriptors in SRAM */

  FAR dw_gdma_link_list_item_t         *lli_items;
  size_t                                lli_count;

  /* Frame buffers */

  struct esp32p4_csi_buffer_s           buffers[ESP32P4_CSI_MAX_BUFFERS];
  uint8_t                               buf_count;
  uint8_t                               current_buf_idx;

  /* ISR signaling and stats */

  volatile bool                         frame_ready;
  volatile int                          last_dma_status;
  struct esp32p4_csi_dma_stats_s        stats;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32p4_csi_dev_s g_csi_dev =
{
  .lock            = NXMUTEX_INITIALIZER,
  .initialized     = false,
  .enabled         = false,
  .dma_initialized = false,
  .dma_running     = false,
  .dma_chan        = 0,
  .cpuint          = -1,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_get_time_us
 ****************************************************************************/

static uint64_t esp32p4_get_time_us(void)
{
  struct timespec ts;

  clock_systime_timespec(&ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/****************************************************************************
 * Name: esp32p4_csi_cache_invalidate
 *
 * Description:
 *   Invalidate a large DMA buffer in bounded chunks.  A whole-frame cache
 *   operation can monopolize the external-memory fabric while display DMA
 *   is active, so keep every cache critical section short.
 ****************************************************************************/

static void esp32p4_csi_cache_invalidate(FAR uint8_t *buf, size_t size)
{
  size_t offset;

  for (offset = 0; offset < size; offset += ESP32P4_CSI_CACHE_CHUNK)
    {
      size_t chunk = size - offset;

      if (chunk > ESP32P4_CSI_CACHE_CHUNK)
        {
          chunk = ESP32P4_CSI_CACHE_CHUNK;
        }

      esp_cache_msync(buf + offset, chunk, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    }
}

/****************************************************************************
 * Name: esp32p4_csi_dma_buffer_addr
 *
 * Description:
 *   Return the MMU/cache address consumed by DW-GDMA.  Unlike LLI metadata,
 *   a data buffer must not be converted to the CPU-only non-cache alias:
 *   DW-GDMA terminates that transaction after the first burst.
 ****************************************************************************/

static uint32_t esp32p4_csi_dma_buffer_addr(
  FAR struct esp32p4_csi_dev_s *priv, FAR uint8_t *buf)
{
  (void)priv;
  return (uint32_t)(uintptr_t)buf;
}

/****************************************************************************
 * Name: esp32p4_csi_dma_wait_idle
 ****************************************************************************/

static int esp32p4_csi_dma_wait_idle(FAR struct esp32p4_csi_dev_s *priv)
{
  uint32_t mask = 1u << priv->dma_chan;
  int spins = ESP32P4_CSI_DMA_IDLE_SPINS;

  while ((priv->dma_hal.dev->chen0.val & mask) != 0 && --spins > 0)
    {
      up_udelay(1);
    }

  return spins > 0 ? OK : -ETIMEDOUT;
}

/****************************************************************************
 * Name: esp32p4_csi_isp_enable_input
 *
 * Description:
 *   Open the ESP32-P4 rev3 CSI input gate in the shared ISP pipeline.  The
 *   CSI host can be configured correctly while the bridge still receives no
 *   pixels if mipi_data_en remains at its reset value.
 ****************************************************************************/

static void esp32p4_csi_isp_enable_input(
  FAR struct esp32p4_csi_dev_s *priv)
{
  isp_dev_t *isp = ISP_LL_GET_HW(0);
  uint32_t line_words;

  PERIPH_RCC_ATOMIC()
    {
      isp_ll_enable_module_clock(isp, true);
      isp_ll_select_clk_source(isp, ISP_CLK_SRC_XTAL);
      isp_ll_reset_module_clock(isp);
    }

  isp_ll_init(isp);
  isp_ll_enable(isp, false);
  isp_ll_clk_enable(isp, true);
  isp_ll_set_input_data_source(isp, ISP_INPUT_DATA_SOURCE_CSI);
  isp_ll_set_input_data_color_format(isp, ISP_COLOR_RAW10);
  isp_ll_enable_line_start_packet_exist(isp, false);
  isp_ll_enable_line_end_packet_exist(isp, false);

  /* The ISP stays bypassed for packed RAW10 capture, but its CSI input
   * front end must remain clocked on ESP32-P4 rev3.  In bypass mode HSIZE
   * is the number of 32-bit input words per line, not the pixel count.
   */

  line_words = ((uint32_t)priv->config.frame_width * 10 + 31) / 32;
  isp_ll_set_intput_data_h_pixel_num(isp, line_words);
  isp_ll_set_intput_data_v_row_num(isp, priv->config.frame_height);
}

/****************************************************************************
 * Name: esp32p4_csi_bridge_reset
 ****************************************************************************/

static void esp32p4_csi_bridge_reset(FAR struct esp32p4_csi_dev_s *priv)
{
  csi_brg_dev_t *brg = priv->hal.bridge_dev;
  uint32_t frame_words = priv->stats.frame_bytes / sizeof(uint64_t);
  uint32_t bridge_burst = 512;

  while (bridge_burst > 1 && (frame_words % bridge_burst) != 0)
    {
      bridge_burst >>= 1;
    }

  mipi_csi_brg_ll_enable(brg, false);
  brg->csi_en.csi_brg_rst = 1;
  brg->csi_en.csi_brg_rst = 0;

  /* Every bridge request must divide the packed RAW10 frame length.  A
   * fixed 512-word request leaves a non-emitted tail (128 words at 720p,
   * 32 at 1080p).  Pick the largest power-of-two divisor for this mode.
   */

  mipi_csi_brg_ll_set_burst_len(brg, bridge_burst);

  /* Keep bridge-side frame request generation enabled.  DW-GDMA remains
   * self-flow-controlled below so the programmed block length is still a
   * hard destination bound.
   */

  brg->dma_req_cfg.csi_dma_flow_controller = 1;

  /* SC2336 emits normal CSI-2 long packets without embedded HSYNC words. */

  mipi_csi_brg_ll_enable_has_hsync(brg, false);
  mipi_csi_brg_ll_set_data_type_min(brg, 0x12);
  mipi_csi_brg_ll_set_data_type_max(brg, 0x2f);
  mipi_csi_brg_ll_enable_color_conversion(brg, true);
  mipi_csi_brg_ll_set_color_mode_bypass(brg, true);
  mipi_csi_brg_ll_set_intput_data_h_pixel_num(brg,
                                               priv->config.frame_width);
  mipi_csi_brg_ll_set_intput_data_v_row_num(brg,
                                             priv->config.frame_height);
  mipi_csi_brg_ll_set_flow_ctl_buf_afull_thrd(
    brg, ESP32P4_CSI_DEFAULT_AFULL_THRD);
  mipi_csi_brg_ll_set_output_byte_endian(brg,
                                          priv->config.byte_swap_en);
  brg->host_ctrl.csi_enableclk = 1;
  brg->host_ctrl.csi_cfg_clk_en = 1;
  brg->int_clr.val = UINT32_MAX;
}

/****************************************************************************
 * Name: esp32p4_csi_dma_record_errors
 ****************************************************************************/

static int esp32p4_csi_dma_record_errors(
  FAR struct esp32p4_csi_dev_s *priv, uint32_t status)
{
  int ret = OK;

  if ((status & (DW_GDMA_LL_CHANNEL_EVENT_SRC_DEC_ERR |
                 DW_GDMA_LL_CHANNEL_EVENT_DST_DEC_ERR)) != 0)
    {
      priv->stats.dma_err_dec++;
      ret = -EIO;
    }

  if ((status & (DW_GDMA_LL_CHANNEL_EVENT_SRC_SLV_ERR |
                 DW_GDMA_LL_CHANNEL_EVENT_DST_SLV_ERR)) != 0)
    {
      priv->stats.dma_err_slv++;
      ret = -EIO;
    }

  if ((status & DW_GDMA_LL_CHANNEL_EVENT_SHADOWREG_OR_LLI_INVALID_ERR) != 0)
    {
      priv->stats.dma_err_lli++;
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: esp32p4_csi_arm_buffer
 ****************************************************************************/

static int esp32p4_csi_arm_buffer(
  FAR struct esp32p4_csi_dev_s *priv,
  FAR struct esp32p4_csi_buffer_s *buf_desc)
{
  dw_gdma_dev_t *dev = priv->dma_hal.dev;
  uint32_t beats = buf_desc->payload_len / sizeof(uint64_t);
  uint32_t dma_dst = esp32p4_csi_dma_buffer_addr(priv,
                                                  buf_desc->payload);
  int ret;

  /* Close and flush the bridge before touching a live DMA channel. */

  if (priv->initialized)
    {
      esp32p4_csi_bridge_reset(priv);
    }

  dw_gdma_ll_channel_enable(dev, priv->dma_chan, false);
  ret = esp32p4_csi_dma_wait_idle(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Drop destination cache lines before granting ownership to DMA. */

  esp32p4_csi_cache_invalidate(buf_desc->payload,
                               buf_desc->payload_len);

  dw_gdma_ll_channel_clear_intr(dev, priv->dma_chan, UINT32_MAX);
  dw_gdma_ll_channel_set_src_addr(dev, priv->dma_chan,
                                  MIPI_CSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_dst_addr(dev, priv->dma_chan,
                                  dma_dst);
  dw_gdma_ll_channel_set_trans_block_size(dev, priv->dma_chan, beats);
  dw_gdma_ll_channel_set_src_master_port(dev, priv->dma_chan,
                                         MIPI_CSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_dst_master_port(dev, priv->dma_chan,
                                         dma_dst);
  dw_gdma_ll_channel_set_src_trans_width(dev, priv->dma_chan,
                                         DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_dst_trans_width(dev, priv->dma_chan,
                                         DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_src_burst_items(dev, priv->dma_chan,
                                         DW_GDMA_BURST_ITEMS_512);
  dw_gdma_ll_channel_set_dst_burst_items(dev, priv->dma_chan,
                                         DW_GDMA_BURST_ITEMS_512);
  dw_gdma_ll_channel_set_src_burst_mode(dev, priv->dma_chan,
                                        DW_GDMA_BURST_MODE_FIXED);
  dw_gdma_ll_channel_set_dst_burst_mode(dev, priv->dma_chan,
                                        DW_GDMA_BURST_MODE_INCREMENT);
  dw_gdma_ll_channel_set_src_burst_len(dev, priv->dma_chan, 16);
  dw_gdma_ll_channel_set_dst_burst_len(dev, priv->dma_chan, 16);
  dw_gdma_ll_channel_set_link_list_head_addr(dev, priv->dma_chan, 0);

  priv->frame_ready = false;
  priv->last_dma_status = OK;
  dw_gdma_ll_channel_enable(dev, priv->dma_chan, true);

  if (priv->initialized)
    {
      mipi_csi_brg_ll_enable(priv->hal.bridge_dev, true);
      priv->enabled = true;
    }

  priv->dma_running = true;
  priv->stats.is_capturing = true;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_mipi_csi_init
 ****************************************************************************/

int esp32p4_mipi_csi_init(FAR const struct
                          esp32p4_mipi_csi_config_s *config)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  mipi_csi_hal_config_t hal_cfg;
  uint8_t dref = 0;
  uint8_t mul = 0;
  bool use_rail_voltage = false;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (config != NULL)
    {
      memcpy(&priv->config, config, sizeof(priv->config));
    }
  else
    {
      priv->config.lanes_num          = ESP32P4_CSI_DEFAULT_LANES;
      priv->config.frame_width        = ESP32P4_CSI_DEFAULT_WIDTH;
      priv->config.frame_height       = ESP32P4_CSI_DEFAULT_HEIGHT;
      priv->config.in_bpp             = ESP32P4_CSI_DEFAULT_BPP;
      priv->config.out_bpp            = ESP32P4_CSI_DEFAULT_BPP;
      priv->config.data_type          = ESP32P4_CSI_DT_RAW10;
      priv->config.byte_swap_en       = false;
      priv->config.lane_bit_rate_mbps = ESP32P4_CSI_DEFAULT_MBPS;
    }

  /* 0. Enable MIPI D-PHY LDO power supply (Channel 3 -> Unit 2, 2.5 V) */

  ldo_ll_voltage_to_dref_mul(ESP32P4_CSI_MIPI_PHY_LDO_UNIT,
                             ESP32P4_CSI_MIPI_PHY_LDO_MV,
                             &dref, &mul, &use_rail_voltage);
  ldo_ll_adjust_voltage(ESP32P4_CSI_MIPI_PHY_LDO_UNIT, dref, mul,
                        use_rail_voltage);
  ldo_ll_set_owner(ESP32P4_CSI_MIPI_PHY_LDO_UNIT, LDO_LL_UNIT_OWNER_SW);
  ldo_ll_enable_ripple_suppression(ESP32P4_CSI_MIPI_PHY_LDO_UNIT, true);
  ldo_ll_enable(ESP32P4_CSI_MIPI_PHY_LDO_UNIT, true);
  up_udelay(5000);

  /* 1. Enable module clocks and release resets.
   * PHY config clock must be toggled (disable->enable) to generate a
   * proper reset pulse, matching the official ESP-IDF sequence in
   * esp_cam_ctlr_csi.c:s_csi_claim_controller/esp_cam_new_csi_ctlr.
   */

  PERIPH_RCC_ATOMIC()
    {
      clk_gate_ll_ref_20m_clk_en(true);
      mipi_csi_ll_set_phy_clock_source(0, MIPI_CSI_PHY_CLK_SRC_PLL_F20M);
      mipi_csi_ll_enable_phy_config_clock(0, false);
      mipi_csi_ll_enable_phy_config_clock(0, true);
      mipi_csi_ll_enable_host_bus_clock(0, false);
      mipi_csi_ll_enable_host_bus_clock(0, true);
      mipi_csi_ll_reset_host_clock(0);
      mipi_csi_ll_enable_brg_module_clock(0, true);
      mipi_csi_ll_reset_brg_module_clock(0);
      mipi_csi_brg_ll_enable_clock(MIPI_CSI_BRG_LL_GET_HW(0), true);
    }

  /* 2. Configure HAL layer (PHY PLL, Host controller, and Bridge)
   * Note: mipi_csi_hal_init sets h_pixel_num from frame_height and
   * v_row_num from frame_width.
   */

  hal_cfg.lanes_num          = priv->config.lanes_num;
  hal_cfg.frame_height       = priv->config.frame_width;  /* h_pixel_num */
  hal_cfg.frame_width        = priv->config.frame_height; /* v_row_num */
  hal_cfg.in_bpp             = priv->config.in_bpp;
  hal_cfg.out_bpp            = priv->config.out_bpp;
  hal_cfg.byte_swap_en       = priv->config.byte_swap_en;
  hal_cfg.lane_bit_rate_mbps = priv->config.lane_bit_rate_mbps;

  mipi_csi_hal_init(&priv->hal, &hal_cfg);

  /* ESP32-P4 rev3 gates the CSI input through the shared ISP block. */

  esp32p4_csi_isp_enable_input(priv);

  /* Force standard 2-bit VC0 mode
   * (disable 5-bit VCX extension: 1 is disabled)
   */

  priv->hal.host_dev->vc_extension.vcx = 1;

  /* Force CSI Bridge internal clock gating disabled (clock always on) */

  priv->hal.bridge_dev->clk_en.clk_en = 1;

  /* Configure Bridge DMA burst length (512 items per burst). */

  mipi_csi_brg_ll_set_burst_len(priv->hal.bridge_dev, 512);
  priv->hal.bridge_dev->dma_req_cfg.csi_dma_flow_controller = 1;

  /* Enable color management in bypass mode so RAW data passes directly */

  mipi_csi_brg_ll_enable_color_conversion(priv->hal.bridge_dev, true);
  mipi_csi_brg_ll_set_color_mode_bypass(priv->hal.bridge_dev, true);

  /* 3. Configure CSI Bridge frame formatting, data type filtering
   *    and threshold.  The SC2336 stream has CSI-2 line packets but no
   *    embedded HSYNC data in the payload.
   */

  mipi_csi_brg_ll_enable_has_hsync(priv->hal.bridge_dev, false);
  mipi_csi_brg_ll_set_data_type_min(priv->hal.bridge_dev, 0x12);
  mipi_csi_brg_ll_set_data_type_max(priv->hal.bridge_dev, 0x2f);
  priv->hal.bridge_dev->host_ctrl.csi_enableclk  = 1;
  priv->hal.bridge_dev->host_ctrl.csi_cfg_clk_en = 1;

  mipi_csi_brg_ll_set_flow_ctl_buf_afull_thrd(priv->hal.bridge_dev,
                                             ESP32P4_CSI_DEFAULT_AFULL_THRD);
  mipi_csi_brg_ll_set_output_byte_endian(priv->hal.bridge_dev,
                                         priv->config.byte_swap_en);

  /* 4. Unmask CSI Host and Bridge status registers for full diagnostics */

  priv->hal.host_dev->int_msk_phy_fatal.val         = 0;
  priv->hal.host_dev->int_msk_pkt_fatal.val         = 0;
  priv->hal.host_dev->int_msk_phy.val               = 0;
  priv->hal.host_dev->int_msk_bndry_frame_fatal.val = 0;
  priv->hal.host_dev->int_msk_seq_frame_fatal.val   = 0;
  priv->hal.host_dev->int_msk_crc_frame_fatal.val   = 0;
  priv->hal.host_dev->int_msk_pld_crc_fatal.val     = 0;
  priv->hal.host_dev->int_msk_data_id.val           = 0;
  priv->hal.host_dev->int_msk_ecc_corrected.val     = 0;

  priv->hal.bridge_dev->int_ena.val                 = 0x3f;

  /* 5. Disable CSI bridge initially until DMA is armed */

  mipi_csi_brg_ll_enable(priv->hal.bridge_dev, false);

  priv->initialized = true;
  priv->enabled     = false;

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_csi_enable
 ****************************************************************************/

int esp32p4_mipi_csi_enable(bool enable)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->initialized)
    {
      nxmutex_unlock(&priv->lock);
      return -ENODEV;
    }

  mipi_csi_brg_ll_enable(priv->hal.bridge_dev, enable);
  priv->enabled = enable;

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_csi_get_status
 ****************************************************************************/

int esp32p4_mipi_csi_get_status(FAR struct
                                esp32p4_mipi_csi_status_s *status)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  csi_host_dev_t *host_dev;
  csi_brg_dev_t *brg_dev;
  int ret;

  if (status == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  memset(status, 0, sizeof(*status));
  status->initialized = priv->initialized;
  status->enabled     = priv->enabled;

  if (priv->initialized)
    {
      host_dev = priv->hal.host_dev;
      brg_dev  = priv->hal.bridge_dev;

      status->clk_stopstate  = host_dev->phy_stopstate.phy_stopstateclk;
      status->clk_activehs   = host_dev->phy_rx.phy_rxclkactivehs;
      status->clk_ulpsnot    = host_dev->phy_rx.phy_rxulpsclknot;

      status->data_stopstate =
        (uint8_t)(host_dev->phy_stopstate.phy_stopstatedata_0 & 0x01) |
        (uint8_t)((host_dev->phy_stopstate.phy_stopstatedata_1 & 0x01) << 1);

      status->data_ulpsesc =
        (uint8_t)(host_dev->phy_rx.phy_rxulpsesc_0 & 0x01) |
        (uint8_t)((host_dev->phy_rx.phy_rxulpsesc_1 & 0x01) << 1);

      status->int_st_main      = host_dev->int_st_main.val;
      status->int_st_phy_fatal = host_dev->int_st_phy_fatal.val;
      status->int_st_pkt_fatal = host_dev->int_st_pkt_fatal.val;
      status->bridge_int_st    = brg_dev->int_st.val;
      status->buffer_depth     = brg_dev->buf_flow_ctl.csi_buf_depth;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_csi_dump
 ****************************************************************************/

void esp32p4_mipi_csi_dump(void)
{
  struct esp32p4_mipi_csi_status_s st;
  int ret;

  ret = esp32p4_mipi_csi_get_status(&st);
  if (ret < 0)
    {
      printf("MIPI CSI: get status failed (%d)\n", ret);
      return;
    }

  printf("========================================\n");
  printf(" ESP32-P4 MIPI CSI Controller Status\n");
  printf("========================================\n");
  printf("  Initialized     : %s\n", st.initialized ? "YES" : "NO");
  printf("  Bridge Enabled  : %s\n", st.enabled ? "YES" : "NO");
  printf("  Clock Lane      : %s, %s (ulpsnot=%d)\n",
         st.clk_activehs ? "HS-Active" : "LP/Inactive",
         st.clk_stopstate ? "StopState (LP-11)" : "Active/Transition",
         st.clk_ulpsnot);
  printf("  Data Lane 0     : %s (ulps=%d)\n",
         (st.data_stopstate & 0x01) ? "StopState (LP-11)" : "Active/HS",
         st.data_ulpsesc & 0x01);
  printf("  Data Lane 1     : %s (ulps=%d)\n",
         (st.data_stopstate & 0x02) ? "StopState (LP-11)" : "Active/HS",
         (st.data_ulpsesc >> 1) & 0x01);
  printf("  Host Main INT   : 0x%08" PRIx32 "\n", st.int_st_main);
  printf("  PHY Fatal INT   : 0x%08" PRIx32 "\n", st.int_st_phy_fatal);
  printf("  Packet Fatal INT: 0x%08" PRIx32 "\n", st.int_st_pkt_fatal);
  printf("  Bridge INT      : 0x%08" PRIx32 "\n", st.bridge_int_st);
  printf("  Bridge FIFO Lvl : %u bytes\n", st.buffer_depth);
  printf("========================================\n");
}

/****************************************************************************
 * Name: esp32p4_mipi_csi_deinit
 ****************************************************************************/

int esp32p4_mipi_csi_deinit(void)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->initialized)
    {
      mipi_csi_brg_ll_enable(priv->hal.bridge_dev, false);
      mipi_csi_brg_ll_enable_clock(priv->hal.bridge_dev, false);

      PERIPH_RCC_ATOMIC()
        {
          mipi_csi_ll_enable_brg_module_clock(0, false);
          mipi_csi_ll_enable_host_bus_clock(0, false);
          mipi_csi_ll_enable_phy_config_clock(0, false);
        }

      ldo_ll_enable(ESP32P4_CSI_MIPI_PHY_LDO_UNIT, false);

      priv->initialized = false;
      priv->enabled     = false;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_csi_dma_init
 ****************************************************************************/

int esp32p4_csi_dma_init(FAR const struct
                         esp32p4_csi_capture_config_s *config)
{
  FAR struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  dw_gdma_dev_t *dev;
  size_t frame_bytes;
  size_t words_count;
  size_t guard_size;
  size_t total_buf_size;
  uint8_t chan;
  uint8_t count;
  int ret;
  int i;

  if (config == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->dma_initialized)
    {
      nxmutex_unlock(&priv->lock);
      return -EEXIST;
    }

  memcpy(&priv->dma_cfg, config, sizeof(priv->dma_cfg));
  chan  = config->dma_chan;
  count = (config->buf_count > 0 &&
           config->buf_count <= ESP32P4_CSI_MAX_BUFFERS) ?
           config->buf_count : 1;

  priv->dma_chan        = chan;
  priv->buf_count       = count;
  priv->current_buf_idx = 0;
  priv->frame_ready     = false;

  /* Calculate required sizes:
   * frame_bytes = (width * height * bpp) / 8
   */

  frame_bytes = ((size_t)config->frame_width * config->frame_height *
                 config->bpp) / 8;
  if (frame_bytes == 0 || (frame_bytes % 8) != 0)
    {
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  words_count = frame_bytes / 8; /* 64-bit words for DW-GDMA */
  guard_size  = config->enable_guard ? ESP32P4_CSI_GUARD_SIZE : 0;
  total_buf_size = (frame_bytes + guard_size * 2 + 63) & ~63;

  priv->stats.frame_bytes    = frame_bytes;
  priv->stats.active_channel = chan;

  /* 1. Enable the shared DW-GDMA controller.  Do not reset the global
   * controller here: display or audio can already own another channel.
   */

  PERIPH_RCC_ATOMIC()
    {
      dw_gdma_ll_enable_bus_clock(0, true);
    }

  priv->dma_hal.dev = DW_GDMA_LL_GET_HW(0);
  dev = priv->dma_hal.dev;

  dw_gdma_ll_enable_controller(dev, true);

  /* 2. Allocate 64-byte aligned Link List Item (LLI) descriptors in SRAM */

  priv->lli_items = (FAR dw_gdma_link_list_item_t *)
    kmm_memalign(DW_GDMA_LL_LINK_LIST_ALIGNMENT,
                 sizeof(dw_gdma_link_list_item_t) * count);
  if (priv->lli_items == NULL)
    {
      ret = -ENOMEM;
      goto err_clean;
    }

  memset(priv->lli_items, 0,
         sizeof(dw_gdma_link_list_item_t) * count);
  priv->lli_count = count;

  /* 3. Allocate and initialize frame buffers (SRAM or PSRAM) */

  for (i = 0; i < count; i++)
    {
      FAR uint8_t *buf = NULL;

      if (config->mem_type == ESP32P4_CSI_BUF_PSRAM)
        {
          buf = (FAR uint8_t *)kumm_memalign(64, total_buf_size);
        }
      else
        {
          buf = (FAR uint8_t *)kmm_memalign(64, total_buf_size);
        }

      if (buf == NULL)
        {
          ret = -ENOMEM;
          goto err_clean;
        }

      memset(buf, 0, total_buf_size);

      priv->buffers[i].raw_buf     = buf;
      priv->buffers[i].raw_len     = total_buf_size;
      priv->buffers[i].payload_len = frame_bytes;
      priv->buffers[i].has_guard   = config->enable_guard;

      if (config->enable_guard)
        {
          priv->buffers[i].payload = buf + guard_size;
          *(FAR uint32_t *)buf = ESP32P4_CSI_GUARD_FRONT;
          *(FAR uint32_t *)(buf + guard_size + frame_bytes) =
            ESP32P4_CSI_GUARD_REAR;
        }
      else
        {
          priv->buffers[i].payload = buf;
        }

      /* Clean/invalidate cache for buffer */

      esp_cache_msync((FAR void *)buf, total_buf_size,
                      ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                      ESP_CACHE_MSYNC_FLAG_INVALIDATE);

      /* Configure LLI item */

      dw_gdma_ll_lli_set_src_addr(&priv->lli_items[i],
                                  MIPI_CSI_BRG_MEM_BASE);
      dw_gdma_ll_lli_set_dst_addr(&priv->lli_items[i],
        esp32p4_csi_dma_buffer_addr(priv, priv->buffers[i].payload));
      dw_gdma_ll_lli_set_trans_block_size(&priv->lli_items[i],
                                          words_count);
      dw_gdma_ll_lli_set_src_master_port(&priv->lli_items[i],
                                         MIPI_CSI_BRG_MEM_BASE);
      dw_gdma_ll_lli_set_dst_master_port(&priv->lli_items[i],
        esp32p4_csi_dma_buffer_addr(priv, priv->buffers[i].payload));
      dw_gdma_ll_lli_set_src_trans_width(&priv->lli_items[i],
                                         DW_GDMA_TRANS_WIDTH_64);
      dw_gdma_ll_lli_set_dst_trans_width(&priv->lli_items[i],
                                         DW_GDMA_TRANS_WIDTH_64);
      dw_gdma_ll_lli_set_src_burst_items(&priv->lli_items[i],
                                         DW_GDMA_BURST_ITEMS_512);
      dw_gdma_ll_lli_set_dst_burst_items(&priv->lli_items[i],
                                         DW_GDMA_BURST_ITEMS_512);
      dw_gdma_ll_lli_set_src_burst_mode(&priv->lli_items[i],
                                        DW_GDMA_BURST_MODE_FIXED);
      dw_gdma_ll_lli_set_dst_burst_mode(&priv->lli_items[i],
                                        DW_GDMA_BURST_MODE_INCREMENT);
      dw_gdma_ll_lli_set_src_burst_len(&priv->lli_items[i], 16);
      dw_gdma_ll_lli_set_dst_burst_len(&priv->lli_items[i], 16);

      dw_gdma_ll_lli_set_block_markers(&priv->lli_items[i],
                                       true, (i == count - 1), true);

      dw_gdma_ll_lli_set_link_list_master_port(
        &priv->lli_items[i],
        DW_GDMA_LL_MASTER_PORT_MEMORY);

      if (i + 1 < count)
        {
          dw_gdma_ll_lli_set_next_item_addr(&priv->lli_items[i],
            (uint32_t)&priv->lli_items[i + 1]);
        }
      else
        {
          dw_gdma_ll_lli_set_next_item_addr(&priv->lli_items[i], 0);
        }
    }

  /* Sync LLI descriptors to memory */

  esp_cache_msync((FAR void *)priv->lli_items,
                  sizeof(dw_gdma_link_list_item_t) * count,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                  ESP_CACHE_MSYNC_FLAG_INVALIDATE);

  /* 4. Configure DW-GDMA Channel registers */

  dw_gdma_ll_channel_set_priority(dev, chan, 1);
  dw_gdma_ll_channel_set_trans_flow(dev, chan,
                                    DW_GDMA_ROLE_PERIPH_CSI,
                                    DW_GDMA_ROLE_MEM,
                                    DW_GDMA_FLOW_CTRL_SELF);
  dw_gdma_ll_channel_set_src_handshake_interface(dev, chan,
                                                 DW_GDMA_HANDSHAKE_HW);
  dw_gdma_ll_channel_set_dst_handshake_interface(dev, chan,
                                                 DW_GDMA_HANDSHAKE_HW);
  dw_gdma_ll_channel_set_src_handshake_periph(dev, chan,
                                              DW_GDMA_ROLE_PERIPH_CSI);
  dw_gdma_ll_channel_set_src_multi_block_type(
    dev, chan, DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
  dw_gdma_ll_channel_set_dst_multi_block_type(
    dev, chan, DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS);
  dw_gdma_ll_channel_set_src_outstanding_limit(dev, chan, 5);
  dw_gdma_ll_channel_set_dst_outstanding_limit(dev, chan, 5);
  dw_gdma_ll_channel_set_src_periph_status_addr(dev, chan,
                                                MIPI_CSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_dst_periph_status_addr(dev, chan, 0);
  dw_gdma_ll_channel_set_link_list_master_port(
    dev, chan,
    DW_GDMA_LL_MASTER_PORT_MEMORY);

  /* Direct channel register setup for first block */

  dw_gdma_ll_channel_set_src_addr(dev, chan, MIPI_CSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_dst_addr(dev, chan,
    esp32p4_csi_dma_buffer_addr(priv, priv->buffers[0].payload));
  dw_gdma_ll_channel_set_trans_block_size(dev, chan, words_count);
  dw_gdma_ll_channel_set_src_master_port(dev, chan, MIPI_CSI_BRG_MEM_BASE);
  dw_gdma_ll_channel_set_dst_master_port(dev, chan,
    esp32p4_csi_dma_buffer_addr(priv, priv->buffers[0].payload));
  dw_gdma_ll_channel_set_src_trans_width(dev, chan, DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_dst_trans_width(dev, chan, DW_GDMA_TRANS_WIDTH_64);
  dw_gdma_ll_channel_set_src_burst_items(dev, chan, DW_GDMA_BURST_ITEMS_512);
  dw_gdma_ll_channel_set_dst_burst_items(dev, chan, DW_GDMA_BURST_ITEMS_512);
  dw_gdma_ll_channel_set_src_burst_mode(dev, chan, DW_GDMA_BURST_MODE_FIXED);
  dw_gdma_ll_channel_set_dst_burst_mode(dev, chan,
                                        DW_GDMA_BURST_MODE_INCREMENT);
  dw_gdma_ll_channel_set_src_burst_len(dev, chan, 16);
  dw_gdma_ll_channel_set_dst_burst_len(dev, chan, 16);
  /* Poll channel completion.  The vendored shared level interrupt can
   * remain asserted after the first frame and freeze the system.
   */

  dw_gdma_ll_channel_enable_intr_propagation(dev, chan, UINT32_MAX, false);
  dw_gdma_ll_channel_clear_intr(dev, chan, UINT32_MAX);
  dw_gdma_ll_channel_enable_intr_generation(dev, chan, UINT32_MAX, true);

  priv->dma_initialized = true;
  priv->dma_running     = false;

  nxmutex_unlock(&priv->lock);
  return OK;

err_clean:
  if (priv->lli_items != NULL)
    {
      kmm_free(priv->lli_items);
      priv->lli_items = NULL;
    }

  for (i = 0; i < count; i++)
    {
      if (priv->buffers[i].raw_buf != NULL)
        {
          if (config->mem_type == ESP32P4_CSI_BUF_PSRAM)
            {
              kumm_free(priv->buffers[i].raw_buf);
            }
          else
            {
              kmm_free(priv->buffers[i].raw_buf);
            }

          priv->buffers[i].raw_buf = NULL;
        }
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_csi_dma_start
 ****************************************************************************/

int esp32p4_csi_dma_start(void)
{
  FAR struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  FAR struct esp32p4_csi_buffer_s *buf_desc;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->dma_initialized)
    {
      nxmutex_unlock(&priv->lock);
      return -ENODEV;
    }

  if (priv->dma_running)
    {
      nxmutex_unlock(&priv->lock);
      return -EBUSY;
    }

  buf_desc = &priv->buffers[priv->current_buf_idx];
  ret = esp32p4_csi_arm_buffer(priv, buf_desc);

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_csi_dma_stop
 ****************************************************************************/

int esp32p4_csi_dma_stop(void)
{
  FAR struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  dw_gdma_dev_t *dev;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->dma_initialized)
    {
      nxmutex_unlock(&priv->lock);
      return -ENODEV;
    }

  dev = priv->dma_hal.dev;

  /* Disable channel */

  dw_gdma_ll_channel_enable(dev, priv->dma_chan, false);
  ret = esp32p4_csi_dma_wait_idle(priv);

  if (priv->initialized)
    {
      mipi_csi_brg_ll_enable(priv->hal.bridge_dev, false);
      priv->enabled = false;
    }

  priv->dma_running        = false;
  priv->stats.is_capturing = false;

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_csi_capture_frame
 ****************************************************************************/

int esp32p4_csi_capture_frame(FAR struct esp32p4_csi_frame_s *frame,
                              uint32_t timeout_ms)
{
  FAR struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  FAR struct esp32p4_csi_buffer_s *buf_desc;
  dw_gdma_dev_t *dev;
  struct timespec abstime;
  struct timespec now;
  uint64_t nsec;
  bool guard_ok = true;
  size_t bytes_received = 0;
  uint32_t crc;
  int ret = OK;

  if (frame == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!priv->dma_initialized)
    {
      nxmutex_unlock(&priv->lock);
      return -ENODEV;
    }

  dev      = priv->dma_hal.dev;
  buf_desc = &priv->buffers[priv->current_buf_idx];

  if (!priv->dma_running)
    {
      ret = esp32p4_csi_arm_buffer(priv, buf_desc);
      if (ret < 0)
        {
          nxmutex_unlock(&priv->lock);
          return ret;
        }
    }

  /* Calculate timeout deadline */

  clock_systime_timespec(&now);
  nsec = (uint64_t)now.tv_nsec + (uint64_t)(timeout_ms % 1000) * 1000000ULL;
  abstime.tv_sec  = now.tv_sec + (timeout_ms / 1000) +
                    (nsec / 1000000000ULL);
  abstime.tv_nsec = nsec % 1000000000ULL;

  /* In self-flow-control mode hardware clears CH_EN after exactly one
   * programmed frame. Interrupt status is sampled for errors.
   */

  while (!priv->frame_ready)
    {
      uint32_t istat =
        dw_gdma_ll_channel_get_intr_status(dev, priv->dma_chan);
      uint32_t mask = 1u << priv->dma_chan;
      uint32_t dar = dev->ch[priv->dma_chan].dar0.dar0;
      uint32_t lo = esp32p4_csi_dma_buffer_addr(priv, buf_desc->payload);

      ret = esp32p4_csi_dma_record_errors(priv, istat);
      if (ret < 0)
        {
          break;
        }

      if (dar < lo || dar > lo + buf_desc->payload_len)
        {
          printf("CSI DMA runaway: DAR=0x%08" PRIx32
                 " buffer=0x%08" PRIx32 "+%zu\n",
                 dar, lo, buf_desc->payload_len);
          priv->stats.dma_guard_errors++;
          ret = -EOVERFLOW;
          break;
        }

      if ((dev->chen0.val & mask) == 0)
        {
          bytes_received = dar - lo;
          if (bytes_received != buf_desc->payload_len)
            {
              printf("CSI DMA short frame: DAR=0x%08" PRIx32
                     " bytes=%zu/%zu\n", dar, bytes_received,
                     buf_desc->payload_len);
              ret = -EMSGSIZE;
              break;
            }

          dw_gdma_ll_channel_clear_intr(dev, priv->dma_chan, istat);
          priv->stats.frames_captured++;
          priv->stats.last_frame_time_us = esp32p4_get_time_us();
          priv->frame_ready              = true;
          priv->last_dma_status          = OK;
          break;
        }

      clock_systime_timespec(&now);
      if (now.tv_sec > abstime.tv_sec ||
          (now.tv_sec == abstime.tv_sec && now.tv_nsec >= abstime.tv_nsec))
        {
          ret = -ETIMEDOUT;
          break;
        }

      nxsig_usleep(2000); /* 2 ms poll sleep */
    }

  if (ret < 0)
    {
      uint32_t istat =
        dw_gdma_ll_channel_get_intr_status(dev, priv->dma_chan);
      uint32_t tamt =
        dw_gdma_ll_channel_get_trans_amount(dev, priv->dma_chan);
      uint32_t fifo =
        dw_gdma_ll_channel_get_fifo_remain(dev, priv->dma_chan);
      uint32_t chen = dev->chen0.val;
      uint32_t bdepth = (priv->initialized) ?
        priv->hal.bridge_dev->buf_flow_ctl.csi_buf_depth : 0;
      uint32_t bint_raw = (priv->initialized) ?
        priv->hal.bridge_dev->int_raw.val : 0;
      uint32_t bint_st = (priv->initialized) ?
        priv->hal.bridge_dev->int_st.val : 0;
      uint32_t h_main = (priv->initialized) ?
        priv->hal.host_dev->int_st_main.val : 0;
      uint32_t h_phy_f = (priv->initialized) ?
        priv->hal.host_dev->int_st_phy_fatal.val : 0;
      uint32_t h_pkt_f = (priv->initialized) ?
        priv->hal.host_dev->int_st_pkt_fatal.val : 0;
      uint32_t h_phy = (priv->initialized) ?
        priv->hal.host_dev->int_st_phy.val : 0;
      uint32_t h_did = (priv->initialized) ?
        priv->hal.host_dev->int_st_data_id.val : 0;
      uint32_t h_crc = (priv->initialized) ?
        priv->hal.host_dev->int_st_crc_frame_fatal.val : 0;
      uint32_t clk_hs = (priv->initialized) ?
        priv->hal.host_dev->phy_rx.phy_rxclkactivehs : 0;
      uint32_t clk_stop = (priv->initialized) ?
        priv->hal.host_dev->phy_stopstate.phy_stopstateclk : 0;
      uint32_t d_stop = (priv->initialized) ?
        ((priv->hal.host_dev->phy_stopstate.phy_stopstatedata_0 & 1) |
         ((priv->hal.host_dev->phy_stopstate.phy_stopstatedata_1 & 1)
          << 1)) : 0;
      uint32_t dar = dev->ch[priv->dma_chan].dar0.dar0;
      uint32_t sar = dev->ch[priv->dma_chan].sar0.sar0;
      uint32_t block_ts = dev->ch[priv->dma_chan].block_ts0.block_ts;

      /* Stop DMA channel */

      dw_gdma_ll_channel_enable(dev, priv->dma_chan, false);
      esp32p4_csi_dma_wait_idle(priv);
      if (priv->initialized)
        {
          mipi_csi_brg_ll_enable(priv->hal.bridge_dev, false);
          priv->enabled = false;
        }

      priv->dma_running        = false;
      priv->stats.is_capturing = false;

      printf("DEBUG: DMA Timeout! istat=0x%08" PRIx32 ", tamt=%" PRIu32
             ", fifo=%" PRIu32 ", chen=0x%08" PRIx32 ", bdepth=%" PRIu32
             ", brg_raw=0x%08" PRIx32 ", brg_st=0x%08" PRIx32
             ", h_main=0x%08" PRIx32 ", h_phy=0x%08" PRIx32
             ", h_did=0x%08" PRIx32 ", h_crc=0x%08" PRIx32
             ", h_phy_f=0x%08" PRIx32 ", h_pkt_f=0x%08" PRIx32
             ", clk_hs=%" PRIu32 ", clk_stop=%" PRIu32
             ", d_stop=0x%02" PRIx32 ", dar=0x%08" PRIx32
             ", sar=0x%08" PRIx32 ", ts=%" PRIu32 "\n",
             istat, tamt, fifo, chen, bdepth, bint_raw, bint_st,
             h_main, h_phy, h_did, h_crc, h_phy_f, h_pkt_f,
             clk_hs, clk_stop, d_stop, dar, sar, block_ts);

      if (priv->initialized)
        {
          printf("  CSI Host: ver=0x%08" PRIx32 " lanes=0x%08" PRIx32
                 " rstn=0x%08" PRIx32 " rx=0x%08" PRIx32
                 " stop=0x%08" PRIx32 "\n"
                 "            tst0=0x%08" PRIx32 " tst1=0x%08" PRIx32
                 " vcx=0x%08" PRIx32 "\n",
                 (uint32_t)priv->hal.host_dev->version.val,
                 (uint32_t)priv->hal.host_dev->n_lanes.val,
                 (uint32_t)priv->hal.host_dev->csi2_resetn.val,
                 (uint32_t)priv->hal.host_dev->phy_rx.val,
                 (uint32_t)priv->hal.host_dev->phy_stopstate.val,
                 (uint32_t)priv->hal.host_dev->phy_test_ctrl0.val,
                 (uint32_t)priv->hal.host_dev->phy_test_ctrl1.val,
                 (uint32_t)priv->hal.host_dev->vc_extension.val);

          printf("  CSI Brg : clk=0x%08" PRIx32 " en=0x%08" PRIx32
                 " dma_cfg=0x%08" PRIx32 " buf=0x%08" PRIx32 "\n"
                 "            dt=0x%08" PRIx32 " frm=0x%08" PRIx32
                 " end=0x%08" PRIx32 " blk=0x%08" PRIx32 "\n"
                 "            h_ctrl=0x%08" PRIx32 " h_cm=0x%08" PRIx32
                 " h_sz=0x%08" PRIx32 "\n",
                 (uint32_t)priv->hal.bridge_dev->clk_en.val,
                 (uint32_t)priv->hal.bridge_dev->csi_en.val,
                 (uint32_t)priv->hal.bridge_dev->dma_req_cfg.val,
                 (uint32_t)priv->hal.bridge_dev->buf_flow_ctl.val,
                 (uint32_t)priv->hal.bridge_dev->data_type_cfg.val,
                 (uint32_t)priv->hal.bridge_dev->frame_cfg.val,
                 (uint32_t)priv->hal.bridge_dev->endian_mode.val,
                 (uint32_t)priv->hal.bridge_dev->dmablk_size.val,
                 (uint32_t)priv->hal.bridge_dev->host_ctrl.val,
                 (uint32_t)priv->hal.bridge_dev->host_cm_ctrl.val,
                 (uint32_t)priv->hal.bridge_dev->host_size_ctrl.val);

          printf("  DW-GDMA : ctl0=0x%08" PRIx32 " ctl1=0x%08" PRIx32
                 " cfg0=0x%08" PRIx32 " cfg1=0x%08" PRIx32 "\n",
                 (uint32_t)dev->ch[priv->dma_chan].ctl0.val,
                 (uint32_t)dev->ch[priv->dma_chan].ctl1.val,
                 (uint32_t)dev->ch[priv->dma_chan].cfg0.val,
                 (uint32_t)dev->ch[priv->dma_chan].cfg1.val);
        }

      fflush(stdout);

      priv->stats.dma_timeouts++;
      nxmutex_unlock(&priv->lock);
      return ret;
    }

  /* Stop DMA channel */

  dw_gdma_ll_channel_enable(dev, priv->dma_chan, false);
  esp32p4_csi_dma_wait_idle(priv);
  if (priv->initialized)
    {
      mipi_csi_brg_ll_enable(priv->hal.bridge_dev, false);
      priv->enabled = false;
    }

  priv->dma_running        = false;
  priv->stats.is_capturing = false;

  if (priv->last_dma_status != OK)
    {
      nxmutex_unlock(&priv->lock);
      return priv->last_dma_status;
    }

  /* Invalidate cache so CPU reads freshly captured data from PSRAM/SRAM */

  esp32p4_csi_cache_invalidate(buf_desc->payload,
                               buf_desc->payload_len);

  /* Verify front and rear guard patterns */

  if (buf_desc->has_guard)
    {
      uint32_t front = *(FAR uint32_t *)buf_desc->raw_buf;
      uint32_t rear  = *(FAR uint32_t *)(buf_desc->payload +
                                         buf_desc->payload_len);

      if (front != ESP32P4_CSI_GUARD_FRONT || rear != ESP32P4_CSI_GUARD_REAR)
        {
          priv->stats.dma_guard_errors++;
          guard_ok = false;
        }
    }

  /* Calculate CRC32 checksum over the payload */

  crc = crc32((FAR const uint8_t *)buf_desc->payload,
              buf_desc->payload_len);

  /* Populate output frame structure */

  frame->seq_no         = priv->stats.frames_captured;
  frame->timestamp_us   = priv->stats.last_frame_time_us;
  frame->buffer         = buf_desc->payload;
  frame->buflen         = buf_desc->payload_len;
  frame->bytes_received = bytes_received;
  frame->crc32          = crc;
  frame->guard_valid    = guard_ok;
  frame->status         = OK;

  /* Switch to next buffer for multi-buffer modes */

  priv->current_buf_idx = (priv->current_buf_idx + 1) % priv->buf_count;

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_csi_get_dma_stats
 ****************************************************************************/

int esp32p4_csi_get_dma_stats(FAR struct esp32p4_csi_dma_stats_s *stats)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  int ret;

  if (stats == NULL)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  memcpy(stats, &priv->stats, sizeof(*stats));

  nxmutex_unlock(&priv->lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_csi_reset_dma_stats
 ****************************************************************************/

void esp32p4_csi_reset_dma_stats(void)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  size_t bytes;
  uint8_t chan;

  if (nxmutex_lock(&priv->lock) < 0)
    {
      return;
    }

  bytes = priv->stats.frame_bytes;
  chan  = priv->stats.active_channel;
  memset(&priv->stats, 0, sizeof(priv->stats));
  priv->stats.frame_bytes    = bytes;
  priv->stats.active_channel = chan;

  nxmutex_unlock(&priv->lock);
}

/****************************************************************************
 * Name: esp32p4_csi_dump_dma
 ****************************************************************************/

void esp32p4_csi_dump_dma(void)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  struct esp32p4_csi_dma_stats_s st;
  dw_gdma_dev_t *dev;
  uint32_t chen;
  uint32_t int_st;
  int ret;
  int i;

  ret = esp32p4_csi_get_dma_stats(&st);
  if (ret < 0)
    {
      printf("CSI DMA: get stats failed (%d)\n", ret);
      return;
    }

  dev = priv->dma_hal.dev;
  chen   = (dev != NULL) ? dev->chen0.val : 0;
  int_st = (dev != NULL) ?
           dw_gdma_ll_channel_get_intr_status(dev, priv->dma_chan) : 0;

  printf("========================================\n");
  printf(" ESP32-P4 CSI DW-GDMA Channel Status\n");
  printf("========================================\n");
  printf("  DMA Initialized : %s\n", priv->dma_initialized ? "YES" : "NO");
  printf("  DMA Running     : %s\n", priv->dma_running ? "YES" : "NO");
  printf("  Channel Number  : %u\n", priv->dma_chan);
  printf("  Buffer Count    : %u\n", priv->buf_count);
  printf("  Memory Type     : %s\n",
         (priv->dma_cfg.mem_type == ESP32P4_CSI_BUF_PSRAM) ?
         "PSRAM" : "Internal SRAM");
  printf("  Expected Frame  : %" PRIu32 " bytes\n", st.frame_bytes);
  printf("  Guard Enabled   : %s\n",
         priv->dma_cfg.enable_guard ? "YES" : "NO");
  printf("  Hardware CHEN0  : 0x%08" PRIx32 "\n", chen);
  printf("  Channel INT ST0 : 0x%08" PRIx32 "\n", int_st);
  printf("----------------------------------------\n");
  printf(" Buffer & Descriptor Layout:\n");
  for (i = 0; i < priv->buf_count; i++)
    {
      printf("  Buf[%d]: raw=%p, payload=%p, len=%zu\n",
             i, priv->buffers[i].raw_buf,
             priv->buffers[i].payload, priv->buffers[i].payload_len);
      if (priv->lli_items != NULL)
        {
          printf("  LLI[%d]: @%p (sar=0x%08" PRIx32 ", dar=0x%08" PRIx32
                 ", block_ts=%" PRIu32 ")\n",
                 i, &priv->lli_items[i],
                 (uint32_t)priv->lli_items[i].sar_lo.sar0,
                 (uint32_t)priv->lli_items[i].dar_lo.dar0,
                 (uint32_t)priv->lli_items[i].block_ts_lo.block_ts + 1);
        }
    }

  printf("----------------------------------------\n");
  printf(" Diagnostic Statistics:\n");
  printf("  Frames Captured : %" PRIu32 "\n", st.frames_captured);
  printf("  Frames Dropped  : %" PRIu32 "\n", st.frames_dropped);
  printf("  DMA Timeouts    : %" PRIu32 "\n", st.dma_timeouts);
  printf("  Decode Errors   : %" PRIu32 "\n", st.dma_err_dec);
  printf("  Slave Errors    : %" PRIu32 "\n", st.dma_err_slv);
  printf("  LLI Errors      : %" PRIu32 "\n", st.dma_err_lli);
  printf("  Guard Errors    : %" PRIu32 "\n", st.dma_guard_errors);
  printf("  Last Frame Time : %" PRIu64 " us\n", st.last_frame_time_us);
  printf("========================================\n");
}

/****************************************************************************
 * Name: esp32p4_csi_dma_deinit
 ****************************************************************************/

int esp32p4_csi_dma_deinit(void)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  dw_gdma_dev_t *dev;
  int ret;
  int i;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->dma_initialized)
    {
      dev = priv->dma_hal.dev;

      /* Disable channel and interrupts */

      if (dev != NULL)
        {
          dw_gdma_ll_channel_enable(dev, priv->dma_chan, false);
          dw_gdma_ll_channel_enable_intr_propagation(dev, priv->dma_chan,
                                                     UINT32_MAX, false);
          dw_gdma_ll_channel_clear_intr(dev, priv->dma_chan, UINT32_MAX);
        }

      if (priv->cpuint >= 0)
        {
          up_disable_irq(ESP_SOURCE2IRQ(DW_GDMA_INTR_SOURCE));
          esp_teardown_irq(DW_GDMA_INTR_SOURCE, priv->cpuint);
          priv->cpuint = -1;
        }

      /* Free LLI items */

      if (priv->lli_items != NULL)
        {
          kmm_free(priv->lli_items);
          priv->lli_items = NULL;
        }

      /* Free frame buffers */

      for (i = 0; i < priv->buf_count; i++)
        {
          if (priv->buffers[i].raw_buf != NULL)
            {
              if (priv->dma_cfg.mem_type == ESP32P4_CSI_BUF_PSRAM)
                {
                  kumm_free(priv->buffers[i].raw_buf);
                }
              else
                {
                  kmm_free(priv->buffers[i].raw_buf);
                }

              priv->buffers[i].raw_buf = NULL;
            }
        }

      priv->dma_initialized = false;
      priv->dma_running     = false;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}
