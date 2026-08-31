/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp_mipi_dsi.c
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
#include <nuttx/spinlock.h>

#include "esp_mipi_dsi.h"

#include "esp_ldo_regulator.h"
#include "esp_clk_tree.h"
#include "esp_cache.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/dw_gdma.h"

#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "hal/mipi_dsi_host_ll.h"
#include "hal/mipi_dsi_phy_ll.h"
#include "hal/mipi_dsi_brg_ll.h"
#include "hal/dw_gdma_ll.h"
#include "hal/lcd_types.h"

#include "soc/reg_base.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DSI_BUS_ID                  0
#define DSI_VIRTUAL_CHANNEL         0

/* TxClkEsc must be 2..20 MHz, timeout clock is conventionally 10 MHz */

#define DSI_TIMEOUT_CLOCK_FREQ_MHZ  10
#define DSI_ESCAPE_CLOCK_FREQ_MHZ   18

/* Poll limit (iterations x 1ms) for PHY PLL lock / lane stop waits */

#define DSI_PHY_WAIT_MAX_MS         100

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_dsi_dev_s
{
  mipi_dsi_hal_context_t hal;
  esp_ldo_channel_handle_t ldo;
  dw_gdma_channel_handle_t dma_chan;
  uint8_t *fb;
  size_t fb_size;
  bool initialized;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp_dsi_dev_s g_dsi_dev;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mipi_dsi_init
 ****************************************************************************/

int esp_mipi_dsi_init(uint8_t num_lanes, uint32_t lane_rate_mbps,
                      int phy_ldo_chan, uint32_t phy_ldo_mv)
{
  struct esp_dsi_dev_s *dev = &g_dsi_dev;
  mipi_dsi_hal_context_t *hal = &dev->hal;
  mipi_dsi_hal_config_t hal_config;
  soc_module_clk_t pllref_clk;
  soc_module_clk_t cfg_clk;
  uint32_t phy_clk_freq_hz = 0;
  int wait;

  if (dev->initialized)
    {
      return OK;
    }

  /* Power the MIPI DPHY rail from the on-chip LDO */

  if (phy_ldo_chan >= 0)
    {
      esp_ldo_channel_config_t ldo_cfg;

      memset(&ldo_cfg, 0, sizeof(ldo_cfg));
      ldo_cfg.chan_id    = phy_ldo_chan;
      ldo_cfg.voltage_mv = (int)phy_ldo_mv;

      if (esp_ldo_acquire_channel(&ldo_cfg, &dev->ldo) != 0)
        {
          lcderr("ERROR: failed to acquire DPHY LDO channel %d\n",
                 phy_ldo_chan);
          return -EIO;
        }
    }

  /* Enable the APB clock for the DSI host/bridge register access */

  PERIPH_RCC_ATOMIC()
    {
      mipi_dsi_ll_enable_bus_clock(DSI_BUS_ID, true);
      mipi_dsi_ll_reset_register(DSI_BUS_ID);
    }

  /* Enable and route the PHY configuration and PLL reference clocks */

  pllref_clk = (soc_module_clk_t)MIPI_DSI_PHY_PLLREF_CLK_SRC_DEFAULT;
  cfg_clk    = (soc_module_clk_t)MIPI_DSI_PHY_CFG_CLK_SRC_DEFAULT;

  esp_clk_tree_enable_src(pllref_clk, true);
  esp_clk_tree_enable_src(cfg_clk, true);

  PERIPH_RCC_ATOMIC()
    {
      mipi_dsi_ll_set_phy_config_clock_source(DSI_BUS_ID,
                                        MIPI_DSI_PHY_CFG_CLK_SRC_DEFAULT);
      mipi_dsi_ll_enable_phy_config_clock(DSI_BUS_ID, true);
      mipi_dsi_ll_set_phy_pllref_clock_source(DSI_BUS_ID,
                                        MIPI_DSI_PHY_PLLREF_CLK_SRC_DEFAULT);
      mipi_dsi_ll_set_phy_pll_ref_clock_div(DSI_BUS_ID, 1);
      mipi_dsi_ll_enable_phy_pllref_clock(DSI_BUS_ID, true);
    }

  /* Initialize the HAL context and configure the DPHY PLL */

  memset(&hal_config, 0, sizeof(hal_config));
  hal_config.bus_id             = DSI_BUS_ID;
  hal_config.lane_bit_rate_mbps = lane_rate_mbps;
  hal_config.num_data_lanes     = num_lanes;

  mipi_dsi_hal_init(hal, &hal_config);

  esp_clk_tree_src_get_freq_hz(pllref_clk,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                               &phy_clk_freq_hz);

  mipi_dsi_hal_configure_phy_pll(hal, phy_clk_freq_hz, lane_rate_mbps);

  /* Wait for the PHY PLL to lock and the lanes to enter stop state */

  wait = DSI_PHY_WAIT_MAX_MS;
  while (!mipi_dsi_phy_ll_is_pll_locked(hal->host) && --wait > 0)
    {
      nxsig_usleep(1000);
    }

  if (wait <= 0)
    {
      lcderr("ERROR: DSI PHY PLL failed to lock\n");
      goto errout;
    }

  wait = DSI_PHY_WAIT_MAX_MS;
  while (!mipi_dsi_phy_ll_are_lanes_stopped(hal->host, num_lanes) &&
         --wait > 0)
    {
      nxsig_usleep(1000);
    }

  if (wait <= 0)
    {
      lcderr("ERROR: DSI lanes did not reach stop state\n");
      goto errout;
    }

  /* Start in command mode; video mode is enabled by esp_mipi_dsi_dpi_start */

  mipi_dsi_host_ll_enable_video_mode(hal->host, false);
  mipi_dsi_host_ll_set_clock_lane_state(hal->host,
                                        MIPI_DSI_LL_CLOCK_LANE_STATE_AUTO);
  mipi_dsi_phy_ll_set_switch_time(hal->host, 50, 104, 46, 128);

  mipi_dsi_host_ll_enable_rx_crc(hal->host, true);
  mipi_dsi_host_ll_enable_rx_ecc(hal->host, true);
  mipi_dsi_host_ll_enable_tx_eotp(hal->host, true, false);

  mipi_dsi_host_ll_set_timeout_clock_division(hal->host,
      (lane_rate_mbps + 8 * DSI_TIMEOUT_CLOCK_FREQ_MHZ / 2) /
      (8 * DSI_TIMEOUT_CLOCK_FREQ_MHZ));
  mipi_dsi_host_ll_set_escape_clock_division(hal->host,
      (lane_rate_mbps + 8 * DSI_ESCAPE_CLOCK_FREQ_MHZ / 2) /
      (8 * DSI_ESCAPE_CLOCK_FREQ_MHZ));
  mipi_dsi_host_ll_set_timeout_count(hal->host, 0, 0, 0, 0, 0, 0, 0);
  mipi_dsi_phy_ll_set_max_read_time(hal->host, 6000);
  mipi_dsi_phy_ll_set_stop_wait_time(hal->host, 0x3f);

  /* Configure the generic packet interface for low-power DCS commands */

  mipi_dsi_host_ll_enable_te_ack(hal->host, false);
  mipi_dsi_host_ll_enable_cmd_ack(hal->host, true);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(hal->host, 0,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(hal->host, 1,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_wr_speed_mode(hal->host, 2,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_long_wr_speed_mode(hal->host,
                                              MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_rd_speed_mode(hal->host, 0,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_rd_speed_mode(hal->host, 1,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_gen_short_rd_speed_mode(hal->host, 2,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_short_wr_speed_mode(hal->host, 0,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_short_wr_speed_mode(hal->host, 1,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_long_wr_speed_mode(hal->host,
                                              MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_dcs_short_rd_speed_mode(hal->host, 0,
                                               MIPI_DSI_LL_TRANS_SPEED_LP);
  mipi_dsi_host_ll_set_mrps_speed_mode(hal->host,
                                       MIPI_DSI_LL_TRANS_SPEED_LP);

  dev->initialized = true;
  return OK;

errout:
  if (dev->ldo != NULL)
    {
      esp_ldo_release_channel(dev->ldo);
      dev->ldo = NULL;
    }

  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: esp_mipi_dsi_dcs_write
 ****************************************************************************/

int esp_mipi_dsi_dcs_write(uint8_t cmd, const void *param, size_t len)
{
  struct esp_dsi_dev_s *dev = &g_dsi_dev;

  if (!dev->initialized)
    {
      return -EPERM;
    }

  mipi_dsi_hal_host_gen_write_dcs_command(&dev->hal, DSI_VIRTUAL_CHANNEL,
                                          cmd, 1, param, len);
  return OK;
}

/****************************************************************************
 * Name: esp_mipi_dsi_dcs_read
 ****************************************************************************/

int esp_mipi_dsi_dcs_read(uint8_t cmd, uint8_t *buf, size_t len)
{
  struct esp_dsi_dev_s *dev = &g_dsi_dev;
  mipi_dsi_hal_context_t *hal = &dev->hal;
  int count = 0;
  bool was_video;
  int wait;

  if (!dev->initialized || buf == NULL || len == 0)
    {
      return -EINVAL;
    }

  /* Same sequence as the HAL read helper, but with bounded waits so a
   * missing/unpowered panel results in -ETIMEDOUT instead of a hang.
   *
   * A DCS read has to drop out of video mode and turn the bus around, so
   * both of those are saved and put back on every exit path.  Leaving BTA
   * enabled lets the peripheral turn the bus around during video
   * transmission, which is out of spec and corrupts the stream; this used
   * to persist for the whole session because nothing ever cleared it.
   */

  was_video = !hal->host->mode_cfg.cmd_video_mode;

  mipi_dsi_hal_host_gen_write_short_packet(hal, DSI_VIRTUAL_CHANNEL,
      MIPI_DSI_DT_SET_MAXIMUM_RETURN_PKT, len);
  mipi_dsi_host_ll_enable_video_mode(hal->host, false);
  mipi_dsi_host_ll_enable_bta(hal->host, true);
  mipi_dsi_host_ll_gen_set_rx_vcid(hal->host, DSI_VIRTUAL_CHANNEL);
  mipi_dsi_hal_host_gen_write_short_packet(hal, DSI_VIRTUAL_CHANNEL,
      MIPI_DSI_DT_DCS_READ_0, cmd);

  wait = 200;
  while (mipi_dsi_host_ll_gen_is_read_cmd_busy(hal->host) && --wait > 0)
    {
      nxsig_usleep(1000);
    }

  if (wait <= 0)
    {
      count = -ETIMEDOUT;
      goto restore;
    }

  wait = 200;
  while (mipi_dsi_host_ll_gen_is_read_fifo_empty(hal->host) && --wait > 0)
    {
      nxsig_usleep(1000);
    }

  if (wait <= 0)
    {
      count = -ETIMEDOUT;
      goto restore;
    }

  while (!mipi_dsi_host_ll_gen_is_read_fifo_empty(hal->host) &&
         count < len)
    {
      uint32_t word = mipi_dsi_host_ll_gen_read_payload_fifo(hal->host);
      int i;

      for (i = 0; i < 4 && count < len; i++)
        {
          buf[count++] = (word >> (8 * i)) & 0xff;
        }
    }

restore:
  mipi_dsi_host_ll_enable_bta(hal->host, false);
  mipi_dsi_host_ll_enable_video_mode(hal->host, was_video);
  return (int)count;
}

/****************************************************************************
 * Name: esp_mipi_dsi_dump_status
 *
 * Description:
 *   Log the raw DSI host PHY/interrupt status registers.  int_st0 carries
 *   peripheral acknowledge-error reports (panel is alive but unhappy) and
 *   DPHY errors; int_st1 carries transmission timeouts and ECC/CRC errors.
 *   Reading the interrupt status registers clears them.
 *
 ****************************************************************************/

void esp_mipi_dsi_dump_status(void)
{
  struct esp_dsi_dev_s *dev = &g_dsi_dev;
  mipi_dsi_hal_context_t *hal = &dev->hal;
  dw_gdma_dev_t *dma = DW_GDMA_LL_GET_HW(0);

  if (!dev->initialized)
    {
      return;
    }

  syslog(LOG_INFO,
         "DSI status: phy=0x%08x int_st0=0x%08x int_st1=0x%08x "
         "lanes_stopped=%d pll_locked=%d\n",
         (unsigned)hal->host->phy_status.val,
         (unsigned)hal->host->int_st0.val,
         (unsigned)hal->host->int_st1.val,
         mipi_dsi_phy_ll_are_lanes_stopped(hal->host, 2),
         mipi_dsi_phy_ll_is_pll_locked(hal->host));

  /* Video path: the DSI bridge only streams while the DW-GDMA channel is
   * enabled and feeding its FIFO.  chen0 bit N is channel N's enable, and
   * sar0 advances through the framebuffer while a block is in flight.
   * The DW-GDMA register block is unclocked until esp_mipi_dsi_dpi_start()
   * allocates the channel, so reading it before that faults.
   */

  if (dev->dma_chan == NULL)
    {
      return;
    }

  syslog(LOG_INFO,
         "DSI video: cmd_mode=%d brg_int=0x%08x chen=0x%08x "
         "ch0_int=0x%08x sar=0x%08x blk_ts=%u\n",
         (int)hal->host->mode_cfg.cmd_video_mode,
         (unsigned)mipi_dsi_brg_ll_get_interrupt_status(hal->bridge),
         (unsigned)dma->chen0.val,
         (unsigned)dma->ch[0].int_st0.val,
         (unsigned)dma->ch[0].sar0.sar0,
         (unsigned)dma->ch[0].block_ts0.block_ts);
}

/****************************************************************************
 * Name: esp_mipi_dsi_dpi_start
 ****************************************************************************/

int esp_mipi_dsi_dpi_start(const struct esp_mipi_dsi_video_s *vt,
                           uint8_t *fb)
{
  struct esp_dsi_dev_s *dev = &g_dsi_dev;
  mipi_dsi_hal_context_t *hal = &dev->hal;
  dw_gdma_channel_alloc_config_t dma_alloc_config;
  dw_gdma_block_transfer_config_t dma_transfer_config;
  soc_module_clk_t dpi_clk;
  uint32_t dpi_clk_src_freq_hz = 0;
  uint32_t dpi_div;

  if (!dev->initialized || vt == NULL || fb == NULL)
    {
      return -EINVAL;
    }

  dev->fb = fb;
  dev->fb_size = (size_t)vt->h_size * vt->v_size * 2;

  /* Route and enable the DPI pixel clock */

  dpi_clk = (soc_module_clk_t)MIPI_DSI_DPI_CLK_SRC_DEFAULT;

  esp_clk_tree_src_get_freq_hz(dpi_clk,
                               ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
                               &dpi_clk_src_freq_hz);
  dpi_div = mipi_dsi_hal_host_dpi_calculate_divider(
                hal, dpi_clk_src_freq_hz / 1000000.0f,
                (float)vt->dpi_clk_mhz);
  esp_clk_tree_enable_src(dpi_clk, true);

  PERIPH_RCC_ATOMIC()
    {
      mipi_dsi_ll_set_dpi_clock_source(DSI_BUS_ID,
                                       MIPI_DSI_DPI_CLK_SRC_DEFAULT);
      mipi_dsi_ll_set_dpi_clock_div(DSI_BUS_ID, dpi_div);
      mipi_dsi_ll_enable_dpi_clock(DSI_BUS_ID, true);
    }

  /* Allocate the DW-GDMA channel that feeds the DSI bridge.  Both ends
   * use the hardware auto-reload block mode: the same whole-frame block
   * restarts forever without any CPU/interrupt involvement.
   */

  memset(&dma_alloc_config, 0, sizeof(dma_alloc_config));
  dma_alloc_config.src.block_transfer_type = DW_GDMA_BLOCK_TRANSFER_RELOAD;
  dma_alloc_config.src.role = DW_GDMA_ROLE_MEM;
  dma_alloc_config.src.handshake_type = DW_GDMA_HANDSHAKE_HW;
  dma_alloc_config.src.num_outstanding_requests = 5;
  dma_alloc_config.dst.block_transfer_type = DW_GDMA_BLOCK_TRANSFER_RELOAD;
  dma_alloc_config.dst.role = DW_GDMA_ROLE_PERIPH_DSI;
  dma_alloc_config.dst.handshake_type = DW_GDMA_HANDSHAKE_HW;
  dma_alloc_config.dst.num_outstanding_requests = 2;
  dma_alloc_config.flow_controller = DW_GDMA_FLOW_CTRL_SELF;
  dma_alloc_config.chan_priority = 1;

  if (dw_gdma_new_channel(&dma_alloc_config, &dev->dma_chan) != 0)
    {
      lcderr("ERROR: failed to create DW-GDMA channel\n");
      return -ENOMEM;
    }

  /* DPI host configuration: RGB565 video, burst mode with sync pulses */

  mipi_dsi_host_ll_dpi_set_vcid(hal->host, DSI_VIRTUAL_CHANNEL);
  mipi_dsi_host_ll_dpi_set_color_coding(hal->host, LCD_COLOR_FMT_RGB565, 0);
  mipi_dsi_host_ll_dpi_set_timing_polarity(hal->host, false, false, false,
                                           false, false);
  mipi_dsi_host_ll_dpi_enable_lp_horizontal_timing(hal->host, true, true);
  mipi_dsi_host_ll_dpi_enable_lp_vertical_timing(hal->host, true, true,
                                                 true, true);
  mipi_dsi_host_ll_dpi_enable_lp_command(hal->host, true);
  mipi_dsi_host_ll_dpi_enable_frame_ack(hal->host, true);
  mipi_dsi_host_ll_dpi_set_video_burst_type(hal->host,
      MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);
  mipi_dsi_host_ll_dpi_set_video_packet_pixel_num(hal->host, vt->h_size);
  mipi_dsi_host_ll_dpi_set_trunks_num(hal->host, 0);
  mipi_dsi_host_ll_dpi_set_null_packet_size(hal->host, 0);
  mipi_dsi_hal_host_dpi_set_horizontal_timing(hal, vt->hsw, vt->hbp,
                                              vt->h_size, vt->hfp);
  mipi_dsi_hal_host_dpi_set_vertical_timing(hal, vt->vsw, vt->vbp,
                                            vt->v_size, vt->vfp);

  /* DSI bridge: frame geometry, color format and DMA flow control */

  mipi_dsi_brg_ll_set_num_pixel_bits(hal->bridge,
      (uint32_t)vt->h_size * vt->v_size * 16);
  mipi_dsi_brg_ll_set_underrun_discard_count(hal->bridge, vt->h_size);
  mipi_dsi_brg_ll_set_input_color_format(hal->bridge, LCD_COLOR_FMT_RGB565);
  mipi_dsi_brg_ll_set_output_color_format(hal->bridge, LCD_COLOR_FMT_RGB565,
                                          0);
  mipi_dsi_brg_ll_set_flow_controller(hal->bridge,
                                      MIPI_DSI_LL_FLOW_CONTROLLER_DMA);
  mipi_dsi_brg_ll_set_multi_block_number(hal->bridge, 1);
  mipi_dsi_brg_ll_set_burst_len(hal->bridge, 256);
  mipi_dsi_brg_ll_set_empty_threshold(hal->bridge, 1024 - 256);
  mipi_dsi_brg_ll_enable(hal->bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(hal->bridge);

  /* One DMA block carries the whole frame: fb -> DSI bridge FIFO */

  memset(&dma_transfer_config, 0, sizeof(dma_transfer_config));
  dma_transfer_config.src.addr = (uint32_t)(uintptr_t)fb;
  dma_transfer_config.src.burst_mode = DW_GDMA_BURST_MODE_INCREMENT;
  dma_transfer_config.src.burst_items = DW_GDMA_BURST_ITEMS_512;
  dma_transfer_config.src.burst_len = 16;
  dma_transfer_config.src.width = DW_GDMA_TRANS_WIDTH_64;
  dma_transfer_config.dst.addr = MIPI_DSI_BRG_MEM_BASE;
  dma_transfer_config.dst.burst_mode = DW_GDMA_BURST_MODE_FIXED;
  dma_transfer_config.dst.burst_items = DW_GDMA_BURST_ITEMS_256;
  dma_transfer_config.dst.burst_len = 16;
  dma_transfer_config.dst.width = DW_GDMA_TRANS_WIDTH_64;
  dma_transfer_config.size = dev->fb_size * 8 / 64;

  dw_gdma_channel_config_transfer(dev->dma_chan, &dma_transfer_config);
  dw_gdma_channel_enable_ctrl(dev->dma_chan, true);

  /* Switch the host to video mode and open the bridge DPI output */

  mipi_dsi_host_ll_enable_video_mode(hal->host, true);
  mipi_dsi_brg_ll_enable_dpi_output(hal->bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(hal->bridge);

  return OK;
}

/****************************************************************************
 * Name: esp_mipi_dsi_flush
 ****************************************************************************/

void esp_mipi_dsi_flush(const void *start, size_t len)
{
  esp_cache_msync((void *)start, len,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                  ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}
