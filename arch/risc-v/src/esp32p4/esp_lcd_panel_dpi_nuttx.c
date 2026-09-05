/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_lcd_panel_dpi_nuttx.c
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
#include <math.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/wqueue.h>

#include "esp_idf_shim.h"
#include "esp_lcd_panel_dpi_nuttx.h"
#include "esp_dw_gdma_nuttx.h"
#include "esp_lcd_dsi_bus_nuttx.h"

#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "hal/mipi_dsi_host_ll.h"
#include "hal/mipi_dsi_brg_ll.h"
#include "hal/dw_gdma_ll.h"
#include "hal/color_types.h"
#include "esp_private/periph_ctrl.h"
#include "esp_cache.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIPI_DSI_BRG_MEM_BASE  0x50105000  /* Bridge FIFO address */
#define DPI_PANEL_FB_BPP       24          /* RGB888: 3 bytes per pixel */
#define DPI_CLK_SRC_MHZ        240         /* PLL_F240M */

/* Color format — use HAL's lcd_color_format_t enum from lcd_types.h.
 * Do NOT redefine LCD_COLOR_FMT_RGB888 — it's a FOURCC code, not 5.
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp_lcd_dpi_panel_t
{
  esp_lcd_dsi_bus_handle_t    bus;
  dw_gdma_channel_handle_t   dma_chan;
  dw_gdma_link_list_handle_t link_list;
  uint8_t                    *fb;
  size_t                      fb_size;
  uint32_t                    h_pixels;
  uint32_t                    v_pixels;
  uint32_t                    bits_per_pixel;
  volatile bool               refresh_running;
  struct work_s               refresh_work;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Static panel instance — only one DPI panel on ESP32-P4 */

static struct esp_lcd_dpi_panel_t g_dpi_panel;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dsi_dma_refresh_thread
 *
 * Description:
 *   Kernel thread that polls DMA channel 1 completion and re-arms.
 *   Runs at ~60fps (16ms per frame).
 ****************************************************************************/

static int dsi_dma_refresh_thread(int argc, char *argv[])
{
  struct esp_lcd_dpi_panel_t *panel = &g_dpi_panel;
  dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);

  syslog(LOG_INFO, "[DPI] Refresh thread started\n");

  while (panel->refresh_running)
    {
      /* Wait ~16ms (one frame at 60fps) */

      usleep(16000);

      /* Check if channel 1 is idle (bit1 in CHEN) */

      uint32_t chen = dev->chen0.val;
      if (!(chen & (1u << 1)))
        {
          /* Clear pending interrupt status */

          uint32_t st = dw_gdma_ll_channel_get_intr_status(dev, 1);
          if (st)
            {
              dw_gdma_ll_channel_clear_intr(dev, 1, st);
            }

          /* Re-arm DMA */

          dw_gdma_block_markers_t m;
          m.is_valid = true;
          m.is_last  = true;
          dw_gdma_lli_set_block_markers(
              dw_gdma_link_list_get_item(panel->link_list, 0), m);
          dw_gdma_channel_use_link_list(panel->dma_chan, panel->link_list);
          dw_gdma_channel_enable_ctrl(panel->dma_chan, true);
        }
    }

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_lcd_new_panel_dpi
 ****************************************************************************/

int esp_lcd_new_panel_dpi(esp_lcd_dsi_bus_handle_t bus,
                          const esp_lcd_dpi_panel_config_t *config,
                          esp_lcd_dpi_panel_handle_t *ret_panel)
{
  struct esp_lcd_dpi_panel_t *panel = &g_dpi_panel;
  uint32_t h_pixels;
  uint32_t v_pixels;
  uint32_t dpi_div;
  int ret;

  if (!bus || !config || !ret_panel)
    {
      return -EINVAL;
    }

  memset(panel, 0, sizeof(*panel));

  /* Store bus reference and pixel dimensions */

  panel->bus            = bus;
  panel->bits_per_pixel = DPI_PANEL_FB_BPP;
  h_pixels              = config->video_timing.h_size;
  v_pixels              = config->video_timing.v_size;
  panel->h_pixels       = h_pixels;
  panel->v_pixels       = v_pixels;

  /* Compute framebuffer size: h * v * 3 bytes (RGB888) */

  panel->fb_size = (size_t)h_pixels * v_pixels * (DPI_PANEL_FB_BPP / 8);

  /* Allocate framebuffer — 64-byte aligned for cache line */

  panel->fb = (uint8_t *)kmm_memalign(64, panel->fb_size);
  if (!panel->fb)
    {
      syslog(LOG_ERR, "[DPI] Failed to allocate FB (%zu bytes)\n",
             panel->fb_size);
      return -ENOMEM;
    }

  memset(panel->fb, 0, panel->fb_size);

  /* Flush zeros to physical memory (cache writeback) */

  esp_cache_msync(panel->fb, panel->fb_size,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  /* Configure DPI clock divider from PLL_F240M */

  dpi_div = mipi_dsi_hal_host_dpi_calculate_divider(
      &bus->hal, (float)DPI_CLK_SRC_MHZ, config->dpi_clock_freq_mhz);

  PERIPH_RCC_ATOMIC()
  {
    mipi_dsi_ll_set_dpi_clock_source(bus->bus_id,
                                     MIPI_DSI_DPI_CLK_SRC_PLL_F240M);
    mipi_dsi_ll_set_dpi_clock_div(bus->bus_id, dpi_div);
    mipi_dsi_ll_enable_dpi_clock(bus->bus_id, true);
  }

  /* Configure Host DPI parameters */

  mipi_dsi_host_ll_dpi_set_vcid(bus->hal.host,
                                config->virtual_channel);
  mipi_dsi_host_ll_dpi_set_color_coding(bus->hal.host,
                                        LCD_COLOR_FMT_RGB888, 0);

  /* Timing signal polarities: all active low */

  mipi_dsi_host_ll_dpi_set_timing_polarity(bus->hal.host,
                                           false, false,
                                           false, false, false);

  /* Enable low-power transitions during blanking periods */

  mipi_dsi_host_ll_dpi_enable_lp_horizontal_timing(bus->hal.host,
                                                   true, true);
  mipi_dsi_host_ll_dpi_enable_lp_vertical_timing(bus->hal.host,
                                                 true, true,
                                                 true, true);

  /* Commands transmitted in low-power mode */

  mipi_dsi_host_ll_dpi_enable_lp_command(bus->hal.host, true);

  /* Enable frame ACK from panel */

  mipi_dsi_host_ll_dpi_enable_frame_ack(bus->hal.host, true);

  /* Video mode: burst with sync pulses (energy-efficient) */

  mipi_dsi_host_ll_dpi_set_video_burst_type(bus->hal.host,
      MIPI_DSI_LL_VIDEO_BURST_WITH_SYNC_PULSES);

  /* Active line pixel count */

  mipi_dsi_host_ll_dpi_set_video_packet_pixel_num(bus->hal.host,
                                                  h_pixels);

  /* Disable multi-packets and null packets */

  mipi_dsi_host_ll_dpi_set_trunks_num(bus->hal.host, 0);
  mipi_dsi_host_ll_dpi_set_null_packet_size(bus->hal.host, 0);

  /* Set horizontal timing */

  mipi_dsi_hal_host_dpi_set_horizontal_timing(&bus->hal,
      config->video_timing.hsync_pulse_width,
      config->video_timing.hsync_back_porch,
      config->video_timing.h_size,
      config->video_timing.hsync_front_porch);

  /* Set vertical timing */

  mipi_dsi_hal_host_dpi_set_vertical_timing(&bus->hal,
      config->video_timing.vsync_pulse_width,
      config->video_timing.vsync_back_porch,
      config->video_timing.v_size,
      config->video_timing.vsync_front_porch);

  /* Configure Bridge */

  mipi_dsi_brg_ll_set_num_pixel_bits(bus->hal.bridge,
      h_pixels * v_pixels * DPI_PANEL_FB_BPP);
  mipi_dsi_brg_ll_set_underrun_discard_count(bus->hal.bridge,
      h_pixels);
  mipi_dsi_brg_ll_set_flow_controller(bus->hal.bridge,
      MIPI_DSI_LL_FLOW_CONTROLLER_DMA);
  mipi_dsi_brg_ll_set_multi_block_number(bus->hal.bridge, 1);
  mipi_dsi_brg_ll_set_burst_len(bus->hal.bridge, 256);
  mipi_dsi_brg_ll_set_empty_threshold(bus->hal.bridge, 1024 - 256);

  /* Enable DSI bridge */

  mipi_dsi_brg_ll_enable(bus->hal.bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(bus->hal.bridge);

  /* DMA channel will be created in esp_lcd_dpi_panel_init() —
   * AFTER Camera CSI init (which resets GDMA and clears all channels).
   */

  *ret_panel = panel;

  syslog(LOG_INFO,
         "[DPI] Panel created: %lux%lu, fb=%p (%zu bytes)\n",
         (unsigned long)h_pixels, (unsigned long)v_pixels,
         panel->fb, panel->fb_size);

  return 0;
}

/****************************************************************************
 * Name: esp_lcd_dpi_panel_init
 *
 * Description:
 *   Start the DPI panel refresh loop.
 *   Critical sequence: start DMA → enable video mode → enable DPI output.
 ****************************************************************************/

int esp_lcd_dpi_panel_init(esp_lcd_dpi_panel_handle_t panel)
{
  dw_gdma_block_transfer_config_t xfer;
  int ret;

  if (!panel || !panel->fb)
    {
      return -EINVAL;
    }

  /* Create DMA channel (must be done AFTER Camera CSI init which
   * resets GDMA and clears all channel configurations).
   */

  dw_gdma_channel_alloc_config_t dma_cfg;
  memset(&dma_cfg, 0, sizeof(dma_cfg));

  dma_cfg.src.block_transfer_type      = DW_GDMA_BLOCK_TRANSFER_LIST;
  dma_cfg.src.role                     = DW_GDMA_ROLE_MEM;
  dma_cfg.src.handshake_type           = DW_GDMA_HANDSHAKE_HW;
  dma_cfg.src.num_outstanding_requests = 5;

  dma_cfg.dst.block_transfer_type      = DW_GDMA_BLOCK_TRANSFER_LIST;
  dma_cfg.dst.role                     = DW_GDMA_ROLE_PERIPH_DSI;
  dma_cfg.dst.handshake_type           = DW_GDMA_HANDSHAKE_HW;
  dma_cfg.dst.num_outstanding_requests = 2;

  dma_cfg.flow_controller              = DW_GDMA_FLOW_CTRL_SELF;
  dma_cfg.chan_priority                 = 1;

  ret = dw_gdma_new_channel(&dma_cfg, &panel->dma_chan);
  if (ret < 0)
    {
      syslog(LOG_ERR, "[DPI] Failed to create DMA channel: %d\n", ret);
      return ret;
    }

  /* Create DMA link list (1 item, self-cycling) */

  dw_gdma_link_list_config_t ll_cfg;
  memset(&ll_cfg, 0, sizeof(ll_cfg));
  ll_cfg.num_items = 1;

  ret = dw_gdma_new_link_list(&ll_cfg, &panel->link_list);
  if (ret < 0)
    {
      syslog(LOG_ERR, "[DPI] Failed to create link list: %d\n", ret);
      return ret;
    }

  /* Register DMA callback — not used with polling approach, skip */

  /* Start the refresh polling thread */

  panel->refresh_running = true;
  kthread_create("dsi_refresh", 200, 2048,
                 dsi_dma_refresh_thread, NULL);

  /* Configure DMA transfer parameters in the LLI:
   *   Source: framebuffer in PSRAM/SRAM (incrementing)
   *   Dest:   Bridge FIFO at fixed address
   */

  memset(&xfer, 0, sizeof(xfer));

  xfer.src.addr       = (uint32_t)(uintptr_t)panel->fb;
  xfer.src.burst_mode = DW_GDMA_BURST_MODE_INCREMENT;
  xfer.src.burst_items = DW_GDMA_BURST_ITEMS_512;
  xfer.src.burst_len  = 16;
  xfer.src.width      = DW_GDMA_TRANS_WIDTH_64;

  xfer.dst.addr       = MIPI_DSI_BRG_MEM_BASE;
  xfer.dst.burst_mode = DW_GDMA_BURST_MODE_FIXED;
  xfer.dst.burst_items = DW_GDMA_BURST_ITEMS_256;
  xfer.dst.burst_len  = 16;
  xfer.dst.width      = DW_GDMA_TRANS_WIDTH_64;

  /* Size in 64-bit transfer items */

  xfer.size = panel->fb_size * 8 / 64;

  dw_gdma_lli_config_transfer(
      dw_gdma_link_list_get_item(panel->link_list, 0), &xfer);

  /* Mark LLI as valid and last. DMA will complete one frame then stop.
   * CSI ISR calls esp_dsi_dma_isr_handler() to re-arm for next frame.
   */

  dw_gdma_block_markers_t markers;
  markers.is_valid = true;
  markers.is_last  = true;

  dw_gdma_lli_set_block_markers(
      dw_gdma_link_list_get_item(panel->link_list, 0), markers);

  /* Attach link list and start DMA */

  dw_gdma_channel_use_link_list(panel->dma_chan, panel->link_list);
  dw_gdma_channel_enable_ctrl(panel->dma_chan, true);

  /* Enable video mode — THIS triggers the PHY LP→HS transition */

  mipi_dsi_host_ll_enable_video_mode(panel->bus->hal.host, true);

  /* Enable DPI output on the bridge */

  mipi_dsi_brg_ll_enable_dpi_output(panel->bus->hal.bridge, true);
  mipi_dsi_brg_ll_update_dpi_config(panel->bus->hal.bridge);

  /* Inline re-arm loop: keep DMA running for first 5 seconds
   * to verify display works before handing off to refresh thread.
   */

  {
    dw_gdma_dev_t *dev = DW_GDMA_LL_GET_HW(0);
    int frames = 0;
    for (int i = 0; i < 300; i++)  /* 300 * 16ms = ~5 seconds */
      {
        usleep(16000);
        uint32_t chen = dev->chen0.val;
        if (!(chen & (1u << 1)))
          {
            /* Channel idle — re-arm */

            uint32_t st = dw_gdma_ll_channel_get_intr_status(dev, 1);
            if (st)
              {
                dw_gdma_ll_channel_clear_intr(dev, 1, st);
              }

            dw_gdma_block_markers_t m2;
            m2.is_valid = true;
            m2.is_last  = true;
            dw_gdma_lli_set_block_markers(
                dw_gdma_link_list_get_item(panel->link_list, 0), m2);
            dw_gdma_channel_use_link_list(panel->dma_chan,
                                          panel->link_list);
            dw_gdma_channel_enable_ctrl(panel->dma_chan, true);
            frames++;
          }
      }

    syslog(LOG_INFO, "[DPI] Inline re-arm: %d frames in 5s\n", frames);
  }

  syslog(LOG_INFO, "[DPI] Init: DMA started, video mode active\n");
  return 0;
}

/****************************************************************************
 * Name: esp_lcd_dpi_panel_get_fb
 ****************************************************************************/

uint8_t *esp_lcd_dpi_panel_get_fb(esp_lcd_dpi_panel_handle_t panel)
{
  return panel ? panel->fb : NULL;
}

/****************************************************************************
 * Name: esp_lcd_dpi_panel_get_fb_size
 ****************************************************************************/

size_t esp_lcd_dpi_panel_get_fb_size(esp_lcd_dpi_panel_handle_t panel)
{
  return panel ? panel->fb_size : 0;
}
