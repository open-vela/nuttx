/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp32p4_mipi_csi.c
 *
 * Native NuttX MIPI-CSI driver (no FreeRTOS dependency).
 *
 * The register sequence follows the official esp-hal-3rdparty
 * upper_hal_cam/csi driver; frame capture reuses the in-tree
 * mipi_csi_hal and dw_gdma for continuous streaming: when a
 * frame completes, the ISR rotates the buffer and notifies the
 * upper layer via frame_cb (ISR context: lightweight
 * semaphore/flag operations only).
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>

#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_private/dw_gdma.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk_tree_common.h"

#include "esp_clk_tree.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_ldo_regulator.h"

#include "hal/mipi_csi_ll.h"
#include "hal/mipi_csi_hal.h"
#include "hal/mipi_csi_brg_ll.h"

#include "soc/clk_tree_defs.h"
#include "soc/reg_base.h"

#include "esp32p4_mipi_csi.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct esp32p4_mipi_csi_dev_s
{
  mipi_csi_hal_context_t hal;
  dw_gdma_channel_handle_t dma_chan;
  uint32_t h_res;
  uint32_t v_res;
  uint32_t in_bpp;
  size_t frame_bytes;       /* bytes per frame = h*v*bpp/8 */
  size_t csi_transfer_size; /* dw_gdma count = h*v*bpp/64 */
  void *fb[2];              /* double buffers */
  int cur_fb;
  uint32_t frame_count;
  esp32p4_mipi_csi_frame_cb_t frame_cb;
  void *cb_arg;
  bool started;
};

static struct esp32p4_mipi_csi_dev_s g_csi;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

size_t esp32p4_mipi_csi_framelen(void)
{
  return g_csi.frame_bytes;
}

uint32_t esp32p4_mipi_csi_frame_count(void)
{
  return g_csi.frame_count;
}

/* Return the latest completed, safe-to-read frame buffer */

void *esp32p4_mipi_csi_get_frame(void)
{
  return g_csi.fb[g_csi.cur_fb ^ 1];
}

/****************************************************************************
 * Name: csi_dma_done_cb
 *
 * Per-frame DMA transfer-done callback (ISR context): rotate
 * the buffer, re-arm DMA and notify the upper layer.
 ****************************************************************************/

static bool csi_dma_done_cb(dw_gdma_channel_handle_t chan,
                            const dw_gdma_trans_done_event_data_t *
                            event_data, void *user_data)
{
  struct esp32p4_mipi_csi_dev_s *csi = user_data;
  dw_gdma_block_transfer_config_t t;
  void *done;

  (void)event_data;

  done = csi->fb[csi->cur_fb];
  csi->cur_fb ^= 1;

  /* Make the DMA-written PSRAM frame visible to the CPU cache */

  esp_cache_msync(done, csi->frame_bytes, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  memset(&t, 0, sizeof(t));
  t.src.addr          = MIPI_CSI_BRG_MEM_BASE;
  t.src.burst_mode    = DW_GDMA_BURST_MODE_FIXED;
  t.src.burst_items   = DW_GDMA_BURST_ITEMS_512;
  t.src.burst_len     = 16;
  t.src.width         = DW_GDMA_TRANS_WIDTH_64;
  t.dst.addr          = (uint32_t)csi->fb[csi->cur_fb];
  t.dst.burst_mode    = DW_GDMA_BURST_MODE_INCREMENT;
  t.dst.burst_items   = DW_GDMA_BURST_ITEMS_512;
  t.dst.burst_len     = 16;
  t.dst.width         = DW_GDMA_TRANS_WIDTH_64;
  t.size              = csi->csi_transfer_size;
  dw_gdma_channel_config_transfer(chan, &t);
  dw_gdma_channel_enable_ctrl(chan, true);

  csi->frame_count++;
  if (csi->frame_cb != NULL)
    {
      csi->frame_cb(done, csi->frame_bytes, csi->cb_arg);
    }

  return false;
}

/****************************************************************************
 * Name: esp32p4_mipi_csi_initialize
 ****************************************************************************/

int esp32p4_mipi_csi_initialize(const struct esp32p4_mipi_csi_config_s *cfg)
{
  mipi_csi_hal_config_t hc;
  dw_gdma_channel_alloc_config_t ac;
  dw_gdma_event_callbacks_t cbs;
  esp_ldo_channel_handle_t ldo_phy = NULL;
  esp_ldo_channel_config_t ldo_cfg;

  if (cfg == NULL || cfg->h_res == 0 || cfg->v_res == 0 ||
      cfg->lanes_num == 0 || cfg->in_bpp == 0)
    {
      return -EINVAL;
    }

  /* 0. The MIPI CSI D-PHY needs an LDO rail (ESP32-P4: LDO
   * channel 3 at 2.5V); it must be enabled before the clock/HAL
   * setup, or the D-PHY will not work.
   */

  memset(&ldo_cfg, 0, sizeof(ldo_cfg));
  ldo_cfg.chan_id    = 3;    /* LDO_VO3, per TRM/vendor example */
  ldo_cfg.voltage_mv = 2500; /* CSI PHY supply, 2.5V */
  if (esp_ldo_acquire_channel(&ldo_cfg, &ldo_phy) != ESP_OK)
    {
      _err("CSI: failed to acquire LDO channel for MIPI-CSI PHY\n");
      return -EIO;
    }

  memset(&g_csi, 0, sizeof(g_csi));
  g_csi.h_res  = cfg->h_res;
  g_csi.v_res  = cfg->v_res;
  g_csi.in_bpp = cfg->in_bpp;
  g_csi.frame_bytes =
    (size_t)cfg->h_res * cfg->v_res * cfg->in_bpp / 8;
  g_csi.csi_transfer_size =
    (size_t)cfg->h_res * cfg->v_res * cfg->in_bpp / 64;

  /* 1. Clocks: enable and reset CSI D-PHY/Host/Bridge */

  esp_clk_tree_enable_src((soc_module_clk_t)MIPI_CSI_PHY_CLK_SRC_DEFAULT,
                          true);
  PERIPH_RCC_ATOMIC()
    {
      mipi_csi_ll_set_phy_clock_source(0, MIPI_CSI_PHY_CLK_SRC_DEFAULT);
      mipi_csi_ll_enable_phy_config_clock(0, 0);
      mipi_csi_ll_enable_phy_config_clock(0, 1);
      mipi_csi_ll_enable_host_bus_clock(0, 1);
      mipi_csi_ll_reset_host_clock(0);
      mipi_csi_ll_enable_brg_module_clock(0, 1);
      mipi_csi_ll_reset_brg_module_clock(0);
    }

  /* 2. HAL init (DPHY band, lanes, bridge size/data type) */

  memset(&hc, 0, sizeof(hc));
  hc.lanes_num         = cfg->lanes_num;
  hc.frame_width       = cfg->v_res;
  hc.frame_height      = cfg->h_res;
  hc.in_bpp            = cfg->in_bpp;
  hc.out_bpp           = cfg->in_bpp;
  hc.byte_swap_en      = false;
  hc.lane_bit_rate_mbps = cfg->lane_bit_rate_mbps;
  mipi_csi_hal_init(&g_csi.hal, &hc);
  mipi_csi_brg_ll_set_burst_len(g_csi.hal.bridge_dev, 512);

  /* 3. Color format: in == out (RAW), bridge bypass mode */

  mipi_csi_brg_ll_enable_color_conversion(g_csi.hal.bridge_dev, true);
  mipi_csi_brg_ll_set_color_mode_bypass(g_csi.hal.bridge_dev, true);

  /* 4. DW-GDMA channel (src = CSI bridge FIFO, dst = memory) */

  memset(&ac, 0, sizeof(ac));
  ac.src.block_transfer_type  = DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS;
  ac.src.role                 = DW_GDMA_ROLE_PERIPH_CSI;
  ac.src.handshake_type       = DW_GDMA_HANDSHAKE_HW;
  ac.src.num_outstanding_requests = 5;
  ac.src.status_fetch_addr    = MIPI_CSI_BRG_MEM_BASE;
  ac.dst.block_transfer_type  = DW_GDMA_BLOCK_TRANSFER_CONTIGUOUS;
  ac.dst.role                 = DW_GDMA_ROLE_MEM;
  ac.dst.handshake_type       = DW_GDMA_HANDSHAKE_HW;
  ac.dst.num_outstanding_requests = 5;
  ac.flow_controller          = DW_GDMA_FLOW_CTRL_SRC;
  ac.chan_priority            = 1;

  if (dw_gdma_new_channel(&ac, &g_csi.dma_chan) != ESP_OK)
    {
      _err("CSI: failed to alloc dwgdma channel\n");
      return -ENODEV;
    }

  memset(&cbs, 0, sizeof(cbs));
  cbs.on_full_trans_done = csi_dma_done_cb;
  if (dw_gdma_channel_register_event_callbacks(g_csi.dma_chan, &cbs,
                                               &g_csi) != ESP_OK)
    {
      _err("CSI: failed to register dwgdma callback\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_csi_start
 ****************************************************************************/

int esp32p4_mipi_csi_start(esp32p4_mipi_csi_frame_cb_t frame_cb, void *arg)
{
  dw_gdma_block_transfer_config_t t;
  const size_t align = 64;

  if (g_csi.started)
    {
      return -EBUSY;
    }

  if (g_csi.fb[0] == NULL)
    {
      g_csi.fb[0] =
        heap_caps_aligned_alloc(align, g_csi.frame_bytes,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
      g_csi.fb[1] =
        heap_caps_aligned_alloc(align, g_csi.frame_bytes,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
      if (g_csi.fb[0] == NULL || g_csi.fb[1] == NULL)
        {
          _err("CSI: no mem for frame buffers (need %zu bytes each)\n",
               g_csi.frame_bytes);
          return -ENOMEM;
        }

      _info("CSI: frame buffers %p %p, %zu bytes\n",
            g_csi.fb[0], g_csi.fb[1], g_csi.frame_bytes);
    }

  g_csi.cur_fb      = 0;
  g_csi.frame_count = 0;
  g_csi.frame_cb    = frame_cb;
  g_csi.cb_arg      = arg;

  memset(&t, 0, sizeof(t));
  t.src.addr          = MIPI_CSI_BRG_MEM_BASE;
  t.src.burst_mode    = DW_GDMA_BURST_MODE_FIXED;
  t.src.burst_items   = DW_GDMA_BURST_ITEMS_512;
  t.src.burst_len     = 16;
  t.src.width         = DW_GDMA_TRANS_WIDTH_64;
  t.dst.addr          = (uint32_t)g_csi.fb[0];
  t.dst.burst_mode    = DW_GDMA_BURST_MODE_INCREMENT;
  t.dst.burst_items   = DW_GDMA_BURST_ITEMS_512;
  t.dst.burst_len     = 16;
  t.dst.width         = DW_GDMA_TRANS_WIDTH_64;
  t.size              = g_csi.csi_transfer_size;

  if (dw_gdma_channel_config_transfer(g_csi.dma_chan, &t) != ESP_OK ||
      dw_gdma_channel_enable_ctrl(g_csi.dma_chan, true) != ESP_OK)
    {
      _err("CSI: failed to start dwgdma\n");
      return -EIO;
    }

  /* Finally enable the CSI bridge; data flows into DMA */

  mipi_csi_brg_ll_enable(g_csi.hal.bridge_dev, true);

  g_csi.started = true;
  return OK;
}

/****************************************************************************
 * Name: esp32p4_mipi_csi_stop
 ****************************************************************************/

int esp32p4_mipi_csi_stop(void)
{
  if (!g_csi.started)
    {
      return -EINVAL;
    }

  mipi_csi_brg_ll_enable(g_csi.hal.bridge_dev, false);
  dw_gdma_channel_enable_ctrl(g_csi.dma_chan, false);
  g_csi.started = false;
  return OK;
}
