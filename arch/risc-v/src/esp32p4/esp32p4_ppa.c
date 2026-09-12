/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_ppa.c
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
#include <nuttx/mutex.h>

#include "soc/hp_sys_clkrst_struct.h"
#include "soc/ppa_struct.h"
#include "soc/ppa_reg.h"
#include "soc/dma2d_channel.h"
#include "hal/ppa_types.h"
#include "hal/ppa_ll.h"
#include "hal/ppa_hal.h"
#include "hal/dma2d_types.h"
#include "esp_cache.h"
#include "esp32p4_dma2d.h"
#include "esp32p4_ppa.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PPA_TIMEOUT_US 500000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32p4_ppa_dev_s
{
  mutex_t           lock;
  bool              initialized;
  ppa_hal_context_t hal;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32p4_ppa_dev_s g_ppa_dev =
{
  .lock = NXMUTEX_INITIALIZER,
  .initialized = false,
};

static dma2d_descriptor_t g_tx_desc[2] __attribute__((aligned(8)));
static dma2d_descriptor_t g_rx_desc    __attribute__((aligned(8)));

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_ppa_to_srm_cm
 ****************************************************************************/

static inline ppa_srm_color_mode_t esp32p4_ppa_to_srm_cm(int color_fmt)
{
  switch (color_fmt)
    {
      case ESP32P4_PPA_COLOR_RGB565:
        return PPA_SRM_COLOR_MODE_RGB565;

      case ESP32P4_PPA_COLOR_RGB888:
        return PPA_SRM_COLOR_MODE_RGB888;

      case ESP32P4_PPA_COLOR_ARGB8888:
        return PPA_SRM_COLOR_MODE_ARGB8888;

      case ESP32P4_PPA_COLOR_GRAY8:
        return PPA_SRM_COLOR_MODE_GRAY8;

      case ESP32P4_PPA_COLOR_YUV420:
        return PPA_SRM_COLOR_MODE_YUV420;

      case ESP32P4_PPA_COLOR_YUV422:
      default:
        return PPA_SRM_COLOR_MODE_YUV422_UYVY;
    }
}

/****************************************************************************
 * Name: esp32p4_ppa_to_blend_cm
 ****************************************************************************/

static inline ppa_blend_color_mode_t esp32p4_ppa_to_blend_cm(int color_fmt)
{
  switch (color_fmt)
    {
      case ESP32P4_PPA_COLOR_RGB565:
        return PPA_BLEND_COLOR_MODE_RGB565;

      case ESP32P4_PPA_COLOR_RGB888:
        return PPA_BLEND_COLOR_MODE_RGB888;

      case ESP32P4_PPA_COLOR_ARGB8888:
        return PPA_BLEND_COLOR_MODE_ARGB8888;

      case ESP32P4_PPA_COLOR_GRAY8:
        return PPA_BLEND_COLOR_MODE_GRAY8;

      case ESP32P4_PPA_COLOR_YUV420:
        return PPA_BLEND_COLOR_MODE_YUV420;

      case ESP32P4_PPA_COLOR_YUV422:
      default:
        return PPA_BLEND_COLOR_MODE_YUV422_UYVY;
    }
}

/****************************************************************************
 * Name: esp32p4_ppa_to_fill_cm
 ****************************************************************************/

static inline ppa_fill_color_mode_t esp32p4_ppa_to_fill_cm(int color_fmt)
{
  switch (color_fmt)
    {
      case ESP32P4_PPA_COLOR_RGB565:
        return PPA_FILL_COLOR_MODE_RGB565;

      case ESP32P4_PPA_COLOR_RGB888:
        return PPA_FILL_COLOR_MODE_RGB888;

      case ESP32P4_PPA_COLOR_ARGB8888:
        return PPA_FILL_COLOR_MODE_ARGB8888;

      case ESP32P4_PPA_COLOR_GRAY8:
        return PPA_FILL_COLOR_MODE_GRAY8;

      case ESP32P4_PPA_COLOR_YUV422:
      default:
        return PPA_FILL_COLOR_MODE_YUV422_UYVY;
    }
}

/****************************************************************************
 * Name: esp32p4_ppa_get_pbyte
 ****************************************************************************/

static inline uint32_t esp32p4_ppa_get_pbyte(int color_fmt)
{
  switch (color_fmt)
    {
      case ESP32P4_PPA_COLOR_GRAY8:
        return DMA2D_DESCRIPTOR_PBYTE_1B0_PER_PIXEL;

      case ESP32P4_PPA_COLOR_RGB565:
      case ESP32P4_PPA_COLOR_YUV422:
        return DMA2D_DESCRIPTOR_PBYTE_2B0_PER_PIXEL;

      case ESP32P4_PPA_COLOR_RGB888:
        return DMA2D_DESCRIPTOR_PBYTE_3B0_PER_PIXEL;

      case ESP32P4_PPA_COLOR_ARGB8888:
        return DMA2D_DESCRIPTOR_PBYTE_4B0_PER_PIXEL;

      case ESP32P4_PPA_COLOR_YUV420:
        return DMA2D_DESCRIPTOR_PBYTE_1B5_PER_PIXEL;

      default:
        return DMA2D_DESCRIPTOR_PBYTE_2B0_PER_PIXEL;
    }
}

/****************************************************************************
 * Name: esp32p4_ppa_get_bpp
 ****************************************************************************/

static inline size_t esp32p4_ppa_get_bpp(int color_fmt)
{
  switch (color_fmt)
    {
      case ESP32P4_PPA_COLOR_GRAY8:
        return 1;

      case ESP32P4_PPA_COLOR_RGB565:
      case ESP32P4_PPA_COLOR_YUV422:
        return 2;

      case ESP32P4_PPA_COLOR_RGB888:
        return 3;

      case ESP32P4_PPA_COLOR_ARGB8888:
        return 4;

      case ESP32P4_PPA_COLOR_YUV420:
        return 2;

      default:
        return 2;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_ppa_init
 ****************************************************************************/

int esp32p4_ppa_init(void)
{
  nxmutex_lock(&g_ppa_dev.lock);

  if (g_ppa_dev.initialized)
    {
      nxmutex_unlock(&g_ppa_dev.lock);
      return OK;
    }

  /* Initialize underlying 2D-DMA controller */

  esp32p4_dma2d_init();

  /* Enable PPA bus clock in HP System Clock & Reset controller */

  HP_SYS_CLKRST.soc_clk_ctrl1.reg_ppa_sys_clk_en = 1;

  /* Reset PPA hardware module */

  HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_ppa = 1;
  HP_SYS_CLKRST.hp_rst_en1.reg_rst_en_ppa = 0;

  /* Initialize HAL context */

  ppa_hal_init(&g_ppa_dev.hal);

  g_ppa_dev.initialized = true;

  nxmutex_unlock(&g_ppa_dev.lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_ppa_deinit
 ****************************************************************************/

int esp32p4_ppa_deinit(void)
{
  nxmutex_lock(&g_ppa_dev.lock);

  if (!g_ppa_dev.initialized)
    {
      nxmutex_unlock(&g_ppa_dev.lock);
      return OK;
    }

  /* Disable PPA bus clock to save power */

  HP_SYS_CLKRST.soc_clk_ctrl1.reg_ppa_sys_clk_en = 0;

  ppa_hal_deinit(&g_ppa_dev.hal);

  g_ppa_dev.initialized = false;

  nxmutex_unlock(&g_ppa_dev.lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_ppa_scale
 ****************************************************************************/

int esp32p4_ppa_scale(const void *src, uint16_t src_w, uint16_t src_h,
                      void *dst, uint16_t dst_w, uint16_t dst_h,
                      int color_fmt)
{
  if (src == NULL || dst == NULL || src_w == 0 || src_h == 0 ||
      dst_w == 0 || dst_h == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_ppa_dev.lock);
  if (!g_ppa_dev.initialized)
    {
      esp32p4_ppa_init();
    }

  size_t src_size = (size_t)src_w * src_h * esp32p4_ppa_get_bpp(color_fmt);
  size_t dst_size = (size_t)dst_w * dst_h * esp32p4_ppa_get_bpp(color_fmt);

  /* Flush CPU cache for source and prepare destination */

  esp_cache_msync((void *)src, src_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  /* Prepare 2D-DMA TX descriptor (Input to SRM) */

  memset(&g_tx_desc[0], 0, sizeof(dma2d_descriptor_t));
  g_tx_desc[0].vb_size   = src_h;
  g_tx_desc[0].hb_length = src_w;
  g_tx_desc[0].dma2d_en  = 1;
  g_tx_desc[0].suc_eof   = 1;
  g_tx_desc[0].owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_tx_desc[0].va_size   = src_h;
  g_tx_desc[0].ha_length = src_w;
  g_tx_desc[0].pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_tx_desc[0].mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_tx_desc[0].buffer    = (void *)src;
  g_tx_desc[0].next      = NULL;

  /* Prepare 2D-DMA RX descriptor (Output from SRM) */

  memset(&g_rx_desc, 0, sizeof(dma2d_descriptor_t));
  g_rx_desc.vb_size   = 2;
  g_rx_desc.hb_length = 2;
  g_rx_desc.dma2d_en  = 1;
  g_rx_desc.suc_eof   = 1;
  g_rx_desc.owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_rx_desc.va_size   = dst_h;
  g_rx_desc.ha_length = dst_w;
  g_rx_desc.pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_rx_desc.mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_rx_desc.buffer    = (void *)dst;
  g_rx_desc.next      = NULL;

  esp_cache_msync(&g_tx_desc[0], sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync(&g_rx_desc, sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  /* Configure 2D-DMA Channels */

  esp32p4_dma2d_setup_tx(0, DMA2D_TRIG_PERIPH_PPA_SRM,
                         SOC_DMA2D_TRIG_PERIPH_PPA_SRM_TX,
                         &g_tx_desc[0], DMA2D_DATA_BURST_LENGTH_128);

  esp32p4_dma2d_setup_rx(0, DMA2D_TRIG_PERIPH_PPA_SRM,
                         SOC_DMA2D_TRIG_PERIPH_PPA_SRM_RX,
                         &g_rx_desc, DMA2D_DATA_BURST_LENGTH_128);

  ppa_srm_color_mode_t ppa_cm = esp32p4_ppa_to_srm_cm(color_fmt);
  uint32_t blk_h = 0;
  uint32_t blk_v = 0;
  ppa_ll_srm_get_dma_dscr_port_mode_block_size(&PPA, ppa_cm,
                                               ppa_ll_srm_get_mb_size(&PPA),
                                               &blk_h, &blk_v);
  esp32p4_dma2d_enable_tx_dscr_port(0, blk_h, blk_v);

  esp32p4_dma2d_start_tx(0);
  esp32p4_dma2d_start_rx(0);

  /* Configure PPA SRM hardware engine */

  ppa_ll_srm_reset(&PPA);
  ppa_ll_srm_set_rx_color_mode(&PPA, ppa_cm);
  ppa_ll_srm_set_tx_color_mode(&PPA, ppa_cm);
  ppa_ll_srm_set_rotation_angle(&PPA, PPA_SRM_ROTATION_ANGLE_0);

  uint32_t scale_x_int = (uint32_t)(dst_w / src_w);
  uint32_t scale_x_frag =
    ((uint64_t)(dst_w % src_w) * PPA_LL_SRM_SCALING_FRAG_MAX) / src_w;
  uint32_t scale_y_int = (uint32_t)(dst_h / src_h);
  uint32_t scale_y_frag =
    ((uint64_t)(dst_h % src_h) * PPA_LL_SRM_SCALING_FRAG_MAX) / src_h;

  ppa_ll_srm_set_scaling_x(&PPA, scale_x_int, scale_x_frag);
  ppa_ll_srm_set_scaling_y(&PPA, scale_y_int, scale_y_frag);
  ppa_ll_srm_enable_mirror_x(&PPA, false);
  ppa_ll_srm_enable_mirror_y(&PPA, false);

  /* Hardware bug workaround (DIG-734) */

  uint32_t w_out = src_w * scale_x_int +
                   src_w * scale_x_frag / PPA_LL_SRM_SCALING_FRAG_MAX;
  uint32_t w_divisor = (ppa_cm == PPA_SRM_COLOR_MODE_ARGB8888 ||
                        ppa_cm == PPA_SRM_COLOR_MODE_RGB888) ? 32 : 64;
  uint32_t w_left = w_out % w_divisor;
  w_left = (w_left == 0) ? w_divisor : w_left;
  uint32_t h_mb =
    (ppa_ll_srm_get_mb_size(&PPA) == PPA_LL_SRM_MB_SIZE_16_16) ? 16 : 32;
  uint32_t h_in_left = src_h % h_mb;
  h_in_left = (h_in_left == 0) ? h_mb : h_in_left;
  uint32_t h_left = h_in_left * scale_y_int +
                    h_in_left * scale_y_frag / PPA_LL_SRM_SCALING_FRAG_MAX;
  const uint32_t dma2d_fifo_depth_bits = 12 * 128;
  uint32_t out_pixel_depth = (uint32_t)esp32p4_ppa_get_bpp(color_fmt) * 8;
  bool bypass_mb_order = false;
  if (((w_out > w_divisor) || (src_h > h_mb)) &&
      ((w_left * h_left * out_pixel_depth) < dma2d_fifo_depth_bits))
    {
      bypass_mb_order = true;
    }

  ppa_ll_srm_bypass_mb_order(&PPA, bypass_mb_order);

  /* Trigger PPA SRM hardware acceleration */

  ppa_ll_srm_start(&PPA);

  /* Wait for 2D-DMA transfer completion */

  int ret = esp32p4_dma2d_wait_rx_done(0, PPA_TIMEOUT_US);

  esp32p4_dma2d_stop_tx(0);
  esp32p4_dma2d_stop_rx(0);

  /* Invalidate CPU cache for destination buffer */

  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  nxmutex_unlock(&g_ppa_dev.lock);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_ppa_rotate
 ****************************************************************************/

int esp32p4_ppa_rotate(const void *src, uint16_t w, uint16_t h,
                       void *dst, int rotation, int color_fmt)
{
  if (src == NULL || dst == NULL || w == 0 || h == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_ppa_dev.lock);
  if (!g_ppa_dev.initialized)
    {
      esp32p4_ppa_init();
    }

  uint16_t dst_w = (rotation == ESP32P4_PPA_ROT_90 ||
                    rotation == ESP32P4_PPA_ROT_270) ? h : w;
  uint16_t dst_h = (rotation == ESP32P4_PPA_ROT_90 ||
                    rotation == ESP32P4_PPA_ROT_270) ? w : h;

  size_t src_size = (size_t)w * h * esp32p4_ppa_get_bpp(color_fmt);
  size_t dst_size = (size_t)dst_w * dst_h * esp32p4_ppa_get_bpp(color_fmt);

  esp_cache_msync((void *)src, src_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  /* Prepare 2D-DMA TX descriptor */

  memset(&g_tx_desc[0], 0, sizeof(dma2d_descriptor_t));
  g_tx_desc[0].vb_size   = h;
  g_tx_desc[0].hb_length = w;
  g_tx_desc[0].dma2d_en  = 1;
  g_tx_desc[0].suc_eof   = 1;
  g_tx_desc[0].owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_tx_desc[0].va_size   = h;
  g_tx_desc[0].ha_length = w;
  g_tx_desc[0].pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_tx_desc[0].mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_tx_desc[0].buffer    = (void *)src;
  g_tx_desc[0].next      = NULL;

  /* Prepare 2D-DMA RX descriptor */

  memset(&g_rx_desc, 0, sizeof(dma2d_descriptor_t));
  g_rx_desc.vb_size   = 2;
  g_rx_desc.hb_length = 2;
  g_rx_desc.dma2d_en  = 1;
  g_rx_desc.suc_eof   = 1;
  g_rx_desc.owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_rx_desc.va_size   = dst_h;
  g_rx_desc.ha_length = dst_w;
  g_rx_desc.pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_rx_desc.mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_rx_desc.buffer    = (void *)dst;
  g_rx_desc.next      = NULL;

  esp_cache_msync(&g_tx_desc[0], sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync(&g_rx_desc, sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  esp32p4_dma2d_setup_tx(0, DMA2D_TRIG_PERIPH_PPA_SRM,
                         SOC_DMA2D_TRIG_PERIPH_PPA_SRM_TX,
                         &g_tx_desc[0], DMA2D_DATA_BURST_LENGTH_128);

  esp32p4_dma2d_setup_rx(0, DMA2D_TRIG_PERIPH_PPA_SRM,
                         SOC_DMA2D_TRIG_PERIPH_PPA_SRM_RX,
                         &g_rx_desc, DMA2D_DATA_BURST_LENGTH_128);

  ppa_srm_color_mode_t ppa_cm = esp32p4_ppa_to_srm_cm(color_fmt);
  uint32_t blk_h = 0;
  uint32_t blk_v = 0;
  ppa_ll_srm_get_dma_dscr_port_mode_block_size(&PPA, ppa_cm,
                                               ppa_ll_srm_get_mb_size(&PPA),
                                               &blk_h, &blk_v);
  esp32p4_dma2d_enable_tx_dscr_port(0, blk_h, blk_v);

  esp32p4_dma2d_start_tx(0);
  esp32p4_dma2d_start_rx(0);

  ppa_ll_srm_reset(&PPA);
  ppa_ll_srm_set_rx_color_mode(&PPA, ppa_cm);
  ppa_ll_srm_set_tx_color_mode(&PPA, ppa_cm);
  ppa_ll_srm_set_rotation_angle(&PPA, (ppa_srm_rotation_angle_t)rotation);
  ppa_ll_srm_set_scaling_x(&PPA, 1, 0);
  ppa_ll_srm_set_scaling_y(&PPA, 1, 0);
  ppa_ll_srm_enable_mirror_x(&PPA, false);
  ppa_ll_srm_enable_mirror_y(&PPA, false);

  ppa_ll_srm_start(&PPA);

  int ret = esp32p4_dma2d_wait_rx_done(0, PPA_TIMEOUT_US);

  esp32p4_dma2d_stop_tx(0);
  esp32p4_dma2d_stop_rx(0);

  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  nxmutex_unlock(&g_ppa_dev.lock);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_ppa_blend
 ****************************************************************************/

int esp32p4_ppa_blend(const void *bg, const void *fg, void *dst,
                      uint16_t w, uint16_t h, uint8_t alpha,
                      int color_fmt)
{
  if (bg == NULL || fg == NULL || dst == NULL || w == 0 || h == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_ppa_dev.lock);
  if (!g_ppa_dev.initialized)
    {
      esp32p4_ppa_init();
    }

  size_t img_size = (size_t)w * h * esp32p4_ppa_get_bpp(color_fmt);

  esp_cache_msync((void *)bg, img_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync((void *)fg, img_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync((void *)dst, img_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  /* TX BG descriptor on TX CH1 */

  memset(&g_tx_desc[0], 0, sizeof(dma2d_descriptor_t));
  g_tx_desc[0].vb_size   = h;
  g_tx_desc[0].hb_length = w;
  g_tx_desc[0].dma2d_en  = 1;
  g_tx_desc[0].suc_eof   = 1;
  g_tx_desc[0].owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_tx_desc[0].va_size   = h;
  g_tx_desc[0].ha_length = w;
  g_tx_desc[0].pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_tx_desc[0].mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_tx_desc[0].buffer    = (void *)bg;
  g_tx_desc[0].next      = NULL;

  /* TX FG descriptor on TX CH2 */

  memset(&g_tx_desc[1], 0, sizeof(dma2d_descriptor_t));
  g_tx_desc[1].vb_size   = h;
  g_tx_desc[1].hb_length = w;
  g_tx_desc[1].dma2d_en  = 1;
  g_tx_desc[1].suc_eof   = 1;
  g_tx_desc[1].owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_tx_desc[1].va_size   = h;
  g_tx_desc[1].ha_length = w;
  g_tx_desc[1].pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_tx_desc[1].mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_tx_desc[1].buffer    = (void *)fg;
  g_tx_desc[1].next      = NULL;

  /* RX descriptor on RX CH0 */

  memset(&g_rx_desc, 0, sizeof(dma2d_descriptor_t));
  g_rx_desc.vb_size   = h;
  g_rx_desc.hb_length = w;
  g_rx_desc.dma2d_en  = 1;
  g_rx_desc.suc_eof   = 1;
  g_rx_desc.owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_rx_desc.va_size   = h;
  g_rx_desc.ha_length = w;
  g_rx_desc.pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_rx_desc.mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_rx_desc.buffer    = (void *)dst;
  g_rx_desc.next      = NULL;

  esp_cache_msync(&g_tx_desc[0], sizeof(dma2d_descriptor_t) * 2,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync(&g_rx_desc, sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  /* Setup 2D-DMA Channels: BG on TX1, FG on TX2, Output on RX0 */

  esp32p4_dma2d_setup_tx(1, DMA2D_TRIG_PERIPH_PPA_BLEND,
                         SOC_DMA2D_TRIG_PERIPH_PPA_BLEND_BG_TX,
                         &g_tx_desc[0], DMA2D_DATA_BURST_LENGTH_128);

  esp32p4_dma2d_setup_tx(2, DMA2D_TRIG_PERIPH_PPA_BLEND,
                         SOC_DMA2D_TRIG_PERIPH_PPA_BLEND_FG_TX,
                         &g_tx_desc[1], DMA2D_DATA_BURST_LENGTH_128);

  esp32p4_dma2d_setup_rx(0, DMA2D_TRIG_PERIPH_PPA_BLEND,
                         SOC_DMA2D_TRIG_PERIPH_PPA_BLEND_RX,
                         &g_rx_desc, DMA2D_DATA_BURST_LENGTH_128);

  esp32p4_dma2d_start_tx(1);
  esp32p4_dma2d_start_tx(2);
  esp32p4_dma2d_start_rx(0);

  /* Configure PPA Blend Engine */

  ppa_blend_color_mode_t ppa_bcm = esp32p4_ppa_to_blend_cm(color_fmt);
  ppa_ll_blend_reset(&PPA);
  ppa_ll_blend_set_rx_bg_color_mode(&PPA, ppa_bcm);
  ppa_ll_blend_configure_rx_bg_alpha(&PPA, PPA_ALPHA_FIX_VALUE,
                                     255 - alpha);
  ppa_ll_blend_set_rx_fg_color_mode(&PPA, ppa_bcm);
  ppa_ll_blend_configure_rx_fg_alpha(&PPA, PPA_ALPHA_FIX_VALUE, alpha);
  ppa_ll_blend_set_tx_color_mode(&PPA, ppa_bcm);

  ppa_ll_blend_start(&PPA, PPA_LL_BLEND_TRANS_MODE_BLEND);

  int ret = esp32p4_dma2d_wait_rx_done(0, PPA_TIMEOUT_US);

  esp32p4_dma2d_stop_tx(1);
  esp32p4_dma2d_stop_tx(2);
  esp32p4_dma2d_stop_rx(0);

  esp_cache_msync((void *)dst, img_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  nxmutex_unlock(&g_ppa_dev.lock);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_ppa_fill
 ****************************************************************************/

int esp32p4_ppa_fill(void *dst, uint16_t w, uint16_t h,
                     uint32_t color, int color_fmt)
{
  if (dst == NULL || w == 0 || h == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_ppa_dev.lock);
  if (!g_ppa_dev.initialized)
    {
      esp32p4_ppa_init();
    }

  size_t dst_size = (size_t)w * h * esp32p4_ppa_get_bpp(color_fmt);

  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  /* RX descriptor for destination */

  memset(&g_rx_desc, 0, sizeof(dma2d_descriptor_t));
  g_rx_desc.vb_size   = h;
  g_rx_desc.hb_length = w;
  g_rx_desc.dma2d_en  = 1;
  g_rx_desc.suc_eof   = 1;
  g_rx_desc.owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_rx_desc.va_size   = h;
  g_rx_desc.ha_length = w;
  g_rx_desc.pbyte     = esp32p4_ppa_get_pbyte(color_fmt);
  g_rx_desc.mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_rx_desc.buffer    = (void *)dst;
  g_rx_desc.next      = NULL;

  esp_cache_msync(&g_rx_desc, sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  esp32p4_dma2d_setup_rx(0, DMA2D_TRIG_PERIPH_PPA_BLEND,
                         SOC_DMA2D_TRIG_PERIPH_PPA_BLEND_RX,
                         &g_rx_desc, DMA2D_DATA_BURST_LENGTH_128);

  esp32p4_dma2d_start_rx(0);

  /* Configure PPA Blend Engine in Fill Mode */

  ppa_fill_color_mode_t ppa_fcm = esp32p4_ppa_to_fill_cm(color_fmt);
  ppa_ll_blend_reset(&PPA);
  ppa_ll_blend_configure_filling_block(&PPA, ppa_fcm, (void *)&color, w, h);
  ppa_ll_blend_set_tx_color_mode(&PPA, (ppa_blend_color_mode_t)ppa_fcm);

  ppa_ll_blend_start(&PPA, PPA_LL_BLEND_TRANS_MODE_FILL);

  int ret = esp32p4_dma2d_wait_rx_done(0, PPA_TIMEOUT_US);

  esp32p4_dma2d_stop_rx(0);

  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  nxmutex_unlock(&g_ppa_dev.lock);
  return ret;
}

/****************************************************************************
 * Name: esp32p4_ppa_csc
 ****************************************************************************/

int esp32p4_ppa_csc(const void *src, void *dst, uint16_t w, uint16_t h,
                    int src_fmt, int dst_fmt)
{
  if (src == NULL || dst == NULL || w == 0 || h == 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_ppa_dev.lock);
  if (!g_ppa_dev.initialized)
    {
      esp32p4_ppa_init();
    }

  size_t src_size = (size_t)w * h * esp32p4_ppa_get_bpp(src_fmt);
  size_t dst_size = (size_t)w * h * esp32p4_ppa_get_bpp(dst_fmt);

  esp_cache_msync((void *)src, src_size, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  /* TX descriptor on TX CH0 */

  memset(&g_tx_desc[0], 0, sizeof(dma2d_descriptor_t));
  g_tx_desc[0].vb_size   = h;
  g_tx_desc[0].hb_length = w;
  g_tx_desc[0].dma2d_en  = 1;
  g_tx_desc[0].suc_eof   = 1;
  g_tx_desc[0].owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_tx_desc[0].va_size   = h;
  g_tx_desc[0].ha_length = w;
  g_tx_desc[0].pbyte     = esp32p4_ppa_get_pbyte(src_fmt);
  g_tx_desc[0].mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_tx_desc[0].buffer    = (void *)src;
  g_tx_desc[0].next      = NULL;

  /* RX descriptor on RX CH0 */

  memset(&g_rx_desc, 0, sizeof(dma2d_descriptor_t));
  g_rx_desc.vb_size   = 2;
  g_rx_desc.hb_length = 2;
  g_rx_desc.dma2d_en  = 1;
  g_rx_desc.suc_eof   = 1;
  g_rx_desc.owner     = DMA2D_DESCRIPTOR_BUFFER_OWNER_DMA;
  g_rx_desc.va_size   = h;
  g_rx_desc.ha_length = w;
  g_rx_desc.pbyte     = esp32p4_ppa_get_pbyte(dst_fmt);
  g_rx_desc.mode      = DMA2D_DESCRIPTOR_BLOCK_RW_MODE_SINGLE;
  g_rx_desc.buffer    = (void *)dst;
  g_rx_desc.next      = NULL;

  esp_cache_msync(&g_tx_desc[0], sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  esp_cache_msync(&g_rx_desc, sizeof(dma2d_descriptor_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M);

  esp32p4_dma2d_setup_tx(0, DMA2D_TRIG_PERIPH_PPA_SRM,
                         SOC_DMA2D_TRIG_PERIPH_PPA_SRM_TX,
                         &g_tx_desc[0], DMA2D_DATA_BURST_LENGTH_128);

  esp32p4_dma2d_setup_rx(0, DMA2D_TRIG_PERIPH_PPA_SRM,
                         SOC_DMA2D_TRIG_PERIPH_PPA_SRM_RX,
                         &g_rx_desc, DMA2D_DATA_BURST_LENGTH_128);

  ppa_srm_color_mode_t in_cm  = esp32p4_ppa_to_srm_cm(src_fmt);
  ppa_srm_color_mode_t out_cm = esp32p4_ppa_to_srm_cm(dst_fmt);

  uint32_t blk_h = 0;
  uint32_t blk_v = 0;
  ppa_ll_srm_get_dma_dscr_port_mode_block_size(&PPA, in_cm,
                                               ppa_ll_srm_get_mb_size(&PPA),
                                               &blk_h, &blk_v);
  esp32p4_dma2d_enable_tx_dscr_port(0, blk_h, blk_v);

  esp32p4_dma2d_start_tx(0);
  esp32p4_dma2d_start_rx(0);

  ppa_ll_srm_reset(&PPA);
  ppa_ll_srm_set_rx_color_mode(&PPA, in_cm);
  ppa_ll_srm_set_tx_color_mode(&PPA, out_cm);
  ppa_ll_srm_set_rotation_angle(&PPA, PPA_SRM_ROTATION_ANGLE_0);
  ppa_ll_srm_set_scaling_x(&PPA, 1, 0);
  ppa_ll_srm_set_scaling_y(&PPA, 1, 0);
  ppa_ll_srm_enable_mirror_x(&PPA, false);
  ppa_ll_srm_enable_mirror_y(&PPA, false);

  ppa_ll_srm_start(&PPA);

  int ret = esp32p4_dma2d_wait_rx_done(0, PPA_TIMEOUT_US);

  esp32p4_dma2d_stop_tx(0);
  esp32p4_dma2d_stop_rx(0);

  esp_cache_msync((void *)dst, dst_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  nxmutex_unlock(&g_ppa_dev.lock);
  return ret;
}
