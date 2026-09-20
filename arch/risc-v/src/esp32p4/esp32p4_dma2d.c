/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_dma2d.c
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
#include "soc/dma2d_struct.h"
#include "soc/dma2d_reg.h"
#include "soc/dma2d_channel.h"
#include "hal/dma2d_ll.h"
#include "hal/dma2d_hal.h"
#include "esp32p4_dma2d.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32p4_dma2d_priv_s
{
  mutex_t           lock;
  bool              initialized;
  dma2d_hal_context_t hal;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32p4_dma2d_priv_s g_dma2d_priv =
{
  .lock = NXMUTEX_INITIALIZER,
  .initialized = false,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_dma2d_init
 ****************************************************************************/

int esp32p4_dma2d_init(void)
{
  nxmutex_lock(&g_dma2d_priv.lock);

  if (g_dma2d_priv.initialized)
    {
      nxmutex_unlock(&g_dma2d_priv.lock);
      return OK;
    }

  /* Enable 2D-DMA bus clock in HP System Clock & Reset controller */

  HP_SYS_CLKRST.soc_clk_ctrl1.reg_dma2d_sys_clk_en = 1;

  /* Reset 2D-DMA hardware module */

  HP_SYS_CLKRST.hp_rst_en0.reg_rst_en_dma2d = 1;
  HP_SYS_CLKRST.hp_rst_en0.reg_rst_en_dma2d = 0;

  /* Enable 2D-DMA controller clock and reset AXI FIFOs */

  dma2d_ll_hw_enable(&DMA2D, true);

  /* Initialize HAL context */

  dma2d_hal_init(&g_dma2d_priv.hal, 0);

  g_dma2d_priv.initialized = true;

  nxmutex_unlock(&g_dma2d_priv.lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_deinit
 ****************************************************************************/

int esp32p4_dma2d_deinit(void)
{
  nxmutex_lock(&g_dma2d_priv.lock);

  if (!g_dma2d_priv.initialized)
    {
      nxmutex_unlock(&g_dma2d_priv.lock);
      return OK;
    }

  /* Disable 2D-DMA bus clock */

  HP_SYS_CLKRST.soc_clk_ctrl1.reg_dma2d_sys_clk_en = 0;

  g_dma2d_priv.initialized = false;

  nxmutex_unlock(&g_dma2d_priv.lock);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_setup_tx
 ****************************************************************************/

int esp32p4_dma2d_setup_tx(uint32_t channel,
                           dma2d_trigger_peripheral_t periph,
                           int periph_sel_id,
                           dma2d_descriptor_t *desc,
                           dma2d_data_burst_length_t burst_len)
{
  if (channel >= ESP32P4_DMA2D_TX_CH_MAX || desc == NULL)
    {
      return -EINVAL;
    }

  if (!g_dma2d_priv.initialized)
    {
      esp32p4_dma2d_init();
    }

  /* Abort and reset TX channel FSM and FIFO */

  dma2d_hal_tx_reset_channel(&g_dma2d_priv.hal, channel);

  /* Connect TX channel to target peripheral */

  dma2d_ll_tx_connect_to_periph(&DMA2D, channel, periph, periph_sel_id);

  /* Set transfer parameters */

  dma2d_ll_tx_set_data_burst_length(&DMA2D, channel, burst_len);
  dma2d_ll_tx_enable_descriptor_burst(&DMA2D, channel, true);
  dma2d_ll_tx_set_macro_block_size(&DMA2D, channel,
                                   DMA2D_MACRO_BLOCK_SIZE_NONE);

  /* Set TX descriptor address */

  dma2d_ll_tx_set_desc_addr(&DMA2D, channel, (uint32_t)(uintptr_t)desc);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_setup_rx
 ****************************************************************************/

int esp32p4_dma2d_setup_rx(uint32_t channel,
                           dma2d_trigger_peripheral_t periph,
                           int periph_sel_id,
                           dma2d_descriptor_t *desc,
                           dma2d_data_burst_length_t burst_len)
{
  if (channel >= ESP32P4_DMA2D_RX_CH_MAX || desc == NULL)
    {
      return -EINVAL;
    }

  if (!g_dma2d_priv.initialized)
    {
      esp32p4_dma2d_init();
    }

  /* Abort and reset RX channel FSM and FIFO */

  dma2d_hal_rx_reset_channel(&g_dma2d_priv.hal, channel);

  /* Connect RX channel to source peripheral */

  dma2d_ll_rx_connect_to_periph(&DMA2D, channel, periph, periph_sel_id);

  /* Set transfer parameters */

  dma2d_ll_rx_set_data_burst_length(&DMA2D, channel, burst_len);
  dma2d_ll_rx_enable_descriptor_burst(&DMA2D, channel, true);
  dma2d_ll_rx_set_macro_block_size(&DMA2D, channel,
                                   DMA2D_MACRO_BLOCK_SIZE_NONE);

  /* Set RX descriptor address */

  dma2d_ll_rx_set_desc_addr(&DMA2D, channel, (uint32_t)(uintptr_t)desc);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_enable_tx_dscr_port
 ****************************************************************************/

int esp32p4_dma2d_enable_tx_dscr_port(uint32_t channel,
                                      uint32_t blk_h,
                                      uint32_t blk_v)
{
  if (channel >= ESP32P4_DMA2D_TX_CH_MAX)
    {
      return -EINVAL;
    }

  dma2d_ll_tx_enable_dscr_port(&DMA2D, channel, true);
  dma2d_ll_tx_set_dscr_port_block_size(&DMA2D, channel, blk_h, blk_v);

  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_start_tx
 ****************************************************************************/

int esp32p4_dma2d_start_tx(uint32_t channel)
{
  if (channel >= ESP32P4_DMA2D_TX_CH_MAX)
    {
      return -EINVAL;
    }

  dma2d_ll_tx_start(&DMA2D, channel);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_start_rx
 ****************************************************************************/

int esp32p4_dma2d_start_rx(uint32_t channel)
{
  if (channel >= ESP32P4_DMA2D_RX_CH_MAX)
    {
      return -EINVAL;
    }

  dma2d_ll_rx_clear_interrupt_status(&DMA2D, channel,
                                     DMA2D_LL_RX_EVENT_MASK);
  dma2d_ll_rx_start(&DMA2D, channel);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_stop_tx
 ****************************************************************************/

int esp32p4_dma2d_stop_tx(uint32_t channel)
{
  if (channel >= ESP32P4_DMA2D_TX_CH_MAX)
    {
      return -EINVAL;
    }

  dma2d_ll_tx_stop(&DMA2D, channel);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_stop_rx
 ****************************************************************************/

int esp32p4_dma2d_stop_rx(uint32_t channel)
{
  if (channel >= ESP32P4_DMA2D_RX_CH_MAX)
    {
      return -EINVAL;
    }

  dma2d_ll_rx_stop(&DMA2D, channel);
  return OK;
}

/****************************************************************************
 * Name: esp32p4_dma2d_wait_rx_done
 ****************************************************************************/

int esp32p4_dma2d_wait_rx_done(uint32_t channel, uint32_t timeout_us)
{
  if (channel >= ESP32P4_DMA2D_RX_CH_MAX)
    {
      return -EINVAL;
    }

  uint32_t elapsed = 0;

  while (elapsed < timeout_us)
    {
      uint32_t st = dma2d_ll_rx_get_interrupt_status(&DMA2D, channel);

      if ((st & (DMA2D_LL_EVENT_RX_SUC_EOF | DMA2D_LL_EVENT_RX_DONE)) != 0)
        {
          dma2d_ll_rx_clear_interrupt_status(&DMA2D, channel,
                                             DMA2D_LL_RX_EVENT_MASK);
          return OK;
        }

      if ((st & (DMA2D_LL_EVENT_RX_DESC_ERROR |
                 DMA2D_LL_EVENT_RX_ERR_EOF)) != 0)
        {
          dma2d_ll_rx_clear_interrupt_status(&DMA2D, channel,
                                             DMA2D_LL_RX_EVENT_MASK);
          dmainfo("ERROR: 2D-DMA RX channel %" PRIu32
                  " error st=0x%08" PRIx32 "\n",
                  channel, st);
          return -EIO;
        }

      up_udelay(1);
      elapsed++;
    }

  dmainfo("ERROR: 2D-DMA RX channel %" PRIu32 " timed out\n", channel);
  dma2d_ll_rx_stop(&DMA2D, channel);
  return -ETIMEDOUT;
}
