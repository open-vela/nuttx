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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>

#include "esp32p4_mipi_csi.h"
#include "esp_private/periph_ctrl.h"
#include "hal/mipi_csi_hal.h"
#include "hal/mipi_csi_ll.h"
#include "hal/mipi_csi_periph.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32P4_CSI_DEFAULT_AFULL_THRD 960

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct esp32p4_csi_dev_s
{
  mutex_t                          lock;
  bool                             initialized;
  bool                             enabled;
  mipi_csi_hal_context_t           hal;
  struct esp32p4_mipi_csi_config_s config;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct esp32p4_csi_dev_s g_csi_dev =
{
  .lock = NXMUTEX_INITIALIZER,
  .initialized = false,
  .enabled = false,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_mipi_csi_init
 ****************************************************************************/

int esp32p4_mipi_csi_init(const struct esp32p4_mipi_csi_config_s *config)
{
  struct esp32p4_csi_dev_s *priv = &g_csi_dev;
  mipi_csi_hal_config_t hal_cfg;
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

  /* 1. Enable module clocks and release resets */

  PERIPH_RCC_ATOMIC()
    {
      mipi_csi_ll_set_phy_clock_source(0, MIPI_CSI_PHY_CLK_SRC_PLL_F20M);
      mipi_csi_ll_enable_phy_config_clock(0, true);
      mipi_csi_ll_enable_host_bus_clock(0, true);
      mipi_csi_ll_reset_host_clock(0);
      mipi_csi_ll_enable_brg_module_clock(0, true);
      mipi_csi_ll_reset_brg_module_clock(0);
      mipi_csi_brg_ll_enable_clock(MIPI_CSI_BRG_LL_GET_HW(0), true);
    }

  /* 2. Configure HAL layer (PHY PLL, Host controller, and Bridge) */

  hal_cfg.lanes_num          = priv->config.lanes_num;
  hal_cfg.frame_width        = priv->config.frame_width;
  hal_cfg.frame_height       = priv->config.frame_height;
  hal_cfg.in_bpp             = priv->config.in_bpp;
  hal_cfg.out_bpp            = priv->config.out_bpp;
  hal_cfg.byte_swap_en       = priv->config.byte_swap_en;
  hal_cfg.lane_bit_rate_mbps = priv->config.lane_bit_rate_mbps;

  mipi_csi_hal_init(&priv->hal, &hal_cfg);

  /* 3. Configure CSI Bridge data type filtering and threshold */

  if (priv->config.data_type != 0)
    {
      mipi_csi_brg_ll_set_data_type_min(priv->hal.bridge_dev,
                                        priv->config.data_type);
      mipi_csi_brg_ll_set_data_type_max(priv->hal.bridge_dev,
                                        priv->config.data_type);
    }
  else
    {
      mipi_csi_brg_ll_set_data_type_min(priv->hal.bridge_dev, 0x12);
      mipi_csi_brg_ll_set_data_type_max(priv->hal.bridge_dev, 0x2f);
    }

  mipi_csi_brg_ll_set_flow_ctl_buf_afull_thrd(priv->hal.bridge_dev,
                                             ESP32P4_CSI_DEFAULT_AFULL_THRD);
  mipi_csi_brg_ll_set_output_byte_endian(priv->hal.bridge_dev,
                                         priv->config.byte_swap_en);

  /* 4. Enable CSI bridge */

  mipi_csi_brg_ll_enable(priv->hal.bridge_dev, true);

  priv->initialized = true;
  priv->enabled     = true;

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

int esp32p4_mipi_csi_get_status(struct esp32p4_mipi_csi_status_s *status)
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

      priv->initialized = false;
      priv->enabled     = false;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}
