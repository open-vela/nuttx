/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_lcd_dsi_bus_nuttx.c
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

#include "esp_idf_shim.h"
#include "esp_lcd_dsi_bus_nuttx.h"

#include "hal/mipi_dsi_hal.h"
#include "hal/mipi_dsi_ll.h"
#include "hal/mipi_dsi_host_ll.h"
#include "hal/mipi_dsi_phy_ll.h"
#include "hal/mipi_dsi_brg_ll.h"
#include "esp_private/periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIPI_DSI_DEFAULT_TIMEOUT_CLOCK_FREQ_MHZ  10
#define MIPI_DSI_DEFAULT_ESCAPE_CLOCK_FREQ_MHZ   18
#define DSI_BUS_ID  0

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Static bus instance — ESP32-P4 has only one DSI bus */

static esp_lcd_dsi_bus_t g_dsi_bus;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp_lcd_new_dsi_bus
 *
 * Description:
 *   Initialize the MIPI-DSI bus following the exact ESP-IDF sequence:
 *   PHY power-up, PLL configuration, Host setup.
 *
 *   The initialization order is critical — the PHY has internal state
 *   machines that require precise timing. Do NOT reorder calls.
 ****************************************************************************/

int esp_lcd_new_dsi_bus(const esp_lcd_dsi_bus_config_t *config,
                        esp_lcd_dsi_bus_handle_t *ret_bus)
{
  esp_lcd_dsi_bus_t *bus;
  int bus_id;
  int num_data_lanes;
  mipi_dsi_hal_config_t hal_config;
  uint32_t phy_clk_src_freq_hz;
  mipi_dsi_phy_pllref_clock_source_t phy_clk_src;

  /* Step 1: Validate arguments */

  if (!config || !ret_bus)
    {
      return -EINVAL;
    }

  if (config->lane_bit_rate_mbps < MIPI_DSI_LL_MIN_PHY_MBPS ||
      config->lane_bit_rate_mbps > MIPI_DSI_LL_MAX_PHY_MBPS)
    {
      syslog(LOG_ERR, "[DSI-BUS] Invalid lane bit rate: %.2f\n",
             config->lane_bit_rate_mbps);
      return -EINVAL;
    }

  /* Step 2: Allocate bus struct (static, single instance) */

  bus_id = config->bus_id;
  bus = &g_dsi_bus;
  memset(bus, 0, sizeof(esp_lcd_dsi_bus_t));
  bus->bus_id = bus_id;

  /* Step 3: Enable APB clock for DSI host and bridge registers */

  PERIPH_RCC_ATOMIC()
    {
      mipi_dsi_ll_enable_bus_clock(bus_id, true);
      mipi_dsi_ll_reset_register(bus_id);
    }

  /* Step 4: Determine PHY clock source */

  phy_clk_src = (mipi_dsi_phy_pllref_clock_source_t)config->phy_clk_src;
  if (phy_clk_src == 0)
    {
      phy_clk_src = MIPI_DSI_PHY_PLLREF_CLK_SRC_DEFAULT;
    }

  /* Step 5: Enable PHY PLL reference clock source (no-op via shim) */

  esp_clk_tree_enable_src((soc_module_clk_t)phy_clk_src, true);

  /* Step 6: Enable PHY config clock source (no-op via shim) */

  esp_clk_tree_enable_src((soc_module_clk_t)MIPI_DSI_PHY_CFG_CLK_SRC_DEFAULT,
                          true);

  /* Step 7: Configure clock sources in PERIPH_RCC_ATOMIC block */

  PERIPH_RCC_ATOMIC()
    {
      mipi_dsi_ll_set_phy_config_clock_source(bus_id,
                                              MIPI_DSI_PHY_CFG_CLK_SRC_DEFAULT);
      mipi_dsi_ll_enable_phy_config_clock(bus_id, true);
      mipi_dsi_ll_set_phy_pllref_clock_source(bus_id, phy_clk_src);
      mipi_dsi_ll_set_phy_pll_ref_clock_div(bus_id, 1);
      mipi_dsi_ll_enable_phy_pllref_clock(bus_id, true);
    }

  /* Step 8: Determine number of data lanes */

  num_data_lanes = config->num_data_lanes;
  if (num_data_lanes == 0)
    {
      num_data_lanes = MIPI_DSI_LL_MAX_DATA_LANES;
    }

  /* Step 9: Initialize HAL context */

  hal_config.bus_id = bus_id;
  hal_config.lane_bit_rate_mbps = config->lane_bit_rate_mbps;
  hal_config.num_data_lanes = num_data_lanes;

  /* Step 10: mipi_dsi_hal_init */

  mipi_dsi_hal_init(&bus->hal, &hal_config);

  /* Step 11: Get PHY clock source frequency (XTAL 40MHz on ESP32-P4) */

  phy_clk_src_freq_hz = 40000000;

  /* Step 12: Configure PHY PLL */

  mipi_dsi_hal_configure_phy_pll(&bus->hal, phy_clk_src_freq_hz,
                                 config->lane_bit_rate_mbps);

  /* Step 13: Wait for PLL lock */

  while (!mipi_dsi_phy_ll_is_pll_locked(bus->hal.host))
    {
      usleep(1000);
    }

  syslog(LOG_INFO, "[DSI-BUS] PLL locked\n");

  /* Step 14: Wait for lanes in stop state */

  while (!mipi_dsi_phy_ll_are_lanes_stopped(bus->hal.host, num_data_lanes))
    {
      usleep(1000);
    }

  syslog(LOG_INFO, "[DSI-BUS] Lanes in stop state\n");

  /* Step 15: Start in command mode */

  mipi_dsi_host_ll_enable_video_mode(bus->hal.host, false);

  /* Step 16: Set clock lane state */

  mipi_dsi_host_ll_set_clock_lane_state(bus->hal.host,
      config->flags.clock_lane_force_hs ?
        MIPI_DSI_LL_CLOCK_LANE_STATE_HS :
        MIPI_DSI_LL_CLOCK_LANE_STATE_AUTO);

  /* Step 17: Set HS/LP switch times */

  mipi_dsi_phy_ll_set_switch_time(bus->hal.host, 50, 104, 46, 128);

  /* Step 18: Enable CRC reception */

  mipi_dsi_host_ll_enable_rx_crc(bus->hal.host, true);

  /* Step 19: Enable ECC reception */

  mipi_dsi_host_ll_enable_rx_ecc(bus->hal.host, true);

  /* Step 20: Enable EoTp for HS mode */

  mipi_dsi_host_ll_enable_tx_eotp(bus->hal.host, true, false);

  /* Step 21: Timeout clock division */

  mipi_dsi_host_ll_set_timeout_clock_division(bus->hal.host,
      (uint32_t)roundf(config->lane_bit_rate_mbps / 8.0f /
                       MIPI_DSI_DEFAULT_TIMEOUT_CLOCK_FREQ_MHZ));

  /* Step 22: Escape clock division */

  mipi_dsi_host_ll_set_escape_clock_division(bus->hal.host,
      (uint32_t)roundf(config->lane_bit_rate_mbps / 8.0f /
                       MIPI_DSI_DEFAULT_ESCAPE_CLOCK_FREQ_MHZ));

  /* Step 23: Disable timeout mechanism */

  mipi_dsi_host_ll_set_timeout_count(bus->hal.host, 0, 0, 0, 0, 0, 0, 0);

  /* Step 24: Max read time */

  mipi_dsi_phy_ll_set_max_read_time(bus->hal.host, 6000);

  /* Step 25: Stop wait time */

  mipi_dsi_phy_ll_set_stop_wait_time(bus->hal.host, 0x3F);

  *ret_bus = bus;
  syslog(LOG_INFO, "[DSI-BUS] Init complete: %d lanes, %.0f Mbps\n",
         num_data_lanes, config->lane_bit_rate_mbps);
  return 0;
}

/****************************************************************************
 * Name: esp_lcd_del_dsi_bus
 *
 * Description:
 *   Delete DSI bus — disable clocks and release resources.
 ****************************************************************************/

int esp_lcd_del_dsi_bus(esp_lcd_dsi_bus_handle_t bus)
{
  int bus_id;

  if (!bus)
    {
      return -EINVAL;
    }

  bus_id = bus->bus_id;

  /* Disable PHY clocks */

  PERIPH_RCC_ATOMIC()
    {
      mipi_dsi_ll_enable_phy_pllref_clock(bus_id, false);
      mipi_dsi_ll_enable_phy_config_clock(bus_id, false);
    }

  /* Disable APB bus clock */

  PERIPH_RCC_ATOMIC()
    {
      mipi_dsi_ll_enable_bus_clock(bus_id, false);
    }

  /* Clear static instance */

  memset(bus, 0, sizeof(esp_lcd_dsi_bus_t));

  syslog(LOG_INFO, "[DSI-BUS] Bus deleted\n");
  return 0;
}
