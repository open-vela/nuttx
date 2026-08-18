/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_lcd_dsi_bus_nuttx.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_LCD_DSI_BUS_NUTTX_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_LCD_DSI_BUS_NUTTX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hal/mipi_dsi_hal.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DSI bus context */

typedef struct
{
  int bus_id;
  mipi_dsi_hal_context_t hal;
} esp_lcd_dsi_bus_t;

typedef esp_lcd_dsi_bus_t *esp_lcd_dsi_bus_handle_t;

/* Bus configuration */

typedef struct
{
  int bus_id;
  int num_data_lanes;
  float lane_bit_rate_mbps;
  int phy_clk_src;  /* 0 = default (XTAL 40MHz) */
  struct
  {
    bool clock_lane_force_hs;
  } flags;
} esp_lcd_dsi_bus_config_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp_lcd_new_dsi_bus
 *
 * Description:
 *   Initialize the MIPI-DSI bus (PHY power-up, PLL config, Host setup).
 *   Follows the exact ESP-IDF initialization sequence.
 *
 * Input Parameters:
 *   config  - Bus configuration (lanes, bit rate, clock source)
 *   ret_bus - Returned bus handle on success
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int esp_lcd_new_dsi_bus(const esp_lcd_dsi_bus_config_t *config,
                        esp_lcd_dsi_bus_handle_t *ret_bus);

/****************************************************************************
 * Name: esp_lcd_del_dsi_bus
 *
 * Description:
 *   Delete DSI bus, disable clocks, free resources.
 *
 * Input Parameters:
 *   bus - Bus handle previously created by esp_lcd_new_dsi_bus
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int esp_lcd_del_dsi_bus(esp_lcd_dsi_bus_handle_t bus);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_LCD_DSI_BUS_NUTTX_H */
