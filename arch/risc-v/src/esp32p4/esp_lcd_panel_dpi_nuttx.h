/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_lcd_panel_dpi_nuttx.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_LCD_PANEL_DPI_NUTTX_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_LCD_PANEL_DPI_NUTTX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include "esp_lcd_dsi_bus_nuttx.h"
#include "esp_dw_gdma_nuttx.h"

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Video timing */

typedef struct
{
  uint32_t h_size;
  uint32_t v_size;
  uint32_t hsync_pulse_width;
  uint32_t hsync_back_porch;
  uint32_t hsync_front_porch;
  uint32_t vsync_pulse_width;
  uint32_t vsync_back_porch;
  uint32_t vsync_front_porch;
} esp_lcd_video_timing_t;

/* DPI panel configuration */

typedef struct
{
  int virtual_channel;
  float dpi_clock_freq_mhz;
  esp_lcd_video_timing_t video_timing;
} esp_lcd_dpi_panel_config_t;

/* Opaque panel handle */

typedef struct esp_lcd_dpi_panel_t *esp_lcd_dpi_panel_handle_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp_lcd_new_panel_dpi
 *
 * Description:
 *   Create a new DPI panel instance. Allocates framebuffer, configures
 *   DPI clock, Host DPI parameters, Bridge, and DMA link list.
 *
 * Input Parameters:
 *   bus       - DSI bus handle (already initialized)
 *   config    - DPI panel configuration (timing, clock, virtual channel)
 *   ret_panel - Returned panel handle on success
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int esp_lcd_new_panel_dpi(esp_lcd_dsi_bus_handle_t bus,
                          const esp_lcd_dpi_panel_config_t *config,
                          esp_lcd_dpi_panel_handle_t *ret_panel);

/****************************************************************************
 * Name: esp_lcd_dpi_panel_init
 *
 * Description:
 *   Initialize and start the DPI panel refresh loop.
 *   Critical sequence: start DMA → enable video mode → enable DPI output.
 *
 * Input Parameters:
 *   panel - Panel handle created by esp_lcd_new_panel_dpi
 *
 * Returned Value:
 *   0 on success, negative errno on failure.
 ****************************************************************************/

int esp_lcd_dpi_panel_init(esp_lcd_dpi_panel_handle_t panel);

/****************************************************************************
 * Name: esp_lcd_dpi_panel_get_fb
 *
 * Description:
 *   Get pointer to the framebuffer for direct pixel writes.
 *
 * Input Parameters:
 *   panel - Panel handle
 *
 * Returned Value:
 *   Pointer to framebuffer, or NULL if panel is invalid.
 ****************************************************************************/

uint8_t *esp_lcd_dpi_panel_get_fb(esp_lcd_dpi_panel_handle_t panel);

/****************************************************************************
 * Name: esp_lcd_dpi_panel_get_fb_size
 *
 * Description:
 *   Get framebuffer size in bytes.
 *
 * Input Parameters:
 *   panel - Panel handle
 *
 * Returned Value:
 *   Framebuffer size in bytes, or 0 if panel is invalid.
 ****************************************************************************/

size_t esp_lcd_dpi_panel_get_fb_size(esp_lcd_dpi_panel_handle_t panel);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_LCD_PANEL_DPI_NUTTX_H */
