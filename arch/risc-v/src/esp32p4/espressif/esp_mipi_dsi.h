/****************************************************************************
 * arch/risc-v/src/esp32p4/espressif/esp_mipi_dsi.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_MIPI_DSI_H
#define __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_MIPI_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stddef.h>

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* DPI video timing for an RGB565 panel driven in MIPI-DSI video mode */

struct esp_mipi_dsi_video_s
{
  uint16_t h_size;      /* Horizontal active pixels */
  uint16_t v_size;      /* Vertical active lines */
  uint16_t hsw;         /* HSYNC pulse width, in pixels */
  uint16_t hbp;         /* Horizontal back porch, in pixels */
  uint16_t hfp;         /* Horizontal front porch, in pixels */
  uint16_t vsw;         /* VSYNC pulse width, in lines */
  uint16_t vbp;         /* Vertical back porch, in lines */
  uint16_t vfp;         /* Vertical front porch, in lines */
  uint32_t dpi_clk_mhz; /* DPI pixel clock, in MHz */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_mipi_dsi_init
 *
 * Description:
 *   Power the MIPI DPHY, enable the DSI host/bridge clocks, configure the
 *   PHY PLL for the requested lane bit rate and leave the DSI host in
 *   command mode, ready for DCS panel initialization commands.
 *
 * Input Parameters:
 *   num_lanes     - Number of DSI data lanes (1 or 2)
 *   lane_rate_mbps- Lane bit rate in Mbps
 *   phy_ldo_chan  - On-chip LDO channel powering VDD_MIPI_DPHY (-1 = skip)
 *   phy_ldo_mv    - LDO voltage in millivolts (e.g. 2500)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp_mipi_dsi_init(uint8_t num_lanes, uint32_t lane_rate_mbps,
                      int phy_ldo_chan, uint32_t phy_ldo_mv);

/****************************************************************************
 * Name: esp_mipi_dsi_dcs_write
 *
 * Description:
 *   Send a DCS command (with optional parameters) to the panel in
 *   low-power mode.  Only valid after esp_mipi_dsi_init().
 *
 ****************************************************************************/

int esp_mipi_dsi_dcs_write(uint8_t cmd, const void *param, size_t len);

/****************************************************************************
 * Name: esp_mipi_dsi_dcs_read
 *
 * Description:
 *   Read a DCS register from the panel with a bounded wait.  Only valid
 *   between esp_mipi_dsi_init() and esp_mipi_dsi_dpi_start() (the host
 *   must be in command mode).  A successful read proves that a powered
 *   panel is attached and responding on the DSI link.
 *
 * Returned Value:
 *   Number of bytes read on success; -ETIMEDOUT if the panel did not
 *   respond; other negated errno on failure.
 *
 ****************************************************************************/

int esp_mipi_dsi_dcs_read(uint8_t cmd, uint8_t *buf, size_t len);

/****************************************************************************
 * Name: esp_mipi_dsi_dump_status
 *
 * Description:
 *   Log the raw DSI host PHY and interrupt status registers via syslog,
 *   for link diagnosis.
 *
 ****************************************************************************/

void esp_mipi_dsi_dump_status(void);

/****************************************************************************
 * Name: esp_mipi_dsi_dpi_start
 *
 * Description:
 *   Switch the DSI host to video mode and start continuously streaming the
 *   given RGB565 framebuffer to the panel through the DSI bridge, using a
 *   dedicated DW-GDMA channel that is re-armed on every frame completion.
 *
 * Input Parameters:
 *   vt - DPI video timing description
 *   fb - Framebuffer (h_size * v_size * 2 bytes, cache-line aligned)
 *
 ****************************************************************************/

int esp_mipi_dsi_dpi_start(const struct esp_mipi_dsi_video_s *vt,
                           uint8_t *fb);

/****************************************************************************
 * Name: esp_mipi_dsi_flush
 *
 * Description:
 *   Write back the CPU cache for a region of the framebuffer so that the
 *   DMA engine fetches up-to-date pixels.  Call after drawing.
 *
 ****************************************************************************/

void esp_mipi_dsi_flush(const void *start, size_t len);

#ifdef __cplusplus
}
#endif
#undef EXTERN

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_SRC_COMMON_ESPRESSIF_ESP_MIPI_DSI_H */
