/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_mipi_dsi.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP32P4_MIPI_DSI_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP32P4_MIPI_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/video/fb.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Color format for DSI Bridge and DPI interface */

enum esp32p4_dsi_color_e
{
  ESP32P4_DSI_COLOR_RGB565 = 0,
  ESP32P4_DSI_COLOR_RGB888 = 1,
};

/* Video mode horizontal and vertical timing parameters */

struct esp32p4_dsi_timing_s
{
  uint32_t hsw;               /* Horizontal synchronization width (pixels) */
  uint32_t hbp;               /* Horizontal back porch (pixels) */
  uint32_t width;             /* Active display width (pixels) */
  uint32_t hfp;               /* Horizontal front porch (pixels) */
  uint32_t vsw;               /* Vertical synchronization width (lines) */
  uint32_t vbp;               /* Vertical back porch (lines) */
  uint32_t height;            /* Active display height (lines) */
  uint32_t vfp;               /* Vertical front porch (lines) */
  float    dpi_clk_mhz;       /* DPI pixel clock frequency in MHz */
};

/* DSI Controller Configuration */

struct esp32p4_dsi_config_s
{
  uint8_t                     num_lanes;          /* 1 or 2 data lanes */
  float                       lane_bit_rate_mbps; /* Bitrate per lane in Mbps */
  enum esp32p4_dsi_color_e    color_format;       /* RGB565 or RGB888 */
  struct esp32p4_dsi_timing_s timing;             /* Display timing */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp32p4_mipi_dsi_initialize
 *
 * Description:
 *   Initialize the ESP32-P4 MIPI DSI Host, D-PHY TX PLL, and DSI Bridge.
 *
 * Parameters:
 *   config - Pointer to DSI timing and lane configuration.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_dsi_initialize(
    FAR const struct esp32p4_dsi_config_s *config);

/****************************************************************************
 * Name: esp32p4_mipi_dsi_write_dcs
 *
 * Description:
 *   Send a DCS (Display Command Set) write command to the panel.
 *
 * Parameters:
 *   vc         - Virtual channel (usually 0).
 *   cmd        - DCS command byte.
 *   param      - Pointer to parameter buffer (can be NULL if no params).
 *   param_size - Number of parameter bytes.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_dsi_write_dcs(uint8_t vc, uint8_t cmd,
                               FAR const void *param, uint16_t param_size);

/****************************************************************************
 * Name: esp32p4_mipi_dsi_read_dcs
 *
 * Description:
 *   Send a DCS read command and receive returned data from the panel.
 *
 * Parameters:
 *   vc         - Virtual channel (usually 0).
 *   cmd        - DCS command byte.
 *   param      - Pointer to receive buffer.
 *   param_size - Expected receive length in bytes.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_dsi_read_dcs(uint8_t vc, uint8_t cmd,
                              FAR void *param, uint16_t param_size);

/****************************************************************************
 * Name: esp32p4_mipi_dsi_set_framebuffer
 *
 * Description:
 *   Configure the DMA framebuffer memory source for video streaming.
 *
 * Parameters:
 *   fb_addr - Physical address of the frame buffer in PSRAM/SRAM.
 *   fb_size - Total length of the frame buffer in bytes.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_dsi_set_framebuffer(uintptr_t fb_addr, size_t fb_size);

/****************************************************************************
 * Name: esp32p4_mipi_dsi_start_video
 *
 * Description:
 *   Start DPI continuous video streaming from framebuffer to panel.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_dsi_start_video(void);

/****************************************************************************
 * Name: esp32p4_mipi_dsi_stop_video
 *
 * Description:
 *   Stop DPI continuous video streaming.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_dsi_stop_video(void);

/****************************************************************************
 * Name: esp32p4_fb_initialize
 *
 * Description:
 *   Initialize the NuttX Framebuffer driver (/dev/fb0) for ESP32-P4 DSI.
 *
 * Parameters:
 *   display - Display number (0 for primary display).
 *
 * Returned Value:
 *   Pointer to struct fb_vtable_s on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct fb_vtable_s *esp32p4_fb_initialize(int display);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP32P4_MIPI_DSI_H */
