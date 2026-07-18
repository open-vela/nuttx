/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_mipi_dsi.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* EK79007 Panel Parameters */

#define ESP_DSI_HRES              1024
#define ESP_DSI_VRES              600
#define ESP_DSI_HSYNC             10
#define ESP_DSI_HBP               120
#define ESP_DSI_HFP               120
#define ESP_DSI_VSYNC             1
#define ESP_DSI_VBP               20
#define ESP_DSI_VFP               20

/* DSI Configuration */

#define ESP_DSI_NUM_DATA_LANES    2
#define ESP_DSI_LANE_BITRATE_MBPS 1000
#define ESP_DSI_DPI_CLK_MHZ       48
#define ESP_DSI_DPI_CLK_SRC_MHZ   240   /* PLL_F240M */
#define ESP_DSI_BUS_ID            0
#define ESP_DSI_PHY_CLK_SRC_FREQ  40000000  /* 40MHz XTAL */

/* Color format: RGB565 input, RGB565 output (no conversion) */

#define ESP_DSI_FB_BPP            16    /* RGB565 = 16 bits per pixel */

/* Framebuffer size */

#define ESP_DSI_FB_SIZE           (ESP_DSI_HRES * ESP_DSI_VRES * \
                                   (ESP_DSI_FB_BPP / 8))

/* PHY LDO Configuration */

#define ESP_DSI_PHY_LDO_CHAN      3
#define ESP_DSI_PHY_LDO_MV        2500

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp_mipi_dsi_initialize
 *
 * Description:
 *   Initialize the MIPI-DSI controller and panel (EK79007).
 *   This sets up PHY, Host, Bridge, and sends panel init commands.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp_mipi_dsi_initialize(void);

/****************************************************************************
 * Name: esp_mipi_dsi_get_fb
 *
 * Description:
 *   Get the framebuffer address allocated by DSI driver.
 *
 * Returned Value:
 *   Pointer to framebuffer memory, or NULL if not initialized.
 *
 ****************************************************************************/

uint8_t *esp_mipi_dsi_get_fb(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_DSI_H */
