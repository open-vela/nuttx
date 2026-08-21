/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4-function-ev-board.h
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

#ifndef __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_ESP32P4_FUNCTION_EV_BOARD_H
#define __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_ESP32P4_FUNCTION_EV_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdbool.h>
#include <stdint.h>
#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_ESPRESSIF_MIPI_DSI
#  include "espressif/esp_mipi_dsi.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ---- UART (debug console) ---- */

#define FUNEV_GPIO_UART0_TX    37
#define FUNEV_GPIO_UART0_RX    38

/* ---- I2C0: touch GT911, shared with audio codec on function-ev-board ---- */

#define FUNEV_GPIO_I2C0_SCL    8
#define FUNEV_GPIO_I2C0_SDA    7

/* ---- GT911 touch panel (polling mode) ----
 * Per Espressif user guide V1.8 + esp-bsp: INT/RST are NOT wired on this
 * board (BSP_LCD_TOUCH_INT/RST = NC).  INT floating at boot -> I2C addr
 * 0x5D.  Touch events are read by polling /dev/input0 (gt9xx_read does a
 * direct I2C read, no interrupt needed).
 */

#define FUNEV_GT911_I2C_ADDR   0x5d   /* INT floating (unwired) -> 0x5D */
#define FUNEV_GT911_I2C_FREQ   400000

/* ---- EK79007 MIPI-DSI panel ---- */

/* LCD reset: per Espressif user guide V1.8, RST_LCD is wired (dupont) to
 * GPIO27 by default.  Drives EK79007AD GRB (active low) via the adapter. */

#define FUNEV_GPIO_LCD_RST     27

/* Backlight: per user guide, PWM/backlight enable wired to GPIO26 by
 * default (drives the AP3012K boost EN on the LCD adapter board). */

#define FUNEV_GPIO_LCD_BL_EN   26

/* ---- SC2336 MIPI-CSI camera ----
 * SCCB shares I2C0 (SCL=8 SDA=7).  MIPI data lanes are dedicated CSI pads.
 * XVCLK provided by the 24 MHz crystal on the camera adapter board. */

#define FUNEV_SC2336_I2C_ADDR  0x3c
#define FUNEV_SC2336_I2C_FREQ  400000

/* ---- ES8311 audio codec + I2S0 (per Espressif BSP) ---- */

/* I2S0 data lines (per main schematic):
 *   SCLK/BCLK=12  MCLK=13  LRCK/WS=10
 *   ASDOUT (data out -> codec DAC)=11   DSDIN (data in <- mic)=9
 */

#define FUNEV_GPIO_I2S_BCLK    12
#define FUNEV_GPIO_I2S_MCLK    13
#define FUNEV_GPIO_I2S_WS      10
#define FUNEV_GPIO_I2S_DOUT    9    /* P4 TX -> codec DSDIN (bsp I2S_DOUT=GPIO9)  */
#define FUNEV_GPIO_I2S_DIN     11   /* P4 RX <- codec ASDOUT (bsp I2S_DSIN=GPIO11) */

/* Power amplifier enable (active high) */

#define FUNEV_GPIO_PA_EN       53

/* ES8311 I2C address on the shared I2C0 bus (SCL=8 SDA=7) */

#define FUNEV_ES8311_I2C_ADDR  0x18
#define FUNEV_ES8311_I2C_FREQ  400000

/* EK79007AD (source driver + TCON) + EK73217BCGA (gate driver).
 * Panel: AML070JGI50-07403L, 7.0 inch, 1024(RGB) x 600, landscape.
 * MIPI-DSI video mode, 2 data lanes.  The EK73217 gate drivers are driven
 * by the EK79007 built-in TCON, so software only talks to the EK79007AD
 * over MIPI-DSI.  Timing per ESP32-P4 EK79007 reference (ESP-IDF BSP).
 */

/* DSI-DPI scan geometry (pixels/lines as sent on the bus) */

#define FUNEV_MIPI_DSI_H_SCAN               1024
#define FUNEV_MIPI_DSI_V_SCAN               600

/* Visible framebuffer as seen by LVGL (landscape 1024x600) */

#define FUNEV_MIPI_DSI_H_RES               1024
#define FUNEV_MIPI_DSI_V_RES               600

/* DPI timing (EK79007AD, 1024x600 @ ~60Hz, 2-lane, per ESP-IDF BSP) */

#define FUNEV_MIPI_DSI_DPI_CLK_MHZ         52
#define FUNEV_MIPI_DSI_HSYNC_PULSE_WIDTH   10
#define FUNEV_MIPI_DSI_HSYNC_BACK_PORCH    160
#define FUNEV_MIPI_DSI_HSYNC_FRONT_PORCH   160
#define FUNEV_MIPI_DSI_VSYNC_PULSE_WIDTH   1
#define FUNEV_MIPI_DSI_VSYNC_BACK_PORCH    23
#define FUNEV_MIPI_DSI_VSYNC_FRONT_PORCH   12

/* DSI link */

#define FUNEV_MIPI_DSI_LANES               2
#define FUNEV_MIPI_DSI_LANE_BITRATE_MBPS   1000

/* FB format */

#define FUNEV_FB_BPP    16   /* RGB565 - fits in 32 MB PSRAM */
#define FUNEV_FB_WIDTH  FUNEV_MIPI_DSI_H_RES
#define FUNEV_FB_HEIGHT FUNEV_MIPI_DSI_V_RES
#define FUNEV_FB_STRIDE (FUNEV_FB_WIDTH * (FUNEV_FB_BPP / 8))
#define FUNEV_FB_SIZE   (FUNEV_FB_STRIDE * FUNEV_FB_HEIGHT)

/* ---- RMT (unchanged from upstream) ---- */

#define RMT_RXCHANNEL       4
#define RMT_TXCHANNEL       0

#ifdef CONFIG_RMT_LOOP_TEST_MODE
#  define RMT_INPUT_PIN     0
#  define RMT_OUTPUT_PIN    0
#else
#  define RMT_INPUT_PIN     2
#  define RMT_OUTPUT_PIN    8
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

int esp_bringup(void);

#ifdef CONFIG_ESPRESSIF_TWAI
int board_twai_setup(int port);
#endif

#ifdef CONFIG_DEV_GPIO
int esp_gpio_init(void);
#endif

#ifdef CONFIG_ESPRESSIF_EMAC
int board_emac_init(void);
#endif

/* Display (ILI9881C via MIPI-DSI) */

#ifdef CONFIG_ESP32P4_FUNCTION_EV_LCD
int funev_lcd_reset(void);
int funev_lcd_backlight(bool on);
void funev_mipi_dsi_dpi_config(FAR struct esp_mipi_dsi_dpi_config_s *cfg);
FAR struct mipi_dsi_device *
    funev_ek79007_initialize(FAR struct mipi_dsi_host *host);
#endif

/* Touch (GT911 via I2C) */

#ifdef CONFIG_ESP32P4_FUNCTION_EV_TOUCH
int funev_touchscreen_init(void);
#endif

#ifdef __cplusplus
}
#endif

/* Sign voice ROMFS image (signs_romfs.c) */

extern const unsigned char g_signs_romfs[];
extern const unsigned char g_models_romfs[];
extern const unsigned int g_signs_romfs_len;
extern const unsigned int g_models_romfs_len;

/* Audio (ES8311 codec + I2S0) */

#ifdef CONFIG_AUDIO_ES8311
int esp32p4_es8311_initialize(int i2c_port, uint8_t i2c_addr,
                              int i2c_freq, int i2s_port);
#endif

/* Camera (SC2336 MIPI-CSI) */

#ifdef CONFIG_ESP32P4_FUNCTION_EV_CAMERA
int esp32p4_sc2336_initialize(FAR struct i2c_master_s *i2c);
int esp32p4_sc2336_configure(void);
int esp32p4_sc2336_stream(bool on);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_ESP32P4_FUNCTION_EV_BOARD_H */
