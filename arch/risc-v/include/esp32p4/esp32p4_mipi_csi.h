/****************************************************************************
 * arch/risc-v/include/esp32p4/esp32p4_mipi_csi.h
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

#ifndef __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_MIPI_CSI_H
#define __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_MIPI_CSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIPI CSI-2 Data Types (Standard Specification) */

#define ESP32P4_CSI_DT_YUV420_8     0x18
#define ESP32P4_CSI_DT_YUV420_10    0x19
#define ESP32P4_CSI_DT_YUV422_8     0x1e
#define ESP32P4_CSI_DT_YUV422_10    0x1f
#define ESP32P4_CSI_DT_RGB565       0x22
#define ESP32P4_CSI_DT_RGB888       0x24
#define ESP32P4_CSI_DT_RAW8         0x2a
#define ESP32P4_CSI_DT_RAW10        0x2b
#define ESP32P4_CSI_DT_RAW12        0x2c

/* Default configuration values for SC2336 720p 30fps */

#define ESP32P4_CSI_DEFAULT_LANES   2
#define ESP32P4_CSI_DEFAULT_WIDTH   1280
#define ESP32P4_CSI_DEFAULT_HEIGHT  720
#define ESP32P4_CSI_DEFAULT_BPP     10
#define ESP32P4_CSI_DEFAULT_MBPS    480

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* MIPI CSI controller configuration structure */

struct esp32p4_mipi_csi_config_s
{
  uint8_t  lanes_num;          /* Number of active data lanes (1 or 2) */
  uint16_t frame_width;        /* Horizontal resolution in pixels */
  uint16_t frame_height;       /* Vertical resolution in lines */
  uint8_t  in_bpp;             /* Input bits per pixel (e.g., 10 for RAW10) */
  uint8_t  out_bpp;            /* Output bits per pixel (e.g., 10 or 16) */
  uint8_t  data_type;          /* MIPI CSI data type (e.g., 0x2b) */
  bool     byte_swap_en;       /* Byte endian swap enable in bridge */
  int      lane_bit_rate_mbps; /* Lane bitrate in Mbps (e.g., 480) */
};

/* MIPI CSI D-PHY and controller diagnostic status */

struct esp32p4_mipi_csi_status_s
{
  bool     initialized;        /* Controller initialized */
  bool     enabled;            /* Controller / Bridge enabled */
  bool     clk_stopstate;      /* Clock lane in LP-11 stop state */
  bool     clk_activehs;       /* Clock lane receiving high-speed clock */
  bool     clk_ulpsnot;        /* Clock lane not in ULPS */
  uint8_t  data_stopstate;     /* Data lanes in LP-11 stop state mask */
  uint8_t  data_ulpsesc;       /* Data lanes in ULPS escape mask */
  uint32_t int_st_main;        /* CSI Host main interrupt status */
  uint32_t int_st_phy_fatal;   /* CSI Host PHY fatal error status */
  uint32_t int_st_pkt_fatal;   /* CSI Host packet fatal error status */
  uint32_t bridge_int_st;      /* CSI Bridge interrupt status */
  uint16_t buffer_depth;       /* CSI Bridge internal FIFO buffer depth */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp32p4_mipi_csi_init
 *
 * Description:
 *   Initialize the ESP32-P4 MIPI CSI-2 Host controller, D-PHY receiver,
 *   and CSI Bridge with the given configuration.
 *
 * Input Parameters:
 *   config - Pointer to configuration parameters (NULL for default 720p).
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_csi_init(const struct esp32p4_mipi_csi_config_s *config);

/****************************************************************************
 * Name: esp32p4_mipi_csi_enable
 *
 * Description:
 *   Enable or disable the MIPI CSI bridge data reception.
 *
 * Input Parameters:
 *   enable - True to enable, False to disable.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_csi_enable(bool enable);

/****************************************************************************
 * Name: esp32p4_mipi_csi_get_status
 *
 * Description:
 *   Read live diagnostic status from the D-PHY receiver, Host controller,
 *   and CSI Bridge registers.
 *
 * Input Parameters:
 *   status - Pointer to status output structure.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_csi_get_status(struct esp32p4_mipi_csi_status_s *status);

/****************************************************************************
 * Name: esp32p4_mipi_csi_dump
 *
 * Description:
 *   Print detailed hardware register status and D-PHY lane states to stdout.
 *
 ****************************************************************************/

void esp32p4_mipi_csi_dump(void);

/****************************************************************************
 * Name: esp32p4_mipi_csi_deinit
 *
 * Description:
 *   Disable and reset the MIPI CSI Host, D-PHY, and Bridge peripherals,
 *   gating their clocks to save power.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_mipi_csi_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_INCLUDE_ESP32P4_ESP32P4_MIPI_CSI_H */
