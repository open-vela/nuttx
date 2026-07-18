/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_mipi_csi.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_CSI_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_CSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CSI Configuration */

#define ESP_CSI_LANE_NUM          2       /* 2-lane MIPI-CSI */
#define ESP_CSI_LANE_BITRATE_MBPS 200     /* 200 Mbps per lane */
#define ESP_CSI_HRES              1024    /* Horizontal resolution */
#define ESP_CSI_VRES              600     /* Vertical resolution */
#define ESP_CSI_IN_BPP            8       /* RAW8 input */
#define ESP_CSI_OUT_BPP           16      /* RGB565 output */

/* Frame buffer size: 1024 * 600 * 2 bytes (RGB565) */

#define ESP_CSI_FRAME_SIZE        (ESP_CSI_HRES * ESP_CSI_VRES * 2)

/* CSI Bridge FIFO base address */

#define ESP_CSI_BRG_MEM_BASE      0x500d1800

/* PHY LDO Configuration */

#define ESP_CSI_PHY_LDO_CHAN      3       /* LDO channel 3 */
#define ESP_CSI_PHY_LDO_MV        2500    /* 2.5V */

/* Bridge Configuration */

#define ESP_CSI_BRG_BURST_LEN     512     /* DMA burst length */
#define ESP_CSI_BRG_AFULL_THRD    960     /* FIFO almost-full threshold */
#define ESP_CSI_BRG_DT_MIN        0x12    /* Data type filter min */
#define ESP_CSI_BRG_DT_MAX        0x2f    /* Data type filter max */

/* PHY PLL frequency selection for 200 Mbps */

#define ESP_CSI_PHY_HS_FREQ_SEL   0x22

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* CSI driver state */

struct esp_csi_dev_s
{
  bool     initialized;       /* Driver initialized flag */
  bool     streaming;         /* Currently streaming */
  uint32_t h_res;             /* Horizontal resolution */
  uint32_t v_res;             /* Vertical resolution */
  uint8_t  *frame_buffer[2];  /* Double frame buffers in PSRAM */
  uint8_t  active_buf;        /* Currently active DMA buffer index */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_csi_init
 *
 * Description:
 *   Initialize the MIPI-CSI controller (PHY + Host + Bridge).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int esp_csi_init(void);

/****************************************************************************
 * Name: esp_csi_start
 *
 * Description:
 *   Start CSI data reception (enable Bridge).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int esp_csi_start(void);

/****************************************************************************
 * Name: esp_csi_stop
 *
 * Description:
 *   Stop CSI data reception (disable Bridge).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int esp_csi_stop(void);

/****************************************************************************
 * Name: esp_csi_get_frame_buffer
 *
 * Description:
 *   Get the current completed frame buffer address.
 *
 * Returned Value:
 *   Pointer to the frame buffer, or NULL on failure.
 ****************************************************************************/

uint8_t *esp_csi_get_frame_buffer(void);

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_CSI_H */
