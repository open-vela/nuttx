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

/* SC2336 1024x600 RAW8 @ 30fps lane bit rate.
 *
 * Derived from sensor timing: HTS(2400) * VTS(1000) * fps(30) * bpp(8) /
 * lanes(2) = 288 Mbps per lane, matching the sensor's mipi_clk = 288 MHz.
 */

#define ESP_CSI_LANE_BITRATE_MBPS 288     /* 288 Mbps per lane */
#define ESP_CSI_HRES              1024    /* Horizontal resolution */
#define ESP_CSI_VRES              600     /* Vertical resolution */
#define ESP_CSI_IN_BPP            8       /* RAW8 input */
#define ESP_CSI_OUT_BPP           16      /* RGB565 output */

/* Frame buffer size.
 *
 * With ISP configured for RAW8→RGB565 conversion, the DMA output is
 * RGB565 (2 bytes per pixel). The bridge + ISP deliver HRES*VRES*2
 * bytes per frame.
 */

#define ESP_CSI_RAW_FRAME_SIZE    (ESP_CSI_HRES * ESP_CSI_VRES)          /* RAW8 (sensor native) */
#define ESP_CSI_FRAME_SIZE        (ESP_CSI_HRES * ESP_CSI_VRES * 2)      /* RGB565 (ISP output) */

/* DW-GDMA transfer size in 64-bit items for an RGB565 frame
 * (h * v * out_bpp / 64).
 */

#define ESP_CSI_DMA_XFER_ITEMS    (ESP_CSI_HRES * ESP_CSI_VRES * ESP_CSI_OUT_BPP / 64)

/* CSI Bridge DMA source memory base (read by DW-GDMA via HW handshake). */

#define ESP_CSI_BRG_MEM_BASE      0x50104000

/* DW-GDMA controller */

#define ESP_DW_GDMA_BASE          0x50081000
#define ESP_CSI_DMA_CHANNEL       0

/* PHY LDO Configuration */

#define ESP_CSI_PHY_LDO_CHAN      3       /* LDO channel 3 */
#define ESP_CSI_PHY_LDO_MV        2500    /* 2.5V */

/* Bridge Configuration */

#define ESP_CSI_BRG_BURST_LEN     512     /* DMA burst length */
#define ESP_CSI_BRG_AFULL_THRD    960     /* FIFO almost-full threshold */
#define ESP_CSI_BRG_DT_MIN        0x12    /* Data type filter min */
#define ESP_CSI_BRG_DT_MAX        0x2f    /* Data type filter max */

/* PHY PLL HS frequency-range selection for 288 Mbps
 * (soc_mipi_csi_phy_pll_ranges: [270,299] -> 0x04).
 */

#define ESP_CSI_PHY_HS_FREQ_SEL   0x04

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Frame-done callback.
 *
 * Invoked from the DW-GDMA ISR when a frame transfer completes. The
 * implementation (esp_camera) must hand the completed buffer up to the
 * V4L2 layer and return, via next_buf/next_size, the destination buffer
 * for the next frame so the ISR can re-arm the DMA. Returning a NULL
 * next_buf leaves the DMA disarmed (streaming effectively pauses until
 * a buffer is queued again).
 */

typedef int (*esp_csi_frame_cb_t)(FAR void *arg,
                                  FAR uint8_t **next_buf,
                                  FAR uint32_t *next_size);

/* CSI driver state */

struct esp_csi_dev_s
{
  bool     initialized;       /* Driver initialized flag */
  bool     streaming;         /* Currently streaming */
  uint32_t h_res;             /* Horizontal resolution */
  uint32_t v_res;             /* Vertical resolution */
  uint8_t  *frame_buffer[2];  /* Double frame buffers in PSRAM */
  uint8_t  active_buf;        /* Currently active DMA buffer index */

  int      cpuint;            /* Allocated CPU interrupt for DW-GDMA */
  FAR uint8_t *dma_dst;       /* Current DMA destination buffer */
  uint32_t dma_dst_size;      /* Size of current DMA destination buffer */
  esp_csi_frame_cb_t frame_cb;/* Frame-done callback */
  FAR void *frame_cb_arg;     /* Argument for frame-done callback */
  uint32_t frame_count;       /* Completed frame counter (diagnostics) */
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

/****************************************************************************
 * Name: esp_csi_register_frame_cb
 *
 * Description:
 *   Register the frame-done callback invoked from the DW-GDMA ISR.
 ****************************************************************************/

int esp_csi_register_frame_cb(esp_csi_frame_cb_t cb, FAR void *arg);

/****************************************************************************
 * Name: esp_csi_set_buffer
 *
 * Description:
 *   Set the destination buffer used to arm the DMA for the next frame.
 *   Called before esp_csi_start (initial buffer).
 ****************************************************************************/

int esp_csi_set_buffer(FAR uint8_t *buf, uint32_t size);

/****************************************************************************
 * Name: esp_csi_dump_status
 *
 * Description:
 *   Diagnostic dump of DW-GDMA channel + CSI bridge/host state.
 ****************************************************************************/

void esp_csi_dump_status(void);

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_MIPI_CSI_H */
