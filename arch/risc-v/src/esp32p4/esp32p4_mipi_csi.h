/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_mipi_csi.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP32P4_MIPI_CSI_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP32P4_MIPI_CSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MIPI CSI-2 Data Types (Standard Specification) */

#define ESP32P4_CSI_DT_YUV420_8        0x18
#define ESP32P4_CSI_DT_YUV420_10       0x19
#define ESP32P4_CSI_DT_YUV422_8        0x1e
#define ESP32P4_CSI_DT_YUV422_10       0x1f
#define ESP32P4_CSI_DT_RGB565          0x22
#define ESP32P4_CSI_DT_RGB888          0x24
#define ESP32P4_CSI_DT_RAW8            0x2a
#define ESP32P4_CSI_DT_RAW10           0x2b
#define ESP32P4_CSI_DT_RAW12           0x2c

/* Default configuration values for SC2336 720p 30fps */

#define ESP32P4_CSI_DEFAULT_LANES      2
#define ESP32P4_CSI_DEFAULT_WIDTH      1280
#define ESP32P4_CSI_DEFAULT_HEIGHT     720
#define ESP32P4_CSI_DEFAULT_BPP        10
#define ESP32P4_CSI_DEFAULT_MBPS       480

/* Buffer memory types */

#define ESP32P4_CSI_BUF_INTERNAL_SRAM  0
#define ESP32P4_CSI_BUF_PSRAM          1

/* Guard patterns for buffer overrun verification */

#define ESP32P4_CSI_GUARD_FRONT        0xcafebabe
#define ESP32P4_CSI_GUARD_REAR         0xdeadbeef
#define ESP32P4_CSI_GUARD_SIZE         64

/* Default buffer count */

#define ESP32P4_CSI_MAX_BUFFERS        2

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

/* CSI DMA and Frame Capture configuration */

struct esp32p4_csi_capture_config_s
{
  uint16_t frame_width;        /* Frame width (pixels) */
  uint16_t frame_height;       /* Frame height (lines) */
  uint8_t  bpp;                /* Bits per pixel (e.g., 10 for RAW10) */
  uint8_t  dma_chan;           /* DW-GDMA channel (0-3, default 0) */
  uint8_t  buf_count;          /* Number of frame buffers (1 or 2) */
  uint8_t  mem_type;           /* Buffer memory type: SRAM or PSRAM */
  bool     enable_guard;       /* Add 32-bit front/rear guard words */
};

/* Captured frame information */

struct esp32p4_csi_frame_s
{
  uint32_t      seq_no;         /* Frame sequence counter */
  uint64_t      timestamp_us;   /* Capture timestamp in microseconds */
  FAR uint8_t  *buffer;         /* Pointer to raw frame payload */
  size_t        buflen;         /* Total allocated payload length in bytes */
  size_t        bytes_received; /* Actual bytes received by DMA */
  uint32_t      crc32;          /* Computed CRC32 checksum of frame data */
  bool          guard_valid;    /* Front and rear guard patterns intact */
  int           status;         /* 0 = OK, negative = negated errno */
};

/* CSI DMA diagnostic statistics */

struct esp32p4_csi_dma_stats_s
{
  uint32_t frames_captured;    /* Total successfully captured frames */
  uint32_t frames_dropped;     /* Dropped/overrun frames */
  uint32_t dma_timeouts;       /* Capture timeouts */
  uint32_t dma_err_dec;        /* DMA address decode errors */
  uint32_t dma_err_slv;        /* DMA slave interface errors */
  uint32_t dma_err_lli;        /* DMA LLI descriptor invalid errors */
  uint32_t dma_guard_errors;   /* Buffer guard pattern corruptions */
  uint64_t last_frame_time_us; /* Timestamp of last captured frame */
  uint32_t frame_bytes;        /* Expected frame size in bytes */
  uint8_t  active_channel;     /* Active DW-GDMA channel */
  bool     is_capturing;       /* Currently capturing flag */
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

int esp32p4_mipi_csi_init(FAR const struct
                          esp32p4_mipi_csi_config_s *config);

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

int esp32p4_mipi_csi_get_status(FAR struct
                                esp32p4_mipi_csi_status_s *status);

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

/****************************************************************************
 * Name: esp32p4_csi_dma_init
 *
 * Description:
 *   Initialize the DW-GDMA subsystem, allocate LLI link list descriptors,
 *   allocate frame buffers in SRAM or PSRAM, and register interrupt handler.
 *
 * Input Parameters:
 *   config - Pointer to CSI capture configuration.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_csi_dma_init(FAR const struct
                         esp32p4_csi_capture_config_s *config);

/****************************************************************************
 * Name: esp32p4_csi_dma_start
 *
 * Description:
 *   Arm DMA channel descriptors and start data reception.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_csi_dma_start(void);

/****************************************************************************
 * Name: esp32p4_csi_dma_stop
 *
 * Description:
 *   Stop DMA channel data transfer and disable channel.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_csi_dma_stop(void);

/****************************************************************************
 * Name: esp32p4_csi_capture_frame
 *
 * Description:
 *   Synchronously capture a single video frame with timeout.
 *   Performs cache invalidation, buffer guard check, and CRC calculation.
 *
 * Input Parameters:
 *   frame      - Pointer to frame output structure.
 *   timeout_ms - Timeout in milliseconds.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure (e.g., -ETIMEDOUT).
 *
 ****************************************************************************/

int esp32p4_csi_capture_frame(FAR struct esp32p4_csi_frame_s *frame,
                              uint32_t timeout_ms);

/****************************************************************************
 * Name: esp32p4_csi_get_dma_stats
 *
 * Description:
 *   Retrieve CSI DMA capture statistics and error counters.
 *
 * Input Parameters:
 *   stats - Pointer to statistics output structure.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_csi_get_dma_stats(FAR struct esp32p4_csi_dma_stats_s *stats);

/****************************************************************************
 * Name: esp32p4_csi_reset_dma_stats
 *
 * Description:
 *   Reset CSI DMA diagnostic statistics and counters.
 *
 ****************************************************************************/

void esp32p4_csi_reset_dma_stats(void);

/****************************************************************************
 * Name: esp32p4_csi_dump_dma
 *
 * Description:
 *   Print detailed DW-GDMA channel and descriptor status to stdout.
 *
 ****************************************************************************/

void esp32p4_csi_dump_dma(void);

/****************************************************************************
 * Name: esp32p4_csi_dma_deinit
 *
 * Description:
 *   Stop DMA transfer, detach interrupts, and free allocated descriptors
 *   and frame buffers.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int esp32p4_csi_dma_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP32P4_MIPI_CSI_H */
