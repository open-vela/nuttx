/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_dma2d.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP32P4_DMA2D_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP32P4_DMA2D_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "hal/dma2d_types.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ESP32P4_DMA2D_TX_CH_MAX  4
#define ESP32P4_DMA2D_RX_CH_MAX  3

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp32p4_dma2d_init
 *
 * Description:
 *   Initialize the ESP32-P4 2D-DMA hardware controller, enable bus clock,
 *   reset controller and enable AXI FIFO FIFOs.
 *
 * Returned Value:
 *   OK on success, or negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_dma2d_init(void);

/****************************************************************************
 * Name: esp32p4_dma2d_deinit
 *
 * Description:
 *   Deinitialize the ESP32-P4 2D-DMA hardware and gate its bus clock.
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

int esp32p4_dma2d_deinit(void);

/****************************************************************************
 * Name: esp32p4_dma2d_setup_tx
 *
 * Description:
 *   Configure a 2D-DMA TX (memory to peripheral) channel.
 *
 * Parameters:
 *   channel       - TX channel index (0..3)
 *   periph        - Target peripheral type
 *   periph_sel_id - Peripheral selection identifier
 *   desc          - Pointer to 8-byte aligned 2D-DMA descriptor link
 *   burst_len     - AXI data burst length
 *
 * Returned Value:
 *   OK on success, or negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_dma2d_setup_tx(uint32_t channel,
                           dma2d_trigger_peripheral_t periph,
                           int periph_sel_id,
                           dma2d_descriptor_t *desc,
                           dma2d_data_burst_length_t burst_len);

/****************************************************************************
 * Name: esp32p4_dma2d_setup_rx
 *
 * Description:
 *   Configure a 2D-DMA RX (peripheral to memory) channel.
 *
 * Parameters:
 *   channel       - RX channel index (0..2)
 *   periph        - Source peripheral type
 *   periph_sel_id - Peripheral selection identifier
 *   desc          - Pointer to 8-byte aligned 2D-DMA descriptor link
 *   burst_len     - AXI data burst length
 *
 * Returned Value:
 *   OK on success, or negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_dma2d_setup_rx(uint32_t channel,
                           dma2d_trigger_peripheral_t periph,
                           int periph_sel_id,
                           dma2d_descriptor_t *desc,
                           dma2d_data_burst_length_t burst_len);

/****************************************************************************
 * Name: esp32p4_dma2d_enable_tx_dscr_port
 *
 * Description:
 *   Enable descriptor port mode on TX channel (required by PPA SRM engine).
 *
 * Parameters:
 *   channel - TX channel index
 *   blk_h   - Block horizontal width
 *   blk_v   - Block vertical height
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

int esp32p4_dma2d_enable_tx_dscr_port(uint32_t channel,
                                      uint32_t blk_h,
                                      uint32_t blk_v);

/****************************************************************************
 * Name: esp32p4_dma2d_start_tx
 *
 * Description:
 *   Start transmission on specified 2D-DMA TX channel.
 *
 * Parameters:
 *   channel - TX channel index
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

int esp32p4_dma2d_start_tx(uint32_t channel);

/****************************************************************************
 * Name: esp32p4_dma2d_start_rx
 *
 * Description:
 *   Start reception on specified 2D-DMA RX channel.
 *
 * Parameters:
 *   channel - RX channel index
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

int esp32p4_dma2d_start_rx(uint32_t channel);

/****************************************************************************
 * Name: esp32p4_dma2d_stop_tx
 *
 * Description:
 *   Stop specified 2D-DMA TX channel.
 *
 * Parameters:
 *   channel - TX channel index
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

int esp32p4_dma2d_stop_tx(uint32_t channel);

/****************************************************************************
 * Name: esp32p4_dma2d_stop_rx
 *
 * Description:
 *   Stop specified 2D-DMA RX channel.
 *
 * Parameters:
 *   channel - RX channel index
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

int esp32p4_dma2d_stop_rx(uint32_t channel);

/****************************************************************************
 * Name: esp32p4_dma2d_wait_rx_done
 *
 * Description:
 *   Wait for 2D-DMA RX channel to complete receiving data.
 *
 * Parameters:
 *   channel    - RX channel index
 *   timeout_us - Timeout in microseconds
 *
 * Returned Value:
 *   OK on success, -ETIMEDOUT on timeout, -EIO on descriptor error.
 *
 ****************************************************************************/

int esp32p4_dma2d_wait_rx_done(uint32_t channel, uint32_t timeout_us);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP32P4_DMA2D_H */
