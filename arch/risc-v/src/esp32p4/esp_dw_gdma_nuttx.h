/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_dw_gdma_nuttx.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_DW_GDMA_NUTTX_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_DW_GDMA_NUTTX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal/dw_gdma_types.h"
#include "hal/dw_gdma_ll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Opaque handle types */

typedef struct dw_gdma_channel_t *dw_gdma_channel_handle_t;
typedef struct dw_gdma_link_list_t *dw_gdma_link_list_handle_t;

/* Transfer endpoint configuration (src or dst) */

struct dw_gdma_channel_endpoint_config_t
{
  dw_gdma_role_t role;                    /* MEM, DSI, CSI, ISP */
  dw_gdma_block_transfer_type_t block_transfer_type;
  dw_gdma_handshake_type_t handshake_type;
  uint8_t num_outstanding_requests;       /* max outstanding requests */
};

/* Channel allocation config */

struct dw_gdma_channel_alloc_config_t
{
  struct dw_gdma_channel_endpoint_config_t src;
  struct dw_gdma_channel_endpoint_config_t dst;
  dw_gdma_flow_controller_t flow_controller;
  uint8_t chan_priority;                  /* higher = more priority */
};

typedef struct dw_gdma_channel_alloc_config_t dw_gdma_channel_alloc_config_t;

/* Link list configuration */

struct dw_gdma_link_list_config_t
{
  uint32_t num_items;                     /* number of LLI items */
};

typedef struct dw_gdma_link_list_config_t dw_gdma_link_list_config_t;

/* Block transfer endpoint config (for LLI item) */

struct dw_gdma_block_endpoint_config_t
{
  uint32_t addr;                          /* source or destination address */
  dw_gdma_burst_mode_t burst_mode;
  dw_gdma_burst_items_t burst_items;
  uint8_t burst_len;                      /* burst length */
  dw_gdma_transfer_width_t width;
};

/* Block transfer configuration */

struct dw_gdma_block_transfer_config_t
{
  struct dw_gdma_block_endpoint_config_t src;
  struct dw_gdma_block_endpoint_config_t dst;
  uint32_t size;                          /* number of transfer items */
};

typedef struct dw_gdma_block_transfer_config_t dw_gdma_block_transfer_config_t;

/* Block markers */

struct dw_gdma_block_markers_t
{
  bool is_valid;                          /* block is valid */
  bool is_last;                           /* last block in list */
};

typedef struct dw_gdma_block_markers_t dw_gdma_block_markers_t;

/* Callback types */

struct dw_gdma_trans_done_event_data_t
{
  uint32_t reserved;                      /* placeholder */
};

typedef struct dw_gdma_trans_done_event_data_t dw_gdma_trans_done_event_data_t;

typedef bool (*dw_gdma_trans_done_cb_t)(
    dw_gdma_channel_handle_t chan,
    const dw_gdma_trans_done_event_data_t *event_data,
    void *user_data);

struct dw_gdma_event_callbacks_t
{
  dw_gdma_trans_done_cb_t on_block_trans_done;
};

typedef struct dw_gdma_event_callbacks_t dw_gdma_event_callbacks_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: dw_gdma_new_channel
 *
 * Description:
 *   Allocate a DW-GDMA channel. Channel ID is determined by source role:
 *   CSI (camera) → channel 0, DSI (display) → channel 1.
 ****************************************************************************/

int dw_gdma_new_channel(const dw_gdma_channel_alloc_config_t *config,
                        dw_gdma_channel_handle_t *ret_chan);

/****************************************************************************
 * Name: esp_dsi_dma_isr_handler
 *
 * Description:
 *   DSI DMA ISR handler (channel 1). Called from shared DW-GDMA ISR.
 ****************************************************************************/

void esp_dsi_dma_isr_handler(void);

/****************************************************************************
 * Name: esp_csi_dma_isr_handler
 *
 * Description:
 *   CSI DMA ISR handler (channel 0). Called from shared DW-GDMA ISR.
 ****************************************************************************/

void esp_csi_dma_isr_handler(void);

/****************************************************************************
 * Name: dw_gdma_new_link_list
 *
 * Description:
 *   Allocate a linked list for multi-block DMA transfers.
 ****************************************************************************/

int dw_gdma_new_link_list(const dw_gdma_link_list_config_t *config,
                          dw_gdma_link_list_handle_t *ret_list);

/****************************************************************************
 * Name: dw_gdma_link_list_get_item
 *
 * Description:
 *   Get a pointer to a specific LLI item (non-cached address).
 ****************************************************************************/

dw_gdma_link_list_item_t *dw_gdma_link_list_get_item(
    dw_gdma_link_list_handle_t list, uint32_t item_index);

/****************************************************************************
 * Name: dw_gdma_lli_config_transfer
 *
 * Description:
 *   Configure source/destination/size in a link list item.
 ****************************************************************************/

int dw_gdma_lli_config_transfer(
    dw_gdma_link_list_item_t *lli,
    const dw_gdma_block_transfer_config_t *config);

/****************************************************************************
 * Name: dw_gdma_lli_set_block_markers
 *
 * Description:
 *   Set block valid/last markers on a link list item.
 ****************************************************************************/

int dw_gdma_lli_set_block_markers(
    dw_gdma_link_list_item_t *lli,
    dw_gdma_block_markers_t markers);

/****************************************************************************
 * Name: dw_gdma_channel_use_link_list
 *
 * Description:
 *   Attach a link list to a channel.
 ****************************************************************************/

int dw_gdma_channel_use_link_list(
    dw_gdma_channel_handle_t chan,
    dw_gdma_link_list_handle_t list);

/****************************************************************************
 * Name: dw_gdma_channel_enable_ctrl
 *
 * Description:
 *   Enable or disable the DMA channel transfer.
 ****************************************************************************/

int dw_gdma_channel_enable_ctrl(
    dw_gdma_channel_handle_t chan, bool en);

/****************************************************************************
 * Name: dw_gdma_channel_register_event_callbacks
 *
 * Description:
 *   Register event callbacks (trans done) for the channel.
 ****************************************************************************/

int dw_gdma_channel_register_event_callbacks(
    dw_gdma_channel_handle_t chan,
    const dw_gdma_event_callbacks_t *cbs,
    void *user_data);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_DW_GDMA_NUTTX_H */
