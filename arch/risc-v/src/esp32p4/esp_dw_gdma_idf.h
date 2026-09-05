/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_dw_gdma_idf.h
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

/****************************************************************************
 * NuttX-ported DW-GDMA high-level driver for ESP32-P4.
 * API-compatible with ESP-IDF esp_private/dw_gdma.h
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_DW_GDMA_IDF_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_DW_GDMA_IDF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_idf_shim.h"
#include "hal/dw_gdma_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Forward declarations - opaque handle types */

typedef struct dw_gdma_channel_t *dw_gdma_channel_handle_t;
typedef struct dw_gdma_link_list_t *dw_gdma_link_list_handle_t;
typedef struct dw_gdma_link_list_item_t *dw_gdma_lli_handle_t;

/**
 * @brief Link list type
 */
typedef enum {
    DW_GDMA_LINKED_LIST_TYPE_SINGLY = 0,
    DW_GDMA_LINKED_LIST_TYPE_CIRCULAR = 1,
} dw_gdma_linked_list_type_t;

/* Use the same name as ESP-IDF for compatibility */

typedef dw_gdma_linked_list_type_t dw_gdma_link_list_type_t;

/**
 * @brief Channel static end configuration
 */
typedef struct {
    dw_gdma_block_transfer_type_t block_transfer_type;
    dw_gdma_role_t role;
    dw_gdma_handshake_type_t handshake_type;
    uint8_t num_outstanding_requests;
    uint32_t status_fetch_addr;
} dw_gdma_channel_static_config_t;

/**
 * @brief Channel allocation configuration
 */
typedef struct {
    dw_gdma_channel_static_config_t src;
    dw_gdma_channel_static_config_t dst;
    dw_gdma_flow_controller_t flow_controller;
    int chan_priority;
    int intr_priority;
} dw_gdma_channel_alloc_config_t;

/**
 * @brief Channel dynamic end configuration (per-transfer)
 */
typedef struct {
    uint32_t addr;
    dw_gdma_transfer_width_t width;
    dw_gdma_burst_mode_t burst_mode;
    dw_gdma_burst_items_t burst_items;
    uint8_t burst_len;
    struct {
        uint32_t en_status_write_back: 1;
    } flags;
} dw_gdma_channel_dynamic_config_t;

/**
 * @brief Block transfer configuration
 */
typedef struct {
    dw_gdma_channel_dynamic_config_t src;
    dw_gdma_channel_dynamic_config_t dst;
    size_t size;
} dw_gdma_block_transfer_config_t;

/**
 * @brief Block markers
 */
typedef struct {
    uint32_t is_last: 1;
    uint32_t is_valid: 1;
    uint32_t en_trans_done_intr: 1;
} dw_gdma_block_markers_t;

/**
 * @brief Link list configuration
 */
typedef struct {
    uint32_t num_items;
    dw_gdma_link_list_type_t link_type;
} dw_gdma_link_list_config_t;

/**
 * @brief Trans done event data
 */
typedef struct {
} dw_gdma_trans_done_event_data_t;

/**
 * @brief Break event data
 */
typedef struct {
    dw_gdma_lli_handle_t invalid_lli;
} dw_gdma_break_event_data_t;

/**
 * @brief Event callback types
 */
typedef bool (*dw_gdma_trans_done_event_callback_t)(
    dw_gdma_channel_handle_t chan,
    const dw_gdma_trans_done_event_data_t *event_data,
    void *user_data);

typedef bool (*dw_gdma_break_event_callback_t)(
    dw_gdma_channel_handle_t chan,
    const dw_gdma_break_event_data_t *event_data,
    void *user_data);

/**
 * @brief Group of supported event callbacks
 */
typedef struct {
    dw_gdma_trans_done_event_callback_t on_block_trans_done;
    dw_gdma_trans_done_event_callback_t on_full_trans_done;
    dw_gdma_break_event_callback_t on_invalid_block;
} dw_gdma_event_callbacks_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

esp_err_t dw_gdma_new_channel(
    const dw_gdma_channel_alloc_config_t *config,
    dw_gdma_channel_handle_t *ret_chan);

esp_err_t dw_gdma_del_channel(dw_gdma_channel_handle_t chan);

esp_err_t dw_gdma_channel_get_id(
    dw_gdma_channel_handle_t chan, int *channel_id);

esp_err_t dw_gdma_channel_enable_ctrl(
    dw_gdma_channel_handle_t chan, bool en_or_dis);

esp_err_t dw_gdma_channel_suspend_ctrl(
    dw_gdma_channel_handle_t chan, bool enter_or_exit);

esp_err_t dw_gdma_channel_abort(dw_gdma_channel_handle_t chan);

esp_err_t dw_gdma_channel_lock(
    dw_gdma_channel_handle_t chan, dw_gdma_lock_level_t level);

esp_err_t dw_gdma_channel_unlock(dw_gdma_channel_handle_t chan);

esp_err_t dw_gdma_channel_continue(dw_gdma_channel_handle_t chan);

esp_err_t dw_gdma_channel_config_transfer(
    dw_gdma_channel_handle_t chan,
    const dw_gdma_block_transfer_config_t *config);

esp_err_t dw_gdma_channel_set_block_markers(
    dw_gdma_channel_handle_t chan, dw_gdma_block_markers_t markers);

esp_err_t dw_gdma_channel_register_event_callbacks(
    dw_gdma_channel_handle_t chan,
    dw_gdma_event_callbacks_t *cbs,
    void *user_data);

esp_err_t dw_gdma_new_link_list(
    const dw_gdma_link_list_config_t *config,
    dw_gdma_link_list_handle_t *ret_list);

esp_err_t dw_gdma_del_link_list(dw_gdma_link_list_handle_t list);

esp_err_t dw_gdma_channel_use_link_list(
    dw_gdma_channel_handle_t chan,
    dw_gdma_link_list_handle_t list);

dw_gdma_lli_handle_t dw_gdma_link_list_get_item(
    dw_gdma_link_list_handle_t list, int item_index);

esp_err_t dw_gdma_lli_config_transfer(
    dw_gdma_lli_handle_t lli,
    const dw_gdma_block_transfer_config_t *config);

esp_err_t dw_gdma_lli_set_block_markers(
    dw_gdma_lli_handle_t lli, dw_gdma_block_markers_t markers);

esp_err_t dw_gdma_lli_set_next(
    dw_gdma_lli_handle_t lli, dw_gdma_lli_handle_t next);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_DW_GDMA_IDF_H */
