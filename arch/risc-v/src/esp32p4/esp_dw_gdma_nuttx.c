/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_dw_gdma_nuttx.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/kmalloc.h>
#include <nuttx/spinlock.h>

#include "esp_idf_shim.h"
#include "esp_dw_gdma_nuttx.h"
#include "hal/dw_gdma_ll.h"
#include "esp_private/periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Non-cached address offset for ESP32-P4 SRAM */

#define SOC_NON_CACHEABLE_OFFSET_SRAM  0x40000000

#define DW_GDMA_NON_CACHE_ADDR(addr) \
  ((uint32_t)(addr) + SOC_NON_CACHEABLE_OFFSET_SRAM)

#define DW_GDMA_CACHE_ADDR(addr) \
  ((uint32_t)(addr) - SOC_NON_CACHEABLE_OFFSET_SRAM)

/* DSI uses channel 1; channel 0 is reserved for CSI */

#define DW_GDMA_DSI_CHANNEL_ID  1

/* DW-GDMA group (only one group on ESP32-P4) */

#define DW_GDMA_GROUP_ID        0

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dw_gdma_channel_t
{
  int chan_id;                            /* channel number (1 for DSI) */
  dw_gdma_dev_t *dev;                    /* HW register base */
  dw_gdma_trans_done_cb_t on_block_trans_done;
  void *user_data;
  spinlock_t spinlock;
};

struct dw_gdma_link_list_t
{
  uint32_t num_items;
  dw_gdma_link_list_item_t *items;       /* cached address */
  dw_gdma_link_list_item_t *items_nc;    /* non-cached address */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dw_gdma_channel_t g_dsi_channel;
static bool g_dsi_channel_allocated = false;
static bool g_gdma_clock_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dw_gdma_ensure_clock_enabled
 *
 * Description:
 *   No-op: Camera CSI already enabled GDMA bus clock via
 *   PERIPH_RCC_ATOMIC + dw_gdma_ll_enable_bus_clock.
 *   We only need dw_gdma_ll_enable_controller (in dw_gdma_new_channel).
 ****************************************************************************/

static void dw_gdma_ensure_clock_enabled(void)
{
  g_gdma_clock_enabled = true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dw_gdma_new_channel
 ****************************************************************************/

int dw_gdma_new_channel(const dw_gdma_channel_alloc_config_t *config,
                        dw_gdma_channel_handle_t *ret_chan)
{
  struct dw_gdma_channel_t *chan;
  dw_gdma_dev_t *dev;
  int ch = DW_GDMA_DSI_CHANNEL_ID;

  if (config == NULL || ret_chan == NULL)
    {
      return -EINVAL;
    }

  if (g_dsi_channel_allocated)
    {
      return -EBUSY;
    }

  /* Ensure DW-GDMA clock is enabled */

  dw_gdma_ensure_clock_enabled();

  /* Use the static channel struct */

  chan = &g_dsi_channel;
  memset(chan, 0, sizeof(*chan));
  chan->chan_id = ch;
  chan->dev = DW_GDMA_LL_GET_HW(DW_GDMA_GROUP_ID);
  spin_lock_init(&chan->spinlock);

  dev = chan->dev;

  /* Enable the DMA controller (idempotent if already enabled by CSI) */

  dw_gdma_ll_enable_controller(dev, true);
  dw_gdma_ll_enable_intr_global(dev, true);

  /* Disable channel before configuration */

  dw_gdma_ll_channel_enable(dev, ch, false);

  /* Set transfer flow controller */

  dw_gdma_ll_channel_set_trans_flow(dev, ch,
                                    config->src.role,
                                    config->dst.role,
                                    config->flow_controller);
  /* Set multi-block transfer types */

  dw_gdma_ll_channel_set_src_multi_block_type(dev, ch,
      config->src.block_transfer_type);
  dw_gdma_ll_channel_set_dst_multi_block_type(dev, ch,
      config->dst.block_transfer_type);

  /* Set handshake interface type */

  dw_gdma_ll_channel_set_src_handshake_interface(dev, ch,
      config->src.handshake_type);
  dw_gdma_ll_channel_set_dst_handshake_interface(dev, ch,
      config->dst.handshake_type);

  /* Set handshake peripheral if not memory role */

  if (config->src.role != DW_GDMA_ROLE_MEM)
    {
      dw_gdma_ll_channel_set_src_handshake_periph(dev, ch,
          config->src.role);
    }

  if (config->dst.role != DW_GDMA_ROLE_MEM)
    {
      dw_gdma_ll_channel_set_dst_handshake_periph(dev, ch,
          config->dst.role);
    }

  /* Set channel priority */

  dw_gdma_ll_channel_set_priority(dev, ch, config->chan_priority);

  /* Set outstanding request limits */

  if (config->src.num_outstanding_requests > 0)
    {
      dw_gdma_ll_channel_set_src_outstanding_limit(dev, ch,
          config->src.num_outstanding_requests);
    }

  if (config->dst.num_outstanding_requests > 0)
    {
      dw_gdma_ll_channel_set_dst_outstanding_limit(dev, ch,
          config->dst.num_outstanding_requests);
    }

  /* Disable intr propagation for ch1 — use timer-based polling re-arm
   * instead to avoid interrupt storm with shared CSI ISR.
   */

  dw_gdma_ll_channel_enable_intr_generation(dev, ch, UINT32_MAX, true);
  dw_gdma_ll_channel_enable_intr_propagation(dev, ch, UINT32_MAX, false);

  g_dsi_channel_allocated = true;
  *ret_chan = chan;

  return 0;
}

/****************************************************************************
 * Name: dw_gdma_new_link_list
 ****************************************************************************/

int dw_gdma_new_link_list(const dw_gdma_link_list_config_t *config,
                          dw_gdma_link_list_handle_t *ret_list)
{
  struct dw_gdma_link_list_t *list;
  dw_gdma_link_list_item_t *items;
  dw_gdma_link_list_item_t *items_nc;
  size_t alloc_size;

  if (config == NULL || ret_list == NULL || config->num_items == 0)
    {
      return -EINVAL;
    }

  /* Allocate the list struct */

  list = kmm_zalloc(sizeof(struct dw_gdma_link_list_t));
  if (list == NULL)
    {
      return -ENOMEM;
    }

  /* Allocate LLI items with 64-byte alignment (required by HW) */

  alloc_size = config->num_items * sizeof(dw_gdma_link_list_item_t);
  items = kmm_memalign(DW_GDMA_LL_LINK_LIST_ALIGNMENT, alloc_size);
  if (items == NULL)
    {
      kmm_free(list);
      return -ENOMEM;
    }

  memset(items, 0, alloc_size);

  /* Compute non-cached address for DMA to read */

  items_nc = (dw_gdma_link_list_item_t *)DW_GDMA_NON_CACHE_ADDR(items);

  /* For a single-item list, set up self-cycling (next points to self) */

  if (config->num_items == 1)
    {
      dw_gdma_ll_lli_set_next_item_addr(items_nc, (uint32_t)items);
      dw_gdma_ll_lli_set_link_list_master_port(items_nc,
          DW_GDMA_LL_MASTER_PORT_MEMORY);
    }
  else
    {
      uint32_t i;

      /* Chain items: each points to the next; last points to first */

      for (i = 0; i < config->num_items; i++)
        {
          uint32_t next_idx = (i + 1) % config->num_items;
          dw_gdma_link_list_item_t *cur_nc = &items_nc[i];
          dw_gdma_link_list_item_t *next_cached =
              &items[next_idx];

          dw_gdma_ll_lli_set_next_item_addr(cur_nc,
              (uint32_t)next_cached);
          dw_gdma_ll_lli_set_link_list_master_port(cur_nc,
              DW_GDMA_LL_MASTER_PORT_MEMORY);
        }
    }

  list->num_items = config->num_items;
  list->items = items;
  list->items_nc = items_nc;

  *ret_list = list;
  return 0;
}

/****************************************************************************
 * Name: dw_gdma_link_list_get_item
 ****************************************************************************/

dw_gdma_link_list_item_t *dw_gdma_link_list_get_item(
    dw_gdma_link_list_handle_t list, uint32_t item_index)
{
  if (list == NULL || item_index >= list->num_items)
    {
      return NULL;
    }

  /* Return non-cached pointer so DMA reads correct data */

  return &list->items_nc[item_index];
}

/****************************************************************************
 * Name: dw_gdma_lli_config_transfer
 ****************************************************************************/

int dw_gdma_lli_config_transfer(
    dw_gdma_link_list_item_t *lli,
    const dw_gdma_block_transfer_config_t *config)
{
  if (lli == NULL || config == NULL)
    {
      return -EINVAL;
    }

  /* Set source and destination addresses */

  dw_gdma_ll_lli_set_src_addr(lli, config->src.addr);
  dw_gdma_ll_lli_set_dst_addr(lli, config->dst.addr);

  /* Set master ports based on addresses */

  dw_gdma_ll_lli_set_src_master_port(lli, config->src.addr);
  dw_gdma_ll_lli_set_dst_master_port(lli, config->dst.addr);

  /* Set block transfer size */

  dw_gdma_ll_lli_set_trans_block_size(lli, config->size);

  /* Set source burst parameters */

  dw_gdma_ll_lli_set_src_burst_mode(lli, config->src.burst_mode);
  dw_gdma_ll_lli_set_src_burst_items(lli, config->src.burst_items);
  dw_gdma_ll_lli_set_src_burst_len(lli, config->src.burst_len);
  dw_gdma_ll_lli_set_src_trans_width(lli, config->src.width);

  /* Set destination burst parameters */

  dw_gdma_ll_lli_set_dst_burst_mode(lli, config->dst.burst_mode);
  dw_gdma_ll_lli_set_dst_burst_items(lli, config->dst.burst_items);
  dw_gdma_ll_lli_set_dst_burst_len(lli, config->dst.burst_len);
  dw_gdma_ll_lli_set_dst_trans_width(lli, config->dst.width);

  return 0;
}

/****************************************************************************
 * Name: dw_gdma_lli_set_block_markers
 ****************************************************************************/

int dw_gdma_lli_set_block_markers(
    dw_gdma_link_list_item_t *lli,
    dw_gdma_block_markers_t markers)
{
  if (lli == NULL)
    {
      return -EINVAL;
    }

  /* Set block markers: en_intr=true so we get notified, is_last, is_valid */

  dw_gdma_ll_lli_set_block_markers(lli, true,
                                   markers.is_last,
                                   markers.is_valid);

  return 0;
}

/****************************************************************************
 * Name: dw_gdma_channel_use_link_list
 ****************************************************************************/

int dw_gdma_channel_use_link_list(
    dw_gdma_channel_handle_t chan,
    dw_gdma_link_list_handle_t list)
{
  if (chan == NULL || list == NULL)
    {
      return -EINVAL;
    }

  /* Set the link list head address (cached address — HW reads via
   * non-cached alias internally based on master port config).
   * The LL function expects the cached address per ESP-IDF convention.
   */

  dw_gdma_ll_channel_set_link_list_head_addr(chan->dev, chan->chan_id,
                                             (uint32_t)list->items);

  /* Set the link list master port to memory port 1 */

  dw_gdma_ll_channel_set_link_list_master_port(chan->dev, chan->chan_id,
      DW_GDMA_LL_MASTER_PORT_MEMORY);

  return 0;
}

/****************************************************************************
 * Name: dw_gdma_channel_enable_ctrl
 ****************************************************************************/

int dw_gdma_channel_enable_ctrl(
    dw_gdma_channel_handle_t chan, bool en)
{
  if (chan == NULL)
    {
      return -EINVAL;
    }

  dw_gdma_ll_channel_enable(chan->dev, chan->chan_id, en);

  return 0;
}

/****************************************************************************
 * Name: dw_gdma_channel_register_event_callbacks
 ****************************************************************************/

int dw_gdma_channel_register_event_callbacks(
    dw_gdma_channel_handle_t chan,
    const dw_gdma_event_callbacks_t *cbs,
    void *user_data)
{
  if (chan == NULL || cbs == NULL)
    {
      return -EINVAL;
    }

  chan->on_block_trans_done = cbs->on_block_trans_done;
  chan->user_data = user_data;

  return 0;
}

/****************************************************************************
 * Name: esp_dsi_dma_isr_handler
 *
 * Description:
 *   Called from the shared DW-GDMA ISR (in esp_mipi_csi.c) when channel 1
 *   has pending interrupt status. Clears the interrupt and invokes the
 *   registered trans_done callback to re-arm DMA for continuous refresh.
 ****************************************************************************/

void esp_dsi_dma_isr_handler(void)
{
  struct dw_gdma_channel_t *chan = &g_dsi_channel;
  dw_gdma_dev_t *dev = chan->dev;
  int ch = chan->chan_id;
  uint32_t status;

  if (!g_dsi_channel_allocated || dev == NULL)
    {
      return;
    }

  /* Read and clear channel 1 interrupt status */

  status = dw_gdma_ll_channel_get_intr_status(dev, ch);
  if (status == 0)
    {
      return;
    }

  dw_gdma_ll_channel_clear_intr(dev, ch, status);

  /* If block transfer done, invoke callback to re-arm DMA */

  if (chan->on_block_trans_done)
    {
      dw_gdma_trans_done_event_data_t ev;
      memset(&ev, 0, sizeof(ev));
      chan->on_block_trans_done(chan, &ev, chan->user_data);
    }
}
