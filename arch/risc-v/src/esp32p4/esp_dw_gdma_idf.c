/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_dw_gdma_idf.c
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
 * NuttX port of ESP-IDF DW-GDMA driver (components/esp_driver_dma/src/dw_gdma.c)
 * Adapted: FreeRTOS→NuttX, esp_intr→esp_setup_irq, heap_caps→kmm
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "esp_idf_shim.h"
#include "esp_dw_gdma_idf.h"
#include "esp_irq.h"
#include "esp_cache.h"

#include "soc/soc_caps.h"
#include "soc/interrupts.h"
#include "hal/dw_gdma_hal.h"
#include "hal/dw_gdma_ll.h"
#include "hal/cache_hal.h"
#include "hal/cache_ll.h"
#include "esp_private/periph_ctrl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

ESP_LOG_ATTR_TAG(TAG, "dw-gdma");

#if SOC_CACHE_INTERNAL_MEM_VIA_L1CACHE
#define DW_GDMA_GET_NON_CACHE_ADDR(addr) \
    ((addr) ? CACHE_LL_L2MEM_NON_CACHE_ADDR(addr) : 0)
#define DW_GDMA_GET_CACHE_ADDRESS(nc_addr) \
    ((nc_addr) ? CACHE_LL_L2MEM_CACHE_ADDR(nc_addr) : 0)
#else
#define DW_GDMA_GET_NON_CACHE_ADDR(addr) (addr)
#define DW_GDMA_GET_CACHE_ADDRESS(nc_addr) (nc_addr)
#endif

#define DW_GDMA_MEM_ALLOC_CAPS    MALLOC_CAP_DEFAULT
#define DW_GDMA_ALLOW_INTR_PRIORITY_MASK ESP_INTR_FLAG_LOWMED

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef struct dw_gdma_group_t dw_gdma_group_t;
typedef struct dw_gdma_channel_t dw_gdma_channel_t;

struct dw_gdma_link_list_t {
    uint32_t num_items;
    dw_gdma_link_list_item_t *items;
    dw_gdma_link_list_item_t *items_nc;
};

typedef struct {
    spinlock_t mutex;
    dw_gdma_group_t *groups[DW_GDMA_LL_GROUPS];
    int group_ref_counts[DW_GDMA_LL_GROUPS];
} dw_gdma_platform_t;

struct dw_gdma_group_t {
    int group_id;
    dw_gdma_hal_context_t hal;
    int intr_priority;
    portMUX_TYPE spinlock;
    dw_gdma_channel_t *channels[DW_GDMA_LL_CHANNELS_PER_GROUP];
};

struct dw_gdma_channel_t {
    int chan_id;
    int cpuint;              /* NuttX: CPU interrupt number */
    portMUX_TYPE spinlock;
    dw_gdma_group_t *group;
    void *user_data;
    dw_gdma_event_callbacks_t cbs;
    dw_gdma_block_transfer_type_t src_transfer_type;
    dw_gdma_block_transfer_type_t dst_transfer_type;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static dw_gdma_platform_t s_platform;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static dw_gdma_group_t *dw_gdma_acquire_group_handle(int group_id)
{
  bool new_group = false;
  dw_gdma_group_t *group = NULL;
  irqstate_t flags;

  flags = spin_lock_irqsave(&s_platform.mutex);
  if (!s_platform.groups[group_id])
    {
      group = (dw_gdma_group_t *)heap_caps_calloc(
                  1, sizeof(dw_gdma_group_t), DW_GDMA_MEM_ALLOC_CAPS);
      if (group)
        {
          new_group = true;
          s_platform.groups[group_id] = group;
          PERIPH_RCC_ATOMIC()
            {
              dw_gdma_ll_enable_bus_clock(group_id, true);
              dw_gdma_ll_reset_register(group_id);
            }

          dw_gdma_hal_config_t hal_config = {};
          dw_gdma_hal_init(&group->hal, &hal_config);
        }
    }
  else
    {
      group = s_platform.groups[group_id];
    }

  if (group)
    {
      s_platform.group_ref_counts[group_id]++;
    }

  spin_unlock_irqrestore(&s_platform.mutex, flags);

  if (new_group)
    {
      portMUX_INITIALIZE(&group->spinlock);
      group->group_id = group_id;
      group->intr_priority = -1;
      ESP_LOGD(TAG, "new group (%d) at %p", group_id, group);
    }

  return group;
}

static void dw_gdma_release_group_handle(dw_gdma_group_t *group)
{
  int group_id = group->group_id;
  bool del_group = false;
  irqstate_t flags;

  flags = spin_lock_irqsave(&s_platform.mutex);
  s_platform.group_ref_counts[group_id]--;
  if (s_platform.group_ref_counts[group_id] == 0)
    {
      del_group = true;
      s_platform.groups[group_id] = NULL;
      dw_gdma_hal_deinit(&group->hal);
      PERIPH_RCC_ATOMIC()
        {
          dw_gdma_ll_enable_bus_clock(group_id, false);
        }
    }

  spin_unlock_irqrestore(&s_platform.mutex, flags);

  if (del_group)
    {
      free(group);
      ESP_LOGD(TAG, "delete group (%d)", group_id);
    }
}

static esp_err_t channel_register_to_group(dw_gdma_channel_t *chan)
{
  dw_gdma_group_t *group = NULL;
  int chan_id = -1;
  int i;
  int j;

  for (i = 0; i < DW_GDMA_LL_GROUPS; i++)
    {
      group = dw_gdma_acquire_group_handle(i);
      ESP_RETURN_ON_FALSE(group, ESP_ERR_NO_MEM, TAG,
                          "no mem for group(%d)", i);
      esp_os_enter_critical(&group->spinlock);
      for (j = 0; j < DW_GDMA_LL_CHANNELS_PER_GROUP; j++)
        {
          if (group->channels[j] == NULL)
            {
              group->channels[j] = chan;
              chan_id = j;
              break;
            }
        }

      esp_os_exit_critical(&group->spinlock);
      if (chan_id < 0)
        {
          dw_gdma_release_group_handle(group);
        }
      else
        {
          chan->group = group;
          chan->chan_id = chan_id;
          break;
        }
    }

  ESP_RETURN_ON_FALSE(chan_id >= 0, ESP_ERR_NOT_FOUND, TAG,
                      "no free channels");
  return ESP_OK;
}

static void channel_unregister_from_group(dw_gdma_channel_t *chan)
{
  dw_gdma_group_t *group = chan->group;
  int chan_id = chan->chan_id;

  esp_os_enter_critical(&group->spinlock);
  group->channels[chan_id] = NULL;
  esp_os_exit_critical(&group->spinlock);
  dw_gdma_release_group_handle(group);
}

static esp_err_t channel_destroy(dw_gdma_channel_t *chan)
{
  if (chan->group)
    {
      channel_unregister_from_group(chan);
    }

  if (chan->cpuint >= 0)
    {
      esp_teardown_irq(ETS_DW_GDMA_INTR_SOURCE, chan->cpuint);
    }

  free(chan);
  return ESP_OK;
}

/****************************************************************************
 * ISR
 ****************************************************************************/

static void dw_gdma_channel_default_isr(dw_gdma_channel_t *chan)
{
  dw_gdma_group_t *group = chan->group;
  dw_gdma_hal_context_t *hal = &group->hal;
  int chan_id = chan->chan_id;
  uint32_t intr_status;

  /* Clear pending interrupt event */

  intr_status = dw_gdma_ll_channel_get_intr_status(hal->dev, chan_id);
  dw_gdma_ll_channel_clear_intr(hal->dev, chan_id, intr_status);

  /* Call user callbacks */

  if (intr_status & DW_GDMA_LL_CHANNEL_EVENT_SHADOWREG_OR_LLI_INVALID_ERR)
    {
      if (chan->cbs.on_invalid_block)
        {
          intptr_t invalid_lli_addr =
              dw_gdma_ll_channel_get_current_link_list_item_addr(
                  hal->dev, chan_id);
          dw_gdma_break_event_data_t edata = {
              .invalid_lli = (dw_gdma_lli_handle_t)
                  DW_GDMA_GET_NON_CACHE_ADDR(invalid_lli_addr),
          };

          chan->cbs.on_invalid_block(chan, &edata, chan->user_data);
        }
    }

  if (intr_status & DW_GDMA_LL_CHANNEL_EVENT_BLOCK_TFR_DONE)
    {
      if (chan->cbs.on_block_trans_done)
        {
          dw_gdma_trans_done_event_data_t edata = {};
          chan->cbs.on_block_trans_done(chan, &edata, chan->user_data);
        }
    }

  if (intr_status & DW_GDMA_LL_CHANNEL_EVENT_DMA_TFR_DONE)
    {
      if (chan->cbs.on_full_trans_done)
        {
          dw_gdma_trans_done_event_data_t edata = {};
          chan->cbs.on_full_trans_done(chan, &edata, chan->user_data);
        }
    }
}

static int dw_gdma_isr_wrapper(int irq, void *context, void *arg)
{
  dw_gdma_channel_t *chan = (dw_gdma_channel_t *)arg;
  (void)irq;
  (void)context;
  dw_gdma_channel_default_isr(chan);
  return 0;
}

static esp_err_t dw_gdma_install_channel_interrupt(dw_gdma_channel_t *chan)
{
  dw_gdma_group_t *group = chan->group;
  dw_gdma_hal_context_t *hal = &group->hal;
  int chan_id = chan->chan_id;
  int cpuint;

  /* Clear pending events */

  dw_gdma_ll_channel_enable_intr_propagation(hal->dev, chan_id,
                                             UINT32_MAX, false);
  dw_gdma_ll_channel_clear_intr(hal->dev, chan_id, UINT32_MAX);

  /* Allocate and attach interrupt */

  cpuint = esp_setup_irq(ETS_DW_GDMA_INTR_SOURCE,
                         ESP_IRQ_PRIORITY_DEFAULT,
                         ESP_IRQ_TRIGGER_LEVEL,
                         dw_gdma_isr_wrapper,
                         chan);
  if (cpuint < 0)
    {
      ESP_LOGE(TAG, "alloc interrupt failed");
      return ESP_FAIL;
    }

  up_enable_irq(ESP_SOURCE2IRQ(ETS_DW_GDMA_INTR_SOURCE));
  ESP_LOGD(TAG, "install interrupt for channel (%d,%d)",
           group->group_id, chan_id);
  chan->cpuint = cpuint;
  return ESP_OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

esp_err_t dw_gdma_new_channel(const dw_gdma_channel_alloc_config_t *config,
                              dw_gdma_channel_handle_t *ret_chan)
{
  esp_err_t ret = ESP_OK;
  dw_gdma_channel_t *chan = NULL;

  ESP_RETURN_ON_FALSE(config && ret_chan, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");
  ESP_RETURN_ON_FALSE(
      config->src.num_outstanding_requests >= 1 &&
      config->src.num_outstanding_requests <=
          DW_GDMA_LL_MAX_OUTSTANDING_REQUESTS,
      ESP_ERR_INVALID_ARG, TAG, "invalid num_outstanding_requests");
  ESP_RETURN_ON_FALSE(
      config->dst.num_outstanding_requests >= 1 &&
      config->dst.num_outstanding_requests <=
          DW_GDMA_LL_MAX_OUTSTANDING_REQUESTS,
      ESP_ERR_INVALID_ARG, TAG, "invalid num_outstanding_requests");
  ESP_RETURN_ON_FALSE(
      config->chan_priority >= 0 &&
      config->chan_priority < DW_GDMA_LL_CHANNELS_PER_GROUP,
      ESP_ERR_INVALID_ARG, TAG, "invalid channel priority");

  chan = (dw_gdma_channel_t *)heap_caps_calloc(
             1, sizeof(dw_gdma_channel_t), DW_GDMA_MEM_ALLOC_CAPS);
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_NO_MEM, TAG, "no mem for channel");
  chan->cpuint = -1;

  /* Register channel to group */

  ESP_GOTO_ON_ERROR(channel_register_to_group(chan), err, TAG,
                    "register to group failed");

  dw_gdma_group_t *group = chan->group;
  dw_gdma_hal_context_t *hal = &group->hal;
  int chan_id = chan->chan_id;

  /* Check interrupt priority conflict */

  bool intr_priority_conflict = false;
  esp_os_enter_critical(&group->spinlock);
  if (group->intr_priority == -1)
    {
      group->intr_priority = config->intr_priority;
    }
  else if (config->intr_priority != 0)
    {
      intr_priority_conflict =
          (group->intr_priority != config->intr_priority);
    }

  esp_os_exit_critical(&group->spinlock);
  ESP_GOTO_ON_FALSE(!intr_priority_conflict, ESP_ERR_INVALID_STATE,
                    err, TAG, "intr_priority conflict");

  /* Basic initialization */

  portMUX_INITIALIZE(&chan->spinlock);
  chan->src_transfer_type = config->src.block_transfer_type;
  chan->dst_transfer_type = config->dst.block_transfer_type;

  /* Set transfer flow type */

  dw_gdma_ll_channel_set_trans_flow(hal->dev, chan_id,
      config->src.role, config->dst.role, config->flow_controller);

  /* Set transfer types */

  dw_gdma_ll_channel_set_src_multi_block_type(hal->dev, chan_id,
      config->src.block_transfer_type);
  dw_gdma_ll_channel_set_dst_multi_block_type(hal->dev, chan_id,
      config->dst.block_transfer_type);

  /* Set handshake interface */

  dw_gdma_ll_channel_set_src_handshake_interface(hal->dev, chan_id,
      config->src.handshake_type);
  dw_gdma_ll_channel_set_dst_handshake_interface(hal->dev, chan_id,
      config->dst.handshake_type);

  /* Set handshake peripheral */

  if (config->src.role != DW_GDMA_ROLE_MEM)
    {
      dw_gdma_ll_channel_set_src_handshake_periph(hal->dev, chan_id,
          config->src.role);
    }

  if (config->dst.role != DW_GDMA_ROLE_MEM)
    {
      dw_gdma_ll_channel_set_dst_handshake_periph(hal->dev, chan_id,
          config->dst.role);
    }

  /* Set channel priority */

  dw_gdma_ll_channel_set_priority(hal->dev, chan_id,
      config->chan_priority);

  /* Set outstanding request limits */

  dw_gdma_ll_channel_set_src_outstanding_limit(hal->dev, chan_id,
      config->src.num_outstanding_requests);
  dw_gdma_ll_channel_set_dst_outstanding_limit(hal->dev, chan_id,
      config->dst.num_outstanding_requests);

  /* Set status fetch addresses */

  dw_gdma_ll_channel_set_src_periph_status_addr(hal->dev, chan_id,
      config->src.status_fetch_addr);
  dw_gdma_ll_channel_set_dst_periph_status_addr(hal->dev, chan_id,
      config->dst.status_fetch_addr);

  /* Enable all channel events generation */

  dw_gdma_ll_channel_enable_intr_generation(hal->dev, chan_id,
      UINT32_MAX, true);

  ESP_LOGD(TAG, "new channel (%d,%d) at %p",
           group->group_id, chan_id, chan);
  *ret_chan = chan;
  return ESP_OK;

err:
  if (chan)
    {
      channel_destroy(chan);
    }

  return ret;
}

esp_err_t dw_gdma_del_channel(dw_gdma_channel_handle_t chan)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_LOGD(TAG, "del channel (%d,%d)",
           chan->group->group_id, chan->chan_id);
  return channel_destroy(chan);
}

esp_err_t dw_gdma_channel_enable_ctrl(dw_gdma_channel_handle_t chan,
                                      bool en_or_dis)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  dw_gdma_hal_context_t *hal = &chan->group->hal;
  dw_gdma_ll_channel_enable(hal->dev, chan->chan_id, en_or_dis);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_suspend_ctrl(dw_gdma_channel_handle_t chan,
                                       bool enter_or_exit)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  dw_gdma_hal_context_t *hal = &chan->group->hal;
  dw_gdma_ll_channel_suspend(hal->dev, chan->chan_id, enter_or_exit);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_abort(dw_gdma_channel_handle_t chan)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  dw_gdma_hal_context_t *hal = &chan->group->hal;
  dw_gdma_ll_channel_abort(hal->dev, chan->chan_id);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_lock(dw_gdma_channel_handle_t chan,
                               dw_gdma_lock_level_t level)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  dw_gdma_hal_context_t *hal = &chan->group->hal;
  esp_os_enter_critical(&chan->spinlock);
  dw_gdma_ll_channel_lock(hal->dev, chan->chan_id, level);
  esp_os_exit_critical(&chan->spinlock);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_unlock(dw_gdma_channel_handle_t chan)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  dw_gdma_hal_context_t *hal = &chan->group->hal;
  esp_os_enter_critical(&chan->spinlock);
  dw_gdma_ll_channel_unlock(hal->dev, chan->chan_id);
  esp_os_exit_critical(&chan->spinlock);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_continue(dw_gdma_channel_handle_t chan)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  dw_gdma_hal_context_t *hal = &chan->group->hal;
  dw_gdma_ll_channel_resume_multi_block_transfer(hal->dev, chan->chan_id);
  return ESP_OK;
}

esp_err_t dw_gdma_new_link_list(const dw_gdma_link_list_config_t *config,
                                dw_gdma_link_list_handle_t *ret_list)
{
  esp_err_t ret = ESP_OK;
  dw_gdma_link_list_item_t *items = NULL;
  struct dw_gdma_link_list_t *list = NULL;
  uint32_t num_items;
  size_t i;

  ESP_RETURN_ON_FALSE(config && ret_list, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");

  num_items = config->num_items;
  list = (struct dw_gdma_link_list_t *)heap_caps_calloc(
             1, sizeof(struct dw_gdma_link_list_t), DW_GDMA_MEM_ALLOC_CAPS);
  ESP_GOTO_ON_FALSE(list, ESP_ERR_NO_MEM, err, TAG,
                    "no mem for link list");

  /* Allocate LLI items with proper alignment */

  items = (dw_gdma_link_list_item_t *)heap_caps_aligned_calloc(
              DW_GDMA_LL_LINK_LIST_ALIGNMENT, num_items,
              sizeof(dw_gdma_link_list_item_t),
              MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  ESP_GOTO_ON_FALSE(items, ESP_ERR_NO_MEM, err, TAG,
                    "no mem for link list items");

  /* Cache sync: write back and invalidate so non-cached alias is clean */

  esp_cache_msync(items,
                  num_items * sizeof(dw_gdma_link_list_item_t),
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                  ESP_CACHE_MSYNC_FLAG_INVALIDATE |
                  ESP_CACHE_MSYNC_FLAG_UNALIGNED);

  list->num_items = num_items;
  list->items = items;
  list->items_nc = (dw_gdma_link_list_item_t *)
      DW_GDMA_GET_NON_CACHE_ADDR(items);

  /* Set up the link list chain */

  for (i = 0; i < num_items; i++)
    {
      dw_gdma_ll_lli_set_next_item_addr(list->items_nc + i,
          (uint32_t)(list->items + i + 1));
      dw_gdma_ll_lli_set_link_list_master_port(list->items_nc + i,
          DW_GDMA_LL_MASTER_PORT_MEMORY);
    }

  /* Set terminal link based on type */

  switch (config->link_type)
    {
    case DW_GDMA_LINKED_LIST_TYPE_CIRCULAR:
      dw_gdma_ll_lli_set_next_item_addr(
          list->items_nc + num_items - 1,
          (uint32_t)(list->items));
      break;
    case DW_GDMA_LINKED_LIST_TYPE_SINGLY:
      dw_gdma_ll_lli_set_next_item_addr(
          list->items_nc + num_items - 1, 0);
      break;
    }

  ESP_LOGD(TAG, "new link list @%p, items @%p", list, items);
  *ret_list = list;
  return ESP_OK;

err:
  if (list)
    {
      free(list);
    }

  if (items)
    {
      free(items);
    }

  return ret;
}

esp_err_t dw_gdma_del_link_list(dw_gdma_link_list_handle_t list)
{
  ESP_RETURN_ON_FALSE(list, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_LOGD(TAG, "del link list at %p", list);
  free(list->items);
  free(list);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_use_link_list(dw_gdma_channel_handle_t chan,
                                        dw_gdma_link_list_handle_t list)
{
  ESP_RETURN_ON_FALSE(chan && list, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");
  ESP_RETURN_ON_FALSE(
      chan->src_transfer_type == DW_GDMA_BLOCK_TRANSFER_LIST ||
      chan->dst_transfer_type == DW_GDMA_BLOCK_TRANSFER_LIST,
      ESP_ERR_INVALID_STATE, TAG, "invalid transfer type");

  dw_gdma_hal_context_t *hal = &chan->group->hal;
  int chan_id = chan->chan_id;

  dw_gdma_ll_channel_set_link_list_master_port(hal->dev, chan_id,
      DW_GDMA_LL_MASTER_PORT_MEMORY);
  dw_gdma_ll_channel_set_link_list_head_addr(hal->dev, chan_id,
      (uint32_t)(list->items));
  return ESP_OK;
}

dw_gdma_lli_handle_t dw_gdma_link_list_get_item(
    dw_gdma_link_list_handle_t list, int item_index)
{
  ESP_RETURN_ON_FALSE(list, NULL, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(item_index < (int)list->num_items, NULL, TAG,
                      "invalid item index");
  /* Return non-cached address */

  dw_gdma_link_list_item_t *lli = list->items_nc + item_index;
  return lli;
}

esp_err_t dw_gdma_lli_set_next(dw_gdma_lli_handle_t lli,
                               dw_gdma_lli_handle_t next)
{
  ESP_RETURN_ON_FALSE(lli && next, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");
  dw_gdma_ll_lli_set_next_item_addr(lli,
      DW_GDMA_GET_CACHE_ADDRESS(next));
  return ESP_OK;
}

esp_err_t dw_gdma_channel_config_transfer(
    dw_gdma_channel_handle_t chan,
    const dw_gdma_block_transfer_config_t *config)
{
  ESP_RETURN_ON_FALSE(chan && config, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");
  ESP_RETURN_ON_FALSE(
      chan->src_transfer_type != DW_GDMA_BLOCK_TRANSFER_LIST &&
      chan->dst_transfer_type != DW_GDMA_BLOCK_TRANSFER_LIST,
      ESP_ERR_INVALID_STATE, TAG, "invalid transfer type");

  dw_gdma_hal_context_t *hal = &chan->group->hal;
  int chan_id = chan->chan_id;

  dw_gdma_ll_channel_set_src_addr(hal->dev, chan_id, config->src.addr);
  dw_gdma_ll_channel_set_dst_addr(hal->dev, chan_id, config->dst.addr);
  dw_gdma_ll_channel_set_trans_block_size(hal->dev, chan_id, config->size);
  dw_gdma_ll_channel_set_src_master_port(hal->dev, chan_id,
      config->src.addr);
  dw_gdma_ll_channel_set_dst_master_port(hal->dev, chan_id,
      config->dst.addr);
  dw_gdma_ll_channel_set_src_trans_width(hal->dev, chan_id,
      config->src.width);
  dw_gdma_ll_channel_set_dst_trans_width(hal->dev, chan_id,
      config->dst.width);
  dw_gdma_ll_channel_set_src_burst_items(hal->dev, chan_id,
      config->src.burst_items);
  dw_gdma_ll_channel_set_dst_burst_items(hal->dev, chan_id,
      config->dst.burst_items);
  dw_gdma_ll_channel_set_src_burst_mode(hal->dev, chan_id,
      config->src.burst_mode);
  dw_gdma_ll_channel_set_dst_burst_mode(hal->dev, chan_id,
      config->dst.burst_mode);
  dw_gdma_ll_channel_set_src_burst_len(hal->dev, chan_id,
      config->src.burst_len);
  dw_gdma_ll_channel_set_dst_burst_len(hal->dev, chan_id,
      config->dst.burst_len);
  dw_gdma_ll_channel_enable_src_periph_status_write_back(hal->dev,
      chan_id, config->src.flags.en_status_write_back);
  dw_gdma_ll_channel_enable_dst_periph_status_write_back(hal->dev,
      chan_id, config->dst.flags.en_status_write_back);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_set_block_markers(
    dw_gdma_channel_handle_t chan,
    dw_gdma_block_markers_t markers)
{
  ESP_RETURN_ON_FALSE(chan, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(
      chan->src_transfer_type != DW_GDMA_BLOCK_TRANSFER_LIST &&
      chan->dst_transfer_type != DW_GDMA_BLOCK_TRANSFER_LIST,
      ESP_ERR_INVALID_STATE, TAG, "invalid transfer type");

  dw_gdma_hal_context_t *hal = &chan->group->hal;
  dw_gdma_ll_channel_set_block_markers(hal->dev, chan->chan_id,
      markers.en_trans_done_intr, markers.is_last, markers.is_valid);
  return ESP_OK;
}

esp_err_t dw_gdma_lli_config_transfer(
    dw_gdma_lli_handle_t lli,
    const dw_gdma_block_transfer_config_t *config)
{
  ESP_RETURN_ON_FALSE(lli && config, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");

  dw_gdma_ll_lli_set_src_addr(lli, config->src.addr);
  dw_gdma_ll_lli_set_dst_addr(lli, config->dst.addr);
  dw_gdma_ll_lli_set_trans_block_size(lli, config->size);
  dw_gdma_ll_lli_set_src_master_port(lli, config->src.addr);
  dw_gdma_ll_lli_set_dst_master_port(lli, config->dst.addr);
  dw_gdma_ll_lli_set_src_trans_width(lli, config->src.width);
  dw_gdma_ll_lli_set_dst_trans_width(lli, config->dst.width);
  dw_gdma_ll_lli_set_src_burst_items(lli, config->src.burst_items);
  dw_gdma_ll_lli_set_dst_burst_items(lli, config->dst.burst_items);
  dw_gdma_ll_lli_set_src_burst_mode(lli, config->src.burst_mode);
  dw_gdma_ll_lli_set_dst_burst_mode(lli, config->dst.burst_mode);
  dw_gdma_ll_lli_set_src_burst_len(lli, config->src.burst_len);
  dw_gdma_ll_lli_set_dst_burst_len(lli, config->dst.burst_len);
  dw_gdma_ll_lli_enable_src_periph_status_write_back(lli,
      config->src.flags.en_status_write_back);
  dw_gdma_ll_lli_enable_dst_periph_status_write_back(lli,
      config->dst.flags.en_status_write_back);
  return ESP_OK;
}

esp_err_t dw_gdma_lli_set_block_markers(dw_gdma_lli_handle_t lli,
                                        dw_gdma_block_markers_t markers)
{
  ESP_RETURN_ON_FALSE(lli, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  dw_gdma_ll_lli_set_block_markers(lli, markers.en_trans_done_intr,
      markers.is_last, markers.is_valid);
  return ESP_OK;
}

esp_err_t dw_gdma_channel_register_event_callbacks(
    dw_gdma_channel_handle_t chan,
    dw_gdma_event_callbacks_t *cbs,
    void *user_data)
{
  ESP_RETURN_ON_FALSE(chan && cbs, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");

  dw_gdma_group_t *group = chan->group;
  dw_gdma_hal_context_t *hal = &group->hal;
  int chan_id = chan->chan_id;

  /* Lazy install interrupt service */

  if (chan->cpuint < 0)
    {
      ESP_RETURN_ON_ERROR(
          dw_gdma_install_channel_interrupt(chan), TAG,
          "install interrupt service failed");
    }

  /* Enable interrupt propagation for registered events */

  dw_gdma_ll_channel_enable_intr_propagation(hal->dev, chan_id,
      DW_GDMA_LL_CHANNEL_EVENT_BLOCK_TFR_DONE,
      cbs->on_block_trans_done != NULL);
  dw_gdma_ll_channel_enable_intr_propagation(hal->dev, chan_id,
      DW_GDMA_LL_CHANNEL_EVENT_DMA_TFR_DONE,
      cbs->on_full_trans_done != NULL);
  dw_gdma_ll_channel_enable_intr_propagation(hal->dev, chan_id,
      DW_GDMA_LL_CHANNEL_EVENT_SHADOWREG_OR_LLI_INVALID_ERR,
      cbs->on_invalid_block != NULL);

  chan->user_data = user_data;
  memcpy(&chan->cbs, cbs, sizeof(dw_gdma_event_callbacks_t));
  return ESP_OK;
}

esp_err_t dw_gdma_channel_get_id(dw_gdma_channel_handle_t chan,
                                 int *channel_id)
{
  ESP_RETURN_ON_FALSE(chan && channel_id, ESP_ERR_INVALID_ARG, TAG,
                      "invalid argument");
  *channel_id = chan->chan_id;
  return ESP_OK;
}
