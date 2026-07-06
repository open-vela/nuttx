/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_dmac.c
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
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "arm64_internal.h"
#include "hardware/rk3576_dmac.h"
#include "rk3576_dmac.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA timeout (microseconds) */

#define DMA_TIMEOUT_US             1000000

/* Channel state */

#define DMA_CH_FREE                0
#define DMA_CH_ALLOCATED           1
#define DMA_CH_ACTIVE              2

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA base address */

const uint32_t g_dmac_base = RK3576_DMAC0_ADDR;

/* Channel state */

static int g_dma_ch_state[RK3576_DMA_CHANNELS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_dmac_wait_idle
 *
 * Description:
 *   Wait for DMA channel to become idle.
 *
 ****************************************************************************/

static int rk3576_dmac_wait_idle(int ch)
{
  int timeout = DMA_TIMEOUT_US;

  while (timeout--)
    {
      uint32_t chen = getreg32(g_dmac_base + DMA_CHEN);
      if (!(chen & DMA_CHEN_EN(ch)))
        {
          return OK;
        }

      up_udelay(1);
    }

  dmaerr("DMA%d: idle timeout\n", ch);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_dmac_init
 *
 * Description:
 *   Initialize DMA controller. Disable all channels.
 *
 ****************************************************************************/

void rk3576_dmac_init(void)
{
  /* Disable all channels */

  putreg32(0, g_dmac_base + DMA_CHEN);

  /* Clear all interrupts */

  putreg32(0xffffffff, g_dmac_base + DMA_INT_CLEAR);

  /* Disable all interrupts */

  putreg32(0, g_dmac_base + DMA_INT_ENABLE);

  /* Initialize channel states */

  for (int i = 0; i < RK3576_DMA_CHANNELS; i++)
    {
      g_dma_ch_state[i] = DMA_CH_FREE;
    }

  dmainfo("DMA: initialized %d channels\n", RK3576_DMA_CHANNELS);
}

/****************************************************************************
 * Name: rk3576_dmac_alloc
 *
 * Description:
 *   Allocate a free DMA channel.
 *
 ****************************************************************************/

int rk3576_dmac_alloc(void)
{
  irqstate_t flags = up_irq_save();

  for (int i = 0; i < RK3576_DMA_CHANNELS; i++)
    {
      if (g_dma_ch_state[i] == DMA_CH_FREE)
        {
          g_dma_ch_state[i] = DMA_CH_ALLOCATED;
          up_irq_restore(flags);
          dmainfo("DMA: allocated channel %d\n", i);
          return i;
        }
    }

  up_irq_restore(flags);
  dmaerr("DMA: no free channels\n");
  return -ENOSPC;
}

/****************************************************************************
 * Name: rk3576_dmac_free
 *
 * Description:
 *   Free a DMA channel.
 *
 ****************************************************************************/

void rk3576_dmac_free(int ch)
{
  if (ch < 0 || ch >= RK3576_DMA_CHANNELS)
    {
      return;
    }

  /* Stop the channel first */

  rk3576_dmac_stop(ch);

  g_dma_ch_state[ch] = DMA_CH_FREE;

  dmainfo("DMA: freed channel %d\n", ch);
}

/****************************************************************************
 * Name: rk3576_dmac_transfer
 *
 * Description:
 *   Perform a DMA transfer using the given descriptor.
 *
 ****************************************************************************/

int rk3576_dmac_transfer(int ch, const struct rk3576_dma_xfer_s *xfer)
{
  uint32_t cfg;
  uint32_t ctl;

  if (ch < 0 || ch >= RK3576_DMA_CHANNELS)
    {
      return -EINVAL;
    }

  if (xfer == NULL)
    {
      return -EINVAL;
    }

  if (g_dma_ch_state[ch] != DMA_CH_ALLOCATED)
    {
      dmaerr("DMA%d: not allocated\n", ch);
      return -EPERM;
    }

  /* Stop channel before reconfiguration */

  putreg32(0, g_dmac_base + DMA_CHEN);

  /* Configure channel */

  cfg = 0;

  /* Set transfer mode */

  switch (xfer->mode)
    {
      case RK3576_DMA_MODE_MEM2MEM:
        cfg |= DMA_CH_CFG_MODE_MEM2MEM;
        cfg |= DMA_CH_CFG_SRC_INC;
        cfg |= DMA_CH_CFG_DST_INC;
        break;
      case RK3576_DMA_MODE_MEM2PER:
        cfg |= DMA_CH_CFG_MODE_MEM2PER;
        cfg |= DMA_CH_CFG_SRC_INC;
        cfg |= DMA_CH_CFG_DST_FIX;
        break;
      case RK3576_DMA_MODE_PER2MEM:
        cfg |= DMA_CH_CFG_MODE_PER2MEM;
        cfg |= DMA_CH_CFG_SRC_FIX;
        cfg |= DMA_CH_CFG_DST_INC;
        break;
      default:
        return -EINVAL;
    }

  /* Set transfer width */

  switch (xfer->width)
    {
      case RK3576_DMA_WIDTH_8BIT:
        cfg |= DMA_CH_CFG_SRC_WIDTH_8 | DMA_CH_CFG_DST_WIDTH_8;
        break;
      case RK3576_DMA_WIDTH_16BIT:
        cfg |= DMA_CH_CFG_SRC_WIDTH_16 | DMA_CH_CFG_DST_WIDTH_16;
        break;
      case RK3576_DMA_WIDTH_32BIT:
        cfg |= DMA_CH_CFG_SRC_WIDTH_32 | DMA_CH_CFG_DST_WIDTH_32;
        break;
      default:
        return -EINVAL;
    }

  /* Set burst size to 1 */

  cfg |= DMA_CH_CFG_SRC_BURST_1 | DMA_CH_CFG_DST_BURST_1;

  putreg32(cfg, g_dmac_base + DMA_CH_CFG(ch));

  /* Configure control */

  ctl = DMA_CH_CTL_INT_EN;

  switch (xfer->width)
    {
      case RK3576_DMA_WIDTH_8BIT:
        ctl |= DMA_CH_CTL_TR_WIDTH_8;
        break;
      case RK3576_DMA_WIDTH_16BIT:
        ctl |= DMA_CH_CTL_TR_WIDTH_16;
        break;
      case RK3576_DMA_WIDTH_32BIT:
        ctl |= DMA_CH_CTL_TR_WIDTH_32;
        break;
    }

  putreg32(ctl, g_dmac_base + DMA_CH_CTL(ch));

  /* Set source and destination addresses */

  putreg32(xfer->src_addr, g_dmac_base + DMA_CH_SRC_ADDR(ch));
  putreg32(xfer->dst_addr, g_dmac_base + DMA_CH_DST_ADDR(ch));

  /* Set transfer size */

  putreg32(xfer->length, g_dmac_base + DMA_CH_CTL2(ch));

  /* Enable channel */

  putreg32(DMA_CHEN_EN(ch), g_dmac_base + DMA_CHEN);
  g_dma_ch_state[ch] = DMA_CH_ACTIVE;

  dmainfo("DMA%d: transfer %u bytes, mode %d\n", ch, xfer->length, xfer->mode);

  return OK;
}

/****************************************************************************
 * Name: rk3576_dmac_mem2mem
 *
 * Description:
 *   Perform memory-to-memory DMA transfer.
 *
 ****************************************************************************/

int rk3576_dmac_mem2mem(int ch, uint32_t dst, uint32_t src, uint32_t len)
{
  struct rk3576_dma_xfer_s xfer =
  {
    .src_addr = src,
    .dst_addr = dst,
    .length = len,
    .mode = RK3576_DMA_MODE_MEM2MEM,
    .width = RK3576_DMA_WIDTH_32BIT,
  };

  return rk3576_dmac_transfer(ch, &xfer);
}

/****************************************************************************
 * Name: rk3576_dmac_per2mem
 *
 * Description:
 *   Perform peripheral-to-memory DMA transfer.
 *
 ****************************************************************************/

int rk3576_dmac_per2mem(int ch, uint32_t dst, uint32_t src_per, uint32_t len)
{
  struct rk3576_dma_xfer_s xfer =
  {
    .src_addr = src_per,
    .dst_addr = dst,
    .length = len,
    .mode = RK3576_DMA_MODE_PER2MEM,
    .width = RK3576_DMA_WIDTH_8BIT,
  };

  return rk3576_dmac_transfer(ch, &xfer);
}

/****************************************************************************
 * Name: rk3576_dmac_mem2per
 *
 * Description:
 *   Perform memory-to-peripheral DMA transfer.
 *
 ****************************************************************************/

int rk3576_dmac_mem2per(int ch, uint32_t dst_per, uint32_t src, uint32_t len)
{
  struct rk3576_dma_xfer_s xfer =
  {
    .src_addr = src,
    .dst_addr = dst_per,
    .length = len,
    .mode = RK3576_DMA_MODE_MEM2PER,
    .width = RK3576_DMA_WIDTH_8BIT,
  };

  return rk3576_dmac_transfer(ch, &xfer);
}

/****************************************************************************
 * Name: rk3576_dmac_start
 *
 * Description:
 *   Start DMA transfer on a channel.
 *
 ****************************************************************************/

void rk3576_dmac_start(int ch)
{
  uint32_t chen;

  if (ch < 0 || ch >= RK3576_DMA_CHANNELS)
    {
      return;
    }

  chen = getreg32(g_dmac_base + DMA_CHEN);
  chen |= DMA_CHEN_EN(ch);
  putreg32(chen, g_dmac_base + DMA_CHEN);

  g_dma_ch_state[ch] = DMA_CH_ACTIVE;

  dmainfo("DMA%d: started\n", ch);
}

/****************************************************************************
 * Name: rk3576_dmac_stop
 *
 * Description:
 *   Stop DMA transfer on a channel.
 *
 ****************************************************************************/

void rk3576_dmac_stop(int ch)
{
  uint32_t chen;

  if (ch < 0 || ch >= RK3576_DMA_CHANNELS)
    {
      return;
    }

  chen = getreg32(g_dmac_base + DMA_CHEN);
  chen &= ~DMA_CHEN_EN(ch);
  putreg32(chen, g_dmac_base + DMA_CHEN);

  g_dma_ch_state[ch] = DMA_CH_ALLOCATED;

  dmainfo("DMA%d: stopped\n", ch);
}

/****************************************************************************
 * Name: rk3576_dmac_abort
 *
 * Description:
 *   Abort DMA transfer on a channel.
 *
 ****************************************************************************/

void rk3576_dmac_abort(int ch)
{
  uint32_t chen;

  if (ch < 0 || ch >= RK3576_DMA_CHANNELS)
    {
      return;
    }

  chen = getreg32(g_dmac_base + DMA_CHEN);
  chen |= DMA_CHEN_ABORT(ch);
  putreg32(chen, g_dmac_base + DMA_CHEN);

  /* Wait for abort to complete */

  {
    int timeout = 1000;
    while (timeout--)
      {
        chen = getreg32(g_dmac_base + DMA_CHEN);
        if (!(chen & DMA_CHEN_ABORT(ch)))
          {
            break;
          }

        up_udelay(1);
      }
  }

  g_dma_ch_state[ch] = DMA_CH_ALLOCATED;

  dmainfo("DMA%d: aborted\n", ch);
}

/****************************************************************************
 * Name: rk3576_dmac_is_busy
 *
 * Description:
 *   Check if a DMA channel is busy.
 *
 ****************************************************************************/

int rk3576_dmac_is_busy(int ch)
{
  uint32_t chen;

  if (ch < 0 || ch >= RK3576_DMA_CHANNELS)
    {
      return -EINVAL;
    }

  chen = getreg32(g_dmac_base + DMA_CHEN);
  return (chen & DMA_CHEN_EN(ch)) ? 1 : 0;
}

/****************************************************************************
 * Name: rk3576_dmac_wait
 *
 * Description:
 *   Wait for DMA transfer to complete.
 *
 ****************************************************************************/

void rk3576_dmac_wait(int ch)
{
  if (ch < 0 || ch >= RK3576_DMA_CHANNELS)
    {
      return;
    }

  /* Wait for transfer to complete */

  rk3576_dmac_wait_idle(ch);

  g_dma_ch_state[ch] = DMA_CH_ALLOCATED;
}
