/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_lsia_dma.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_lsia_dma.h"

#ifdef CONFIG_LS2K0300_LSIA_DMA

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SoC connects the I2S controller to DMA channel 0 (receive) and
 * channel 1 (transmit).  The mapping is fixed in hardware.
 */

#define LSIA_DMA_RX_CHANNEL          0
#define LSIA_DMA_TX_CHANNEL          1

#define LSIA_DMA_NCHANNELS           2

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Transfer complete callback.  Called from the context of the DMA
 * interrupt handler with result == OK on success or a negated errno
 * value on failure.
 */

typedef void (*lsia_dma_callback_t)(FAR void *arg, int result);

/* Keep the definitions below in sync with the callers in ls2k0300_i2s.c */

int ls2k0300_lsia_dma_initialize(void);
int ls2k0300_lsia_dma_start(int chn, uint32_t periph, uint32_t mem,
                            uint32_t nbytes, bool tomem,
                            lsia_dma_callback_t cb, FAR void *arg);
void ls2k0300_lsia_dma_stop(int chn);

/* Per channel state */

struct lsia_dma_channel_s
{
  lsia_dma_callback_t callback;  /* Completion callback */
  FAR void *arg;                 /* Callback argument */
  bool active;                   /* A transfer is in flight */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ls2k0300_lsia_dma_isr(int irq, FAR void *context,
                                FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lsia_dma_channel_s g_channels[LSIA_DMA_NCHANNELS];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dma_read / dma_write
 *
 * Description:
 *   Read/write a register of the LSIA DMA controller.
 *
 ****************************************************************************/

static inline uint32_t dma_read(uint32_t offset)
{
  return getreg32(LS2K0300_LSIA_DMA_BASE + offset);
}

static inline void dma_write(uint32_t offset, uint32_t value)
{
  putreg32(value, LS2K0300_LSIA_DMA_BASE + offset);
}

/****************************************************************************
 * Name: ls2k0300_lsia_dma_isr
 *
 * Description:
 *   DMA channel interrupt handler.  The LSIA DMA packs four status bits
 *   per channel into the ISR/IFCR registers; the channel number is
 *   recovered from the arg passed at irq_attach() time.
 *
 ****************************************************************************/

static int ls2k0300_lsia_dma_isr(int irq, FAR void *context,
                                FAR void *arg)
{
  FAR struct lsia_dma_channel_s *ch = (FAR struct lsia_dma_channel_s *)arg;
  int chn = (int)(ch - g_channels);
  uint32_t status;
  uint32_t ccr;
  int result;

  if (chn < 0 || chn >= LSIA_DMA_NCHANNELS)
    {
      return -EINVAL;
    }

  status = (dma_read(LSIA_DMA_ISR) >> (4 * chn)) &
           (LSIA_DMA_TCI | LSIA_DMA_HTI | LSIA_DMA_TEI);

  /* Only the events that are enabled in the channel configuration are
   * reported to the handler.
   */

  ccr = dma_read(LSIA_DMA_CCR(chn));
  status &= ccr;

  if (status == 0)
    {
      return OK;
    }

  /* Clear the reported events */

  dma_write(LSIA_DMA_IFCR, status << (4 * chn));

  result = (status & LSIA_DMA_TEI) ? -EIO : OK;

  if ((status & LSIA_DMA_TCI) && ch->active)
    {
      ch->active = false;

      if (ch->callback != NULL)
        {
          lsia_dma_callback_t cb = ch->callback;
          FAR void *cbarg = ch->arg;

          ch->callback = NULL;
          cb(cbarg, result);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ls2k0300_lsia_dma_initialize
 *
 * Description:
 *   Attach the interrupt handlers of the DMA channels used by the I2S
 *   controller.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ls2k0300_lsia_dma_initialize(void)
{
  int chn;

  for (chn = 0; chn < LSIA_DMA_NCHANNELS; chn++)
    {
      irq_attach(LS2K0300_IRQ_IODMA(chn), ls2k0300_lsia_dma_isr,
                 &g_channels[chn]);
      up_enable_irq(LS2K0300_IRQ_IODMA(chn));
    }

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_lsia_dma_start
 *
 * Description:
 *   Start a transfer on the given channel.  Only one transfer may be in
 *   flight per channel; the previous transfer is stopped.
 *
 * Input Parameters:
 *   chn    - DMA channel number
 *   periph - Physical address of the peripheral data register
 *   mem    - Physical address of the memory buffer
 *   nbytes - Transfer size in bytes.  Must be a multiple of 4.
 *   tomem  - true: peripheral to memory, false: memory to peripheral
 *   cb     - Completion callback
 *   arg    - Callback argument
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ls2k0300_lsia_dma_start(int chn, uint32_t periph, uint32_t mem,
                            uint32_t nbytes, bool tomem,
                            lsia_dma_callback_t cb, FAR void *arg)
{
  uint32_t ccr;
  uint32_t items;

  if (chn < 0 || chn >= LSIA_DMA_NCHANNELS)
    {
      return -EINVAL;
    }

  if (nbytes == 0 || (nbytes & 3) != 0)
    {
      return -EINVAL;
    }

  items = nbytes / 4;
  if (items > 0xffff)
    {
      return -EINVAL;
    }

  /* Stop any transfer that may still be running on this channel */

  ccr = dma_read(LSIA_DMA_CCR(chn));
  dma_write(LSIA_DMA_CCR(chn),
            ccr & ~(LSIA_DMA_CCR_EN | LSIA_DMA_CCR_TCIE |
                    LSIA_DMA_CCR_HTIE | LSIA_DMA_CCR_TEIE));

  g_channels[chn].callback = cb;
  g_channels[chn].arg = arg;
  g_channels[chn].active = true;

  /* High priority, 32 bit peripheral accesses, incrementing memory
   * address, transfer complete interrupt.  Both sides are 32 bit wide:
   * one bus word is one packed stereo frame, which is what the I2S data
   * register serves.  The reference driver keeps the two widths equal,
   * and the manual only describes what happens to the *data* when they
   * differ, not how the memory address advances - so unequal widths are
   * left alone.
   */

  ccr = LSIA_DMA_CCR_PL(2) |
        LSIA_DMA_CCR_MSIZE(LSIA_DMA_WIDTH_32BIT) |
        LSIA_DMA_CCR_PSIZE(LSIA_DMA_WIDTH_32BIT) |
        LSIA_DMA_CCR_MINC |
        LSIA_DMA_CCR_TCIE;

  if (tomem)
    {
      /* 0: read from the peripheral */
      ccr &= ~LSIA_DMA_CCR_DIR;
    }
  else
    {
      /* 1: read from the memory */
      ccr |= LSIA_DMA_CCR_DIR;
    }

  dma_write(LSIA_DMA_CCR(chn), ccr);
  dma_write(LSIA_DMA_CNDTR(chn), items);
  dma_write(LSIA_DMA_CPAR(chn), periph);
  dma_write(LSIA_DMA_CMAR(chn), mem);

  /* Start the transfer */

  dma_write(LSIA_DMA_CCR(chn), ccr | LSIA_DMA_CCR_EN);

  return OK;
}

/****************************************************************************
 * Name: ls2k0300_lsia_dma_stop
 *
 * Description:
 *   Stop the transfer running on the given channel.
 *
 ****************************************************************************/

void ls2k0300_lsia_dma_stop(int chn)
{
  uint32_t ccr;

  if (chn < 0 || chn >= LSIA_DMA_NCHANNELS)
    {
      return;
    }

  ccr = dma_read(LSIA_DMA_CCR(chn));
  dma_write(LSIA_DMA_CCR(chn),
            ccr & ~(LSIA_DMA_CCR_EN | LSIA_DMA_CCR_TCIE |
                    LSIA_DMA_CCR_HTIE | LSIA_DMA_CCR_TEIE));

  g_channels[chn].active = false;
  g_channels[chn].callback = NULL;
}

#endif /* CONFIG_LS2K0300_LSIA_DMA */
