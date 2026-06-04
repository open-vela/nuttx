/****************************************************************************
 * arch/arm/src/t113/t113_dma.c
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>
#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/t113_ccu.h"
#include "hardware/t113_dma.h"
#include "t113_ccu.h"
#include "t113_dma.h"

#ifdef CONFIG_T113_DMA

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* DMA hardware descriptor (must be 4-byte aligned, allocated from SRAM) */

struct t113_dma_desc_s
{
  uint32_t config;
  uint32_t src;
  uint32_t dst;
  uint32_t len;
  uint32_t param;
  uint32_t link;
};

/* Per-channel state.  The embedded descriptor is cache-line aligned so
 * up_flush_dcache() in t113_dmastart() cannot evict unrelated fields of
 * adjacent channels that happen to share a cache line.
 */

struct t113_dma_chan_s
{
  bool              inuse;
  uint8_t           mode;     /* DMAC_MODE_REGN value (handshake bits) */
  dma_callback_t    callback;
  void             *arg;
  struct t113_dma_desc_s desc aligned_data(64);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct t113_dma_chan_s g_dmachan[T113_DMA_NCHANNELS];

/* Channel allocation lock.  Spinlock (not mutex) because callers may run
 * with another spinlock held (e.g. uart_open holds dev->lock when calling
 * up_setup -> t113_dmachannel via the upstream serial framework).  Holding
 * a spinlock across a sleeping mutex deadlocks on SMP - task migrates to
 * another CPU while still owning the spinlock and the rspinlock owner
 * field becomes stale.  Critical section here is O(NCHANNELS) scan,
 * nanoseconds, well within spinlock budget.
 */

static spinlock_t g_dmalock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* IRQ 82 (DMAC_NS) services chans 0-7; IRQ 83 (DMAC_S) is wired to
 * the DSP only and never reaches a non-secure ARM, so chans 8-15
 * are unusable from this CPU and excluded from the allocator pool
 * (see T113_DMA_NCHANNELS in t113_dma.h).  Only PEND_REG0/EN0 ever
 * carry events for us.
 */

static int t113_dma_irq0(int irq, void *context, void *arg)
{
  uint32_t en0   = getreg32(T113_DMAC_IRQ_EN0);
  uint32_t pend0 = getreg32(T113_DMAC_IRQ_PEND0);
  struct t113_dma_chan_s *chan;
  uint32_t bits;
  int n;

  if (pend0 == 0)
    {
      return OK;
    }

  putreg32(pend0, T113_DMAC_IRQ_PEND0);

  for (n = 0; n < T113_DMA_NCHANNELS; n++)
    {
      /* Only fire callback for IRQ sources that are both pending
       * AND enabled.  The T113 DMAC sets PEND bits unconditionally
       * (e.g. HLFDONE at half-transfer) even when not enabled in
       * IRQ_EN - without the mask a spurious HLFDONE at byte 128
       * of a 256-byte transfer would fire callback prematurely.
       */

      bits  = (pend0 >> (n * 4)) & 0xf;
      bits &= (en0   >> (n * 4)) & 0xf;
      if (bits == 0)
        {
          continue;
        }

      chan = &g_dmachan[n];
      if (chan->callback != NULL)
        {
          chan->callback((DMA_HANDLE)chan, (uint8_t)bits, chan->arg);
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_dma_initialize(void)
{
#ifdef CONFIG_T113_RPTUN_SLAVE
  /* DMA0 is statically partitioned to the master in AMP mode.  The
   * slave never allocates a channel, never wires the IRQ, and must
   * not retouch the BGR/MBUS gates that master already configured.
   */

  return;
#else
  memset(g_dmachan, 0, sizeof(g_dmachan));

  /* DMA bus clock gate: deassert reset (bit 16), enable clock (bit 0).
   * Each RMW is serialized by the shared CCU layer.  The udelay between
   * the two writes runs outside the lock by design.
   */

  t113_ccu_modify(T113_CCU_DMA_BGR, 0, (1 << 16));
  up_udelay(20);
  t113_ccu_modify(T113_CCU_DMA_BGR, 0, (1 << 0));

  /* MBUS master clock gating: use fixed value matching boot0 set_mbus().
   * Whole-register write, but routed through the CCU layer so it stays
   * serialized against any future RMW on MBUS_MAT (e.g. CE).
   */

  t113_ccu_modify(T113_CCU_MBUS_MAT, 0xffffffff, T113_CCU_MBUS_DMA_GATING);

  /* Disable DMA auto-gating (channel + common + mclk).  Polarity per
   * T113 user manual rev 1.4 section 8 (DMA_AUTO_GATE_REG):
   * 0=auto-gating ENABLED, 1=DISABLED - write 0x7 to disable all
   * three gates.  With auto-gating enabled the DMAC clock is cut a
   * few cycles after a channel's PKGDONE bit is set, before the CPU
   * can ack IRQ 82/83.  GIC treats DMAC IRQs as level-sensitive; the
   * gated-away clock leaves the DMAC IRQ line deasserted while
   * IRQ_PEND keeps the bit set, so GIC ISPENDR never latches and the
   * completion callback never fires -> 5-port parallel UART DMA
   * stalls at the first ttyS5 PKGDONE under load.
   */

  putreg32(0x7, T113_DMAC_AUTO_GATE);

  /* Clear and disable all pending interrupts in group 0.  Group 1
   * (PEND_REG1/EN_REG1) is left untouched - its IRQ line goes to
   * the DSP, so we neither service nor disturb it.
   */

  putreg32(0xffffffff, T113_DMAC_IRQ_PEND0);
  putreg32(0, T113_DMAC_IRQ_EN0);

  irq_attach(T113_IRQ_DMA0, t113_dma_irq0, NULL);
  up_enable_irq(T113_IRQ_DMA0);
#endif /* CONFIG_T113_RPTUN_SLAVE */
}

DMA_HANDLE t113_dmachannel(void)
{
  struct t113_dma_chan_s *chan = NULL;
  irqstate_t flags;
  int n;

  /* DMA is owned by the master image in AMP.  arm_dma_initialize() is a
   * no-op on the slave; allowing slave drivers to call this would hand
   * out an unconfigured channel.
   */

#ifdef CONFIG_T113_RPTUN_SLAVE
  DEBUGPANIC();
  return NULL;
#endif

  flags = spin_lock_irqsave(&g_dmalock);

  for (n = 0; n < T113_DMA_NCHANNELS; n++)
    {
      if (!g_dmachan[n].inuse)
        {
          chan = &g_dmachan[n];
          chan->inuse    = true;
          chan->callback = NULL;
          chan->arg      = NULL;
          break;
        }
    }

  spin_unlock_irqrestore(&g_dmalock, flags);
  return (DMA_HANDLE)chan;
}

void t113_dmafree(DMA_HANDLE handle)
{
  struct t113_dma_chan_s *chan = (struct t113_dma_chan_s *)handle;
  irqstate_t flags;

  DEBUGASSERT(chan != NULL && chan->inuse);

  t113_dmastop(handle);

  flags = spin_lock_irqsave(&g_dmalock);
  chan->inuse = false;
  spin_unlock_irqrestore(&g_dmalock, flags);
}

int t113_dmasetup(DMA_HANDLE handle,
                  uintptr_t src, uintptr_t dst, size_t len,
                  const struct t113_dma_config_s *cfg)
{
  struct t113_dma_chan_s *chan = (struct t113_dma_chan_s *)handle;
  uint32_t config;

  DEBUGASSERT(chan != NULL && chan->inuse && cfg != NULL);
  DEBUGASSERT(len > 0 && len <= DMAC_MAX_XFER_LEN);

  config  = (cfg->src_drq  << DMAC_CFG_SRC_DRQ_SHIFT);
  config |= (cfg->src_width << DMAC_CFG_SRC_WIDTH_SHIFT);
  config |= (cfg->src_burst << DMAC_CFG_SRC_BURST_SHIFT);
  config |= (cfg->src_linear ? DMAC_CFG_SRC_ADDR_LINEAR :
                                DMAC_CFG_SRC_ADDR_IO);
  config |= (cfg->dst_drq  << DMAC_CFG_DST_DRQ_SHIFT);
  config |= (cfg->dst_width << DMAC_CFG_DST_WIDTH_SHIFT);
  config |= (cfg->dst_burst << DMAC_CFG_DST_BURST_SHIFT);
  config |= (cfg->dst_linear ? DMAC_CFG_DST_ADDR_LINEAR :
                                DMAC_CFG_DST_ADDR_IO);
  if (cfg->bmode)
    {
      config |= DMAC_CFG_BMODE_SEL;
    }

  chan->desc.config = config;
  chan->desc.src    = (uint32_t)src;
  chan->desc.dst    = (uint32_t)dst;
  chan->desc.len    = (uint32_t)len;
  chan->desc.param  = DMAC_PARA_NORMAL_WAIT;
  chan->desc.link   = cfg->circular ?
                      (uint32_t)(uintptr_t)&chan->desc :
                      DMAC_DESC_END;
  chan->mode        = cfg->mode;

  return OK;
}

int t113_dmastart(DMA_HANDLE handle,
                  dma_callback_t callback, void *arg)
{
  struct t113_dma_chan_s *chan = (struct t113_dma_chan_s *)handle;
  int n;
  uint32_t irq_en;
  uint32_t irq_bits;
  irqstate_t flags;

  DEBUGASSERT(chan != NULL && chan->inuse);

  n = (int)(chan - g_dmachan);
  DEBUGASSERT(n >= 0 && n < T113_DMA_NCHANNELS);

  chan->callback = callback;
  chan->arg      = arg;

  /* Flush descriptor to memory before DMA reads it */

  up_flush_dcache((uintptr_t)&chan->desc,
                  (uintptr_t)&chan->desc + sizeof(chan->desc));

  flags = enter_critical_section();

  /* Clear any stale pending bits for this channel BEFORE enabling
   * the IRQ.  The T113 DMAC sets PEND bits (especially HLFDONE)
   * unconditionally, even when they are not enabled in IRQ_EN.
   * If a stale HLFDONE is left pending when we enable PKGDONE,
   * the next IRQ entry could see it and fire a spurious callback.
   */

  putreg32(0xf << (n * 4), T113_DMAC_IRQ_PEND0);

  /* Enable interrupts for this channel.
   * For circular mode: enable both HLFDONE + PKGDONE to get
   * callbacks at 50% and 100% of the buffer (ping-pong).
   * For single-shot: only PKGDONE.
   */

  irq_bits = DMAC_IRQ_PKGDONE(n);
  if (chan->desc.link != DMAC_DESC_END)
    {
      irq_bits |= DMAC_IRQ_HLFDONE(n);
    }

  irq_en = getreg32(T113_DMAC_IRQ_EN0);
  irq_en |= irq_bits;
  putreg32(irq_en, T113_DMAC_IRQ_EN0);

  /* Configure handshake mode (required for peripheral DMA).
   * Must be written before enabling the channel.
   */

  putreg32(chan->mode, T113_DMAC_MODE(n));
  putreg32((uint32_t)(uintptr_t)&chan->desc, T113_DMAC_DESC(n));
  putreg32(DMAC_CHAN_ENABLE, T113_DMAC_EN(n));

  leave_critical_section(flags);

  return OK;
}

void t113_dmapause(DMA_HANDLE handle)
{
  struct t113_dma_chan_s *chan = (struct t113_dma_chan_s *)handle;
  int n;
  int retry = 100;

  DEBUGASSERT(chan != NULL && chan->inuse);

  n = (int)(chan - g_dmachan);
  DEBUGASSERT(n >= 0 && n < T113_DMA_NCHANNELS);

  putreg32(DMAC_CHAN_PAUSE, T113_DMAC_PAU(n));
  UP_DSB();

  while ((getreg32(T113_DMAC_STATUS) & (1 << n)) != 0 &&
         --retry > 0)
    {
      up_udelay(1);
    }
}

void t113_dmaresume(DMA_HANDLE handle)
{
  struct t113_dma_chan_s *chan = (struct t113_dma_chan_s *)handle;
  int n;

  DEBUGASSERT(chan != NULL && chan->inuse);

  n = (int)(chan - g_dmachan);
  DEBUGASSERT(n >= 0 && n < T113_DMA_NCHANNELS);

  /* Clear the PAUSE bit so the channel resumes the in-flight transfer.
   * The descriptor and channel-enable bit are still programmed from the
   * preceding t113_dmastart(), so no further setup is needed.
   */

  putreg32(0, T113_DMAC_PAU(n));
  UP_DSB();
}

void t113_dmastop(DMA_HANDLE handle)
{
  struct t113_dma_chan_s *chan = (struct t113_dma_chan_s *)handle;
  int n;
  uint32_t irq_en;
  irqstate_t flags;

  DEBUGASSERT(chan != NULL);

  n = (int)(chan - g_dmachan);
  DEBUGASSERT(n >= 0 && n < T113_DMA_NCHANNELS);

  flags = enter_critical_section();

  putreg32(0, T113_DMAC_EN(n));
  putreg32(0, T113_DMAC_PAU(n));

  irq_en  = getreg32(T113_DMAC_IRQ_EN0);
  irq_en &= ~(DMAC_IRQ_PKGDONE(n) | DMAC_IRQ_HLFDONE(n));
  putreg32(irq_en, T113_DMAC_IRQ_EN0);

  /* Clear any pending IRQ bits for this channel so a stale
   * HLFDONE/PKGDONE left behind by the aborted transfer cannot
   * fire a spurious callback on the next t113_dmastart().
   */

  putreg32(0xf << (n * 4), T113_DMAC_IRQ_PEND0);

  leave_critical_section(flags);
}

size_t t113_dmaresidual(DMA_HANDLE handle)
{
  struct t113_dma_chan_s *chan = (struct t113_dma_chan_s *)handle;
  int n;

  DEBUGASSERT(chan != NULL && chan->inuse);

  n = (int)(chan - g_dmachan);
  DEBUGASSERT(n >= 0 && n < T113_DMA_NCHANNELS);

  return (size_t)getreg32(T113_DMAC_CNT(n));
}

#endif /* CONFIG_T113_DMA */
