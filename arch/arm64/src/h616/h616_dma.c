/****************************************************************************
 * arch/arm64/src/h616/h616_dma.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/* NuttX driver for the Allwinner H616 DMA controller (DMAC).
 *
 * The controller is the sun6i/A100 compatible design with 16 channels
 * driven through linked-list (LLI) descriptors in DRAM.  It implements
 * the generic NuttX struct dma_chan_s vtable (see include/nuttx/dma/dma.h)
 * so that clients such as the 16550 UART driver (CONFIG_16550_UARTx_DMA)
 * can use it through DMA_CONFIG / DMA_START / DMA_START_CYCLIC.
 *
 * Reference: Linux drivers/dma/sun6i-dma.c, compatible
 * "allwinner,sun50i-h616-dma" -> sun50i_a100_dma_cfg
 * (set_drq_h6 / set_mode_h6 bit fields, H3 style clock autogate).
 *
 * Address notes: this port runs flat (VA == PA, DRAM at 0x40000000), so
 * all "physical" addresses the clients hand us are used as-is.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/uart_16550.h>

#include "arm64_internal.h"
#include "hardware/h616_memorymap.h"
#include "hardware/h616_dma.h"
#include "h616_dma.h"

#ifdef CONFIG_H616_DMA

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Channel state (extends struct dma_chan_s) */

struct h616_dmachan_s
{
  struct dma_chan_s    chan;           /* vtable (must be first) */
  uint8_t              num;            /* hardware channel number */
  volatile bool        inuse;          /* channel claimed */
  volatile bool        cyclic;         /* running a cyclic transfer */
  uint8_t              drq;            /* peripheral DRQ port on this channel */

  /* Transfer configuration (from dma_config_s) */

  uint8_t              src_width;      /* bytes, 1/2/4/8 */
  uint8_t              dst_width;
  uint8_t              direction;      /* DMA_MEM_TO_DEV / DMA_DEV_TO_MEM */

  dma_callback_t       callback;       /* completion callback */
  FAR void            *arg;            /* callback argument */

  size_t               xfer_len;       /* total length of the active transfer */
  size_t               period_len;     /* cyclic period length */
  size_t               cyc_total;      /* cyclic: bytes completed so far */

  /* LLI descriptors for the active transfer (static, cache-line aligned) */

  struct h616_dma_lli_s lli[H616_DMAC_MAXLLI] __attribute__((aligned(64)));
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct h616_dmachan_s g_dmachan[H616_DMAC_NCHANNELS];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int h616_dmac_config(FAR struct dma_chan_s *chan,
                            FAR const struct dma_config_s *cfg);
static int h616_dmac_start(FAR struct dma_chan_s *chan,
                           dma_callback_t callback, FAR void *arg,
                           uintptr_t dst, uintptr_t src, size_t len);
static int h616_dmac_start_cyclic(FAR struct dma_chan_s *chan,
                                  dma_callback_t callback, FAR void *arg,
                                  uintptr_t dst, uintptr_t src,
                                  size_t len, size_t period_len);
static int h616_dmac_stop(FAR struct dma_chan_s *chan);
static int h616_dmac_pause(FAR struct dma_chan_s *chan);
static int h616_dmac_resume(FAR struct dma_chan_s *chan);
static size_t h616_dmac_residual(FAR struct dma_chan_s *chan);

static const struct dma_ops_s g_dmac_ops =
{
  h616_dmac_config,
  h616_dmac_start,
  h616_dmac_start_cyclic,
#ifdef CONFIG_DMA_LINK
  NULL,
#endif
  h616_dmac_stop,
  h616_dmac_pause,
  h616_dmac_resume,
  h616_dmac_residual,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uintptr_t h616_chaddr(unsigned int ch, uint32_t offset)
{
  return H616_DMA_ADDR + H616_DMAC_CH_BASE + ch * H616_DMAC_CH_STRIDE +
         offset;
}

/* Convert a byte width to the 2-bit CUR_CFG encoding */

static uint8_t h616_width_code(unsigned int width)
{
  switch (width)
    {
    case 1:
      return 0;
    case 2:
      return 1;
    case 4:
      return 2;
    case 8:
      return 3;
    default:
      return 0;
    }
}

/* Compose the CUR_CFG word for one LLI */

static uint32_t h616_compose_cfg(FAR struct h616_dmachan_s *priv)
{
  uint32_t cfg;

  cfg = H616_DMA_CFG_SRC_WIDTH(h616_width_code(priv->src_width)) |
        H616_DMA_CFG_DST_WIDTH(h616_width_code(priv->dst_width)) |
        H616_DMA_CFG_SRC_BURST(0) | H616_DMA_CFG_DST_BURST(0);

  if (priv->direction == DMA_MEM_TO_DEV)
    {
      /* Memory (SDRAM, linear) -> peripheral (drq, io mode) */

      cfg |= H616_DMA_CFG_SRC_DRQ(H616_DMA_DRQ_SDRAM) |
             H616_DMA_CFG_SRC_MODE(H616_DMA_MODE_LINEAR) |
             H616_DMA_CFG_DST_DRQ(priv->drq) |
             H616_DMA_CFG_DST_MODE(H616_DMA_MODE_IO);
    }
  else
    {
      /* Peripheral (drq, io mode) -> memory (SDRAM, linear) */

      cfg |= H616_DMA_CFG_SRC_DRQ(priv->drq) |
             H616_DMA_CFG_SRC_MODE(H616_DMA_MODE_IO) |
             H616_DMA_CFG_DST_DRQ(H616_DMA_DRQ_SDRAM) |
             H616_DMA_CFG_DST_MODE(H616_DMA_MODE_LINEAR);
    }

  return cfg;
}

/* Per-channel IRQ enable bits: 4 bits (HALF/PKG/QUEUE) per channel,
 * 8 channels per IRQ_EN register.
 */

static void h616_set_irqen(FAR struct h616_dmachan_s *priv, uint32_t bits)
{
  uintptr_t addr = H616_DMA_ADDR +
                   ((priv->num < 8) ? H616_DMAC_IRQ_EN0 : H616_DMAC_IRQ_EN1);
  unsigned int shift = (priv->num & 7) * 4;
  uint32_t val = getreg32(addr);

  val &= ~(0xfu << shift);
  val |= bits << shift;
  putreg32(val, addr);
}

/* Flush the descriptor chain out to DRAM for the DMAC to fetch */

static void h616_clean_lli(FAR struct h616_dmachan_s *priv, int count)
{
  up_clean_dcache((uintptr_t)priv->lli,
                  (uintptr_t)priv->lli + count * sizeof(priv->lli[0]));
}

/* Program the channel with the first LLI of the chain and start it */

static void h616_dmac_kick(FAR struct h616_dmachan_s *priv)
{
  putreg32((uint32_t)(uintptr_t)priv->lli, h616_chaddr(priv->num,
              H616_DMAC_CH_LLI_ADDR));
  putreg32(H616_DMAC_CH_ENABLE_START,
           h616_chaddr(priv->num, H616_DMAC_CH_ENABLE));
}

/* Common completion handling for one channel IRQ */

static void h616_dmac_complete(FAR struct h616_dmachan_s *priv)
{
  dma_callback_t cb = priv->callback;
  FAR void *arg = priv->arg;

  if (priv->cyclic)
    {
      size_t pos;

      /* One more period done; report the write position within the
       * cyclic buffer (this is what the 16550 driver expects).
       */

      priv->cyc_total += priv->period_len;
      pos = priv->cyc_total % priv->xfer_len;

      if (cb != NULL)
        {
          cb(&priv->chan, arg, pos);
        }
    }
  else
    {
      /* One-shot transfer finished: stop the channel, mask its IRQ
       * and report the transferred length.
       */

      putreg32(H616_DMAC_CH_ENABLE_STOP,
               h616_chaddr(priv->num, H616_DMAC_CH_ENABLE));
      h616_set_irqen(priv, 0);

      if (cb != NULL)
        {
          cb(&priv->chan, arg, priv->xfer_len);
        }
    }
}

static int h616_dmac_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t status[2];
  int i;
  int j;

  status[0] = getreg32(H616_DMA_ADDR + H616_DMAC_IRQ_STAT0);
  status[1] = getreg32(H616_DMA_ADDR + H616_DMAC_IRQ_STAT1);

  /* Writing the status back clears the pending bits */

  if (status[0] != 0)
    {
      putreg32(status[0], H616_DMA_ADDR + H616_DMAC_IRQ_STAT0);
    }

  if (status[1] != 0)
    {
      putreg32(status[1], H616_DMA_ADDR + H616_DMAC_IRQ_STAT1);
    }

  for (i = 0; i < 2; i++)
    {
      uint32_t stat = status[i];

      for (j = 0; (j < 8) && (stat != 0); j++, stat >>= 4)
        {
          struct h616_dmachan_s *priv = &g_dmachan[i * 8 + j];

          /* PKG: one LLI completed.  QUEUE: descriptor chain ended
           * (both fire together at the end of a one-shot transfer).
           */

          if ((stat & (H616_DMAC_IRQ_PKG | H616_DMAC_IRQ_QUEUE)) != 0 &&
              priv->inuse)
            {
              h616_dmac_complete(priv);
            }
        }
    }

  return OK;
}

/* vtable: config */

static int h616_dmac_config(FAR struct dma_chan_s *chan,
                            FAR const struct dma_config_s *cfg)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;

  if (cfg == NULL || chan == NULL)
    {
      return -EINVAL;
    }

  if (cfg->direction != DMA_MEM_TO_DEV && cfg->direction != DMA_DEV_TO_MEM)
    {
      return -EINVAL;
    }

  priv->direction = cfg->direction;
  priv->src_width = cfg->src_width > 0 ? cfg->src_width : 1;
  priv->dst_width = cfg->dst_width > 0 ? cfg->dst_width : 1;

  /* The generic dma_config_s may carry the DRQ ids explicitly; the 16550
   * client does not, in that case keep the DRQ bound at channel-get time.
   */

  if (cfg->direction == DMA_MEM_TO_DEV && cfg->dst_drq != 0)
    {
      priv->drq = cfg->dst_drq;
    }
  else if (cfg->direction == DMA_DEV_TO_MEM && cfg->src_drq != 0)
    {
      priv->drq = cfg->src_drq;
    }

  return OK;
}

/* vtable: start a one-shot transfer (one LLI) */

static int h616_dmac_start(FAR struct dma_chan_s *chan,
                           dma_callback_t callback, FAR void *arg,
                           uintptr_t dst, uintptr_t src, size_t len)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;
  struct h616_dma_lli_s *lli = &priv->lli[0];
  uint32_t cfg;
  irqstate_t flags;

  if (chan == NULL || len == 0)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  priv->callback = callback;
  priv->arg      = arg;
  priv->cyclic   = false;
  priv->xfer_len = len;

  cfg = h616_compose_cfg(priv);

  lli->cfg        = cfg;
  lli->src        = (uint32_t)src;
  lli->dst        = (uint32_t)dst;
  lli->len        = (uint32_t)len;
  lli->para       = H616_DMA_LLI_NORMAL_WAIT;
  lli->p_lli_next = H616_DMA_LLI_LAST;

  h616_clean_lli(priv, 1);

  /* End-of-transfer notification: PKG (LLI done) and QUEUE (chain end) */

  h616_set_irqen(priv, H616_DMAC_IRQ_PKG | H616_DMAC_IRQ_QUEUE);
  h616_dmac_kick(priv);

  leave_critical_section(flags);
  return OK;
}

/* vtable: start a cyclic transfer (LLI ring, PKG IRQ per period) */

static int h616_dmac_start_cyclic(FAR struct dma_chan_s *chan,
                                  dma_callback_t callback, FAR void *arg,
                                  uintptr_t dst, uintptr_t src,
                                  size_t len, size_t period_len)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;
  uint32_t cfg;
  size_t nperiods;
  size_t i;
  irqstate_t flags;

  if (chan == NULL || period_len == 0 || len == 0 ||
      (len % period_len) != 0)
    {
      return -EINVAL;
    }

  nperiods = len / period_len;
  if (nperiods > H616_DMAC_MAXLLI)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  priv->callback  = callback;
  priv->arg       = arg;
  priv->cyclic    = true;
  priv->xfer_len  = len;
  priv->period_len = period_len;
  priv->cyc_total = 0;

  cfg = h616_compose_cfg(priv);

  for (i = 0; i < nperiods; i++)
    {
      struct h616_dma_lli_s *lli = &priv->lli[i];

      if (priv->direction == DMA_MEM_TO_DEV)
        {
          lli->src = (uint32_t)(src + period_len * i);
          lli->dst = (uint32_t)dst;
        }
      else
        {
          lli->src = (uint32_t)src;
          lli->dst = (uint32_t)(dst + period_len * i);
        }

      lli->cfg        = cfg;
      lli->len        = (uint32_t)period_len;
      lli->para       = H616_DMA_LLI_NORMAL_WAIT;

      /* Ring the chain: last descriptor points back to the first */

      lli->p_lli_next = (i + 1 < nperiods) ?
                          (uint32_t)(uintptr_t)&priv->lli[i + 1] :
                          (uint32_t)(uintptr_t)&priv->lli[0];
    }

  h616_clean_lli(priv, nperiods);

  h616_set_irqen(priv, H616_DMAC_IRQ_PKG);
  h616_dmac_kick(priv);

  leave_critical_section(flags);
  return OK;
}

/* vtable: stop */

static int h616_dmac_stop(FAR struct dma_chan_s *chan)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;
  irqstate_t flags;

  if (chan == NULL)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();

  putreg32(H616_DMAC_CH_ENABLE_STOP,
           h616_chaddr(priv->num, H616_DMAC_CH_ENABLE));
  h616_set_irqen(priv, 0);
  priv->cyclic   = false;
  priv->callback = NULL;

  leave_critical_section(flags);
  return OK;
}

/* vtable: pause */

static int h616_dmac_pause(FAR struct dma_chan_s *chan)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;

  if (chan == NULL)
    {
      return -EINVAL;
    }

  putreg32(H616_DMAC_CH_PAUSE_PAUSE,
           h616_chaddr(priv->num, H616_DMAC_CH_PAUSE));
  return OK;
}

/* vtable: resume */

static int h616_dmac_resume(FAR struct dma_chan_s *chan)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;

  if (chan == NULL)
    {
      return -EINVAL;
    }

  putreg32(H616_DMAC_CH_PAUSE_RESUME,
           h616_chaddr(priv->num, H616_DMAC_CH_PAUSE));
  return OK;
}

/* vtable: residual */

static size_t h616_dmac_residual(FAR struct dma_chan_s *chan)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;

  if (chan == NULL)
    {
      return 0;
    }

  return getreg32(h616_chaddr(priv->num, H616_DMAC_CH_CUR_CNT));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: h616_dma_initialize
 *
 * Description:
 *   Enable the DMAC clocks (bus + mbus), release it from reset, enable
 *   the clock auto-gate and attach the interrupt handler.
 *
 *   CCU offsets (Linux ccu-sun50i-h616.c):
 *     BUS_DMA  gate: 0x70c bit 0
 *     MBUS_DMA gate: 0x804 bit 0
 *     BUS_DMA  reset: 0x2c0 bit 6 (RST_BUS_DMA = 6)
 *
 ****************************************************************************/

void h616_dma_initialize(void)
{
  uintptr_t icfgr;
  uint32_t regval;
  uint32_t shift;
  uint32_t val;
  int i;

  /* Channel bookkeeping */

  for (i = 0; i < H616_DMAC_NCHANNELS; i++)
    {
      g_dmachan[i].chan.ops = &g_dmac_ops;
      g_dmachan[i].num      = i;
    }

  /* Deassert the DMAC bus reset, then open the bus and mbus clock gates */

  regval = getreg32(H616_CCU_ADDR + 0x2c0);
  regval &= ~(1u << 6);                      /* assert reset */
  putreg32(regval, H616_CCU_ADDR + 0x2c0);
  up_mdelay(1);
  regval |= (1u << 6);                       /* release reset */
  putreg32(regval, H616_CCU_ADDR + 0x2c0);

  regval  = getreg32(H616_CCU_ADDR + 0x70c);
  regval |= (1u << 0);                       /* BUS_DMA gate */
  putreg32(regval, H616_CCU_ADDR + 0x70c);

  regval  = getreg32(H616_CCU_ADDR + 0x804);
  regval |= (1u << 0);                       /* MBUS_DMA gate */
  putreg32(regval, H616_CCU_ADDR + 0x804);

  /* Enable the internal clock auto-gating (H3 style) */

  putreg32(H616_DMAC_GATE_ENABLE, H616_DMA_ADDR + H616_DMAC_GATE);

  /* Clear and attach the DMAC interrupt */

  putreg32(0, H616_DMA_ADDR + H616_DMAC_IRQ_EN0);
  putreg32(0, H616_DMA_ADDR + H616_DMAC_IRQ_EN1);
  putreg32(0xffffffff, H616_DMA_ADDR + H616_DMAC_IRQ_STAT0);
  putreg32(0xffffffff, H616_DMA_ADDR + H616_DMAC_IRQ_STAT1);

  /* The DMAC outputs a level-high interrupt, but the GIC ICFGR left by
   * the bootloader marks all SPIs edge-triggered.  Reconfigure our SPI
   * (H616_DMAC_IRQ) to level so every completion is delivered:  with an
   * edge config, a pending level that re-asserts without a sampled low
   * period can be lost forever.
   */

  icfgr = 0x03021000 + 0xc00 + (H616_DMAC_IRQ / 16) * 4;
  shift = (H616_DMAC_IRQ % 16) * 2;
  val   = getreg32(icfgr) & ~(3u << shift);

  putreg32(val, icfgr);

  irq_attach(H616_DMAC_IRQ, h616_dmac_interrupt, NULL);
  up_enable_irq(H616_DMAC_IRQ);

  /* NOTE: no logging here: this runs from arm64_earlyserialinit(),
   * before syslog/console are functional; a syslog() call at this
   * point hangs the boot.  Use dmainfo (CONFIG_DEBUG_DMA) instead.
   */
}

/****************************************************************************
 * Name: h616_dmachan_get
 *
 ****************************************************************************/

FAR struct dma_chan_s *h616_dmachan_get(unsigned int ch)
{
  if (ch >= H616_DMAC_NCHANNELS)
    {
      return NULL;
    }

  g_dmachan[ch].inuse = true;
  return &g_dmachan[ch].chan;
}

/****************************************************************************
 * Name: h616_dmachan_setdrq
 *
 ****************************************************************************/

void h616_dmachan_setdrq(FAR struct dma_chan_s *chan, uint8_t drq)
{
  FAR struct h616_dmachan_s *priv = (FAR struct h616_dmachan_s *)chan;

  if (chan != NULL)
    {
      priv->drq = drq;
    }
}

#endif /* CONFIG_H616_DMA */

/****************************************************************************
 * Name: uart_dmachan
 *
 * Description:
 *   16550 DMA glue (see include/nuttx/serial/uart_16550.h): map the
 *   channel identity from CONFIG_16550_UART0_DMA_TX/RX to a DMAC channel
 *   and bind the DRQ port of the requesting UART.  The identity is the
 *   hardware channel number (0..15).
 *
 *   UART DRQ ports (sun50i-h616.dtsi dmas = <&dma N>): UART0..5 -> 14..19.
 *
 ****************************************************************************/

#if defined(CONFIG_H616_DMA) && defined(CONFIG_16550_UART) && \
   (defined(CONFIG_16550_UART0_DMA) || defined(CONFIG_16550_UART1_DMA) || \
    defined(CONFIG_16550_UART2_DMA) || defined(CONFIG_16550_UART3_DMA))

FAR struct dma_chan_s *uart_dmachan(FAR struct u16550_s *priv,
                                    unsigned int ident)
{
  FAR struct dma_chan_s *chan;
  uint32_t uartbase;
  uint8_t drq;

  chan = h616_dmachan_get(ident);
  if (chan == NULL)
    {
      return NULL;
    }

  /* DRQ port from the UART base: UART0..5 at 0x05000000 + 0x400 * n */

  uartbase = (uint32_t)priv->uartbase;
  if (uartbase < H616_UART0_ADDR || uartbase > H616_UART5_ADDR ||
      ((uartbase - H616_UART0_ADDR) & 0x3ff) != 0)
    {
      return NULL;
    }

  drq = 14 + (uartbase - H616_UART0_ADDR) / 0x400;
  h616_dmachan_setdrq(chan, drq);

  dmainfo("h616 dmac: uart 0x%x -> channel %d, drq %d\n",
          uartbase, ident, drq);
  return chan;
}

#endif /* CONFIG_H616_DMA && 16550 UART DMA */

/****************************************************************************
 * Name: up_addrenv_va_to_pa / up_addrenv_pa_to_va
 *
 * Description:
 *   Identity implementations for flat (VA == PA) builds.  The 16550 DMA
 *   client calls up_addrenv_va_to_pa() (declared unconditionally in
 *   include/nuttx/arch.h) but arm64 provides no definition when the
 *   kernel runs flat with a single DRAM mapping at 0x40000000.
 *
 ****************************************************************************/

#if defined(CONFIG_H616_DMA) && !defined(CONFIG_ARCH_USE_MMU)

uintptr_t up_addrenv_va_to_pa(FAR void *va)
{
  return (uintptr_t)va;
}

FAR void *up_addrenv_pa_to_va(uintptr_t pa)
{
  return (FAR void *)pa;
}

#endif /* CONFIG_H616_DMA && !CONFIG_ARCH_USE_MMU */
