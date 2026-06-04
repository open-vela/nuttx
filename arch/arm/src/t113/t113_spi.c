/****************************************************************************
 * arch/arm/src/t113/t113_spi.c
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
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/kmalloc.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "t113_ccu.h"
#include "t113_gpio.h"
#include "hardware/t113_spi.h"
#include "hardware/t113_dma.h"
#include "hardware/t113_clk.h"

#include "t113_dma.h"
#include "t113_spi.h"

#if defined(CONFIG_T113_SPI0) || defined(CONFIG_T113_SPI1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPI_DMA_THRESHOLD   4096
#define SPI_DMA_BOUNCE_SIZE 4096

#define PLL_PERI_1X_HZ      T113_SPI_SRC_FREQUENCY

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_spidev_s
{
  struct spi_dev_s  spidev;
  struct qspi_dev_s qspidev;
  uint32_t          base;
  mutex_t           lock;
  uint32_t          frequency;
  uint32_t          actual;
  uint32_t          tcr;   /* Shadow of TCR config bits (no XCH) */
  uint8_t           nbits;
  uint8_t           mode;

#ifdef CONFIG_T113_SPI0_DMA
  DMA_HANDLE        rxdma;
  DMA_HANDLE        txdma;
  sem_t             rxsem;
  sem_t             txsem;
  volatile uint8_t  rxresult;
  volatile uint8_t  txresult;
  uint8_t           src_drq;
  uint8_t           dst_drq;
  FAR uint8_t      *dma_rxbounce;
  FAR uint8_t      *dma_txbounce;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int      spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                 uint32_t frequency);
static void     spi_setmode(FAR struct spi_dev_s *dev,
                             enum spi_mode_e mode);
static void     spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd);
static int      spi_exchange(FAR struct spi_dev_s *dev,
                              FAR const void *txbuffer,
                              FAR void *rxbuffer, size_t nwords);
#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                         size_t nwords);
static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                          size_t nwords);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static inline void spi_putreg(struct t113_spidev_s *priv,
                               uint32_t offset, uint32_t val)
{
  putreg32(val, priv->base + offset);
}

static inline uint32_t spi_getreg(struct t113_spidev_s *priv,
                                    uint32_t offset)
{
  return getreg32(priv->base + offset);
}

/* Reset TX and RX FIFOs.  The reset bits are self-clearing in the
 * SPI clock domain.  At low SPI frequencies (e.g. 10 MHz) the CPU
 * can outrun the reset - writing TX data before reset completes
 * silently drops the data, causing XCH to hang (no TX bytes -> no
 * clocks).  Poll until both bits clear.
 */

static void spi_fifo_reset(struct t113_spidev_s *priv)
{
  spi_putreg(priv, SPI_FCR_REG,
             SPI_FCR_TX_FIFO_RST | SPI_FCR_RF_RST);
  while (spi_getreg(priv, SPI_FCR_REG) &
         (SPI_FCR_TX_FIFO_RST | SPI_FCR_RF_RST));
}

static void spi_select(FAR struct spi_dev_s *dev, uint32_t devid,
                        bool selected)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)dev;

  if (selected)
    {
#ifdef CONFIG_BOOT_RUNFROMISRAM
      /* Ensure minimum CS high time (tCSH) before asserting CS.
       * At 100MHz QSPI with CPU running from SRAM at 1008MHz,
       * back-to-back deselect->select can violate the NAND chip's
       * tCSH requirement (>=20ns for MX35LF series).  A short
       * spin loop guarantees the minimum gap.
       */

      volatile int i;
      for (i = 0; i < 50; i++);
#endif

      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr & ~SPI_TCR_SS_LEVEL);
    }
  else
    {
      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr | SPI_TCR_SS_LEVEL);

      /* Pair the CS-rise TCR write with a barrier so the next
       * transaction's CS-fall TCR write reaches the SPI peripheral
       * as a distinct second event.  Under concurrent USB MUSB
       * DMA load the AXI fabric gets congested; without this
       * barrier the CPU store buffer can batch the two TCR writes
       * such that both arrive at the SPI peripheral within one
       * peripheral-clock period, and CS never actually pulses high
       * between transactions.  NAND MX35LF1GE4AB then sees
       * insufficient tCSH and silently drops/merges commands,
       * causing a PAGE_READ to be absorbed into the prior
       * transaction.  Symptom: FTL RMW read of page N returns
       * page N-1's data under USB-MSC-over-NAND write load.
       *
       * Any of {DSB, DMB, same-addr readback, ~100 CPU cycles
       * of delay} at this location fixes it.  DSB is chosen for
       * consistency with the rest of the T113 arch code and to
       * give the strongest ordering guarantee.
       *
       * Related: the CONFIG_BOOT_RUNFROMISRAM branch above
       * guards the same tCSH requirement from the other side
       * (minimum CS-high duration when CPU runs from SRAM at
       * 1008 MHz).
       */

      __asm__ volatile("dsb sy" ::: "memory");
    }
}

static const struct spi_ops_s g_spiops =
{
  .lock              = spi_lock,
  .select            = spi_select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = NULL,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata           = NULL,
#endif
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
};

#ifdef CONFIG_T113_SPI0
static struct t113_spidev_s g_spi0dev =
{
  .spidev =
    {
      .ops = &g_spiops
    },
  .base     = T113_SPI0_BASE,
  .lock     = NXMUTEX_INITIALIZER,
  .frequency = 0,
  .nbits     = 8,
  .mode      = SPIDEV_MODE0,
#ifdef CONFIG_T113_SPI0_DMA
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
  .src_drq  = DRQ_SPI0_RX,
  .dst_drq  = DRQ_SPI0_TX,
#endif
};
#endif

#ifdef CONFIG_T113_SPI1
static struct t113_spidev_s g_spi1dev =
{
  .spidev =
    {
      .ops = &g_spiops
    },
  .base     = T113_SPI1_BASE,
  .lock     = NXMUTEX_INITIALIZER,
  .frequency = 0,
  .nbits     = 8,
  .mode      = SPIDEV_MODE0,
#ifdef CONFIG_T113_SPI1_DMA
  .rxsem    = SEM_INITIALIZER(0),
  .txsem    = SEM_INITIALIZER(0),
  .src_drq  = DRQ_SPI1_RX,
  .dst_drq  = DRQ_SPI1_TX,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void spi_reset(struct t113_spidev_s *priv)
{
  /* SRST resets ALL SPI registers to defaults (GCR -> 0x80,
   * TCR -> 0x87, etc.).  Must re-initialize after reset completes.
   */

  spi_putreg(priv, SPI_GCR_REG, SPI_GCR_SRST);
  while (spi_getreg(priv, SPI_GCR_REG) & SPI_GCR_SRST);

  spi_putreg(priv, SPI_GCR_REG,
             SPI_GCR_TP_EN | SPI_GCR_MASTER | SPI_GCR_EN);

  /* TCR shadow - all subsequent TCR writes use priv->tcr, never
   * read-modify-write from hardware.  SS_LEVEL excluded so that
   * XCH writes keep CS asserted; spi_select() manages SS_LEVEL.
   */

  priv->tcr = SPI_TCR_SS_OWNER | SPI_TCR_SPOL;
  spi_putreg(priv, SPI_TCR_REG, priv->tcr | SPI_TCR_SS_LEVEL);

  /* Force spi_setfrequency/spi_setmode to re-run on next call --
   * SRST cleared SDC/SDM/CPOL/CPHA in TCR but the cached values
   * still hold old state, causing early-return checks to skip
   * hardware reconfiguration.
   */

  priv->frequency = 0;
  priv->mode = UINT8_MAX;

  spi_fifo_reset(priv);

  spi_putreg(priv, SPI_IER_REG, 0);
  spi_putreg(priv, SPI_ISR_REG, 0xffffffff);
}

static void spi_clock_enable(struct t113_spidev_s *priv)
{
  uint32_t gating = priv->base == T113_SPI0_BASE ?
                    T113_CCU_SPI0_GATING : T113_CCU_SPI1_GATING;
  uint32_t rst    = priv->base == T113_SPI0_BASE ?
                    T113_CCU_SPI0_RST : T113_CCU_SPI1_RST;
  uint32_t clk_reg = priv->base == T113_SPI0_BASE ?
                     T113_CCU_SPI0_CLK : T113_CCU_SPI1_CLK;

  /* Tear down CCU state before rebuild.
   * Zeroing SPI_CLK_REG first replicates the
   * effect of a WDT reset on the clock domain.
   */

  t113_ccu_modify(clk_reg, 0xffffffff, 0);   /* kill SPI clock entirely */

  /* bus gate off + assert reset */

  t113_ccu_modify(T113_CCU_SPI_BGR, gating | rst, 0);

  up_udelay(20);

  /* Rebuild from scratch: clock -> bus gate -> deassert reset */

  /* CLK_EN | PLL_PERI1X | M=6 (600MHz / 6 = 100MHz) */

  t113_ccu_modify(clk_reg, 0xffffffff,
                  T113_CCU_SPI_CLK_GATING |
                  T113_CCU_SPI_CLK_SRC_PLL_PERI1X |
                  T113_CCU_SPI_FACTOR_M(6));

  /* bus gate on */

  t113_ccu_modify(T113_CCU_SPI_BGR, 0, gating);

  /* deassert reset */

  t113_ccu_modify(T113_CCU_SPI_BGR, 0, rst);

  /* Pin mux - selections come from board.h via t113_pinmap.h */

#ifdef CONFIG_T113_SPI0
  if (priv->base == T113_SPI0_BASE)
    {
      t113_gpio_config(T113_SPI0_CLK);
      t113_gpio_config(T113_SPI0_CS);
      t113_gpio_config(T113_SPI0_MOSI);
      t113_gpio_config(T113_SPI0_MISO);
      t113_gpio_config(T113_SPI0_WP);
      t113_gpio_config(T113_SPI0_HOLD);
    }
#endif

#ifdef CONFIG_T113_SPI1
  if (priv->base == T113_SPI1_BASE)
    {
      t113_gpio_config(T113_SPI1_CLK);
      t113_gpio_config(T113_SPI1_CS);
      t113_gpio_config(T113_SPI1_MOSI);
      t113_gpio_config(T113_SPI1_MISO);
      t113_gpio_config(T113_SPI1_HOLD);
      t113_gpio_config(T113_SPI1_WP);
    }
#endif

  up_udelay(10);
}

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev,
                                   uint32_t frequency)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)dev;
  uint32_t clk_reg;
  uint32_t n;
  uint32_t m;
  uint32_t actual;

  if (frequency == priv->frequency)
    {
      return priv->actual;
    }

  clk_reg = priv->base == T113_SPI0_BASE ?
            T113_CCU_SPI0_CLK : T113_CCU_SPI1_CLK;

  /* SPI_CLK = PLL_PERI_1X / (2^N * M), N=0..3, M=1..16
   * Find smallest N then smallest M that gives freq <= target.
   */

  for (n = 0; n < 4; n++)
    {
      m = (PLL_PERI_1X_HZ / (1u << n) + frequency - 1) / frequency;
      if (m <= 16)
        {
          break;
        }
    }

  if (n >= 4)
    {
      n = 3;
      m = 16;
    }

  if (m < 1)
    {
      m = 1;
    }

  actual = PLL_PERI_1X_HZ / ((1u << n) * m);

  t113_ccu_clk_set(clk_reg,
                   T113_CCU_SPI_FACTOR_N_MASK | T113_CCU_SPI_FACTOR_M_MASK,
                   T113_CCU_SPI_FACTOR_N(n) | T113_CCU_SPI_FACTOR_M(m));

  /* Configure sample delay per User Manual Table 9-10 (Old Sample Mode):
   *   >=80MHz: SDC=1, SDM=0 (delay one cycle)
   *   <=40MHz: SDC=0, SDM=0 (delay half cycle)
   *   <=24MHz: SDC=0, SDM=1 (normal sample)
   */

  priv->tcr &= ~(SPI_TCR_SDC | SPI_TCR_SDM);

  if (actual >= 80000000)
    {
      priv->tcr |= SPI_TCR_SDC;
    }
  else if (actual <= 24000000)
    {
      priv->tcr |= SPI_TCR_SDM;
    }

  spi_putreg(priv, SPI_TCR_REG,
             priv->tcr | SPI_TCR_SS_LEVEL);

  priv->frequency = frequency;
  priv->actual    = actual;

  spiinfo("target=%luHz actual=%luHz N=%lu M=%lu\n",
          (unsigned long)frequency, (unsigned long)actual,
          (unsigned long)(1u << n), (unsigned long)m);
  return actual;
}

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)dev;

  if (mode == priv->mode)
    {
      return;
    }

  priv->tcr &= ~(SPI_TCR_CPOL | SPI_TCR_CPHA);

  switch (mode)
    {
      case SPIDEV_MODE0:
        break;
      case SPIDEV_MODE1:
        priv->tcr |= SPI_TCR_CPHA;
        break;
      case SPIDEV_MODE2:
        priv->tcr |= SPI_TCR_CPOL;
        break;
      case SPIDEV_MODE3:
        priv->tcr |= SPI_TCR_CPOL | SPI_TCR_CPHA;
        break;
      default:
        return;
    }

  spi_putreg(priv, SPI_TCR_REG,
             priv->tcr | SPI_TCR_SS_LEVEL);
  priv->mode = mode;
}

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)dev;
  priv->nbits = nbits;
}

static void spi_exchange_pio(FAR struct t113_spidev_s *priv,
                              FAR const void *txbuf,
                              FAR void *rxbuf, size_t nwords)
{
  const uint8_t *tx = (const uint8_t *)txbuf;
  uint8_t       *rx = (uint8_t *)rxbuf;
  uint32_t       fsr;
  size_t         txpos = 0;
  size_t         rxpos = 0;
  size_t         txfill;
  int            timeout;

  spi_fifo_reset(priv);
  spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

  spi_putreg(priv, SPI_MBC_REG, nwords);
  spi_putreg(priv, SPI_MTC_REG, nwords);
  spi_putreg(priv, SPI_BCC_REG, nwords);

  txfill = nwords < SPI_FIFO_DEPTH ? nwords : SPI_FIFO_DEPTH;
  for (size_t i = 0; i < txfill; i++)
    {
      putreg8(tx ? tx[txpos++] : 0xff, priv->base + SPI_TXD_REG);
    }

  spi_putreg(priv, SPI_TCR_REG, priv->tcr | SPI_TCR_XCH);

  timeout = 2000000;
  while (rxpos < nwords && --timeout > 0)
    {
      fsr = spi_getreg(priv, SPI_FSR_REG);

      while (rxpos < nwords && (fsr & 0xff) > 0)
        {
          uint8_t val = getreg8(priv->base + SPI_RXD_REG);
          if (rx)
            {
              rx[rxpos] = val;
            }

          rxpos++;
          fsr--;
        }

      while (txpos < nwords &&
             ((fsr >> 16) & 0xff) < SPI_FIFO_DEPTH)
        {
          putreg8(tx ? tx[txpos] : 0xff,
                  priv->base + SPI_TXD_REG);
          txpos++;
          fsr += (1 << 16);
        }
    }

  if (timeout <= 0)
    {
      spierr("SPI timeout tx=%zu/%zu rx=%zu/%zu "
             "TCR=%08lx FSR=%08lx MBC=%08lx ISR=%08lx\n",
             txpos, nwords, rxpos, nwords,
             (unsigned long)spi_getreg(priv, SPI_TCR_REG),
             (unsigned long)spi_getreg(priv, SPI_FSR_REG),
             (unsigned long)spi_getreg(priv, SPI_MBC_REG),
             (unsigned long)spi_getreg(priv, SPI_ISR_REG));
    }

  /* Wait for XCH auto-clear */

  timeout = 200000;
  while ((spi_getreg(priv, SPI_TCR_REG) & SPI_TCR_XCH)
         && --timeout > 0);

  spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);
}

#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)
static void spi_rxcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)arg;
  priv->rxresult = status | 0x80;
  nxsem_post(&priv->rxsem);
}

static void spi_txcallback(DMA_HANDLE handle, uint8_t status, void *arg)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)arg;
  priv->txresult = status | 0x80;
  nxsem_post(&priv->txsem);
}

static void spi_exchange_dma(FAR struct t113_spidev_s *priv,
                              FAR const void *txbuf,
                              FAR void *rxbuf, size_t nwords)
{
  static uint32_t aligned_data(64) s_dummy_tx = 0xffffffff;
  static uint32_t aligned_data(64) s_dummy_rx;
  struct t113_dma_config_s rxcfg;
  struct t113_dma_config_s txcfg;
  FAR uint8_t *dma_rxbuf = NULL;
  bool bounce = false;
  int xch_timeout;
  int dma_timeout;

  txcfg.src_width  = DMAC_WIDTH_8BIT;
  txcfg.dst_width  = DMAC_WIDTH_8BIT;
  txcfg.src_burst  = DMAC_BURST_1;
  txcfg.dst_burst  = DMAC_BURST_1;
  txcfg.mode       = DMAC_MODE_DST_HANDSHAKE;
  txcfg.circular   = false;
  txcfg.bmode      = false;

  rxcfg.src_drq    = priv->src_drq;
  rxcfg.dst_drq    = DRQ_DRAM;
  rxcfg.src_width  = DMAC_WIDTH_8BIT;
  rxcfg.dst_width  = DMAC_WIDTH_8BIT;
  rxcfg.src_burst  = DMAC_BURST_1;
  rxcfg.dst_burst  = DMAC_BURST_1;
  rxcfg.src_linear = false;
  rxcfg.dst_linear = (rxbuf != NULL);
  rxcfg.mode       = DMAC_MODE_SRC_HANDSHAKE;
  rxcfg.circular   = false;
  rxcfg.bmode      = false;

  txcfg.src_drq    = DRQ_DRAM;
  txcfg.dst_drq    = priv->dst_drq;
  txcfg.src_linear = (txbuf != NULL);
  txcfg.dst_linear = false;

  priv->rxresult = 0;
  priv->txresult = 0;

  if (txbuf)
    {
      up_flush_dcache((uintptr_t)txbuf & ~63ul,
                      ((uintptr_t)txbuf + nwords + 63ul) & ~63ul);
    }

  /* Use a cache-line aligned bounce buffer for RX when the caller's
   * buffer is not 64-byte aligned.  Unaligned invalidate causes stale
   * data at cache-line boundaries on Cortex-A7.
   */

  if (rxbuf && ((uintptr_t)rxbuf & 63))
    {
      if (nwords <= SPI_DMA_BOUNCE_SIZE && priv->dma_rxbounce)
        {
          dma_rxbuf = priv->dma_rxbounce;
          bounce = true;
        }
      else
        {
          dma_rxbuf = memalign(64, nwords);
          if (dma_rxbuf != NULL)
            {
              bounce = true;
            }
          else
            {
              dma_rxbuf = rxbuf;
            }
        }
    }
  else if (rxbuf)
    {
      dma_rxbuf = rxbuf;
    }

  if (dma_rxbuf)
    {
      up_flush_dcache((uintptr_t)dma_rxbuf,
                      (uintptr_t)dma_rxbuf + nwords);
    }

  t113_dmasetup(priv->rxdma,
                priv->base + SPI_RXD_REG,
                dma_rxbuf ? (uintptr_t)dma_rxbuf : (uintptr_t)&s_dummy_rx,
                nwords, &rxcfg);

  t113_dmasetup(priv->txdma,
                txbuf ? (uintptr_t)txbuf : (uintptr_t)&s_dummy_tx,
                priv->base + SPI_TXD_REG,
                nwords, &txcfg);

  /* TX-only: set DHB to discard RX so TP_EN won't stall when
   * RX FIFO fills.  Skip RX DMA entirely.
   */

  if (rxbuf)
    {
      spi_putreg(priv, SPI_FCR_REG,
                 SPI_FCR_RF_DRQ_EN | SPI_FCR_TF_DRQ_EN |
                 SPI_FCR_RX_TRIG(32) | SPI_FCR_TX_TRIG(1));
    }
  else
    {
      spi_putreg(priv, SPI_TCR_REG, priv->tcr | SPI_TCR_DHB);
      spi_putreg(priv, SPI_FCR_REG,
                 SPI_FCR_TF_DRQ_EN | SPI_FCR_TX_TRIG(1));
    }

  spi_putreg(priv, SPI_MBC_REG, nwords);
  spi_putreg(priv, SPI_MTC_REG, nwords);
  spi_putreg(priv, SPI_BCC_REG, nwords);

  /* Start DMA before XCH: DMA waits for DRQ, then XCH triggers
   * the SPI transfer which generates DRQ signals.  This avoids
   * a race where XCH fires before DMA is ready to consume/feed
   * the FIFO.  Matches the safer order used in qspi_memory_dma.
   */

  if (rxbuf)
    {
      t113_dmastart(priv->rxdma, spi_rxcallback, priv);
    }

  t113_dmastart(priv->txdma, spi_txcallback, priv);

  spi_putreg(priv, SPI_TCR_REG,
             priv->tcr |
             (rxbuf ? 0 : SPI_TCR_DHB) | SPI_TCR_XCH);

  /* Poll DMA residual directly.  Under MUSB internal DMA concurrent load
   * the DMAC PKG_DONE IRQ line does not reach the GIC reliably.
   */

  dma_timeout = 2000;  /* up to 2000 * 100us = 200ms */
  while (((t113_dmaresidual(priv->txdma) > 0) ||
          (rxbuf && t113_dmaresidual(priv->rxdma) > 0)) &&
         --dma_timeout > 0)
    {
      up_udelay(100);
    }

  if (t113_dmaresidual(priv->txdma) > 0 ||
      (rxbuf && t113_dmaresidual(priv->rxdma) > 0))
    {
      _err("DMA incomplete tx_left=%zu rx_left=%zu\n",
             t113_dmaresidual(priv->txdma),
             rxbuf ? t113_dmaresidual(priv->rxdma) : 0);
      t113_dmastop(priv->txdma);
      if (rxbuf)
        {
          t113_dmastop(priv->rxdma);
        }
    }

  if (dma_rxbuf)
    {
      up_invalidate_dcache((uintptr_t)dma_rxbuf,
                           (uintptr_t)dma_rxbuf + nwords);
      if (bounce)
        {
          memcpy(rxbuf, dma_rxbuf, nwords);
          if (dma_rxbuf != priv->dma_rxbounce)
            {
              free(dma_rxbuf);
            }
        }
    }

  spi_putreg(priv, SPI_FCR_REG, 0);

  /* Wait XCH=0 before restoring TCR.  Shadow prevents writing XCH
   * back, but the hardware still ignores ALL register writes (TCR,
   * MBC, MTC, BCC) while XCH=1 - the next operation's setup would
   * silently fail.
   */

  xch_timeout = 200000;
  while ((spi_getreg(priv, SPI_TCR_REG) & SPI_TCR_XCH)
         && --xch_timeout > 0);

  spi_putreg(priv, SPI_TCR_REG, priv->tcr);
}
#endif

static int spi_exchange(FAR struct spi_dev_s *dev,
                         FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct t113_spidev_s *priv = (FAR struct t113_spidev_s *)dev;

#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)
  if (nwords > SPI_DMA_THRESHOLD && priv->rxdma != NULL)
    {
      spi_exchange_dma(priv, txbuffer, rxbuffer, nwords);
      return OK;
    }
#endif

  spi_exchange_pio(priv, txbuffer, rxbuffer, nwords);
  return OK;
}

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  uint8_t txbyte = (uint8_t)wd;
  uint8_t rxbyte = 0;
  spi_exchange(dev, &txbyte, &rxbyte, 1);
  return rxbyte;
}

#ifndef CONFIG_SPI_EXCHANGE
static void spi_sndblock(FAR struct spi_dev_s *dev, FAR const void *buffer,
                          size_t nwords)
{
  spi_exchange(dev, buffer, NULL, nwords);
}

static void spi_recvblock(FAR struct spi_dev_s *dev, FAR void *buffer,
                           size_t nwords)
{
  spi_exchange(dev, NULL, buffer, nwords);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct spi_dev_s *t113_spibus_initialize(int bus)
{
  FAR struct t113_spidev_s *priv;

  switch (bus)
    {
#ifdef CONFIG_T113_SPI0
      case 0:
        priv = &g_spi0dev;
        break;
#endif
#ifdef CONFIG_T113_SPI1
      case 1:
        priv = &g_spi1dev;
        break;
#endif
      default:
        return NULL;
    }

  spi_clock_enable(priv);
  spi_reset(priv);

  /* Configure as master, 8-bit, mode 0.
   * TP_EN: pause TX when RX FIFO full - prevents RX overflow in
   * full-duplex PIO.  Write-only paths must clear TP_EN or set DHB
   * to avoid stalling (RX FIFO fills with don't-care data).
   */

  spi_putreg(priv, SPI_GCR_REG,
             SPI_GCR_TP_EN | SPI_GCR_MASTER | SPI_GCR_EN);
  spi_putreg(priv, SPI_FCR_REG,
             SPI_FCR_TX_TRIG(0x40) | SPI_FCR_RX_TRIG(1));

#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)
  if (priv->src_drq != 0)
    {
      priv->rxdma = t113_dmachannel();
      priv->txdma = t113_dmachannel();
      priv->dma_rxbounce = memalign(64, SPI_DMA_BOUNCE_SIZE);
      priv->dma_txbounce = memalign(64, SPI_DMA_BOUNCE_SIZE);
    }
#endif

  return (FAR struct spi_dev_s *)priv;
}

/****************************************************************************
 * QSPI Interface
 ****************************************************************************/

static int qspi_lock(FAR struct qspi_dev_s *dev, bool lock)
{
  FAR struct t113_spidev_s *priv =
    container_of(dev, struct t113_spidev_s, qspidev);

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

static uint32_t qspi_setfrequency(FAR struct qspi_dev_s *dev,
                                   uint32_t frequency)
{
  FAR struct t113_spidev_s *priv =
    container_of(dev, struct t113_spidev_s, qspidev);
  return spi_setfrequency(&priv->spidev, frequency);
}

static void qspi_setmode(FAR struct qspi_dev_s *dev,
                          enum qspi_mode_e mode)
{
  FAR struct t113_spidev_s *priv =
    container_of(dev, struct t113_spidev_s, qspidev);
  spi_setmode(&priv->spidev, (enum spi_mode_e)mode);
}

static void qspi_setbits(FAR struct qspi_dev_s *dev, int nbits)
{
  FAR struct t113_spidev_s *priv =
    container_of(dev, struct t113_spidev_s, qspidev);
  priv->nbits = nbits;
}

static int qspi_command(FAR struct qspi_dev_s *dev,
                        FAR struct qspi_cmdinfo_s *cmdinfo)
{
  FAR struct t113_spidev_s *priv =
    container_of(dev, struct t113_spidev_s, qspidev);
  uint8_t buf[8];
  size_t txlen = 0;
  size_t rxlen = 0;

  buf[txlen++] = (uint8_t)cmdinfo->cmd;

  if (QSPICMD_ISADDRESS(cmdinfo->flags))
    {
      int i;
      for (i = cmdinfo->addrlen - 1; i >= 0; i--)
        {
          buf[txlen++] = (cmdinfo->addr >> (i * 8)) & 0xff;
        }
    }

  DEBUGASSERT(txlen <= sizeof(buf));

  spi_select(&priv->spidev, SPIDEV_FLASH(0), true);

  if (QSPICMD_ISWRITE(cmdinfo->flags) && cmdinfo->buflen > 0)
    {
      /* Single-XCH write: cmd + data in one burst via FSR polling.
       * Must drain RX to prevent TP_EN stall when total > FIFO depth.
       */

      const uint8_t *data = (const uint8_t *)cmdinfo->buffer;
      size_t datalen = cmdinfo->buflen;
      size_t total   = txlen + datalen;
      size_t txpos   = 0;
      size_t rxrem   = total;
      size_t fill;
      uint32_t fsr;
      uint32_t rxcnt;
      uint32_t txcnt;
      int timeout;

      spi_fifo_reset(priv);
      spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

      spi_putreg(priv, SPI_MBC_REG, total);
      spi_putreg(priv, SPI_MTC_REG, total);
      spi_putreg(priv, SPI_BCC_REG, total);

      /* Pre-fill TX FIFO: cmd bytes first, then data bytes */

      fill = total < SPI_FIFO_DEPTH ? total : SPI_FIFO_DEPTH;
      for (size_t i = 0; i < fill; i++)
        {
          putreg8(i < txlen ? buf[i] : data[i - txlen],
                  priv->base + SPI_TXD_REG);
        }

      txpos = fill > txlen ? fill - txlen : 0;

      spi_putreg(priv, SPI_TCR_REG, priv->tcr | SPI_TCR_XCH);

      /* Full-duplex FSR polling: drain RX (discard) + feed TX.
       * TP_EN (GCR bit7) pauses TX when RX FIFO full, so we must
       * drain RX to keep the transfer flowing.
       */

      timeout = 2000000;
      while (rxrem > 0 && --timeout > 0)
        {
          fsr   = spi_getreg(priv, SPI_FSR_REG);
          rxcnt = fsr & 0xff;
          txcnt = (fsr >> 16) & 0xff;

          while (rxrem > 0 && rxcnt > 0)
            {
              getreg8(priv->base + SPI_RXD_REG);
              rxrem--;
              rxcnt--;
            }

          while (txpos < datalen && txcnt < SPI_FIFO_DEPTH)
            {
              putreg8(data[txpos++], priv->base + SPI_TXD_REG);
              txcnt++;
            }
        }

      /* Wait XCH completion */

      timeout = 200000;
      while ((spi_getreg(priv, SPI_TCR_REG) & SPI_TCR_XCH)
             && --timeout > 0);

      spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);
    }
  else if (QSPICMD_ISREAD(cmdinfo->flags) && cmdinfo->buflen > 0)
    {
      /* Single-XCH read: cmd TX + data RX in one burst with DHB.
       * DHB = 1 discards RX during cmd phase (MTC bytes), RX FIFO
       * receives only data phase bytes.  FSR polling drains RX
       * during transfer to support rxlen > FIFO depth.
       */

      uint8_t *rx = (uint8_t *)cmdinfo->buffer;
      size_t rxpos = 0;
      uint32_t rxcnt;
      int timeout;

      rxlen = cmdinfo->buflen;

      spi_fifo_reset(priv);
      spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

      spi_putreg(priv, SPI_MBC_REG, txlen + rxlen);
      spi_putreg(priv, SPI_MTC_REG, txlen);
      spi_putreg(priv, SPI_BCC_REG, txlen);

      for (size_t i = 0; i < txlen; i++)
        {
          putreg8(buf[i], priv->base + SPI_TXD_REG);
        }

      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr | SPI_TCR_DHB | SPI_TCR_XCH);

      /* FSR polling: drain RX as data arrives.  During cmd phase
       * (MTC bytes) RX is discarded by DHB, so rxcnt stays 0.
       * Data phase bytes appear in RX FIFO progressively.
       */

      timeout = 2000000;
      while (rxpos < rxlen && --timeout > 0)
        {
          rxcnt = spi_getreg(priv, SPI_FSR_REG) & 0xff;
          while (rxpos < rxlen && rxcnt > 0)
            {
              rx[rxpos++] = getreg8(priv->base + SPI_RXD_REG);
              rxcnt--;
            }
        }

      /* Wait XCH completion and restore TCR (no DHB, no XCH) */

      timeout = 200000;
      while ((spi_getreg(priv, SPI_TCR_REG) & SPI_TCR_XCH)
             && --timeout > 0);

      spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);
      spi_putreg(priv, SPI_TCR_REG, priv->tcr);
    }
  else
    {
      spi_exchange_pio(priv, buf, NULL, txlen);
    }

  spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
  return OK;
}

#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)

/* Quad/Dual memory read via DMA with DHB=1 (RX FIFO receives data only) */

static int qspi_memory_dma(FAR struct t113_spidev_s *priv,
                           FAR struct qspi_meminfo_s *meminfo,
                           FAR const uint8_t *cmdbuf, size_t cmdlen)
{
  struct t113_dma_config_s rxcfg;
  size_t rxlen = meminfo->buflen;
  uint32_t bcc;
  FAR uint8_t *dma_rxbuf;
  bool bounce = false;
  int dma_timeout;
  int timeout;

  rxcfg.src_drq    = priv->src_drq;
  rxcfg.dst_drq    = DRQ_DRAM;
  rxcfg.src_width  = DMAC_WIDTH_32BIT;
  rxcfg.dst_width  = DMAC_WIDTH_32BIT;
  rxcfg.src_burst  = DMAC_BURST_8;
  rxcfg.dst_burst  = DMAC_BURST_8;
  rxcfg.src_linear = false;
  rxcfg.dst_linear = true;
  rxcfg.mode       = DMAC_MODE_SRC_HANDSHAKE;
  rxcfg.circular   = false;
  rxcfg.bmode      = false;

  if ((uintptr_t)meminfo->buffer & 63)
    {
      if (rxlen <= SPI_DMA_BOUNCE_SIZE && priv->dma_rxbounce)
        {
          dma_rxbuf = priv->dma_rxbounce;
          bounce = true;
        }
      else
        {
          dma_rxbuf = kmm_memalign(64, rxlen);
          if (dma_rxbuf == NULL)
            {
              return -ENOMEM;
            }

          bounce = true;
        }
    }
  else
    {
      dma_rxbuf = meminfo->buffer;
    }

  /* Invalidate before DMA read - discard stale cache lines so
   * post-DMA invalidate sees only DMA-written data.
   */

  up_invalidate_dcache((uintptr_t)dma_rxbuf,
                       (uintptr_t)dma_rxbuf + rxlen);

  priv->rxresult = 0;
  while (nxsem_trywait(&priv->rxsem) == OK);

  /* Reset FIFOs and clear pending interrupts first */

  spi_fifo_reset(priv);
  spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

  /* Single-phase: cmd + data in one XCH.
   * MBC = cmdlen + rxlen, MTC = cmdlen (TX from FIFO),
   * STC = cmdlen (single-wire), QUAD_EN for data phase.
   * DHB=1: only receive RX during auto-dummy (data) period.
   */

  spi_putreg(priv, SPI_MBC_REG, cmdlen + rxlen);
  spi_putreg(priv, SPI_MTC_REG, cmdlen);

  bcc = cmdlen & SPI_BCC_STC_MASK;
  if (QSPIMEM_ISQUADIO(meminfo->flags))
    {
      bcc |= SPI_BCC_QUAD_EN;
    }
  else if (QSPIMEM_ISDUALIO(meminfo->flags))
    {
      bcc |= SPI_BCC_DUAL_EN;
    }

  spi_putreg(priv, SPI_BCC_REG, bcc);

  spi_putreg(priv, SPI_TCR_REG, priv->tcr | SPI_TCR_DHB);

  /* Push cmd bytes into TX FIFO */

  for (size_t i = 0; i < cmdlen; i++)
    {
      putreg8(cmdbuf[i], priv->base + SPI_TXD_REG);
    }

  /* Setup DMA after FIFO reset and cmd push, before enabling DRQ */

  t113_dmasetup(priv->rxdma,
                priv->base + SPI_RXD_REG,
                (uintptr_t)dma_rxbuf,
                rxlen, &rxcfg);

  /* Enable RX FIFO DRQ, start DMA, then trigger XCH */

  spi_putreg(priv, SPI_FCR_REG,
             SPI_FCR_RF_DRQ_EN | SPI_FCR_RX_TRIG(32));

  t113_dmastart(priv->rxdma, spi_rxcallback, priv);

  spi_putreg(priv, SPI_TCR_REG,
             priv->tcr | SPI_TCR_DHB | SPI_TCR_XCH);

  /* Poll TC bit for short transfers to avoid scheduler overhead */

  timeout = 200000;
  while (!(spi_getreg(priv, SPI_ISR_REG) & (SPI_INT_TC | SPI_INT_ERR))
         && --timeout > 0);
  if (spi_getreg(priv, SPI_ISR_REG) & SPI_INT_ERR)
    {
      spierr("QSPI DMA read ISR error: 0x%08lx\n",
             (unsigned long)spi_getreg(priv, SPI_ISR_REG));
    }

  spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);

  /* Wait for DMA to finish draining FIFO to DRAM - TC fires when SPI
   * shift register is done, but DMA may still have in-flight beats.
   */

  dma_timeout = 10000;
  while (t113_dmaresidual(priv->rxdma) > 0 && --dma_timeout > 0);

  spi_putreg(priv, SPI_FCR_REG, 0);

  /* Consume the semaphore posted by spi_rxcallback */

  nxsem_trywait(&priv->rxsem);

  /* Wait XCH=0 before restoring TCR - hardware ignores all register
   * writes while XCH=1, so without this the DHB clear would be lost.
   */

  timeout = 200000;
  while ((spi_getreg(priv, SPI_TCR_REG) & SPI_TCR_XCH)
         && --timeout > 0);

  spi_putreg(priv, SPI_TCR_REG, priv->tcr);

  spi_putreg(priv, SPI_BCC_REG, 0);

  if (t113_dmaresidual(priv->rxdma) > 0)
    {
      spierr("QSPI DMA incomplete rx_left=%zu\n",
             t113_dmaresidual(priv->rxdma));
      t113_dmastop(priv->rxdma);
      if (bounce && dma_rxbuf != priv->dma_rxbounce)
        {
          kmm_free(dma_rxbuf);
        }

      return -EIO;
    }

  up_invalidate_dcache((uintptr_t)dma_rxbuf,
                       (uintptr_t)dma_rxbuf + rxlen);

  if (bounce)
    {
      memcpy(meminfo->buffer, dma_rxbuf, rxlen);
      if (dma_rxbuf != priv->dma_rxbounce)
        {
          kmm_free(dma_rxbuf);
        }
    }

  return OK;
}

/* Quad/Dual memory write via DMA - cmd bytes PIO, data bytes DMA */

static int qspi_memory_dma_write(FAR struct t113_spidev_s *priv,
                                 FAR struct qspi_meminfo_s *meminfo,
                                 FAR const uint8_t *cmdbuf, size_t cmdlen)
{
  struct t113_dma_config_s txcfg;
  size_t datalen = meminfo->buflen;
  size_t total = cmdlen + datalen;
  uint32_t bcc;
  FAR uint8_t *dma_txbuf;
  bool bounce = false;
  int dma_timeout;
  size_t res;

  txcfg.src_drq    = DRQ_DRAM;
  txcfg.dst_drq    = priv->dst_drq;
  txcfg.src_width  = DMAC_WIDTH_32BIT;
  txcfg.dst_width  = DMAC_WIDTH_32BIT;
  txcfg.src_burst  = DMAC_BURST_8;
  txcfg.dst_burst  = DMAC_BURST_8;
  txcfg.src_linear = true;
  txcfg.dst_linear = false;
  txcfg.mode       = DMAC_MODE_DST_HANDSHAKE;
  txcfg.circular   = false;
  txcfg.bmode      = false;

  if ((uintptr_t)meminfo->buffer & 63)
    {
      if (datalen <= SPI_DMA_BOUNCE_SIZE && priv->dma_txbounce)
        {
          dma_txbuf = priv->dma_txbounce;
        }
      else
        {
          dma_txbuf = kmm_memalign(64, datalen);
          if (dma_txbuf == NULL)
            {
              return -ENOMEM;
            }
        }

      bounce = true;
      memcpy(dma_txbuf, meminfo->buffer, datalen);
    }
  else
    {
      dma_txbuf = (FAR uint8_t *)meminfo->buffer;
    }

  up_flush_dcache((uintptr_t)dma_txbuf,
                  (uintptr_t)dma_txbuf + datalen);

  priv->txresult = 0;
  while (nxsem_trywait(&priv->txsem) == OK);

  /* Reset FIFOs and clear pending interrupts */

  spi_fifo_reset(priv);
  spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

  /* MBC = total (cmd + data), MTC = total (all TX from master) */

  spi_putreg(priv, SPI_MBC_REG, total);
  spi_putreg(priv, SPI_MTC_REG, total);

  bcc = cmdlen & SPI_BCC_STC_MASK;
  if (QSPIMEM_ISQUADIO(meminfo->flags))
    {
      bcc |= SPI_BCC_QUAD_EN;
    }
  else if (QSPIMEM_ISDUALIO(meminfo->flags))
    {
      bcc |= SPI_BCC_DUAL_EN;
    }

  spi_putreg(priv, SPI_BCC_REG, bcc);

  /* Set DHB for write: discard RX data so TP_EN won't stall
   * when RX FIFO fills with don't-care bytes during TX.
   */

  spi_putreg(priv, SPI_TCR_REG, priv->tcr | SPI_TCR_DHB);

  /* Push cmd bytes into TX FIFO via PIO */

  for (size_t i = 0; i < cmdlen; i++)
    {
      putreg8(cmdbuf[i], priv->base + SPI_TXD_REG);
    }

  /* Setup TX DMA for data bytes */

  t113_dmasetup(priv->txdma,
                (uintptr_t)dma_txbuf,
                priv->base + SPI_TXD_REG,
                datalen, &txcfg);

  /* Enable TX FIFO DRQ, start DMA, then trigger XCH */

  spi_putreg(priv, SPI_FCR_REG,
             SPI_FCR_TF_DRQ_EN | SPI_FCR_TX_TRIG(32));

  t113_dmastart(priv->txdma, spi_txcallback, priv);

  spi_putreg(priv, SPI_TCR_REG,
             priv->tcr | SPI_TCR_DHB | SPI_TCR_XCH);

  /* Poll DMA residual directly.  Under MUSB internal DMA concurrent load
   * the DMAC PKG_DONE IRQ line does not reach the GIC reliably, so we
   * cannot rely on the semaphore path.  Polling CNT register is safe
   * and fast (a 2KB DMA at 100MHz SPI completes in ~160us).
   */

  dma_timeout = 200;  /* up to 200 * 100us = 20ms */
  while (t113_dmaresidual(priv->txdma) > 0 && --dma_timeout > 0)
    {
      up_udelay(100);
    }

  /* Wait for Transfer Complete */

  int timeout = 100000;
  while (!(spi_getreg(priv, SPI_ISR_REG) & (SPI_INT_TC | SPI_INT_ERR))
         && --timeout > 0);

  if (spi_getreg(priv, SPI_ISR_REG) & SPI_INT_ERR)
    {
      spierr("QSPI DMA write ISR error: 0x%08lx\n",
             (unsigned long)spi_getreg(priv, SPI_ISR_REG));
    }

  spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);

  spi_putreg(priv, SPI_FCR_REG, 0);
  spi_putreg(priv, SPI_BCC_REG, 0);

  timeout = 200000;
  while ((spi_getreg(priv, SPI_TCR_REG) & SPI_TCR_XCH)
         && --timeout > 0);

  spi_putreg(priv, SPI_TCR_REG, priv->tcr);

  res = t113_dmaresidual(priv->txdma);

  if (res > 0)
    {
      spierr("QSPI DMA write incomplete tx_left=%zu\n", res);
      t113_dmastop(priv->txdma);
      if (bounce && dma_txbuf != priv->dma_txbounce)
        {
          kmm_free(dma_txbuf);
        }

      return -EIO;
    }

  if (bounce && dma_txbuf != priv->dma_txbounce)
    {
      kmm_free(dma_txbuf);
    }

  return OK;
}
#endif

static int qspi_memory(FAR struct qspi_dev_s *dev,
                       FAR struct qspi_meminfo_s *meminfo)
{
  FAR struct t113_spidev_s *priv =
    container_of(dev, struct t113_spidev_s, qspidev);
  uint8_t cmdbuf[8];
  size_t cmdlen = 0;
#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)
  bool quad_or_dual;
#endif

  cmdbuf[cmdlen++] = (uint8_t)meminfo->cmd;

  for (int i = meminfo->addrlen - 1; i >= 0; i--)
    {
      cmdbuf[cmdlen++] = (meminfo->addr >> (i * 8)) & 0xff;
    }

  for (int i = 0; i < meminfo->dummies; i++)
    {
      cmdbuf[cmdlen++] = 0xff;
    }

  DEBUGASSERT(cmdlen <= sizeof(cmdbuf));

#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)
  quad_or_dual = QSPIMEM_ISQUADIO(meminfo->flags) ||
                 QSPIMEM_ISDUALIO(meminfo->flags);
#endif

  spi_select(&priv->spidev, SPIDEV_FLASH(0), true);

  if (QSPIMEM_ISREAD(meminfo->flags))
    {
#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)
      if (quad_or_dual && priv->rxdma != NULL)
        {
          int ret = qspi_memory_dma(priv, meminfo,
                                    cmdbuf, cmdlen);
          spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
          return ret;
        }
#endif

      /* PIO read: single-phase XCH, matching DMA read register
       * setup.  MBC = cmd+data, MTC = cmdlen (TX bytes from FIFO),
       * STC = cmdlen (single-wire), QUAD_EN for data phase.
       * DHB=1: discard TX during data phase, only receive RX.
       *
       * The two-phase approach (separate cmd and data XCH) caused
       * 9 KB/s throttle because the data-only XCH (MTC=0) has no
       * TX bytes to drive the clock - the controller trickles
       * clocks without DRQ feedback.  Single-phase avoids this:
       * cmd bytes in TX FIFO drive clocks for the cmd phase, then
       * the controller continues clocking for the remaining MBC-MTC
       * bytes in DHB mode at full SPI speed.
       */

      /* PIO quad read: chunked transfer, <= FIFO_DEPTH bytes per XCH.
       *
       * Problem: DHB quad mode generates clocks at full SPI speed.
       * A single XCH for 2048 bytes overflows the 64-byte RX FIFO
       * (RF_OVF proven by JLink).  RF_DRQ_EN without DMA throttles
       * to ~9 KB/s.
       *
       * Solution: break into FIFO-sized chunks.  Each chunk's MBC
       * <= FIFO_DEPTH so TC fires before overflow.  Drain at leisure.
       *
       * Chunk 0: MBC=cmdlen+data, MTC=cmdlen, push cmd to TX FIFO.
       *   Controller sends cmd on single wire, receives data on quad.
       *   DHB discards cmd-phase RX; FIFO gets chunk0_data bytes.
       *
       * Chunk 1+: MBC=chunk, MTC=0, no TX data.  Controller generates
       *   clocks in pure DHB+quad mode.  MTC=0 causes the controller
       *   to throttle clock generation (~9 KB/s for large MBC), but
       *   with MBC<=64, each chunk completes quickly.
       *   IMPORTANT: MTC>0 would drive IO0 with TX data while the
       *   NAND chip is still driving IO0-IO3 (quad output) -> bus
       *   conflict -> data corruption.  MTC=0 avoids this.
       */

      size_t datalen = meminfo->buflen;
      uint8_t *rx = (uint8_t *)meminfo->buffer;
      size_t rxpos = 0;
      uint32_t bcc_flags = 0;
      int timeout;

      if (QSPIMEM_ISQUADIO(meminfo->flags))
        {
          bcc_flags = SPI_BCC_QUAD_EN;
        }
      else if (QSPIMEM_ISDUALIO(meminfo->flags))
        {
          bcc_flags = SPI_BCC_DUAL_EN;
        }

      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr | SPI_TCR_DHB);

      /* --- Chunk 0: cmd + first data bytes --- */

      size_t chunk0_data = SPI_FIFO_DEPTH - cmdlen;
      if (chunk0_data > datalen)
        {
          chunk0_data = datalen;
        }

      spi_fifo_reset(priv);
      spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

      spi_putreg(priv, SPI_MBC_REG, cmdlen + chunk0_data);
      spi_putreg(priv, SPI_MTC_REG, cmdlen);
      spi_putreg(priv, SPI_BCC_REG,
                 (cmdlen & SPI_BCC_STC_MASK) | bcc_flags);

      for (size_t i = 0; i < cmdlen; i++)
        {
          putreg8(cmdbuf[i], priv->base + SPI_TXD_REG);
        }

      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr | SPI_TCR_DHB | SPI_TCR_XCH);

      timeout = 200000;
      while (!(spi_getreg(priv, SPI_ISR_REG) & (SPI_INT_TC | SPI_INT_ERR))
             && --timeout > 0);
      spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);

      if (timeout <= 0)
        {
          spierr("QSPI PIO read chunk0 timeout\n");
          spi_putreg(priv, SPI_TCR_REG, priv->tcr);
          spi_putreg(priv, SPI_FCR_REG, 0);
          spi_putreg(priv, SPI_BCC_REG, 0);
          spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
          return -ETIMEDOUT;
        }

      for (size_t i = 0; i < chunk0_data; i++)
        {
          rx[rxpos++] = getreg8(priv->base + SPI_RXD_REG);
        }

      /* --- Subsequent chunks: pure DHB, MTC=0 --- */

      while (rxpos < datalen)
        {
          size_t chunk = datalen - rxpos;
          if (chunk > SPI_FIFO_DEPTH)
            {
              chunk = SPI_FIFO_DEPTH;
            }

          spi_fifo_reset(priv);
          spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

          spi_putreg(priv, SPI_MBC_REG, chunk);
          spi_putreg(priv, SPI_MTC_REG, 0);
          spi_putreg(priv, SPI_BCC_REG, bcc_flags);

          spi_putreg(priv, SPI_TCR_REG,
                     priv->tcr | SPI_TCR_DHB | SPI_TCR_XCH);

          timeout = 200000;
          while (!(spi_getreg(priv, SPI_ISR_REG) &
                   (SPI_INT_TC | SPI_INT_ERR)) && --timeout > 0);
          spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);

          if (timeout <= 0)
            {
              spierr("QSPI PIO read chunk timeout at %zu/%zu\n",
                     rxpos, datalen);
              spi_putreg(priv, SPI_TCR_REG, priv->tcr);
              spi_putreg(priv, SPI_FCR_REG, 0);
              spi_putreg(priv, SPI_BCC_REG, 0);
              spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
              return -ETIMEDOUT;
            }

          for (size_t i = 0; i < chunk; i++)
            {
              rx[rxpos++] = getreg8(priv->base + SPI_RXD_REG);
            }
        }

      spi_putreg(priv, SPI_TCR_REG, priv->tcr);
      spi_putreg(priv, SPI_FCR_REG, 0);
      spi_putreg(priv, SPI_BCC_REG, 0);
    }
  else
    {
#if defined(CONFIG_T113_SPI0_DMA) || defined(CONFIG_T113_SPI1_DMA)
      if (quad_or_dual && priv->txdma != NULL)
        {
          int ret = qspi_memory_dma_write(priv, meminfo, cmdbuf, cmdlen);
          spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
          return ret;
        }
#endif

      /* PIO write: single XCH, TX-only feed (no RX drain).
       *
       * The original code drained RX while feeding TX.  In quad
       * mode, reading RX FIFO during TX causes data corruption -
       * likely due to FIFO resource conflicts in the controller.
       * (Stall testing proved the controller pauses the SPI clock
       * when TX FIFO empties, so this is NOT an underrun issue.)
       * Since write doesn't need RX data, we skip RX entirely.
       */

      size_t datalen = meminfo->buflen;
      const uint8_t *tx = (const uint8_t *)meminfo->buffer;
      size_t total = cmdlen + datalen;
      uint32_t bcc;
      int timeout;

      spi_fifo_reset(priv);
      spi_putreg(priv, SPI_ISR_REG, 0xffffffff);

      spi_putreg(priv, SPI_MBC_REG, total);
      spi_putreg(priv, SPI_MTC_REG, total);

      if (QSPIMEM_ISQUADIO(meminfo->flags))
        {
          bcc = (cmdlen & SPI_BCC_STC_MASK) | SPI_BCC_QUAD_EN;
        }
      else if (QSPIMEM_ISDUALIO(meminfo->flags))
        {
          bcc = (cmdlen & SPI_BCC_STC_MASK) | SPI_BCC_DUAL_EN;
        }
      else
        {
          bcc = total & SPI_BCC_STC_MASK;
        }

      spi_putreg(priv, SPI_BCC_REG, bcc);

      /* Set DHB for write: discard RX so TP_EN won't stall
       * when RX FIFO fills with don't-care bytes during TX.
       */

      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr | SPI_TCR_DHB);

      /* Push cmd bytes */

      for (size_t i = 0; i < cmdlen; i++)
        {
          putreg8(cmdbuf[i], priv->base + SPI_TXD_REG);
        }

      /* Pre-fill TX FIFO with data */

      size_t fill = (datalen < (SPI_FIFO_DEPTH - cmdlen)) ?
                     datalen : (SPI_FIFO_DEPTH - cmdlen);
      size_t datapos = 0;
      for (size_t i = 0; i < fill; i++)
        {
          putreg8(tx[datapos++], priv->base + SPI_TXD_REG);
        }

      spi_putreg(priv, SPI_TCR_REG,
                 priv->tcr | SPI_TCR_DHB | SPI_TCR_XCH);

      /* Feed remaining TX data */

      timeout = 2000000;
      while (datapos < datalen && --timeout > 0)
        {
          uint32_t fsr_val = spi_getreg(priv, SPI_FSR_REG);
          uint32_t txcnt = (fsr_val >> 16) & 0xff;
          while (datapos < datalen && txcnt < SPI_FIFO_DEPTH)
            {
              putreg8(tx[datapos++], priv->base + SPI_TXD_REG);
              txcnt++;
            }
        }

      if (timeout <= 0)
        {
          spierr("QSPI PIO write TX-feed timeout at %zu/%zu\n",
                 datapos, datalen);
          spi_putreg(priv, SPI_TCR_REG, priv->tcr);
          spi_putreg(priv, SPI_FCR_REG, 0);
          spi_putreg(priv, SPI_BCC_REG, 0);
          spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
          return -ETIMEDOUT;
        }

      /* Wait TC */

      timeout = 100000;
      while (!(spi_getreg(priv, SPI_ISR_REG) & (SPI_INT_TC | SPI_INT_ERR))
             && --timeout > 0);
      if (spi_getreg(priv, SPI_ISR_REG) & SPI_INT_ERR)
        {
          spierr("QSPI PIO write ISR error: 0x%08lx\n",
                 (unsigned long)spi_getreg(priv, SPI_ISR_REG));
        }

      spi_putreg(priv, SPI_ISR_REG, SPI_INT_TC | SPI_INT_ERR);

      spi_putreg(priv, SPI_FCR_REG, 0);
      spi_putreg(priv, SPI_BCC_REG, 0);

      spi_putreg(priv, SPI_TCR_REG, priv->tcr);

      if (timeout <= 0)
        {
          spierr("QSPI PIO write TC timeout\n");
          spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
          return -ETIMEDOUT;
        }
    }

  spi_select(&priv->spidev, SPIDEV_FLASH(0), false);
  return OK;
}

static FAR void *qspi_alloc(FAR struct qspi_dev_s *dev, size_t buflen)
{
  return kmm_memalign(64, buflen);
}

static void qspi_free(FAR struct qspi_dev_s *dev, FAR void *buffer)
{
  kmm_free(buffer);
}

static const struct qspi_ops_s g_qspiops =
{
  .lock         = qspi_lock,
  .setfrequency = qspi_setfrequency,
  .setmode      = qspi_setmode,
  .setbits      = qspi_setbits,
  .command      = qspi_command,
  .memory       = qspi_memory,
  .alloc        = qspi_alloc,
  .free         = qspi_free,
};

FAR struct qspi_dev_s *t113_qspi_initialize(int bus)
{
  FAR struct spi_dev_s *spi;
  FAR struct t113_spidev_s *priv;

  spi = t113_spibus_initialize(bus);
  if (spi == NULL)
    {
      return NULL;
    }

  priv = container_of(spi, struct t113_spidev_s, spidev);
  priv->qspidev.ops = &g_qspiops;

  return &priv->qspidev;
}

#endif /* CONFIG_T113_SPI0 || CONFIG_T113_SPI1 */
