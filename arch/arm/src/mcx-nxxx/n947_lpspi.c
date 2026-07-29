/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_lpspi.c
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

/* Polling LPSPI master driver for MCX-Nxxx LP_FLEXCOMM instances.  The IP is
 * register-compatible with the NXP LPSPI block used by i.MX RT and S32K.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/mutex.h>
#include <nuttx/spi/spi.h>

#include "arm_internal.h"
#include "chip.h"
#include "hardware/nxxx_flexcomm.h"
#include "hardware/n947/n947_lpspi.h"
#include "hardware/nxxx_memorymap.h"
#include "nxxx_clockconfig.h"
#include "n947_lpspi.h"
#include "nxxx_port.h"

#include <arch/board/board.h>

#ifdef CONFIG_N947_LPSPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

  /* LPSPI functional clock: FRO_HF (48MHz FIRC, enabled at boot) via the
   * DIVFROHFCLK /1 divider.  The old FRO12M source capped SCK at 6MHz
   * (12M / (presc*(scaler+2)), min divide-by-2), which limited sustained
   * transfers to high-bandwidth SPI targets.
   */

#define N947_LPSPI_CLKFREQ      48000000u
#define N947_LPSPI_DEFAULT_HZ   400000u
#define N947_LPSPI_MIN_BITS     2
#define N947_LPSPI_MAX_BITS     16

#ifndef weak_function
#  define weak_function __attribute__((weak))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n947_lpspi_config_s
{
  uintptr_t base;
  struct clock_regs_s clk_regs;
  struct clock_gate_reg_s clk_gate;
  uint32_t clk_source;
  uintptr_t psel_reg;
  uint8_t pcs;
  bool pins;
  port_cfg_t sck_pin;
  port_cfg_t miso_pin;
  port_cfg_t mosi_pin;
};

struct n947_lpspi_priv_s
{
  struct spi_dev_s spidev;
  const struct n947_lpspi_config_s *config;
  mutex_t lock;
  uint32_t frequency;
  uint32_t actual;
  enum spi_mode_e mode;
  uint8_t nbits;
  uint8_t refs;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int n947_lpspi_lock(struct spi_dev_s *dev, bool lock);
static uint32_t n947_lpspi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency);
static void n947_lpspi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode);
static void n947_lpspi_setbits(struct spi_dev_s *dev, int nbits);
#ifdef CONFIG_SPI_HWFEATURES
static int n947_lpspi_hwfeatures(struct spi_dev_s *dev,
                                 spi_hwfeatures_t features);
#endif
static uint32_t n947_lpspi_send(struct spi_dev_s *dev, uint32_t wd);
#ifdef CONFIG_SPI_EXCHANGE
static void n947_lpspi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer, void *rxbuffer,
                                size_t nwords);
#else
static void n947_lpspi_sndblock(struct spi_dev_s *dev,
                                const void *txbuffer, size_t nwords);
static void n947_lpspi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                 size_t nwords);
#endif
#ifdef CONFIG_SPI_CMDDATA
static int n947_lpspi_cmddata(struct spi_dev_s *dev, uint32_t devid,
                              bool cmd);
#endif
static int n947_lpspi_registercallback(struct spi_dev_s *dev,
                                       spi_mediachange_t callback,
                                       void *arg);

/****************************************************************************
 * Weak Board Hooks
 ****************************************************************************/

#ifdef CONFIG_N947_LPSPI0
void weak_function n947_lpspi0select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi0status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_N947_LPSPI1
void weak_function n947_lpspi1select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi1status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_N947_LPSPI2
void weak_function n947_lpspi2select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi2status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_N947_LPSPI3
void weak_function n947_lpspi3select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi3status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_N947_LPSPI4
void weak_function n947_lpspi4select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi4status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_N947_LPSPI5
void weak_function n947_lpspi5select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi5status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_N947_LPSPI6
void weak_function n947_lpspi6select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi6status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_N947_LPSPI7
void weak_function n947_lpspi7select(struct spi_dev_s *dev,
                                     uint32_t devid, bool selected)
{
  (void)dev;
  (void)devid;
  (void)selected;
}

uint8_t weak_function n947_lpspi7status(struct spi_dev_s *dev,
                                        uint32_t devid)
{
  (void)dev;
  (void)devid;
  return SPI_STATUS_PRESENT;
}
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_N947_LPSPI0
static const struct spi_ops_s g_lpspi0_ops =
{
  .lock             = n947_lpspi_lock,
  .select           = n947_lpspi0select,
  .setfrequency     = n947_lpspi_setfrequency,
  .setmode          = n947_lpspi_setmode,
  .setbits          = n947_lpspi_setbits,
#ifdef CONFIG_SPI_HWFEATURES
  .hwfeatures       = n947_lpspi_hwfeatures,
#endif
  .status           = n947_lpspi0status,
#ifdef CONFIG_SPI_CMDDATA
  .cmddata          = n947_lpspi_cmddata,
#endif
  .send             = n947_lpspi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange         = n947_lpspi_exchange,
#else
  .sndblock         = n947_lpspi_sndblock,
  .recvblock        = n947_lpspi_recvblock,
#endif
  .registercallback = n947_lpspi_registercallback,
};

static const struct n947_lpspi_config_s g_lpspi0_config =
{
  .base       = N947_LPSPI0_BASE,
  .clk_regs   = SYSCON_FCCLK0,
  .clk_gate   = CLOCK_GATE_LPFLEXCOMM0,
  .clk_source = FRO_HF_DIV_TO_FLEXCOMM0,
  .psel_reg   = NXXX_FLEXCOMM0_PSELID,
  .pcs        = 0,
#if defined(PORT_LPSPI0_SCK) && defined(PORT_LPSPI0_MISO) && \
    defined(PORT_LPSPI0_MOSI)
  .pins       = true,
  .sck_pin    = PORT_LPSPI0_SCK,
  .miso_pin   = PORT_LPSPI0_MISO,
  .mosi_pin   = PORT_LPSPI0_MOSI,
#endif
};

static struct n947_lpspi_priv_s g_lpspi0_priv =
{
  .spidev =
  {
    .ops = &g_lpspi0_ops
  },
  .config = &g_lpspi0_config,
  .lock   = NXMUTEX_INITIALIZER,
};
#endif

#define N947_LPSPI_OPS(n) \
static const struct spi_ops_s g_lpspi##n##_ops = \
{ \
  .lock             = n947_lpspi_lock, \
  .select           = n947_lpspi##n##select, \
  .setfrequency     = n947_lpspi_setfrequency, \
  .setmode          = n947_lpspi_setmode, \
  .setbits          = n947_lpspi_setbits, \
  IF_SPI_HWFEATURES(.hwfeatures = n947_lpspi_hwfeatures,) \
  .status           = n947_lpspi##n##status, \
  IF_SPI_CMDDATA(.cmddata = n947_lpspi_cmddata,) \
  .send             = n947_lpspi_send, \
  IF_SPI_EXCHANGE(.exchange = n947_lpspi_exchange,) \
  IF_NOT_SPI_EXCHANGE(.sndblock = n947_lpspi_sndblock,) \
  IF_NOT_SPI_EXCHANGE(.recvblock = n947_lpspi_recvblock,) \
  .registercallback = n947_lpspi_registercallback, \
}

/* The IF_* helpers keep the repeated bus tables readable without relying on
 * non-standard conditional designated initializers.
 */

/* These wrappers take a designated-initializer fragment that itself contains
 * a trailing comma, so they must be variadic (a fixed (x) param would treat
 * the comma as a second argument).  Only reached when an LPSPI1..7 instance
 * is enabled (LPSPI0 uses a hand-written ops table).
 */

#ifdef CONFIG_SPI_HWFEATURES
#  define IF_SPI_HWFEATURES(...) __VA_ARGS__
#else
#  define IF_SPI_HWFEATURES(...)
#endif

#ifdef CONFIG_SPI_CMDDATA
#  define IF_SPI_CMDDATA(...) __VA_ARGS__
#else
#  define IF_SPI_CMDDATA(...)
#endif

#ifdef CONFIG_SPI_EXCHANGE
#  define IF_SPI_EXCHANGE(...) __VA_ARGS__
#  define IF_NOT_SPI_EXCHANGE(...)
#else
#  define IF_SPI_EXCHANGE(...)
#  define IF_NOT_SPI_EXCHANGE(...) __VA_ARGS__
#endif

#define N947_LPSPI_CONFIG(n) \
static const struct n947_lpspi_config_s g_lpspi##n##_config = \
{ \
  .base       = N947_LPSPI##n##_BASE, \
  .clk_regs   = SYSCON_FCCLK##n, \
  .clk_gate   = CLOCK_GATE_LPFLEXCOMM##n, \
  .clk_source = FRO_HF_DIV_TO_FLEXCOMM##n, \
  .psel_reg   = NXXX_FLEXCOMM##n##_PSELID, \
  .pcs        = 0, \
}

#define N947_LPSPI_PRIV(n) \
static struct n947_lpspi_priv_s g_lpspi##n##_priv = \
{ \
  .spidev = { .ops = &g_lpspi##n##_ops }, \
  .config = &g_lpspi##n##_config, \
  .lock   = NXMUTEX_INITIALIZER, \
}

#ifdef CONFIG_N947_LPSPI1
N947_LPSPI_OPS(1);
N947_LPSPI_CONFIG(1);
N947_LPSPI_PRIV(1);
#endif

#ifdef CONFIG_N947_LPSPI2
N947_LPSPI_OPS(2);
N947_LPSPI_CONFIG(2);
N947_LPSPI_PRIV(2);
#endif

#ifdef CONFIG_N947_LPSPI3
N947_LPSPI_OPS(3);
N947_LPSPI_CONFIG(3);
N947_LPSPI_PRIV(3);
#endif

#ifdef CONFIG_N947_LPSPI4
N947_LPSPI_OPS(4);
N947_LPSPI_CONFIG(4);
N947_LPSPI_PRIV(4);
#endif

#ifdef CONFIG_N947_LPSPI5
N947_LPSPI_OPS(5);
N947_LPSPI_CONFIG(5);
N947_LPSPI_PRIV(5);
#endif

#ifdef CONFIG_N947_LPSPI6
N947_LPSPI_OPS(6);
N947_LPSPI_CONFIG(6);
N947_LPSPI_PRIV(6);
#endif

#ifdef CONFIG_N947_LPSPI7
N947_LPSPI_OPS(7);
N947_LPSPI_CONFIG(7);
N947_LPSPI_PRIV(7);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t n947_lpspi_getreg(struct n947_lpspi_priv_s *priv,
                                         uint16_t offset)
{
  return getreg32(priv->config->base + offset);
}

static inline void n947_lpspi_putreg(struct n947_lpspi_priv_s *priv,
                                     uint16_t offset, uint32_t value)
{
  putreg32(value, priv->config->base + offset);
}

static inline void n947_lpspi_modifyreg(struct n947_lpspi_priv_s *priv,
                                        uint16_t offset, uint32_t clearbits,
                                        uint32_t setbits)
{
  modifyreg32(priv->config->base + offset, clearbits, setbits);
}

static inline bool n947_lpspi_16bitmode(struct n947_lpspi_priv_s *priv)
{
  return priv->nbits > 8;
}

static void n947_lpspi_wait_idle(struct n947_lpspi_priv_s *priv)
{
  while ((n947_lpspi_getreg(priv, N947_LPSPI_SR_OFFSET) & LPSPI_SR_MBF) != 0)
    {
    }
}

static void n947_lpspi_flush(struct n947_lpspi_priv_s *priv)
{
  n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, 0,
                       LPSPI_CR_RTF | LPSPI_CR_RRF);
  n947_lpspi_putreg(priv, N947_LPSPI_SR_OFFSET, LPSPI_SR_CLEAR);
}

static int n947_lpspi_lock(struct spi_dev_s *dev, bool lock)
{
  struct n947_lpspi_priv_s *priv = (struct n947_lpspi_priv_s *)dev;

  if (lock)
    {
      return nxmutex_lock(&priv->lock);
    }

  return nxmutex_unlock(&priv->lock);
}

static uint32_t n947_lpspi_setfrequency(struct spi_dev_s *dev,
                                        uint32_t frequency)
{
  struct n947_lpspi_priv_s *priv = (struct n947_lpspi_priv_s *)dev;
  uint32_t best_frequency;
  uint32_t best_prescaler;
  uint32_t best_scaler;
  uint32_t prescaler;
  uint32_t scaler;
  uint32_t men;
  uint32_t diff;
  uint32_t min_diff;

  if (frequency == 0)
    {
      return priv->actual;
    }

  if (frequency == priv->frequency)
    {
      return priv->actual;
    }

  best_prescaler = 7;
  best_scaler    = 255;
  best_frequency = N947_LPSPI_CLKFREQ / ((1u << best_prescaler) *
                                         (best_scaler + 2u));
  min_diff = best_frequency > frequency ? best_frequency - frequency :
                                          frequency - best_frequency;

  for (prescaler = 0; prescaler < 8; prescaler++)
    {
      for (scaler = 0; scaler < 256; scaler++)
        {
          uint32_t actual = N947_LPSPI_CLKFREQ /
                            ((1u << prescaler) * (scaler + 2u));

          if (actual <= frequency)
            {
              diff = frequency - actual;
              if (diff < min_diff)
                {
                  min_diff = diff;
                  best_prescaler = prescaler;
                  best_scaler = scaler;
                  best_frequency = actual;
                  if (diff == 0)
                    {
                      break;
                    }
                }
            }
        }

      if (min_diff == 0)
        {
          break;
        }
    }

  men = n947_lpspi_getreg(priv, N947_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
  if (men != 0)
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);
    }

  n947_lpspi_modifyreg(priv, N947_LPSPI_CCR_OFFSET,
                       LPSPI_CCR_SCKDIV_MASK,
                       LPSPI_CCR_SCKDIV(best_scaler));
  n947_lpspi_modifyreg(priv, N947_LPSPI_TCR_OFFSET,
                       LPSPI_TCR_PRESCALE_MASK,
                       LPSPI_TCR_PRESCALE(best_prescaler));

  priv->frequency = frequency;
  priv->actual = best_frequency;

  if (men != 0)
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
    }

  spiinfo("LPSPI%u requested=%" PRIu32 " actual=%" PRIu32 "\n",
          (unsigned int)(priv->config->base - N947_LPSPI0_BASE) / 0x1000,
          frequency, best_frequency);
  return best_frequency;
}

static void n947_lpspi_setmode(struct spi_dev_s *dev, enum spi_mode_e mode)
{
  struct n947_lpspi_priv_s *priv = (struct n947_lpspi_priv_s *)dev;
  uint32_t setbits;
  uint32_t clearbits;
  uint32_t men;

  if (mode == priv->mode)
    {
      return;
    }

  switch (mode)
    {
      case SPIDEV_MODE0:
        setbits = 0;
        clearbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
        break;

      case SPIDEV_MODE1:
        setbits = LPSPI_TCR_CPHA;
        clearbits = LPSPI_TCR_CPOL;
        break;

      case SPIDEV_MODE2:
        setbits = LPSPI_TCR_CPOL;
        clearbits = LPSPI_TCR_CPHA;
        break;

      case SPIDEV_MODE3:
        setbits = LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
        clearbits = 0;
        break;

      default:
        return;
    }

  men = n947_lpspi_getreg(priv, N947_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
  if (men != 0)
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);
    }

  n947_lpspi_modifyreg(priv, N947_LPSPI_TCR_OFFSET, clearbits, setbits);
  priv->mode = mode;

  if (men != 0)
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
    }
}

static void n947_lpspi_setbits(struct spi_dev_s *dev, int nbits)
{
  struct n947_lpspi_priv_s *priv = (struct n947_lpspi_priv_s *)dev;
  uint32_t men;

  if (nbits < N947_LPSPI_MIN_BITS || nbits > N947_LPSPI_MAX_BITS)
    {
      return;
    }

  if (nbits == priv->nbits)
    {
      return;
    }

  men = n947_lpspi_getreg(priv, N947_LPSPI_CR_OFFSET) & LPSPI_CR_MEN;
  if (men != 0)
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, LPSPI_CR_MEN, 0);
    }

  n947_lpspi_modifyreg(priv, N947_LPSPI_TCR_OFFSET,
                       LPSPI_TCR_FRAMESZ_MASK,
                       LPSPI_TCR_FRAMESZ(nbits - 1));
  priv->nbits = nbits;

  if (men != 0)
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
    }
}

#ifdef CONFIG_SPI_HWFEATURES
static int n947_lpspi_hwfeatures(struct spi_dev_s *dev,
                                 spi_hwfeatures_t features)
{
#ifdef CONFIG_SPI_BITORDER
  struct n947_lpspi_priv_s *priv = (struct n947_lpspi_priv_s *)dev;

  if ((features & HWFEAT_LSBFIRST) != 0)
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_TCR_OFFSET, 0, LPSPI_TCR_LSBF);
    }
  else
    {
      n947_lpspi_modifyreg(priv, N947_LPSPI_TCR_OFFSET, LPSPI_TCR_LSBF, 0);
    }

  return (features & ~HWFEAT_LSBFIRST) == 0 ? OK : -ENOSYS;
#else
  return features == 0 ? OK : -ENOSYS;
#endif
}
#endif

/* TX-only 8-bit burst: keep the TX FIFO fed and discard RX as it drains.
 * The generic per-word path waits for RDF after every byte, which leaves the
 * bus idle between bytes; for one-way display blits (SNDBLOCK) that alone
 * halves throughput.  Used by the exchange fast path when rxbuffer == NULL.
 */

static void n947_lpspi_send_burst(struct n947_lpspi_priv_s *priv,
                                  const uint8_t *src, size_t nwords)
{
  while (nwords > 0)
    {
      if ((n947_lpspi_getreg(priv, N947_LPSPI_SR_OFFSET) &
           LPSPI_SR_TDF) != 0)
        {
          n947_lpspi_putreg(priv, N947_LPSPI_TDR_OFFSET, *src++);
          nwords--;
        }

      while ((n947_lpspi_getreg(priv, N947_LPSPI_RSR_OFFSET) &
              LPSPI_RSR_RXEMPTY) == 0)
        {
          (void)n947_lpspi_getreg(priv, N947_LPSPI_RDR_OFFSET);
        }
    }

  n947_lpspi_wait_idle(priv);

  while ((n947_lpspi_getreg(priv, N947_LPSPI_RSR_OFFSET) &
          LPSPI_RSR_RXEMPTY) == 0)
    {
      (void)n947_lpspi_getreg(priv, N947_LPSPI_RDR_OFFSET);
    }

  n947_lpspi_putreg(priv, N947_LPSPI_SR_OFFSET, LPSPI_SR_CLEAR);
}

static uint32_t n947_lpspi_send(struct spi_dev_s *dev, uint32_t wd)
{
  struct n947_lpspi_priv_s *priv = (struct n947_lpspi_priv_s *)dev;
  uint32_t status;
  uint32_t ret;

  while ((n947_lpspi_getreg(priv, N947_LPSPI_SR_OFFSET) & LPSPI_SR_TDF) == 0)
    {
    }

  n947_lpspi_putreg(priv, N947_LPSPI_TDR_OFFSET, wd);

  /* A write-only SPI target may have no MISO connection.  MCXN947 can
   * therefore report frame-complete without ever placing a word in the RX
   * FIFO.  Waiting exclusively for RDF hangs the caller even though the
   * command was clocked out successfully.  Full-duplex users still return
   * RDR whenever it exists; write-only users complete on FCF with dummy 0.
   */

  do
    {
      status = n947_lpspi_getreg(priv, N947_LPSPI_SR_OFFSET);
    }
  while ((status & (LPSPI_SR_RDF | LPSPI_SR_FCF)) == 0);

  ret = (status & LPSPI_SR_RDF) != 0 ?
        n947_lpspi_getreg(priv, N947_LPSPI_RDR_OFFSET) : 0;
  n947_lpspi_putreg(priv, N947_LPSPI_SR_OFFSET, LPSPI_SR_CLEAR);

  return ret;
}

#ifdef CONFIG_SPI_EXCHANGE
static void n947_lpspi_exchange(struct spi_dev_s *dev,
                                const void *txbuffer, void *rxbuffer,
                                size_t nwords)
#else
static void n947_lpspi_exchange_common(struct spi_dev_s *dev,
                                       const void *txbuffer, void *rxbuffer,
                                       size_t nwords)
#endif
{
  struct n947_lpspi_priv_s *priv = (struct n947_lpspi_priv_s *)dev;

  DEBUGASSERT(priv != NULL);

  if (n947_lpspi_16bitmode(priv))
    {
      const uint16_t *src = (const uint16_t *)txbuffer;
      uint16_t *dest = (uint16_t *)rxbuffer;

      while (nwords-- > 0)
        {
          uint16_t word = src != NULL ? *src++ : 0xffff;
          word = (uint16_t)n947_lpspi_send(dev, word);
          if (dest != NULL)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      const uint8_t *src = (const uint8_t *)txbuffer;
      uint8_t *dest = (uint8_t *)rxbuffer;

      if (dest == NULL && src != NULL && nwords > 4)
        {
          n947_lpspi_send_burst(priv, src, nwords);
          nwords = 0;
        }

      while (nwords-- > 0)
        {
          uint8_t word = src != NULL ? *src++ : 0xff;
          word = (uint8_t)n947_lpspi_send(dev, word);
          if (dest != NULL)
            {
              *dest++ = word;
            }
        }
    }

  n947_lpspi_wait_idle(priv);
}

#ifndef CONFIG_SPI_EXCHANGE
static void n947_lpspi_sndblock(struct spi_dev_s *dev,
                                const void *txbuffer, size_t nwords)
{
  n947_lpspi_exchange_common(dev, txbuffer, NULL, nwords);
}

static void n947_lpspi_recvblock(struct spi_dev_s *dev, void *rxbuffer,
                                 size_t nwords)
{
  n947_lpspi_exchange_common(dev, NULL, rxbuffer, nwords);
}
#endif

#ifdef CONFIG_SPI_CMDDATA
/* Optional board hook for a 4-wire SPI display's command/data (D/C) line.
 * A board that wires an LCD D/C GPIO provides a strong implementation that
 * toggles it (cmd==true -> command, D/C low).  If no board provides it the
 * weak symbol is NULL and cmddata stays unsupported (-ENOSYS).
 */

int weak_function n947_lpspi_cmddata_dc(struct spi_dev_s *dev,
                                        uint32_t devid, bool cmd);

static int n947_lpspi_cmddata(struct spi_dev_s *dev, uint32_t devid,
                              bool cmd)
{
  if (n947_lpspi_cmddata_dc != NULL)
    {
      return n947_lpspi_cmddata_dc(dev, devid, cmd);
    }

  return -ENOSYS;
}
#endif

static int n947_lpspi_registercallback(struct spi_dev_s *dev,
                                       spi_mediachange_t callback,
                                       void *arg)
{
  (void)dev;
  (void)callback;
  (void)arg;
  return -ENOSYS;
}

static void n947_lpspi_configure_pins(struct n947_lpspi_priv_s *priv)
{
  if (priv->config->pins)
    {
      nxxx_port_configure(priv->config->sck_pin);
      nxxx_port_configure(priv->config->miso_pin);
      nxxx_port_configure(priv->config->mosi_pin);
    }
}

static void n947_lpspi_hw_initialize(struct n947_lpspi_priv_s *priv)
{
  n947_lpspi_configure_pins(priv);

  /* The FRO_HF_DIV branch divider is halted out of reset; run it at /1
   * so 48MHz FRO_HF reaches the flexcomm clock mux.
   */

  modifyreg32(SYSCON_DIVFROHFCLK, 0,
              SYSCON_CLKDIV_RESET | SYSCON_CLKDIV_HALT);
  putreg32(0, SYSCON_DIVFROHFCLK);

  nxxx_set_periphclock(priv->config->clk_regs, priv->config->clk_source, 1);
  nxxx_set_clock_gate(priv->config->clk_gate, true);
  putreg32(FLEXCOMM_PSELID_PERSEL_SPI, priv->config->psel_reg);

  n947_lpspi_putreg(priv, N947_LPSPI_CR_OFFSET, LPSPI_CR_RST);
  n947_lpspi_putreg(priv, N947_LPSPI_CR_OFFSET, 0);
  n947_lpspi_flush(priv);

  n947_lpspi_putreg(priv, N947_LPSPI_CFGR0_OFFSET, 0);
  n947_lpspi_putreg(priv, N947_LPSPI_CFGR1_OFFSET,
                    LPSPI_CFGR1_MASTER |
                    LPSPI_CFGR1_OUTCFG_RETAIN |
                    LPSPI_CFGR1_PINCFG_SIN_SOUT);
  n947_lpspi_putreg(priv, N947_LPSPI_FCR_OFFSET,
                    LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0));
  n947_lpspi_putreg(priv, N947_LPSPI_TCR_OFFSET,
                    LPSPI_TCR_WIDTH_1BIT | LPSPI_TCR_PCS(priv->config->pcs));

  priv->frequency = 0;
  priv->actual = 0;
  priv->nbits = 0;
  n947_lpspi_setfrequency(&priv->spidev, N947_LPSPI_DEFAULT_HZ);
  n947_lpspi_setbits(&priv->spidev, 8);
  priv->mode = SPIDEV_MODE3;
  n947_lpspi_setmode(&priv->spidev, SPIDEV_MODE0);

  n947_lpspi_modifyreg(priv, N947_LPSPI_CR_OFFSET, 0, LPSPI_CR_MEN);
}

static struct n947_lpspi_priv_s *n947_lpspi_getpriv(int bus)
{
  switch (bus)
    {
#ifdef CONFIG_N947_LPSPI0
      case 0:
        return &g_lpspi0_priv;
#endif
#ifdef CONFIG_N947_LPSPI1
      case 1:
        return &g_lpspi1_priv;
#endif
#ifdef CONFIG_N947_LPSPI2
      case 2:
        return &g_lpspi2_priv;
#endif
#ifdef CONFIG_N947_LPSPI3
      case 3:
        return &g_lpspi3_priv;
#endif
#ifdef CONFIG_N947_LPSPI4
      case 4:
        return &g_lpspi4_priv;
#endif
#ifdef CONFIG_N947_LPSPI5
      case 5:
        return &g_lpspi5_priv;
#endif
#ifdef CONFIG_N947_LPSPI6
      case 6:
        return &g_lpspi6_priv;
#endif
#ifdef CONFIG_N947_LPSPI7
      case 7:
        return &g_lpspi7_priv;
#endif
      default:
        return NULL;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct spi_dev_s *n947_lpspibus_initialize(int bus)
{
  struct n947_lpspi_priv_s *priv;
  irqstate_t flags;

  priv = n947_lpspi_getpriv(bus);
  if (priv == NULL)
    {
      return NULL;
    }

  flags = enter_critical_section();
  if (priv->refs++ == 0)
    {
      n947_lpspi_hw_initialize(priv);
    }

  leave_critical_section(flags);
  return &priv->spidev;
}

#endif /* CONFIG_N947_LPSPI */
