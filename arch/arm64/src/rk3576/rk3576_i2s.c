/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_i2s.c
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
#include "hardware/rk3576_i2s.h"
#include "rk3576_i2s.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2S timeout (microseconds) */

#define I2S_TIMEOUT_US              10000

/* Source clock (from CRU) */

#define I2S_SRC_CLK_HZ             24000000   /* 24MHz */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2S controller base addresses */

const uint32_t g_i2s_base[RK3576_I2S_COUNT] =
{
  RK3576_I2S0_ADDR,
  RK3576_I2S1_ADDR,
  RK3576_I2S2_ADDR,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_i2s_validate
 *
 * Description:
 *   Validate I2S bus number.
 *
 ****************************************************************************/

static inline int rk3576_i2s_validate(int bus)
{
  if (bus < 0 || bus >= RK3576_I2S_COUNT)
    {
      i2serr("I2S: invalid bus %d\n", bus);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_i2s_wait_tx_ready
 *
 * Description:
 *   Wait for TX FIFO to have space.
 *
 ****************************************************************************/

static int rk3576_i2s_wait_tx_ready(int bus)
{
  uint32_t base = g_i2s_base[bus];
  int timeout = I2S_TIMEOUT_US;

  while (timeout--)
    {
      uint32_t fifolr = getreg32(base + I2S_TXFIFOLR);
      if (!(fifolr & (1 << 8)))   /* TXFF bit */
        {
          return OK;
        }

      up_udelay(1);
    }

  i2serr("I2S%d: TX ready timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2s_wait_rx_ready
 *
 * Description:
 *   Wait for RX FIFO to have data.
 *
 ****************************************************************************/

static int rk3576_i2s_wait_rx_ready(int bus)
{
  uint32_t base = g_i2s_base[bus];
  int timeout = I2S_TIMEOUT_US;

  while (timeout--)
    {
      uint32_t fifolr = getreg32(base + I2S_RXFIFOLR);
      if (!(fifolr & (1 << 8)))   /* RXFE bit (empty = 1, not empty = 0) */
        {
          return OK;
        }

      up_udelay(1);
    }

  i2serr("I2S%d: RX ready timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2s_wait_not_busy
 *
 * Description:
 *   Wait for I2S to become idle.
 *
 ****************************************************************************/

static int rk3576_i2s_wait_not_busy(int bus)
{
  uint32_t base = g_i2s_base[bus];
  int timeout = I2S_TIMEOUT_US;

  while (timeout--)
    {
      uint32_t sr = getreg32(base + I2S_SR);
      if (!(sr & (I2S_SR_TXBUSY | I2S_SR_RXBUSY)))
        {
          return OK;
        }

      up_udelay(1);
    }

  i2serr("I2S%d: busy timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2s_calc_divider
 *
 * Description:
 *   Calculate clock divider for the given sample rate.
 *
 ****************************************************************************/

static uint32_t rk3576_i2s_calc_divider(uint32_t sample_rate, int bits,
                                         int channels)
{
  uint32_t sclk_freq;
  uint32_t div;

  /* MCLK = sample_rate * bits * channels * oversampling
   * SCLK = MCLK * 256 (for I2S standard)
   */

  sclk_freq = sample_rate * bits * channels * 256;

  /* Calculate divider from source clock */

  if (sclk_freq == 0)
    {
      return 1;
    }

  div = I2S_SRC_CLK_HZ / sclk_freq;
  if (div < 1)
    {
      div = 1;
    }

  return div;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_i2s_init
 *
 * Description:
 *   Initialize I2S controller. Disable I2S and clear FIFOs.
 *
 ****************************************************************************/

void rk3576_i2s_init(int bus)
{
  uint32_t base;

  if (rk3576_i2s_validate(bus) < 0)
    {
      return;
    }

  base = g_i2s_base[bus];

  /* Disable TX and RX */

  putreg32(0, base + I2S_TXCR);
  putreg32(0, base + I2S_RXCR);

  /* Clear interrupts */

  putreg32(0, base + I2S_IMR);
  putreg32(0xffffffff, base + I2S_ICR);

  /* Clear TX and RX FIFOs */

  putreg32(0xffffffff, base + I2S_CLR_TX);
  putreg32(0xffffffff, base + I2S_CLR_RX);

  /* Disable DMA */

  putreg32(0, base + I2S_DMACR);

  /* Set default: I2S master, stereo, 16-bit */

  putreg32(I2S_CKR_SCLKG_NODELAY |
           I2S_CKR_TXM_MASTER |
           I2S_CKR_RXM_MASTER |
           I2S_CKR_MSS_MASTER,
           base + I2S_CKR);

  i2sinfo("I2S%d: initialized\n", bus);
}

/****************************************************************************
 * Name: rk3576_i2s_set_samplerate
 *
 * Description:
 *   Set audio sample rate. Configures clock divider.
 *
 ****************************************************************************/

int rk3576_i2s_set_samplerate(int bus, uint32_t rate)
{
  uint32_t base;
  uint32_t ckr;
  uint32_t div;
  int ret;

  ret = rk3576_i2s_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2s_base[bus];

  /* Calculate divider for 16-bit stereo */

  div = rk3576_i2s_calc_divider(rate, 16, 2);

  /* Update clock configuration */

  ckr = getreg32(base + I2S_CKR);
  ckr &= ~I2S_CKR_SCLKG_MASK;
  ckr |= I2S_CKR_SCLKG_NODELAY;
  putreg32(ckr, base + I2S_CKR);

  i2sinfo("I2S%d: rate %u Hz, div %u\n", bus, rate, div);
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2s_set_bits
 *
 * Description:
 *   Set sample word size (16, 20, 24, or 32 bits).
 *
 ****************************************************************************/

int rk3576_i2s_set_bits(int bus, int bits)
{
  uint32_t base;
  uint32_t txcr;
  uint32_t rxcr;
  int ret;

  ret = rk3576_i2s_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2s_base[bus];

  /* Map bits to hardware encoding */

  uint32_t dfs;
  uint32_t trc;

  switch (bits)
    {
      case 16:
        dfs = I2S_TXCR_DFS_16;
        trc = I2S_TXCR_TRC_16;
        break;
      case 20:
        dfs = I2S_TXCR_DFS_20;
        trc = I2S_TXCR_TRC_20;
        break;
      case 24:
        dfs = I2S_TXCR_DFS_24;
        trc = I2S_TXCR_TRC_24;
        break;
      case 32:
        dfs = I2S_TXCR_DFS_32;
        trc = I2S_TXCR_TRC_32;
        break;
      default:
        return -EINVAL;
    }

  /* Update TX config */

  txcr = getreg32(base + I2S_TXCR);
  txcr &= ~(I2S_TXCR_DFS_MASK | I2S_TXCR_TRC_MASK);
  txcr |= dfs | trc;
  putreg32(txcr, base + I2S_TXCR);

  /* Update RX config */

  rxcr = getreg32(base + I2S_RXCR);
  rxcr &= ~(I2S_RXCR_DFS_MASK | I2S_RXCR_TRC_MASK);
  rxcr |= dfs | trc;
  putreg32(rxcr, base + I2S_RXCR);

  i2sinfo("I2S%d: %d bits\n", bus, bits);
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2s_set_channels
 *
 * Description:
 *   Set number of audio channels (1=mono, 2=stereo).
 *
 ****************************************************************************/

int rk3576_i2s_set_channels(int bus, int channels)
{
  uint32_t base;
  uint32_t txcr;
  uint32_t rxcr;
  int ret;

  ret = rk3576_i2s_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2s_base[bus];

  /* Update TX config */

  txcr = getreg32(base + I2S_TXCR);
  txcr &= ~I2S_TXCR_STR_MASK;

  if (channels == 1)
    {
      txcr |= I2S_TXCR_STR_MONO;
    }
  else
    {
      txcr |= I2S_TXCR_STR_STEREO;
    }

  putreg32(txcr, base + I2S_TXCR);

  /* Update RX config */

  rxcr = getreg32(base + I2S_RXCR);
  rxcr &= ~I2S_RXCR_STR_MASK;

  if (channels == 1)
    {
      rxcr |= I2S_RXCR_STR_MONO;
    }
  else
    {
      rxcr |= I2S_RXCR_STR_STEREO;
    }

  putreg32(rxcr, base + I2S_RXCR);

  i2sinfo("I2S%d: %d channels\n", bus, channels);
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2s_set_mode
 *
 * Description:
 *   Set I2S transfer mode (TX, RX, or both).
 *
 ****************************************************************************/

int rk3576_i2s_set_mode(int bus, int mode)
{
  uint32_t base;
  uint32_t txcr;
  uint32_t rxcr;
  int ret;

  ret = rk3576_i2s_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2s_base[bus];

  txcr = getreg32(base + I2S_TXCR);
  rxcr = getreg32(base + I2S_RXCR);

  switch (mode)
    {
      case RK3576_I2S_MODE_TX:
        txcr |= I2S_TXCR_TXEN;
        rxcr &= ~I2S_RXCR_RXEN;
        break;

      case RK3576_I2S_MODE_RX:
        txcr &= ~I2S_TXCR_TXEN;
        rxcr |= I2S_RXCR_RXEN;
        break;

      case RK3576_I2S_MODE_BOTH:
        txcr |= I2S_TXCR_TXEN;
        rxcr |= I2S_RXCR_RXEN;
        break;

      default:
        return -EINVAL;
    }

  putreg32(txcr, base + I2S_TXCR);
  putreg32(rxcr, base + I2S_RXCR);

  i2sinfo("I2S%d: mode %d\n", bus, mode);
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2s_set_role
 *
 * Description:
 *   Set I2S role (master or slave).
 *
 ****************************************************************************/

int rk3576_i2s_set_role(int bus, int role)
{
  uint32_t base;
  uint32_t ckr;
  int ret;

  ret = rk3576_i2s_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2s_base[bus];

  ckr = getreg32(base + I2S_CKR);
  ckr &= ~(I2S_CKR_TXM_MASK | I2S_CKR_RXM_MASK | I2S_CKR_MSS_MASK);

  if (role == RK3576_I2S_ROLE_MASTER)
    {
      ckr |= I2S_CKR_TXM_MASTER | I2S_CKR_RXM_MASTER | I2S_CKR_MSS_MASTER;
    }
  else
    {
      ckr |= I2S_CKR_TXM_SLAVE | I2S_CKR_RXM_SLAVE | I2S_CKR_MSS_SLAVE;
    }

  putreg32(ckr, base + I2S_CKR);

  i2sinfo("I2S%d: role %s\n", bus, role == 0 ? "master" : "slave");
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2s_start
 *
 * Description:
 *   Start I2S transmission/reception.
 *
 ****************************************************************************/

void rk3576_i2s_start(int bus)
{
  uint32_t base;

  if (rk3576_i2s_validate(bus) < 0)
    {
      return;
    }

  base = g_i2s_base[bus];

  /* Enable TX/RX based on current config */

  {
    uint32_t txcr = getreg32(base + I2S_TXCR);
    uint32_t rxcr = getreg32(base + I2S_RXCR);
    putreg32(txcr, base + I2S_TXCR);
    putreg32(rxcr, base + I2S_RXCR);
  }

  i2sinfo("I2S%d: started\n", bus);
}

/****************************************************************************
 * Name: rk3576_i2s_stop
 *
 * Description:
 *   Stop I2S transmission/reception.
 *
 ****************************************************************************/

void rk3576_i2s_stop(int bus)
{
  uint32_t base;

  if (rk3576_i2s_validate(bus) < 0)
    {
      return;
    }

  base = g_i2s_base[bus];

  /* Disable TX and RX */

  {
    uint32_t txcr = getreg32(base + I2S_TXCR);
    uint32_t rxcr = getreg32(base + I2S_RXCR);
    txcr &= ~I2S_TXCR_TXEN;
    rxcr &= ~I2S_RXCR_RXEN;
    putreg32(txcr, base + I2S_TXCR);
    putreg32(rxcr, base + I2S_RXCR);
  }

  /* Clear FIFOs */

  putreg32(0xffffffff, base + I2S_CLR_TX);
  putreg32(0xffffffff, base + I2S_CLR_RX);

  i2sinfo("I2S%d: stopped\n", bus);
}

/****************************************************************************
 * Name: rk3576_i2s_send
 *
 * Description:
 *   Send audio data (blocking). TX only.
 *
 ****************************************************************************/

int rk3576_i2s_send(int bus, const uint16_t *data, int len)
{
  uint32_t base;
  int ret;

  ret = rk3576_i2s_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  if (data == NULL || len <= 0)
    {
      return -EINVAL;
    }

  base = g_i2s_base[bus];

  /* Enable TX */

  {
    uint32_t txcr = getreg32(base + I2S_TXCR);
    txcr |= I2S_TXCR_TXEN;
    putreg32(txcr, base + I2S_TXCR);
  }

  /* Send data */

  for (int i = 0; i < len; i++)
    {
      ret = rk3576_i2s_wait_tx_ready(bus);
      if (ret < 0)
        {
          break;
        }

      putreg32(data[i], base + I2S_TXDR);
    }

  /* Wait for last sample to be sent */

  rk3576_i2s_wait_not_busy(bus);

  return ret;
}

/****************************************************************************
 * Name: rk3576_i2s_recv
 *
 * Description:
 *   Receive audio data (blocking). RX only.
 *
 ****************************************************************************/

int rk3576_i2s_recv(int bus, uint16_t *data, int len)
{
  uint32_t base;
  int ret;

  ret = rk3576_i2s_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  if (data == NULL || len <= 0)
    {
      return -EINVAL;
    }

  base = g_i2s_base[bus];

  /* Enable RX */

  {
    uint32_t rxcr = getreg32(base + I2S_RXCR);
    rxcr |= I2S_RXCR_RXEN;
    putreg32(rxcr, base + I2S_RXCR);
  }

  /* Receive data */

  for (int i = 0; i < len; i++)
    {
      ret = rk3576_i2s_wait_rx_ready(bus);
      if (ret < 0)
        {
          break;
        }

      data[i] = (uint16_t)(getreg32(base + I2S_RXDR) & 0xffff);
    }

  return ret;
}

/****************************************************************************
 * Name: rk3576_i2s_dma_enable
 *
 * Description:
 *   Enable DMA for TX and/or RX.
 *
 ****************************************************************************/

void rk3576_i2s_dma_enable(int bus, bool tx, bool rx)
{
  uint32_t base;
  uint32_t dmacr;

  if (rk3576_i2s_validate(bus) < 0)
    {
      return;
    }

  base = g_i2s_base[bus];

  dmacr = getreg32(base + I2S_DMACR);

  if (tx)
    {
      dmacr |= I2S_DMACR_TXDMAE;
    }

  if (rx)
    {
      dmacr |= I2S_DMACR_RXDMAE;
    }

  putreg32(dmacr, base + I2S_DMACR);
}

/****************************************************************************
 * Name: rk3576_i2s_dma_disable
 *
 * Description:
 *   Disable DMA for TX and/or RX.
 *
 ****************************************************************************/

void rk3576_i2s_dma_disable(int bus, bool tx, bool rx)
{
  uint32_t base;
  uint32_t dmacr;

  if (rk3576_i2s_validate(bus) < 0)
    {
      return;
    }

  base = g_i2s_base[bus];

  dmacr = getreg32(base + I2S_DMACR);

  if (tx)
    {
      dmacr &= ~I2S_DMACR_TXDMAE;
    }

  if (rx)
    {
      dmacr &= ~I2S_DMACR_RXDMAE;
    }

  putreg32(dmacr, base + I2S_DMACR);
}
