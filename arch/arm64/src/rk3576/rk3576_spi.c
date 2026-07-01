/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_spi.c
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
#include "hardware/rk3576_spi.h"
#include "rk3576_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPI timeout (microseconds) */

#define SPI_TIMEOUT_US              10000

/* Source clock (from CRU) */

#define SPI_SRC_CLK_HZ             24000000   /* 24MHz */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* SPI controller base addresses */

const uint32_t g_spi_base[RK3576_SPI_COUNT] =
{
  RK3576_SPI0_ADDR,
  RK3576_SPI1_ADDR,
  RK3576_SPI2_ADDR,
  RK3576_SPI3_ADDR,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_spi_validate
 *
 * Description:
 *   Validate SPI bus number.
 *
 ****************************************************************************/

static inline int rk3576_spi_validate(int bus)
{
  if (bus < 0 || bus >= RK3576_SPI_COUNT)
    {
      spierr("SPI: invalid bus %d\n", bus);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_spi_wait_not_busy
 *
 * Description:
 *   Wait for SPI to become idle.
 *
 ****************************************************************************/

static int rk3576_spi_wait_not_busy(int bus)
{
  uint32_t base = g_spi_base[bus];
  int timeout = SPI_TIMEOUT_US;

  while (timeout--)
    {
      if (!(getreg32(base + SPI_SR) & SPI_SR_BUSY))
        {
          return OK;
        }

      up_udelay(1);
    }

  spierr("SPI%d: busy timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_spi_drain_rx
 *
 * Description:
 *   Drain all data from the RX FIFO.
 *
 ****************************************************************************/

static void rk3576_spi_drain_rx(int bus)
{
  uint32_t base = g_spi_base[bus];

  while (getreg32(base + SPI_SR) & SPI_SR_RFNE)
    {
      (void)getreg32(base + SPI_RXDR);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_spi_init
 *
 * Description:
 *   Initialize SPI controller. Disable SPI and clear FIFOs.
 *
 ****************************************************************************/

void rk3576_spi_init(int bus)
{
  uint32_t base;

  if (rk3576_spi_validate(bus) < 0)
    {
      return;
    }

  base = g_spi_base[bus];

  /* Disable SPI */

  putreg32(0, base + SPI_CTRLR0);

  /* Clear interrupts */

  putreg32(0, base + SPI_IMR);
  putreg32(0xffffffff, base + SPI_ICR);

  /* Drain RX FIFO */

  rk3576_spi_drain_rx(bus);

  /* Set default: 8-bit, mode 0, SPI master */

  putreg32(SPI_CTRLR0_DFS(8) |
           SPI_CTRLR0_FRF_MOTOROLA |
           SPI_CTRLR0_TMOD_TR,
           base + SPI_CTRLR0);

  /* Set default baud rate divider */

  putreg32(24, base + SPI_BAUDR);   /* 24MHz / 24 = 1MHz */

  spiinfo("SPI%d: initialized\n", bus);
}

/****************************************************************************
 * Name: rk3576_spi_set_mode
 *
 * Description:
 *   Set SPI mode (CPOL, CPHA).
 *
 ****************************************************************************/

int rk3576_spi_set_mode(int bus, int mode)
{
  uint32_t base;
  uint32_t ctrl0;
  int ret;

  ret = rk3576_spi_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_spi_base[bus];

  ctrl0 = getreg32(base + SPI_CTRLR0);
  ctrl0 &= ~(SPI_CTRLR0_SCPH | SPI_CTRLR0_SCPOl);

  switch (mode)
    {
      case RK3576_SPI_MODE_0:   /* CPOL=0, CPHA=0 */
        break;
      case RK3576_SPI_MODE_1:   /* CPOL=0, CPHA=1 */
        ctrl0 |= SPI_CTRLR0_SCPH;
        break;
      case RK3576_SPI_MODE_2:   /* CPOL=1, CPHA=0 */
        ctrl0 |= SPI_CTRLR0_SCPOl;
        break;
      case RK3576_SPI_MODE_3:   /* CPOL=1, CPHA=1 */
        ctrl0 |= SPI_CTRLR0_SCPH | SPI_CTRLR0_SCPOl;
        break;
      default:
        return -EINVAL;
    }

  putreg32(ctrl0, base + SPI_CTRLR0);

  spiinfo("SPI%d: mode %d\n", bus, mode);
  return OK;
}

/****************************************************************************
 * Name: rk3576_spi_set_bits
 *
 * Description:
 *   Set SPI word size (4-16 bits).
 *
 ****************************************************************************/

int rk3576_spi_set_bits(int bus, int bits)
{
  uint32_t base;
  uint32_t ctrl0;
  int ret;

  ret = rk3576_spi_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  if (bits < 4 || bits > 16)
    {
      return -EINVAL;
    }

  base = g_spi_base[bus];

  ctrl0 = getreg32(base + SPI_CTRLR0);
  ctrl0 &= ~SPI_CTRLR0_DFS_MASK;
  ctrl0 |= SPI_CTRLR0_DFS(bits);
  putreg32(ctrl0, base + SPI_CTRLR0);

  spiinfo("SPI%d: %d bits\n", bus, bits);
  return OK;
}

/****************************************************************************
 * Name: rk3576_spi_set_frequency
 *
 * Description:
 *   Set SPI clock frequency.
 *
 ****************************************************************************/

int rk3576_spi_set_frequency(int bus, uint32_t freq)
{
  uint32_t base;
  uint32_t div;
  int ret;

  ret = rk3576_spi_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_spi_base[bus];

  /* Calculate baud rate divider:
   * SCLK = SRC_CLK / DIV
   * DIV = SRC_CLK / SCLK
   */

  div = SPI_SRC_CLK_HZ / freq;
  if (div < 2)
    {
      div = 2;   /* Minimum divider */
    }

  if (div > 65534)
    {
      div = 65534;   /* Maximum even divider */
    }

  /* Ensure divider is even (hardware requirement) */

  div = (div + 1) & ~1;

  putreg32(div, base + SPI_BAUDR);

  spiinfo("SPI%d: freq %u Hz, div %u\n", bus, freq, div);
  return OK;
}

/****************************************************************************
 * Name: rk3576_spi_exchange
 *
 * Description:
 *   Full duplex SPI transfer. Simultaneously sends and receives data.
 *
 ****************************************************************************/

int rk3576_spi_exchange(int bus, const uint8_t *txbuf,
                        uint8_t *rxbuf, int len)
{
  uint32_t base;
  int tx_count = 0;
  int rx_count = 0;
  int ret;

  ret = rk3576_spi_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  if (len <= 0)
    {
      return 0;
    }

  base = g_spi_base[bus];

  /* Clear any pending interrupts/status */

  putreg32(0xffffffff, base + SPI_ICR);

  /* Set transfer mode to TX&RX */

  {
    uint32_t ctrl0 = getreg32(base + SPI_CTRLR0);
    ctrl0 &= ~SPI_CTRLR0_TMOD_MASK;
    ctrl0 |= SPI_CTRLR0_TMOD_TR;
    putreg32(ctrl0, base + SPI_CTRLR0);
  }

  /* Transfer loop */

  while (tx_count < len || rx_count < len)
    {
      /* Fill TX FIFO while there's space and data to send */

      while (tx_count < len &&
             (getreg32(base + SPI_SR) & SPI_SR_TFNF))
        {
          uint8_t byte = txbuf ? txbuf[tx_count] : 0xff;
          putreg32(byte, base + SPI_TXDR);
          tx_count++;
        }

      /* Drain RX FIFO while there's data */

      while (rx_count < len &&
             (getreg32(base + SPI_SR) & SPI_SR_RFNE))
        {
          uint8_t byte = (uint8_t)(getreg32(base + SPI_RXDR) & 0xff);
          if (rxbuf)
            {
              rxbuf[rx_count] = byte;
            }

          rx_count++;
        }
    }

  /* Wait for transfer to complete */

  ret = rk3576_spi_wait_not_busy(bus);
  if (ret < 0)
    {
      return ret;
    }

  /* Drain any remaining RX data */

  while (getreg32(base + SPI_SR) & SPI_SR_RFNE)
    {
      if (rxbuf && rx_count < len)
        {
          rxbuf[rx_count++] = (uint8_t)(getreg32(base + SPI_RXDR) & 0xff);
        }
      else
        {
          (void)getreg32(base + SPI_RXDR);
        }
    }

  return len;
}

/****************************************************************************
 * Name: rk3576_spi_send
 *
 * Description:
 *   Send data (TX only, ignores received data).
 *
 ****************************************************************************/

int rk3576_spi_send(int bus, const uint8_t *data, int len)
{
  return rk3576_spi_exchange(bus, data, NULL, len);
}

/****************************************************************************
 * Name: rk3576_spi_recv
 *
 * Description:
 *   Receive data (sends 0xFF for each byte).
 *
 ****************************************************************************/

int rk3576_spi_recv(int bus, uint8_t *data, int len)
{
  return rk3576_spi_exchange(bus, NULL, data, len);
}

/****************************************************************************
 * Name: rk3576_spi_sendrecv
 *
 * Description:
 *   Send one byte and receive one byte (full duplex).
 *
 ****************************************************************************/

uint8_t rk3576_spi_sendrecv(int bus, uint8_t data)
{
  uint8_t rx;
  int ret;

  ret = rk3576_spi_exchange(bus, &data, &rx, 1);
  if (ret < 0)
    {
      return 0xff;
    }

  return rx;
}

/****************************************************************************
 * Name: rk3576_spi_reset
 *
 * Description:
 *   Reset SPI controller.
 *
 ****************************************************************************/

int rk3576_spi_reset(int bus)
{
  int ret;

  ret = rk3576_spi_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  /* Disable SPI */

  putreg32(0, g_spi_base[bus] + SPI_CTRLR0);

  /* Clear interrupts */

  putreg32(0xffffffff, g_spi_base[bus] + SPI_ICR);

  /* Drain RX FIFO */

  rk3576_spi_drain_rx(bus);

  /* Re-initialize */

  rk3576_spi_init(bus);

  spiinfo("SPI%d: reset\n", bus);
  return OK;
}
