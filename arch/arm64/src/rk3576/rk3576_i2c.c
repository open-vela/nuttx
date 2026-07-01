/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_i2c.c
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
#include "hardware/rk3576_i2c.h"
#include "rk3576_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C timeout (microseconds) */

#define I2C_TIMEOUT_US              10000
#define I2C_TX_TIMEOUT_US           1000
#define I2C_BUS_CLEAR_TIMEOUT       9

/* FIFO depth */

#define I2C_FIFO_DEPTH              32

/* Source clock (from CRU) */

#define I2C_SRC_CLK_HZ             24000000   /* 24MHz */

/* Register access helpers */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* I2C controller base addresses */

const uint32_t g_i2c_base[RK3576_I2C_COUNT] =
{
  RK3576_I2C0_ADDR,
  RK3576_I2C1_ADDR,
  RK3576_I2C2_ADDR,
  RK3576_I2C3_ADDR,
  RK3576_I2C4_ADDR,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_i2c_validate
 *
 * Description:
 *   Validate I2C bus number.
 *
 ****************************************************************************/

static inline int rk3576_i2c_validate(int bus)
{
  if (bus < 0 || bus >= RK3576_I2C_COUNT)
    {
      i2cerr("I2C: invalid bus %d\n", bus);
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: rk3576_i2c_wait_bus_idle
 *
 * Description:
 *   Wait for I2C bus to become idle.
 *
 ****************************************************************************/

static int rk3576_i2c_wait_bus_idle(int bus)
{
  uint32_t base = g_i2c_base[bus];
  int timeout = I2C_TIMEOUT_US;

  while (timeout--)
    {
      if (!(getreg32(base + I2C_SR) & I2C_SR_BUSY))
        {
          return OK;
        }

      up_udelay(1);
    }

  i2cerr("I2C%d: bus busy timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2c_wait_tx_ready
 *
 * Description:
 *   Wait for TX FIFO to have space.
 *
 ****************************************************************************/

static int rk3576_i2c_wait_tx_ready(int bus)
{
  uint32_t base = g_i2c_base[bus];
  int timeout = I2C_TX_TIMEOUT_US;

  while (timeout--)
    {
      if (getreg32(base + I2C_SR) & I2C_SR_TX_EMPTY)
        {
          return OK;
        }

      up_udelay(1);
    }

  i2cerr("I2C%d: TX ready timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2c_wait_rx_ready
 *
 * Description:
 *   Wait for RX FIFO to have data.
 *
 ****************************************************************************/

static int rk3576_i2c_wait_rx_ready(int bus)
{
  uint32_t base = g_i2c_base[bus];
  int timeout = I2C_TIMEOUT_US;

  while (timeout--)
    {
      /* Check if RX FIFO is not empty (TX_EMPTY=1 means RX has data too) */

      if (!(getreg32(base + I2C_SR) & I2C_SR_TX_EMPTY))
        {
          return OK;
        }

      up_udelay(1);
    }

  i2cerr("I2C%d: RX ready timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2c_send_start
 *
 * Description:
 *   Generate I2C START condition.
 *
 ****************************************************************************/

static int rk3576_i2c_send_start(int bus)
{
  uint32_t base = g_i2c_base[bus];
  uint32_t con;
  int timeout = I2C_TIMEOUT_US;

  /* Wait for bus idle */

  rk3576_i2c_wait_bus_idle(bus);

  /* Generate START */

  con = getreg32(base + I2C_CON);
  con |= I2C_CON_START | I2C_CON_EN;
  putreg32(con, base + I2C_CON);

  /* Wait for START to complete */

  while (timeout--)
    {
      if (!(getreg32(base + I2C_CON) & I2C_CON_START))
        {
          return OK;
        }

      up_udelay(1);
    }

  i2cerr("I2C%d: START timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2c_send_stop
 *
 * Description:
 *   Generate I2C STOP condition.
 *
 ****************************************************************************/

static int rk3576_i2c_send_stop(int bus)
{
  uint32_t base = g_i2c_base[bus];
  uint32_t con;
  int timeout = I2C_TIMEOUT_US;

  con = getreg32(base + I2C_CON);
  con |= I2C_CON_STOP | I2C_CON_EN;
  putreg32(con, base + I2C_CON);

  /* Wait for STOP to complete */

  while (timeout--)
    {
      if (!(getreg32(base + I2C_CON) & I2C_CON_STOP))
        {
          return OK;
        }

      up_udelay(1);
    }

  i2cerr("I2C%d: STOP timeout\n", bus);
  return -ETIMEDOUT;
}

/****************************************************************************
 * Name: rk3576_i2c_send_byte
 *
 * Description:
 *   Send a single byte.
 *
 ****************************************************************************/

static int rk3576_i2c_send_byte(int bus, uint8_t byte)
{
  uint32_t base = g_i2c_base[bus];
  int ret;

  ret = rk3576_i2c_wait_tx_ready(bus);
  if (ret < 0)
    {
      return ret;
    }

  putreg32(byte, base + I2C_TFR);
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2c_recv_byte
 *
 * Description:
 *   Receive a single byte.
 *
 ****************************************************************************/

static int rk3576_i2c_recv_byte(int bus, uint8_t *byte)
{
  uint32_t base = g_i2c_base[bus];
  int ret;

  ret = rk3576_i2c_wait_rx_ready(bus);
  if (ret < 0)
    {
      return ret;
    }

  *byte = (uint8_t)(getreg32(base + I2C_RFR) & 0xff);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rk3576_i2c_init
 *
 * Description:
 *   Initialize an I2C controller.
 *
 ****************************************************************************/

void rk3576_i2c_init(int bus)
{
  uint32_t base;

  if (rk3576_i2c_validate(bus) < 0)
    {
      return;
    }

  base = g_i2c_base[bus];

  /* Disable I2C */

  putreg32(0, base + I2C_CON);

  /* Clear interrupts */

  putreg32(0xffffffff, base + I2C_INTCLR);

  /* Disable interrupts */

  putreg32(0, base + I2C_INTEN);

  /* Clear status */

  putreg32(0xffffffff, base + I2C_INTSTS);

  /* Enable I2C */

  putreg32(I2C_CON_EN, base + I2C_CON);

  i2cinfo("I2C%d: initialized\n", bus);
}

/****************************************************************************
 * Name: rk3576_i2c_set_speed
 *
 * Description:
 *   Set I2C clock speed.
 *
 ****************************************************************************/

int rk3576_i2c_set_speed(int bus, uint32_t speed)
{
  uint32_t base;
  uint32_t div;
  int ret;

  ret = rk3576_i2c_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2c_base[bus];

  /* Calculate clock divisor:
   * SCL frequency = SRC_CLK / (8 * (CLKDIV + 1))
   * CLKDIV = (SRC_CLK / (8 * SCL)) - 1
   */

  div = (I2C_SRC_CLK_HZ / (8 * speed)) - 1;
  if (div > 0xffff)
    {
      div = 0xffff;
    }

  putreg32(div, base + I2C_CLKDIV);

  i2cinfo("I2C%d: speed %u Hz, div %u\n", bus, speed, div);
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2c_set_address
 *
 * Description:
 *   Set the slave address for the next transfer.
 *
 ****************************************************************************/

int rk3576_i2c_set_address(int bus, uint16_t addr)
{
  uint32_t base;
  int ret;

  ret = rk3576_i2c_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2c_base[bus];

  /* Set slave address (7-bit, shifted left by 1) */

  putreg32((addr & 0x7f) << 1, base + I2C_MADDR);

  i2cinfo("I2C%d: addr 0x%02x\n", bus, addr);
  return OK;
}

/****************************************************************************
 * Name: rk3576_i2c_write
 *
 * Description:
 *   Write data to I2C slave (START + addr/W + data + STOP).
 *
 ****************************************************************************/

int rk3576_i2c_write(int bus, const uint8_t *data, int len)
{
  uint32_t base = g_i2c_base[bus];
  uint32_t con;
  int i;
  int ret;

  ret = rk3576_i2c_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  if (data == NULL || len <= 0)
    {
      return -EINVAL;
    }

  /* Wait for bus idle */

  ret = rk3576_i2c_wait_bus_idle(bus);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear status */

  putreg32(0xffffffff, base + I2C_INTSTS);

  /* Set TX mode and enable */

  con = getreg32(base + I2C_CON);
  con |= I2C_CON_EN | I2C_CON_MODE_TX;
  con &= ~I2C_CON_INT_EN;
  putreg32(con, base + I2C_CON);

  /* Generate START */

  ret = rk3576_i2c_send_start(bus);
  if (ret < 0)
    {
      goto err_out;
    }

  /* Send data bytes */

  for (i = 0; i < len; i++)
    {
      ret = rk3576_i2c_send_byte(bus, data[i]);
      if (ret < 0)
        {
          goto err_out;
        }
    }

  /* Wait for last byte to be sent */

  ret = rk3576_i2c_wait_tx_ready(bus);
  if (ret < 0)
    {
      goto err_out;
    }

err_out:
  /* Generate STOP regardless of error */

  rk3576_i2c_send_stop(bus);

  /* Disable I2C */

  putreg32(0, base + I2C_CON);

  return ret;
}

/****************************************************************************
 * Name: rk3576_i2c_read
 *
 * Description:
 *   Read data from I2C slave (START + addr/R + data + STOP).
 *
 ****************************************************************************/

int rk3576_i2c_read(int bus, uint8_t *data, int len)
{
  uint32_t base = g_i2c_base[bus];
  uint32_t con;
  int i;
  int ret;

  ret = rk3576_i2c_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  if (data == NULL || len <= 0)
    {
      return -EINVAL;
    }

  /* Wait for bus idle */

  ret = rk3576_i2c_wait_bus_idle(bus);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear status */

  putreg32(0xffffffff, base + I2C_INTSTS);

  /* Set RX mode and enable */

  con = getreg32(base + I2C_CON);
  con |= I2C_CON_EN | I2C_CON_MODE_RX;
  con &= ~I2C_CON_INT_EN;
  putreg32(con, base + I2C_CON);

  /* Generate START */

  ret = rk3576_i2c_send_start(bus);
  if (ret < 0)
    {
      goto err_out;
    }

  /* Receive data bytes */

  for (i = 0; i < len; i++)
    {
      ret = rk3576_i2c_recv_byte(bus, &data[i]);
      if (ret < 0)
        {
          goto err_out;
        }

      /* Send NACK after last byte */

      if (i == len - 1)
        {
          con = getreg32(base + I2C_CON);
          con |= I2C_CON_ACK;
          putreg32(con, base + I2C_CON);
        }
    }

err_out:
  /* Generate STOP */

  rk3576_i2c_send_stop(bus);

  /* Disable I2C */

  putreg32(0, base + I2C_CON);

  return ret;
}

/****************************************************************************
 * Name: rk3576_i2c_write_reg
 *
 * Description:
 *   Write data to a specific register on the I2C slave.
 *
 ****************************************************************************/

int rk3576_i2c_write_reg(int bus, uint8_t reg, const uint8_t *data, int len)
{
  uint8_t buf[I2C_FIFO_DEPTH + 1];
  int total;

  if (data == NULL || len <= 0)
    {
      return -EINVAL;
    }

  if (len + 1 > I2C_FIFO_DEPTH)
    {
      return -E2BIG;
    }

  /* Build buffer: register address + data */

  buf[0] = reg;
  memcpy(&buf[1], data, len);
  total = len + 1;

  return rk3576_i2c_write(bus, buf, total);
}

/****************************************************************************
 * Name: rk3576_i2c_read_reg
 *
 * Description:
 *   Read data from a specific register on the I2C slave.
 *
 ****************************************************************************/

int rk3576_i2c_read_reg(int bus, uint8_t reg, uint8_t *data, int len)
{
  /* Write register address, then read data */

  return rk3576_i2c_writeread(bus, &reg, 1, data, len);
}

/****************************************************************************
 * Name: rk3576_i2c_writeread
 *
 * Description:
 *   Combined write-then-read operation.
 *   Common pattern for reading registers from I2C devices.
 *
 ****************************************************************************/

int rk3576_i2c_writeread(int bus, const uint8_t *wbuf, int wlen,
                         uint8_t *rbuf, int rlen)
{
  uint32_t base = g_i2c_base[bus];
  uint32_t con;
  int i;
  int ret;

  ret = rk3576_i2c_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  if (wbuf == NULL || wlen <= 0 || rbuf == NULL || rlen <= 0)
    {
      return -EINVAL;
    }

  /* Wait for bus idle */

  ret = rk3576_i2c_wait_bus_idle(bus);
  if (ret < 0)
    {
      return ret;
    }

  /* Clear status */

  putreg32(0xffffffff, base + I2C_INTSTS);

  /* Set TX mode and enable */

  con = getreg32(base + I2C_CON);
  con |= I2C_CON_EN | I2C_CON_MODE_TX;
  con &= ~I2C_CON_INT_EN;
  putreg32(con, base + I2C_CON);

  /* Generate START */

  ret = rk3576_i2c_send_start(bus);
  if (ret < 0)
    {
      goto err_out;
    }

  /* Send write data (register address) */

  for (i = 0; i < wlen; i++)
    {
      ret = rk3576_i2c_send_byte(bus, wbuf[i]);
      if (ret < 0)
        {
          goto err_out;
        }
    }

  /* Wait for write to complete */

  ret = rk3576_i2c_wait_tx_ready(bus);
  if (ret < 0)
    {
      goto err_out;
    }

  /* Generate repeated START for read */

  putreg32(0xffffffff, base + I2C_INTSTS);

  con = getreg32(base + I2C_CON);
  con |= I2C_CON_START;
  putreg32(con, base + I2C_CON);

  /* Wait for START */

  {
    int timeout = I2C_TIMEOUT_US;
    while (timeout--)
      {
        if (!(getreg32(base + I2C_CON) & I2C_CON_START))
          {
            break;
          }

        up_udelay(1);
      }
  }

  /* Switch to RX mode */

  con = getreg32(base + I2C_CON);
  con &= ~I2C_CON_MODE_TX;
  con |= I2C_CON_MODE_RX;
  putreg32(con, base + I2C_CON);

  /* Receive data bytes */

  for (i = 0; i < rlen; i++)
    {
      ret = rk3576_i2c_recv_byte(bus, &rbuf[i]);
      if (ret < 0)
        {
          goto err_out;
        }

      /* Send NACK after last byte */

      if (i == rlen - 1)
        {
          con = getreg32(base + I2C_CON);
          con |= I2C_CON_ACK;
          putreg32(con, base + I2C_CON);
        }
    }

err_out:
  /* Generate STOP */

  rk3576_i2c_send_stop(bus);

  /* Disable I2C */

  putreg32(0, base + I2C_CON);

  return ret;
}

/****************************************************************************
 * Name: rk3576_i2c_reset
 *
 * Description:
 *   Reset I2C controller and release bus.
 *
 ****************************************************************************/

int rk3576_i2c_reset(int bus)
{
  uint32_t base;
  int ret;

  ret = rk3576_i2c_validate(bus);
  if (ret < 0)
    {
      return ret;
    }

  base = g_i2c_base[bus];

  /* Disable I2C */

  putreg32(0, base + I2C_CON);

  /* Clear all interrupts */

  putreg32(0xffffffff, base + I2C_INTCLR);
  putreg32(0xffffffff, base + I2C_INTSTS);

  /* Send 9 clock pulses to clear any stuck slave */

  for (int i = 0; i < I2C_BUS_CLEAR_TIMEOUT; i++)
    {
      /* Toggle SCL via GPIO bit-bang would be needed here */
      /* For now, just re-enable the controller */
    }

  /* Re-initialize */

  rk3576_i2c_init(bus);

  i2cinfo("I2C%d: reset\n", bus);
  return OK;
}
