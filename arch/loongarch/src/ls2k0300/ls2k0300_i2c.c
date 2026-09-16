/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_i2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain the copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by law or agreed to in writing, software
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
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_i2c.h"

#if defined(CONFIG_LS2K0300_I2C)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ls2k0300_i2c_priv_s
{
  struct i2c_master_s dev;
  uintptr_t base;
  uint32_t frequency;
  sem_t sem;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ls2k0300_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count);
static int ls2k0300_i2c_setup(struct i2c_master_s *dev);
static int ls2k0300_i2c_shutdown(struct i2c_master_s *dev);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_i2c_ops =
{
  .transfer  = ls2k0300_i2c_transfer,
  .setup     = ls2k0300_i2c_setup,
  .shutdown  = ls2k0300_i2c_shutdown,
};

#ifdef CONFIG_LS2K0300_I2C0
static struct ls2k0300_i2c_priv_s g_i2c0_priv =
{
  .dev =
    {
      .ops = &g_i2c_ops,
    },
  .base      = LS2K0300_I2C0_BASE,
  .frequency = I2C_SPEED_STANDARD,
  .sem       = SEM_INITIALIZER(1),
};
#endif

#ifdef CONFIG_LS2K0300_I2C1
static struct ls2k0300_i2c_priv_s g_i2c1_priv =
{
  .dev =
    {
      .ops = &g_i2c_ops,
    },
  .base      = LS2K0300_I2C1_BASE,
  .frequency = I2C_SPEED_STANDARD,
  .sem       = SEM_INITIALIZER(1),
};
#endif

#ifdef CONFIG_LS2K0300_I2C2
static struct ls2k0300_i2c_priv_s g_i2c2_priv =
{
  .dev =
    {
      .ops = &g_i2c_ops,
    },
  .base      = LS2K0300_I2C2_BASE,
  .frequency = I2C_SPEED_STANDARD,
  .sem       = SEM_INITIALIZER(1),
};
#endif

#ifdef CONFIG_LS2K0300_I2C3
static struct ls2k0300_i2c_priv_s g_i2c3_priv =
{
  .dev =
    {
      .ops = &g_i2c_ops,
    },
  .base      = LS2K0300_I2C3_BASE,
  .frequency = I2C_SPEED_STANDARD,
  .sem       = SEM_INITIALIZER(1),
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t i2c_getreg(struct ls2k0300_i2c_priv_s *priv,
                                  uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void i2c_putreg(struct ls2k0300_i2c_priv_s *priv,
                              uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

static void i2c_set_bits(struct ls2k0300_i2c_priv_s *priv,
                         uint32_t offset, uint32_t mask)
{
  uint32_t regval = i2c_getreg(priv, offset);
  regval |= mask;
  i2c_putreg(priv, offset, regval);
}

static void i2c_clr_bits(struct ls2k0300_i2c_priv_s *priv,
                         uint32_t offset, uint32_t mask)
{
  uint32_t regval = i2c_getreg(priv, offset);
  regval &= ~mask;
  i2c_putreg(priv, offset, regval);
}

static void ls2k0300_i2c_hw_config(struct ls2k0300_i2c_priv_s *priv)
{
  uint32_t val;
  uint32_t ccr = 0;

  i2c_set_bits(priv, LS2K0300_I2C_CR2, I2C_CR2_FREQ_MASK);
  i2c_set_bits(priv, LS2K0300_I2C_TRISE, I2C_CR2_FREQ_MASK);

  if (priv->frequency <= I2C_SPEED_STANDARD)
    {
      val = (LS2K0300_I2C_INPUT_CLK + 100000 * 2 - 1) / (100000 * 2);
    }
  else
    {
      val = (LS2K0300_I2C_INPUT_CLK + 400000 * 3 - 1) / (400000 * 3);
      ccr |= I2C_CCR_FS;
    }

  ccr |= val & 0xfff;
  i2c_putreg(priv, LS2K0300_I2C_CCR, ccr);

  i2c_putreg(priv, LS2K0300_I2C_CR1, I2C_CR1_PE);
}

static int ls2k0300_i2c_wait_status(struct ls2k0300_i2c_priv_s *priv,
                                   uint32_t mask, bool set,
                                   uint32_t timeout_us)
{
  uint32_t elapsed = 0;
  uint32_t status;

  while (elapsed < timeout_us)
    {
      status = i2c_getreg(priv, LS2K0300_I2C_SR1);

      if (set && (status & mask))
        {
          return OK;
        }

      if (!set && !(status & mask))
        {
          return OK;
        }

      up_udelay(10);
      elapsed += 10;
    }

  return -ETIMEDOUT;
}

static int ls2k0300_i2c_wait_free_bus(struct ls2k0300_i2c_priv_s *priv)
{
  uint32_t elapsed = 0;
  uint32_t status;

  while (elapsed < LS2K0300_I2C_TIMEOUT_US)
    {
      status = i2c_getreg(priv, LS2K0300_I2C_SR2);
      if (!(status & I2C_SR2_BUSY))
        {
          return OK;
        }

      up_udelay(10);
      elapsed += 10;
    }

  /* Bus stuck in busy state - attempt recovery:
   * 1. Disable I2C peripheral
   * 2. Clear all status flags
   * 3. Re-enable I2C peripheral
   */

  i2c_clr_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_PE);
  up_udelay(100);

  /* Clear status registers by reading them */

  i2c_getreg(priv, LS2K0300_I2C_SR1);
  i2c_getreg(priv, LS2K0300_I2C_SR2);

  /* Re-enable peripheral */

  i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_PE);
  up_udelay(100);

  /* Check if bus is now free */

  status = i2c_getreg(priv, LS2K0300_I2C_SR2);
  if (!(status & I2C_SR2_BUSY))
    {
      return OK;
    }

  return -EBUSY;
}

static void ls2k0300_i2c_clear_addr(struct ls2k0300_i2c_priv_s *priv)
{
  i2c_getreg(priv, LS2K0300_I2C_SR1);
  i2c_getreg(priv, LS2K0300_I2C_SR2);
}

static int ls2k0300_i2c_check_errors(struct ls2k0300_i2c_priv_s *priv,
                                    uint32_t status)
{
  if (status & I2C_SR1_ARLO)
    {
      i2c_clr_bits(priv, LS2K0300_I2C_SR1, I2C_SR1_ARLO);
      return -EAGAIN;
    }

  if (status & I2C_SR1_AF)
    {
      i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
      i2c_clr_bits(priv, LS2K0300_I2C_SR1, I2C_SR1_AF);
      return -EIO;
    }

  if (status & I2C_SR1_BERR)
    {
      i2c_clr_bits(priv, LS2K0300_I2C_SR1, I2C_SR1_BERR);
      return -EIO;
    }

  return OK;
}

static int ls2k0300_i2c_send_addr(struct ls2k0300_i2c_priv_s *priv,
                                 struct i2c_msg_s *msg)
{
  uint8_t addr;
  int ret;

  ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_SB, true,
                                 LS2K0300_I2C_TIMEOUT_US);
  if (ret < 0)
    {
      return ret;
    }

  if (msg->flags & I2C_M_READ)
    {
      addr = I2C_READADDR8(msg->addr);
    }
  else
    {
      addr = I2C_WRITEADDR8(msg->addr);
    }

  i2c_putreg(priv, LS2K0300_I2C_DR, addr);

  ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_ADDR, true,
                                 LS2K0300_I2C_TIMEOUT_US);
  if (ret < 0)
    {
      uint32_t sr1 = i2c_getreg(priv, LS2K0300_I2C_SR1);
      int err = ls2k0300_i2c_check_errors(priv, sr1);
      return err < 0 ? err : ret;
    }

  return OK;
}

static int ls2k0300_i2c_read(struct ls2k0300_i2c_priv_s *priv,
                            struct i2c_msg_s *msg, bool is_last)
{
  uint32_t count = msg->length;
  FAR uint8_t *buf = msg->buffer;
  int ret;

  if (count == 0)
    {
      if (is_last)
        {
          i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
        }

      ls2k0300_i2c_clear_addr(priv);
      return OK;
    }

  if (count == 1)
    {
      i2c_clr_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_ACK | I2C_CR1_POS);

      ls2k0300_i2c_clear_addr(priv);

      if (is_last)
        {
          i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
        }

      ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_RXNE, true,
                                     LS2K0300_I2C_TIMEOUT_US);
      if (ret < 0)
        {
          return ret;
        }

      *buf = (uint8_t)i2c_getreg(priv, LS2K0300_I2C_DR);
      return OK;
    }

  if (count == 2)
    {
      i2c_clr_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_ACK);
      i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_POS);

      ls2k0300_i2c_clear_addr(priv);

      while (!(i2c_getreg(priv, LS2K0300_I2C_SR1) & I2C_SR1_BTF))
        ;

      if (is_last)
        {
          i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
        }

      i2c_clr_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_POS);

      *buf++ = (uint8_t)i2c_getreg(priv, LS2K0300_I2C_DR);
      *buf   = (uint8_t)i2c_getreg(priv, LS2K0300_I2C_DR);
      return OK;
    }

  i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_ACK);
  i2c_clr_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_POS);

  ls2k0300_i2c_clear_addr(priv);

  while (count > 3)
    {
      ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_BTF, true,
                                     LS2K0300_I2C_TIMEOUT_US);
      if (ret < 0)
        {
          return ret;
        }

      *buf++ = (uint8_t)i2c_getreg(priv, LS2K0300_I2C_DR);
      count--;
    }

  ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_BTF, true,
                                 LS2K0300_I2C_TIMEOUT_US);
  if (ret < 0)
    {
      return ret;
    }

  i2c_clr_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_ACK);

  (void)i2c_getreg(priv, LS2K0300_I2C_SR1);

  if (is_last)
    {
      i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
    }

  *buf++ = (uint8_t)i2c_getreg(priv, LS2K0300_I2C_DR);
  *buf++ = (uint8_t)i2c_getreg(priv, LS2K0300_I2C_DR);
  count -= 2;

  while (count > 0)
    {
      ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_RXNE, true,
                                     LS2K0300_I2C_TIMEOUT_US);
      if (ret < 0)
        {
          return ret;
        }

      *buf++ = (uint8_t)i2c_getreg(priv, LS2K0300_I2C_DR);
      count--;
    }

  return OK;
}

static int ls2k0300_i2c_write(struct ls2k0300_i2c_priv_s *priv,
                             struct i2c_msg_s *msg, bool is_last)
{
  uint32_t count = msg->length;
  FAR uint8_t *buf = msg->buffer;
  int ret;
  uint32_t sr1;

  ls2k0300_i2c_clear_addr(priv);

  while (count > 0)
    {
      i2c_putreg(priv, LS2K0300_I2C_DR, *buf++);
      count--;

      if (count == 0)
        {
          ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_BTF, true,
                                         LS2K0300_I2C_TIMEOUT_US);
          if (ret < 0)
            {
              return ret;
            }

          if (is_last)
            {
              i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
            }
        }
      else
        {
          ret = ls2k0300_i2c_wait_status(priv, I2C_SR1_TXE, true,
                                         LS2K0300_I2C_TIMEOUT_US);
          if (ret < 0)
            {
              return ret;
            }
        }

      sr1 = i2c_getreg(priv, LS2K0300_I2C_SR1);
      ret = ls2k0300_i2c_check_errors(priv, sr1);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

static int ls2k0300_i2c_transfer_msg(struct ls2k0300_i2c_priv_s *priv,
                                    struct i2c_msg_s *msg, bool is_last)
{
  int ret;
  uint32_t sr1;

  i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_START);

  ret = ls2k0300_i2c_send_addr(priv, msg);
  if (ret < 0)
    {
      i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
      return ret;
    }

  sr1 = i2c_getreg(priv, LS2K0300_I2C_SR1);
  ret = ls2k0300_i2c_check_errors(priv, sr1);
  if (ret < 0)
    {
      i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
      return ret;
    }

  if (msg->flags & I2C_M_READ)
    {
      ret = ls2k0300_i2c_read(priv, msg, is_last);
    }
  else
    {
      ret = ls2k0300_i2c_write(priv, msg, is_last);
    }

  if (ret < 0)
    {
      i2c_set_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_STOP);
      return ret;
    }

  return OK;
}

static int ls2k0300_i2c_transfer(struct i2c_master_s *dev,
                                struct i2c_msg_s *msgs, int count)
{
  struct ls2k0300_i2c_priv_s *priv =
      (struct ls2k0300_i2c_priv_s *)dev;
  int ret;
  int i;

  if (count <= 0 || msgs == NULL)
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&priv->sem);
  if (ret < 0)
    {
      return ret;
    }

  if (msgs[0].frequency != priv->frequency)
    {
      priv->frequency = msgs[0].frequency;
      ls2k0300_i2c_hw_config(priv);
    }

  ret = ls2k0300_i2c_wait_free_bus(priv);
  if (ret < 0)
    {
      nxsem_post(&priv->sem);
      return ret;
    }

  for (i = 0; i < count; i++)
    {
      bool is_last = (i == count - 1);

      if (msgs[i].flags & I2C_M_NOSTART)
        {
          continue;
        }

      ret = ls2k0300_i2c_transfer_msg(priv, &msgs[i], is_last);
      if (ret < 0)
        {
          nxsem_post(&priv->sem);
          return ret;
        }
    }

  nxsem_post(&priv->sem);
  return count;
}

static int ls2k0300_i2c_setup(struct i2c_master_s *dev)
{
  struct ls2k0300_i2c_priv_s *priv =
      (struct ls2k0300_i2c_priv_s *)dev;

  ls2k0300_i2c_hw_config(priv);
  return OK;
}

static int ls2k0300_i2c_shutdown(struct i2c_master_s *dev)
{
  struct ls2k0300_i2c_priv_s *priv =
      (struct ls2k0300_i2c_priv_s *)dev;

  i2c_clr_bits(priv, LS2K0300_I2C_CR1, I2C_CR1_PE);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct i2c_master_s *ls2k0300_i2c_initialize(int port)
{
  struct ls2k0300_i2c_priv_s *priv = NULL;

  switch (port)
    {
#ifdef CONFIG_LS2K0300_I2C0
      case 0:
        priv = &g_i2c0_priv;
        break;
#endif

#ifdef CONFIG_LS2K0300_I2C1
      case 1:
        priv = &g_i2c1_priv;
        break;
#endif

#ifdef CONFIG_LS2K0300_I2C2
      case 2:
        priv = &g_i2c2_priv;
        break;
#endif

#ifdef CONFIG_LS2K0300_I2C3
      case 3:
        priv = &g_i2c3_priv;
        break;
#endif

      default:
        return NULL;
    }

  ls2k0300_i2c_hw_config(priv);
  return &priv->dev;
}

#endif /* CONFIG_LS2K0300_I2C */
