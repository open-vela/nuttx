/****************************************************************************
 * arch/arm/src/t113/t113_i2c.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/i2c/i2c_master.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "t113_ccu.h"
#include "t113_clk.h"
#include "t113_gpio.h"
#include "hardware/t113_i2c.h"
#include "hardware/t113_clk.h"
#include "t113_i2c.h"

#if defined(CONFIG_T113_TWI0) || defined(CONFIG_T113_TWI1) || \
    defined(CONFIG_T113_TWI2) || defined(CONFIG_T113_TWI3)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TWI_TIMEOUT_MS  100

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* I2C transfer state machine states */

enum twi_state_e
{
  TWI_IDLE = 0,
  TWI_START,
  TWI_ADDR,
  TWI_DATA_TX,
  TWI_DATA_RX,
  TWI_STOP,
  TWI_ERROR,
};

struct t113_i2cdev_s
{
  struct i2c_master_s    i2cdev;   /* Externally visible I2C interface */
  uint32_t               base;     /* TWI controller base address */
  int                    irq;      /* TWI interrupt number */
  mutex_t                lock;     /* Bus-level mutex */
  sem_t                  sem;      /* Transfer completion semaphore */
  FAR struct i2c_msg_s  *msgs;     /* Current message array */
  int                    nmsgs;    /* Number of messages */
  int                    msgidx;   /* Current message index */
  int                    byteidx;  /* Byte index within current message */
  int                    result;   /* Transfer result (0=OK, <0=error) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int twi_transfer(FAR struct i2c_master_s *dev,
                         FAR struct i2c_msg_s *msgs, int count);
#ifdef CONFIG_I2C_RESET
static int twi_reset(FAR struct i2c_master_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2c_ops_s g_twiops =
{
  .transfer = twi_transfer,
#ifdef CONFIG_I2C_RESET
  .reset    = twi_reset,
#endif
};

#define DEFINE_TWI(n, base_addr, irq_num)                \
  static struct t113_i2cdev_s g_twi##n##dev =            \
  {                                                       \
    .i2cdev = { .ops = &g_twiops },                      \
    .base   = base_addr,                                  \
    .irq    = irq_num,                                    \
    .lock   = NXMUTEX_INITIALIZER,                        \
    .sem    = SEM_INITIALIZER(0),                         \
  }

#ifdef CONFIG_T113_TWI0
DEFINE_TWI(0, T113_TWI0_BASE, T113_IRQ_TWI0);
#endif
#ifdef CONFIG_T113_TWI1
DEFINE_TWI(1, T113_TWI1_BASE, T113_IRQ_TWI1);
#endif
#ifdef CONFIG_T113_TWI2
DEFINE_TWI(2, T113_TWI2_BASE, T113_IRQ_TWI2);
#endif
#ifdef CONFIG_T113_TWI3
DEFINE_TWI(3, T113_TWI3_BASE, T113_IRQ_TWI3);
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void twi_putreg(struct t113_i2cdev_s *priv,
                               uint32_t offset, uint32_t val)
{
  putreg32(val, priv->base + offset);
}

static inline uint32_t twi_getreg(struct t113_i2cdev_s *priv,
                                    uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static void twi_reset_hw(struct t113_i2cdev_s *priv)
{
  twi_putreg(priv, TWI_SRST_REG, TWI_SRST_RESET);
  up_udelay(10);
  twi_putreg(priv, TWI_SRST_REG, 0);

  /* Forget any in-flight transfer so a later ISR cannot touch stale
   * msgs.
   */

  priv->msgs    = NULL;
  priv->msgidx  = 0;
  priv->byteidx = 0;
}

static void twi_setclock(struct t113_i2cdev_s *priv, uint32_t freq)
{
  uint32_t n;
  uint32_t m;
  uint32_t div;
  uint32_t clk_in = t113_apb1_freq();   /* live APB1 (TWI shares it) */

  DEBUGASSERT(freq != 0);

  /* Fscl = Fin / (2^N * (M+1) * 10)
   * Try N=2, solve for M: M = (Fin / (10 * Fscl * 4)) - 1
   */

  for (n = 0; n <= 7; n++)
    {
      div = clk_in / (freq * 10 * (1 << n));
      if (div >= 1 && div <= 16)
        {
          m = div - 1;
          twi_putreg(priv, TWI_CCR_REG,
                     (m << TWI_CCR_CLK_M_SHIFT) |
                     (n << TWI_CCR_CLK_N_SHIFT));
          return;
        }
    }

  /* Fallback: use N=2, M=11 -> ~50kHz */

  twi_putreg(priv, TWI_CCR_REG,
             (11 << TWI_CCR_CLK_M_SHIFT) | (2 << TWI_CCR_CLK_N_SHIFT));
}

static void twi_clock_enable(struct t113_i2cdev_s *priv)
{
  int bit;

  /* TWI BGR at CCU + 0x091C: bits[3:0]=gate, bits[19:16]=reset */

  if (priv->base == T113_TWI0_BASE)
    {
      bit = 0;
    }
  else if (priv->base == T113_TWI1_BASE)
    {
      bit = 1;
    }
  else if (priv->base == T113_TWI2_BASE)
    {
      bit = 2;
    }
  else
    {
      bit = 3;
    }

  /* Cycle reset: clear, brief delay, set.  Then cycle bus gate the same
   * way.  Each step is a locked RMW on the shared TWI_BGR.
   */

  t113_ccu_modify(T113_CCU_TWI_BGR, (1 << (16 + bit)), 0);
  up_udelay(1);
  t113_ccu_modify(T113_CCU_TWI_BGR, 0, (1 << (16 + bit)));

  t113_ccu_modify(T113_CCU_TWI_BGR, (1 << bit), 0);
  up_udelay(1);
  t113_ccu_modify(T113_CCU_TWI_BGR, 0, (1 << bit));

  /* Pin mux - selections come from board.h via t113_pinmap.h */

#ifdef CONFIG_T113_TWI0
  if (priv->base == T113_TWI0_BASE)
    {
      t113_gpio_config(T113_TWI0_SCK);
      t113_gpio_config(T113_TWI0_SDA);
    }
#endif

#ifdef CONFIG_T113_TWI1
  if (priv->base == T113_TWI1_BASE)
    {
      t113_gpio_config(T113_TWI1_SCK);
      t113_gpio_config(T113_TWI1_SDA);
    }
#endif

#ifdef CONFIG_T113_TWI2
  if (priv->base == T113_TWI2_BASE)
    {
      t113_gpio_config(T113_TWI2_SCK);
      t113_gpio_config(T113_TWI2_SDA);
    }
#endif

#ifdef CONFIG_T113_TWI3
  if (priv->base == T113_TWI3_BASE)
    {
      t113_gpio_config(T113_TWI3_SCK);
      t113_gpio_config(T113_TWI3_SDA);
    }
#endif
}

static void twi_send_start(struct t113_i2cdev_s *priv)
{
  uint32_t cntr;

  /* INT_FL is W1C: writing 1 acks any pending interrupt so the controller
   * actually advances to send the (re)START condition.
   */

  cntr = TWI_CNTR_INT_FL | TWI_CNTR_INT_EN | TWI_CNTR_BUS_EN |
         TWI_CNTR_START | TWI_CNTR_A_ACK;
  twi_putreg(priv, TWI_CNTR_REG, cntr);
}

static void twi_send_stop(struct t113_i2cdev_s *priv)
{
  uint32_t cntr;

  cntr = TWI_CNTR_INT_FL | TWI_CNTR_INT_EN | TWI_CNTR_BUS_EN |
         TWI_CNTR_STOP | TWI_CNTR_A_ACK;
  twi_putreg(priv, TWI_CNTR_REG, cntr);
}

static void twi_clear_irq(struct t113_i2cdev_s *priv)
{
  uint32_t cntr;

  /* INT_FL is write-1-to-clear.  Setting the bit acks the pending
   * interrupt and lets the controller advance to the next state.
   */

  cntr = twi_getreg(priv, TWI_CNTR_REG);
  cntr |= TWI_CNTR_INT_FL | TWI_CNTR_INT_EN |
          TWI_CNTR_BUS_EN | TWI_CNTR_A_ACK;
  twi_putreg(priv, TWI_CNTR_REG, cntr);
}

static int twi_irq_handler(int irq, void *context, void *arg)
{
  FAR struct t113_i2cdev_s *priv = (FAR struct t113_i2cdev_s *)arg;
  uint32_t stat;
  FAR struct i2c_msg_s *msg;
  bool done = false;

  stat = twi_getreg(priv, TWI_STAT_REG);

  switch (stat)
    {
      case TWI_STAT_START:
      case TWI_STAT_RSTART:
        msg = &priv->msgs[priv->msgidx];
        twi_putreg(priv, TWI_DATA_REG,
                   (msg->addr << 1) |
                   ((msg->flags & I2C_M_READ) ? 1 : 0));
        twi_clear_irq(priv);
        break;

      case TWI_STAT_ADDR_W_ACK:
        msg = &priv->msgs[priv->msgidx];
        priv->byteidx = 0;
        if (msg->length == 0)
          {
            /* Zero-length write (e.g. SMBus quick / ping) - nothing to
             * send.  Either start the next message or stop.
             */

            priv->msgidx++;
            if (priv->msgidx < priv->nmsgs)
              {
                twi_send_start(priv);
              }
            else
              {
                twi_send_stop(priv);
                done = true;
              }
          }
        else
          {
            twi_putreg(priv, TWI_DATA_REG, msg->buffer[priv->byteidx++]);
            twi_clear_irq(priv);
          }
        break;

      case TWI_STAT_DATA_T_ACK:
        msg = &priv->msgs[priv->msgidx];
        if (priv->byteidx < msg->length)
          {
            twi_putreg(priv, TWI_DATA_REG,
                       msg->buffer[priv->byteidx++]);
            twi_clear_irq(priv);
          }
        else
          {
            priv->msgidx++;
            if (priv->msgidx < priv->nmsgs)
              {
                twi_send_start(priv);
              }
            else
              {
                twi_send_stop(priv);
                done = true;
              }
          }
        break;

      case TWI_STAT_ADDR_R_ACK:
        msg = &priv->msgs[priv->msgidx];
        priv->byteidx = 0;
        if (msg->length == 1)
          {
            /* Single-byte read: clear A_ACK before the byte arrives so
             * the controller NAKs it (I2C protocol requires the master
             * to NAK the last byte of a read transfer).  INT_FL is W1C.
             */

            uint32_t cntr = twi_getreg(priv, TWI_CNTR_REG);
            cntr &= ~TWI_CNTR_A_ACK;
            cntr |= TWI_CNTR_INT_FL | TWI_CNTR_INT_EN | TWI_CNTR_BUS_EN;
            twi_putreg(priv, TWI_CNTR_REG, cntr);
          }
        else
          {
            twi_clear_irq(priv);
          }
        break;

      case TWI_STAT_DATA_R_ACK:
        msg = &priv->msgs[priv->msgidx];
        msg->buffer[priv->byteidx++] = (uint8_t)twi_getreg(priv,
                                                             TWI_DATA_REG);
        if (priv->byteidx < msg->length - 1)
          {
            twi_clear_irq(priv);
          }
        else
          {
            /* Clear A_ACK so the next-to-last data byte gets NAKed.
             * INT_FL is W1C - write 1 to acknowledge.
             */

            uint32_t cntr = twi_getreg(priv, TWI_CNTR_REG);
            cntr &= ~TWI_CNTR_A_ACK;
            cntr |= TWI_CNTR_INT_FL | TWI_CNTR_INT_EN | TWI_CNTR_BUS_EN;
            twi_putreg(priv, TWI_CNTR_REG, cntr);
          }
        break;

      case TWI_STAT_DATA_R_NAK:
        msg = &priv->msgs[priv->msgidx];
        msg->buffer[priv->byteidx++] = (uint8_t)twi_getreg(priv,
                                                             TWI_DATA_REG);
        priv->msgidx++;
        if (priv->msgidx < priv->nmsgs)
          {
            twi_send_start(priv);
          }
        else
          {
            twi_send_stop(priv);
            done = true;
          }
        break;

      case TWI_STAT_ADDR_W_NAK:
      case TWI_STAT_ADDR_R_NAK:
        priv->result = -ENXIO;
        twi_send_stop(priv);
        done = true;
        break;

      case TWI_STAT_DATA_T_NAK:
        priv->result = -EIO;
        twi_send_stop(priv);
        done = true;
        break;

      case TWI_STAT_ARB_LOST:
        priv->result = -EAGAIN;
        twi_send_stop(priv);
        done = true;
        break;

      case TWI_STAT_BUS_ERR:
        priv->result = -EIO;
        twi_reset_hw(priv);
        twi_send_stop(priv);
        done = true;
        break;

      case TWI_STAT_IDLE:
        done = true;
        break;

      default:
        twi_clear_irq(priv);
        break;
    }

  if (done)
    {
      nxsem_post(&priv->sem);
    }

  return OK;
}

static int twi_transfer(FAR struct i2c_master_s *dev,
                         FAR struct i2c_msg_s *msgs, int count)
{
  FAR struct t113_i2cdev_s *priv = (FAR struct t113_i2cdev_s *)dev;
  int ret;

  DEBUGASSERT(msgs != NULL && count > 0);

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  priv->msgs    = msgs;
  priv->nmsgs   = count;
  priv->msgidx  = 0;
  priv->byteidx = 0;
  priv->result  = 0;

  twi_setclock(priv, msgs[0].frequency);
  twi_send_start(priv);

  ret = nxsem_tickwait_uninterruptible(&priv->sem,
            MSEC2TICK(TWI_TIMEOUT_MS));
  if (ret < 0)
    {
      /* Mask the IRQ before tearing down the controller so a pending
       * ISR cannot run against the next caller's state.
       */

      up_disable_irq(priv->irq);
      twi_clear_irq(priv);
      twi_reset_hw(priv);
      up_enable_irq(priv->irq);
      ret = -ETIMEDOUT;
    }
  else
    {
      ret = priv->result;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

#ifdef CONFIG_I2C_RESET

/* Replace the FUNC field of a pinset while keeping port/pin/pull/drv
 * intact.  Used to flip the TWI SCL/SDA pads between I2C alt function
 * and GPIO in/out for bit-bang recovery.
 */

#define PINSET_WITH_FUNC(p, f) \
  (((uint16_t)(p) & (uint16_t)~T113_GPIO_FUNC_MASK) | \
   (((uint16_t)(f) << T113_GPIO_FUNC_SHIFT) & T113_GPIO_FUNC_MASK))

/****************************************************************************
 * Name: twi_bus_recover
 *
 * Description:
 *   Free a bus where a slave has stuck SDA low (e.g. after an MCU reset
 *   mid-transfer).  Temporarily re-muxes SCL/SDA to GPIO, clocks SCL up
 *   to 9 times until SDA floats high, then issues a STOP.  The pads are
 *   returned to TWI alt function on exit.
 *
 ****************************************************************************/

static void twi_bus_recover(uint16_t scl_pin, uint16_t sda_pin)
{
  uint16_t scl_out = PINSET_WITH_FUNC(scl_pin, T113_GPIO_OUTPUT);
  uint16_t sda_in  = PINSET_WITH_FUNC(sda_pin, T113_GPIO_INPUT);
  uint16_t sda_out = PINSET_WITH_FUNC(sda_pin, T113_GPIO_OUTPUT);
  int i;

  /* Drive SCL high to start, SDA released (input with pull-up from pad
   * config) so we can sample whether the slave still holds it low.
   */

  t113_gpio_write(scl_out, true);
  t113_gpio_config(scl_out);
  t113_gpio_config(sda_in);

  for (i = 0; i < 9; i++)
    {
      t113_gpio_write(scl_out, false);
      up_udelay(5);
      t113_gpio_write(scl_out, true);
      up_udelay(5);

      if (t113_gpio_read(sda_in))
        {
          break;
        }
    }

  /* Generate a STOP: SDA low while SCL high, then SDA high. */

  t113_gpio_write(sda_out, false);
  t113_gpio_config(sda_out);
  up_udelay(5);
  t113_gpio_write(scl_out, true);
  up_udelay(5);
  t113_gpio_write(sda_out, true);
  up_udelay(5);

  /* Restore TWI alt function. */

  t113_gpio_config(scl_pin);
  t113_gpio_config(sda_pin);
}

static int twi_reset(FAR struct i2c_master_s *dev)
{
  FAR struct t113_i2cdev_s *priv = (FAR struct t113_i2cdev_s *)dev;

  /* Bit-bang a recovery sequence before the controller reset so a
   * slave holding SDA low gets unstuck.
   */

#ifdef CONFIG_T113_TWI0
  if (priv->base == T113_TWI0_BASE)
    {
      twi_bus_recover(T113_TWI0_SCK, T113_TWI0_SDA);
    }
#endif

#ifdef CONFIG_T113_TWI1
  if (priv->base == T113_TWI1_BASE)
    {
      twi_bus_recover(T113_TWI1_SCK, T113_TWI1_SDA);
    }
#endif

#ifdef CONFIG_T113_TWI2
  if (priv->base == T113_TWI2_BASE)
    {
      twi_bus_recover(T113_TWI2_SCK, T113_TWI2_SDA);
    }
#endif

#ifdef CONFIG_T113_TWI3
  if (priv->base == T113_TWI3_BASE)
    {
      twi_bus_recover(T113_TWI3_SCK, T113_TWI3_SDA);
    }
#endif

  twi_reset_hw(priv);
  twi_putreg(priv, TWI_CNTR_REG,
             TWI_CNTR_INT_EN | TWI_CNTR_BUS_EN | TWI_CNTR_A_ACK);
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct i2c_master_s *t113_i2cbus_initialize(int bus)
{
  FAR struct t113_i2cdev_s *priv;

  switch (bus)
    {
#ifdef CONFIG_T113_TWI0
      case 0: priv = &g_twi0dev; break;
#endif
#ifdef CONFIG_T113_TWI1
      case 1: priv = &g_twi1dev; break;
#endif
#ifdef CONFIG_T113_TWI2
      case 2: priv = &g_twi2dev; break;
#endif
#ifdef CONFIG_T113_TWI3
      case 3: priv = &g_twi3dev; break;
#endif
      default: return NULL;
    }

  twi_clock_enable(priv);
  twi_reset_hw(priv);
  twi_setclock(priv, 100000);   /* Default 100kHz */

  twi_putreg(priv, TWI_CNTR_REG,
             TWI_CNTR_INT_EN | TWI_CNTR_BUS_EN | TWI_CNTR_A_ACK);

  irq_attach(priv->irq, twi_irq_handler, priv);
  up_enable_irq(priv->irq);

  return (FAR struct i2c_master_s *)priv;
}

int t113_i2cbus_uninitialize(FAR struct i2c_master_s *dev)
{
  FAR struct t113_i2cdev_s *priv = (FAR struct t113_i2cdev_s *)dev;

  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
  twi_putreg(priv, TWI_CNTR_REG, 0);

  return OK;
}

#endif /* CONFIG_T113_TWI* */
