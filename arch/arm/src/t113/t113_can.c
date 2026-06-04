/****************************************************************************
 * arch/arm/src/t113/t113_can.c
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

/* T113-S3 CAN controller driver (Allwinner sun20i-d1 variant).
 *
 * This driver directly implements the NuttX CAN lower-half (can_ops_s)
 * interface.  The T113 CAN IP is an Allwinner custom design (not a
 * standard SJA1000) with:
 *   - 32-bit word-aligned registers (not byte-addressed)
 *   - Single packed BTIME register (not two BTR0/BTR1)
 *   - D1-variant acceptance filter at 0x28/0x2C
 *   - Combined error counter register
 *   - No CLOCK_DIVIDER/EXT_MODE register
 *
 * Loopback mode note:
 *   The T113 CAN "self-test" mode (MSEL bit 2 + CMD SELF_REQ) only
 *   suppresses ACK checking - it does NOT provide internal digital
 *   loopback.  TX still drives the physical pin and monitors RX,
 *   so a transceiver or TX-RX short is still required to avoid bit
 *   errors.  When CONFIG_CAN_LOOPBACK is enabled, this driver
 *   implements pure software loopback: t113_can_send() echoes the
 *   frame directly to can_receive() without touching the hardware
 *   TX path, providing reliable loopback testing without any
 *   physical bus connection.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/can/can.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "t113_clk.h"
#include "t113_gpio.h"
#include "hardware/t113_can.h"
#include "hardware/t113_ccu.h"
#include "t113_can.h"
#include "t113_ccu.h"

#if defined(CONFIG_T113_CAN0) || defined(CONFIG_T113_CAN1)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define T113_CAN_MODE_RETRIES   100

#ifndef CONFIG_T113_CAN_BITRATE
#  define CONFIG_T113_CAN_BITRATE 500000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_can_priv_s
{
  uint32_t          base;       /* MMIO base address */
  int               irq;        /* IRQ number */
  int               port;       /* Port number (0 or 1) */
  bool              loopback;   /* Loopback mode enabled */
  struct can_dev_s  dev;        /* NuttX CAN device */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void t113_can_reset(FAR struct can_dev_s *dev);
static int  t113_can_setup(FAR struct can_dev_s *dev);
static void t113_can_shutdown(FAR struct can_dev_s *dev);
static void t113_can_rxint(FAR struct can_dev_s *dev, bool enable);
static void t113_can_txint(FAR struct can_dev_s *dev, bool enable);
static int  t113_can_ioctl(FAR struct can_dev_s *dev,
                           int cmd, unsigned long arg);
static int  t113_can_send(FAR struct can_dev_s *dev,
                          FAR struct can_msg_s *msg);
static bool t113_can_txready(FAR struct can_dev_s *dev);
static bool t113_can_txempty(FAR struct can_dev_s *dev);

static int  t113_can_isr(int irq, FAR void *context, FAR void *arg);
static int  t113_can_enter_reset(struct t113_can_priv_s *priv);
static int  t113_can_leave_reset(struct t113_can_priv_s *priv);
static int  t113_can_set_bittiming(struct t113_can_priv_s *priv,
                                   uint32_t bitrate);
static int  t113_can_busoff_recovery(struct t113_can_priv_s *priv);
static void t113_can_ccu_enable(int port);
static void t113_can_gpio_config(int port);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct can_ops_s g_t113_can_ops =
{
  .co_reset         = t113_can_reset,
  .co_setup         = t113_can_setup,
  .co_shutdown      = t113_can_shutdown,
  .co_rxint         = t113_can_rxint,
  .co_txint         = t113_can_txint,
  .co_ioctl         = t113_can_ioctl,
  .co_remoterequest = NULL,
  .co_send          = t113_can_send,
  .co_txready       = t113_can_txready,
  .co_txempty       = t113_can_txempty,
  .co_cancel        = NULL,
  .co_errhandle     = NULL,
};

#ifdef CONFIG_T113_CAN0
static struct t113_can_priv_s g_can0priv =
{
  .base     = T113_CAN0_BASE,
  .irq      = T113_IRQ_CAN0,
  .port     = 0,
#ifdef CONFIG_CAN_LOOPBACK
  .loopback = true,
#else
  .loopback = false,
#endif
};
#endif

#ifdef CONFIG_T113_CAN1
static struct t113_can_priv_s g_can1priv =
{
  .base     = T113_CAN1_BASE,
  .irq      = T113_IRQ_CAN1,
  .port     = 1,
#ifdef CONFIG_CAN_LOOPBACK
  .loopback = true,
#else
  .loopback = false,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t t113_can_getreg(struct t113_can_priv_s *priv,
                                       uint32_t offset)
{
  return getreg32(priv->base + offset);
}

static inline void t113_can_putreg(struct t113_can_priv_s *priv,
                                   uint32_t offset, uint32_t value)
{
  putreg32(value, priv->base + offset);
}

/****************************************************************************
 * Name: t113_can_enter_reset
 ****************************************************************************/

static int t113_can_enter_reset(struct t113_can_priv_s *priv)
{
  int retry = T113_CAN_MODE_RETRIES;
  uint32_t msel;

  do
    {
      msel = t113_can_getreg(priv, T113_CAN_MSEL_OFFSET);
      msel |= T113_CAN_MSEL_RESET;
      t113_can_putreg(priv, T113_CAN_MSEL_OFFSET, msel);
      up_udelay(1);
    }
  while (retry-- > 0 &&
         !(t113_can_getreg(priv, T113_CAN_MSEL_OFFSET) &
           T113_CAN_MSEL_RESET));

  if (!(t113_can_getreg(priv, T113_CAN_MSEL_OFFSET) &
        T113_CAN_MSEL_RESET))
    {
      canerr("ERROR: CAN%d enter reset mode failed\n", priv->port);
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: t113_can_leave_reset
 ****************************************************************************/

static int t113_can_leave_reset(struct t113_can_priv_s *priv)
{
  int retry = T113_CAN_MODE_RETRIES;
  uint32_t msel;

  do
    {
      msel = t113_can_getreg(priv, T113_CAN_MSEL_OFFSET);
      msel &= ~T113_CAN_MSEL_RESET;
      t113_can_putreg(priv, T113_CAN_MSEL_OFFSET, msel);
      up_udelay(1);
    }
  while (retry-- > 0 &&
         (t113_can_getreg(priv, T113_CAN_MSEL_OFFSET) &
          T113_CAN_MSEL_RESET));

  msel = t113_can_getreg(priv, T113_CAN_MSEL_OFFSET);

  if (msel & T113_CAN_MSEL_RESET)
    {
      canerr("ERROR: CAN%d leave reset mode failed (MSEL=0x%08lx)\n",
             priv->port, (unsigned long)msel);
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: t113_can_busoff_recovery
 *
 * Description:
 *   Recover from bus-off by toggling reset mode.  Entering reset clears
 *   the transmit/receive error counters (they drop to ErrorActive once
 *   reset is released), which is the hardware-defined recovery path.
 *
 ****************************************************************************/

static int t113_can_busoff_recovery(struct t113_can_priv_s *priv)
{
  int ret;

  ret = t113_can_enter_reset(priv);
  if (ret < 0)
    {
      return ret;
    }

  up_udelay(10);

  return t113_can_leave_reset(priv);
}

/****************************************************************************
 * Name: t113_can_set_bittiming
 *
 * Description:
 *   Configure CAN bus timing.  Must be called in reset mode.
 *
 *   Bit rate = clk_freq / (brp * (1 + tseg1 + tseg2))
 *   Note: Manual suggests a 2x factor but hardware validation shows no 2x
 *   is needed; the formula above is correct for this silicon.
 *
 ****************************************************************************/

static int t113_can_set_bittiming(struct t113_can_priv_s *priv,
                                  uint32_t bitrate)
{
  uint32_t clk_freq = t113_apb1_freq();   /* live APB1 (CAN shares it) */
  uint32_t best_brp = 0;
  uint32_t best_tseg1 = 0;
  uint32_t best_tseg2 = 0;
  uint32_t best_err = UINT32_MAX;
  uint32_t tolerance;
  uint32_t sjw;
  uint32_t brp;
  uint32_t cfg;

  /* Reject candidates whose actual bitrate deviates > 1% from requested */

  tolerance = bitrate / 100;
  if (tolerance == 0)
    {
      tolerance = 1;
    }

  for (brp = 1; brp <= 64; brp++)
    {
      uint32_t tq_per_bit;
      uint32_t actual;
      uint32_t err;
      uint32_t tseg1;
      uint32_t tseg2;

      tq_per_bit = clk_freq / (brp * bitrate);
      if (tq_per_bit < 3 || tq_per_bit > 25)
        {
          continue;
        }

      /* ~70% sample point */

      tseg1 = (tq_per_bit * 7 + 5) / 10 - 1;
      tseg2 = tq_per_bit - 1 - tseg1;

      if (tseg1 < 1)  tseg1 = 1;
      if (tseg1 > 16) tseg1 = 16;
      if (tseg2 < 1)  tseg2 = 1;
      if (tseg2 > 8)  tseg2 = 8;

      actual = clk_freq / (brp * (1 + tseg1 + tseg2));
      err = actual > bitrate ? actual - bitrate : bitrate - actual;

      if (err > tolerance)
        {
          continue;
        }

      if (err < best_err)
        {
          best_err   = err;
          best_brp   = brp;
          best_tseg1 = tseg1;
          best_tseg2 = tseg2;
          if (err == 0)
            {
              break;
            }
        }
    }

  if (best_brp == 0)
    {
      canerr("ERROR: CAN%d no bittiming within 1%% tolerance for %lu bps\n",
             priv->port, (unsigned long)bitrate);
      return -EINVAL;
    }

  /* SJW is limited by TSEG2 (max 4 per spec) */

  sjw = best_tseg2 > 4 ? 4 : best_tseg2;

  caninfo("CAN%d: bitrate=%lu brp=%lu tseg1=%lu tseg2=%lu sjw=%lu "
          "(err=%lu)\n",
          priv->port, (unsigned long)bitrate,
          (unsigned long)best_brp, (unsigned long)best_tseg1,
          (unsigned long)best_tseg2, (unsigned long)sjw,
          (unsigned long)best_err);

  /* Pack into BTIME register: all fields are (value - 1) encoded */

  cfg = ((best_brp - 1) & 0x3ff) |
        (((sjw - 1) & 0x3) << T113_CAN_BTIME_SJW_SHIFT) |
        (((best_tseg1 - 1) & 0xf) << T113_CAN_BTIME_TSEG1_SHIFT) |
        (((best_tseg2 - 1) & 0x7) << T113_CAN_BTIME_TSEG2_SHIFT);

  t113_can_putreg(priv, T113_CAN_BTIME_OFFSET, cfg);
  return OK;
}

/****************************************************************************
 * Name: t113_can_selftest
 *
 * Description:
 *   Register-level self-test in reset mode.  Validates CAN controller
 *   register access without requiring bus activity or transceiver.
 *
 ****************************************************************************/

static int t113_can_selftest(struct t113_can_priv_s *priv)
{
  int pass = 1;
  uint32_t val;

  /* TEWL register (default 0x60, writable in reset mode) */

  t113_can_putreg(priv, T113_CAN_TEWL_OFFSET, 0x50);
  val = t113_can_getreg(priv, T113_CAN_TEWL_OFFSET) & 0xff;
  if (val != 0x50)
    {
      canerr("CAN%d selftest: TEWL wrote 0x50 read 0x%02lx\n",
             priv->port, (unsigned long)val);
      pass = 0;
    }

  t113_can_putreg(priv, T113_CAN_TEWL_OFFSET, 0x60);

  /* ACPC register (D1 offset 0x28) */

  t113_can_putreg(priv, T113_CAN_ACPC_OFFSET, 0xaa55aa55);
  val = t113_can_getreg(priv, T113_CAN_ACPC_OFFSET);
  if (val != 0xaa55aa55)
    {
      canerr("CAN%d selftest: ACPC wrote 0xaa55aa55 read 0x%08lx\n",
             priv->port, (unsigned long)val);
      pass = 0;
    }

  /* ACPM register */

  t113_can_putreg(priv, T113_CAN_ACPM_OFFSET, 0x12345678);
  val = t113_can_getreg(priv, T113_CAN_ACPM_OFFSET);
  if (val != 0x12345678)
    {
      canerr("CAN%d selftest: ACPM wrote 0x12345678 read 0x%08lx\n",
             priv->port, (unsigned long)val);
      pass = 0;
    }

  /* BTIME register (should be non-zero after set_bittiming) */

  val = t113_can_getreg(priv, T113_CAN_BTIME_OFFSET);
  if (val == 0)
    {
      canerr("CAN%d selftest: BTIME is zero\n", priv->port);
      pass = 0;
    }

  /* Restore acceptance filter to accept all */

  t113_can_putreg(priv, T113_CAN_ACPC_OFFSET, 0x00000000);
  t113_can_putreg(priv, T113_CAN_ACPM_OFFSET, 0xffffffff);

  _alert("CAN%d: register selftest %s\n",
         priv->port, pass ? "PASSED" : "FAILED");

  return pass ? OK : -EIO;
}

/****************************************************************************
 * Name: t113_can_reset
 ****************************************************************************/

static void t113_can_reset(FAR struct can_dev_s *dev)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;

  t113_can_enter_reset(priv);
  t113_can_putreg(priv, T113_CAN_INTEN_OFFSET, 0);
  t113_can_putreg(priv, T113_CAN_ACPC_OFFSET, 0x00000000);
  t113_can_putreg(priv, T113_CAN_ACPM_OFFSET, 0xffffffff);
}

/****************************************************************************
 * Name: t113_can_setup
 *
 * Description:
 *   Configure the CAN.  Sequence: enter_reset - filters - counters -
 *   bittiming - mode - leave_reset:
 *     enter_reset - filters - counters - bittiming - mode - leave
 *
 ****************************************************************************/

static int t113_can_setup(FAR struct can_dev_s *dev)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  uint32_t msel;
  int ret;

  caninfo("CAN%d setup: base=0x%08lx irq=%d loopback=%d\n",
          priv->port, (unsigned long)priv->base, priv->irq,
          priv->loopback);

  ret = t113_can_enter_reset(priv);
  if (ret < 0)
    {
      return ret;
    }

  /* Set acceptance filter: accept all */

  t113_can_putreg(priv, T113_CAN_ACPC_OFFSET, 0x00000000);
  t113_can_putreg(priv, T113_CAN_ACPM_OFFSET, 0xffffffff);

  /* Clear error counters */

  t113_can_putreg(priv, T113_CAN_ERRC_OFFSET, 0);

  /* Configure bus timing (must be in reset mode) */

  ret = t113_can_set_bittiming(priv, CONFIG_T113_CAN_BITRATE);
  if (ret < 0)
    {
      return ret;
    }

  /* Set operating mode (still in reset) */

  msel = T113_CAN_MSEL_RESET;
  t113_can_putreg(priv, T113_CAN_MSEL_OFFSET, msel);

  /* Run register-level self-test while still in reset mode */

  ret = t113_can_selftest(priv);
  if (ret < 0)
    {
      canwarn("CAN%d: selftest failed, continuing anyway\n", priv->port);
    }

#ifdef CONFIG_CAN_LOOPBACK
  if (priv->loopback)
    {
      /* Software loopback: keep controller in reset mode, no IRQ needed.
       * The bus is never used - all TX/RX goes through software echo.
       */

      caninfo("CAN%d: software loopback mode (hardware stays in reset)\n",
              priv->port);
      return OK;
    }
#endif

  /* Clear pending interrupts (W1C) and enable interrupt sources */

  t113_can_putreg(priv, T113_CAN_INT_OFFSET, 0xff);
  t113_can_putreg(priv, T113_CAN_INTEN_OFFSET,
                  T113_CAN_INTEN_RX |
                  T113_CAN_INTEN_TX |
                  T113_CAN_INTEN_ERR_WARNING |
                  T113_CAN_INTEN_ERR_PASSIVE |
                  T113_CAN_INTEN_OVERRUN |
                  T113_CAN_INTEN_BUSERR);

  irq_attach(priv->irq, t113_can_isr, dev);
  up_enable_irq(priv->irq);

  /* Leave reset mode */

  ret = t113_can_leave_reset(priv);
  if (ret < 0)
    {
      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
      return ret;
    }

  caninfo("CAN%d: operational MSEL=0x%08lx STA=0x%08lx\n",
          priv->port,
          (unsigned long)t113_can_getreg(priv, T113_CAN_MSEL_OFFSET),
          (unsigned long)t113_can_getreg(priv, T113_CAN_STA_OFFSET));

  return OK;
}

/****************************************************************************
 * Name: t113_can_shutdown
 ****************************************************************************/

static void t113_can_shutdown(FAR struct can_dev_s *dev)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;

#ifdef CONFIG_CAN_LOOPBACK
  if (priv->loopback)
    {
      /* Software loopback never enabled IRQ or left reset mode */

      return;
    }
#endif

  t113_can_enter_reset(priv);
  t113_can_putreg(priv, T113_CAN_INTEN_OFFSET, 0);
  up_disable_irq(priv->irq);
  irq_detach(priv->irq);
}

/****************************************************************************
 * Name: t113_can_rxint
 ****************************************************************************/

static void t113_can_rxint(FAR struct can_dev_s *dev, bool enable)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  irqstate_t flags;
  uint32_t inten;

  flags = up_irq_save();
  inten = t113_can_getreg(priv, T113_CAN_INTEN_OFFSET);
  if (enable)
    {
      inten |= T113_CAN_INTEN_RX;
    }
  else
    {
      inten &= ~T113_CAN_INTEN_RX;
    }

  t113_can_putreg(priv, T113_CAN_INTEN_OFFSET, inten);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: t113_can_txint
 ****************************************************************************/

static void t113_can_txint(FAR struct can_dev_s *dev, bool enable)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  irqstate_t flags;
  uint32_t inten;

  flags = up_irq_save();
  inten = t113_can_getreg(priv, T113_CAN_INTEN_OFFSET);
  if (enable)
    {
      inten |= T113_CAN_INTEN_TX;
    }
  else
    {
      inten &= ~T113_CAN_INTEN_TX;
    }

  t113_can_putreg(priv, T113_CAN_INTEN_OFFSET, inten);
  up_irq_restore(flags);
}

/****************************************************************************
 * Name: t113_can_ioctl
 ****************************************************************************/

static int t113_can_ioctl(FAR struct can_dev_s *dev,
                          int cmd, unsigned long arg)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  int ret;

  switch (cmd)
    {
      case CANIOC_GET_BITTIMING:
        {
          FAR struct canioc_bittiming_s *bt =
            (FAR struct canioc_bittiming_s *)arg;
          uint32_t cfg;
          uint32_t brp;
          uint32_t tseg1;
          uint32_t tseg2;

          if (bt == NULL)
            {
              return -EINVAL;
            }

          cfg = t113_can_getreg(priv, T113_CAN_BTIME_OFFSET);
          brp   = (cfg & T113_CAN_BTIME_BRP_MASK) + 1;
          tseg1 = ((cfg & T113_CAN_BTIME_TSEG1_MASK)
                   >> T113_CAN_BTIME_TSEG1_SHIFT) + 1;
          tseg2 = ((cfg & T113_CAN_BTIME_TSEG2_MASK)
                   >> T113_CAN_BTIME_TSEG2_SHIFT) + 1;

          bt->bt_baud  = t113_apb1_freq() / (brp * (1 + tseg1 + tseg2));
          bt->bt_tseg1 = tseg1;
          bt->bt_tseg2 = tseg2;
          bt->bt_sjw   = ((cfg & T113_CAN_BTIME_SJW_MASK)
                          >> T113_CAN_BTIME_SJW_SHIFT) + 1;
          return OK;
        }

      case CANIOC_SET_BITTIMING:
        {
          FAR const struct canioc_bittiming_s *bt =
            (FAR const struct canioc_bittiming_s *)arg;

          if (bt == NULL || bt->bt_baud == 0)
            {
              return -EINVAL;
            }

          ret = t113_can_enter_reset(priv);
          if (ret < 0)
            {
              return ret;
            }

          ret = t113_can_set_bittiming(priv, bt->bt_baud);
          if (ret < 0)
            {
              t113_can_leave_reset(priv);
              return ret;
            }

          return t113_can_leave_reset(priv);
        }

      case CANIOC_BUSOFF_RECOVERY:
        return t113_can_busoff_recovery(priv);

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Name: t113_can_send
 *
 * Description:
 *   Send one CAN message.  Frame layout in TX buffer:
 *     BUF0: frame info (EFF flag | RTR flag | DLC)
 *     BUF1-2 (SFF) or BUF1-4 (EFF): arbitration ID
 *     BUF3-10 (SFF) or BUF5-12 (EFF): data bytes
 *
 ****************************************************************************/

static int t113_can_send(FAR struct can_dev_s *dev,
                         FAR struct can_msg_s *msg)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  uint32_t id;
  uint8_t  dlc;
  uint8_t  frame_info;
  int      dreg;
  int      i;

#ifdef CONFIG_CAN_LOOPBACK
  if (priv->loopback)
    {
      /* Software loopback: echo frame directly to RX path without
       * touching the hardware.  The Allwinner CAN "self-test" mode
       * does NOT provide internal digital loopback - it still drives
       * the physical bus pins, causing bit errors without a transceiver.
       */

      can_receive(dev, &msg->cm_hdr, msg->cm_data);
      can_txdone(dev);
      return OK;
    }
#endif

  /* Check TX buffer is ready before writing frame data */

  if (!(t113_can_getreg(priv, T113_CAN_STA_OFFSET) &
        T113_CAN_STA_TBUF_RDY))
    {
      return -EBUSY;
    }

  id  = msg->cm_hdr.ch_id;
  dlc = msg->cm_hdr.ch_dlc;
  if (dlc > 8)
    {
      return -EOPNOTSUPP;
    }

  frame_info = dlc;

  if (msg->cm_hdr.ch_rtr)
    {
      frame_info |= T113_CAN_BUF0_RTR;
    }

#ifdef CONFIG_CAN_EXTID
  if (msg->cm_hdr.ch_extid)
    {
      frame_info |= T113_CAN_BUF0_EFF;
      dreg = 5;

      t113_can_putreg(priv, T113_CAN_BUF_OFFSET(1),
                      (id >> 21) & 0xff);
      t113_can_putreg(priv, T113_CAN_BUF_OFFSET(2),
                      (id >> 13) & 0xff);
      t113_can_putreg(priv, T113_CAN_BUF_OFFSET(3),
                      (id >> 5) & 0xff);
      t113_can_putreg(priv, T113_CAN_BUF_OFFSET(4),
                      (id << 3) & 0xf8);
    }
  else
#endif
    {
      dreg = 3;

      t113_can_putreg(priv, T113_CAN_BUF_OFFSET(1),
                      (id >> 3) & 0xff);
      t113_can_putreg(priv, T113_CAN_BUF_OFFSET(2),
                      (id << 5) & 0xe0);
    }

  /* RTR frames carry no payload - do not write data bytes */

  if (!msg->cm_hdr.ch_rtr)
    {
      for (i = 0; i < dlc; i++)
        {
          t113_can_putreg(priv, T113_CAN_BUF_OFFSET(dreg + i),
                          msg->cm_data[i]);
        }
    }

  /* Write frame info last (the controller latches on the info byte write) */

  t113_can_putreg(priv, T113_CAN_BUF_OFFSET(0), frame_info);

  /* Drop any stale TX_DONE from a prior transmission before enabling
   * the TX IRQ to avoid a spurious can_txdone() for this request.
   */

  t113_can_putreg(priv, T113_CAN_INT_OFFSET, T113_CAN_INT_TX_DONE);
  t113_can_txint(dev, true);

  /* Normal transmission */

  t113_can_putreg(priv, T113_CAN_CMD_OFFSET,
                  T113_CAN_CMD_TRANS_REQ);

  return OK;
}

/****************************************************************************
 * Name: t113_can_txready
 ****************************************************************************/

static bool t113_can_txready(FAR struct can_dev_s *dev)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;

#ifdef CONFIG_CAN_LOOPBACK
  if (priv->loopback)
    {
      return true;
    }
#endif

  return (t113_can_getreg(priv, T113_CAN_STA_OFFSET) &
          T113_CAN_STA_TBUF_RDY) != 0;
}

/****************************************************************************
 * Name: t113_can_txempty
 ****************************************************************************/

static bool t113_can_txempty(FAR struct can_dev_s *dev)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  uint32_t sta;

#ifdef CONFIG_CAN_LOOPBACK
  if (priv->loopback)
    {
      return true;
    }
#endif

  sta = t113_can_getreg(priv, T113_CAN_STA_OFFSET);
  return (sta & T113_CAN_STA_TBUF_RDY) != 0 &&
         (sta & T113_CAN_STA_TRANS_BUSY) == 0;
}

/****************************************************************************
 * Name: t113_can_rx
 ****************************************************************************/

static void t113_can_rx(FAR struct can_dev_s *dev)
{
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  struct can_hdr_s hdr;
  uint8_t data[CAN_MAXDATALEN];
  uint32_t fi;
  uint32_t id;
  int dreg;
  int i;

  memset(&hdr, 0, sizeof(hdr));

  fi = t113_can_getreg(priv, T113_CAN_BUF_OFFSET(0));
  hdr.ch_dlc = fi & T113_CAN_BUF0_DLC_MASK;
  if (hdr.ch_dlc > 8)
    {
      hdr.ch_dlc = 8;
    }

  hdr.ch_rtr = (fi & T113_CAN_BUF0_RTR) ? 1 : 0;

#ifdef CONFIG_CAN_EXTID
  if (fi & T113_CAN_BUF0_EFF)
    {
      hdr.ch_extid = 1;
      dreg = 5;

      id = ((uint32_t)t113_can_getreg(priv, T113_CAN_BUF_OFFSET(1)) << 21) |
           ((uint32_t)t113_can_getreg(priv, T113_CAN_BUF_OFFSET(2)) << 13) |
           ((uint32_t)t113_can_getreg(priv, T113_CAN_BUF_OFFSET(3)) << 5) |
           ((uint32_t)t113_can_getreg(priv, T113_CAN_BUF_OFFSET(4)) >> 3);
      id &= 0x1fffffff;
    }
  else
#endif
    {
#ifdef CONFIG_CAN_EXTID
      hdr.ch_extid = 0;
#endif
      dreg = 3;

      id = ((uint32_t)t113_can_getreg(priv, T113_CAN_BUF_OFFSET(1)) << 3) |
           ((uint32_t)t113_can_getreg(priv, T113_CAN_BUF_OFFSET(2)) >> 5);
      id &= 0x7ff;
    }

  hdr.ch_id = id;

  if (!hdr.ch_rtr)
    {
      for (i = 0; i < hdr.ch_dlc; i++)
        {
          data[i] = (uint8_t)t113_can_getreg(priv,
                                              T113_CAN_BUF_OFFSET(dreg + i));
        }
    }
  else
    {
      memset(data, 0, sizeof(data));
    }

  t113_can_putreg(priv, T113_CAN_CMD_OFFSET, T113_CAN_CMD_RELEASE_BUF);
  can_receive(dev, &hdr, data);
}

/****************************************************************************
 * Name: t113_can_isr
 ****************************************************************************/

static int t113_can_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct can_dev_s *dev = (FAR struct can_dev_s *)arg;
  struct t113_can_priv_s *priv = (struct t113_can_priv_s *)dev->cd_priv;
  uint32_t iflags;
  uint32_t status;

  iflags = t113_can_getreg(priv, T113_CAN_INT_OFFSET);
  status = t113_can_getreg(priv, T113_CAN_STA_OFFSET);

  /* W1C clear interrupt flags + dummy read to flush */

  t113_can_putreg(priv, T113_CAN_INT_OFFSET, iflags);
  t113_can_getreg(priv, T113_CAN_INT_OFFSET);

  if (iflags & T113_CAN_INT_RX_DONE)
    {
      while (t113_can_getreg(priv, T113_CAN_STA_OFFSET) &
             T113_CAN_STA_RBUF_RDY)
        {
          t113_can_rx(dev);
        }
    }

  if (iflags & T113_CAN_INT_TX_DONE)
    {
      t113_can_txint(dev, false);
      can_txdone(dev);
    }

  if (iflags & T113_CAN_INT_DATA_OVERRUN)
    {
      canwarn("CAN%d: data overrun\n", priv->port);
      t113_can_putreg(priv, T113_CAN_CMD_OFFSET,
                      T113_CAN_CMD_CLR_OVERRUN);
    }

  if (iflags & T113_CAN_INT_ERR_WARNING)
    {
      if (status & T113_CAN_STA_BUSOFF)
        {
          canerr("CAN%d: bus-off (ERRC=0x%08lx)\n", priv->port,
                 (unsigned long)t113_can_getreg(priv,
                                                T113_CAN_ERRC_OFFSET));

          /* Reduce interrupts to RX+TX only to prevent error storm.
           * Recovery is handled out-of-band via CANIOC_BUSOFF_RECOVERY
           * so application code controls when to re-join the bus.
           * Do NOT call can_txdone() here - no transmission actually
           * completed.
           */

          t113_can_putreg(priv, T113_CAN_INTEN_OFFSET,
                          T113_CAN_INTEN_RX | T113_CAN_INTEN_TX);
          t113_can_txint(dev, false);
        }
      else if (status & T113_CAN_STA_ERR)
        {
          canwarn("CAN%d: error warning\n", priv->port);
        }
    }

  if (iflags & T113_CAN_INT_ERR_PASSIVE)
    {
      canwarn("CAN%d: error passive (ERRC=0x%08lx)\n", priv->port,
              (unsigned long)t113_can_getreg(priv, T113_CAN_ERRC_OFFSET));
    }

  if (iflags & T113_CAN_INT_BUSERR)
    {
      canwarn("CAN%d: bus error (STA=0x%08lx)\n", priv->port,
              (unsigned long)status);
    }

  return OK;
}

/****************************************************************************
 * Name: t113_can_ccu_enable
 ****************************************************************************/

static void t113_can_ccu_enable(int port)
{
  uint32_t gate_bit;
  uint32_t rst_bit;

  if (port == 0)
    {
      gate_bit = T113_CCU_CAN0_GATING;
      rst_bit  = T113_CCU_CAN0_RST;
    }
  else
    {
      gate_bit = T113_CCU_CAN1_GATING;
      rst_bit  = T113_CCU_CAN1_RST;
    }

  /* Standard CCU enable sequence:
   * 1. Assert reset, gate off (known state after warm boot)
   * 2. Enable gate, reset still asserted
   * 3. De-assert reset
   *
   * Each step is a locked RMW so the sibling CAN port (or any other
   * driver sharing CAN_BGR/CCU) cannot lose updates.
   */

  t113_ccu_modify(T113_CCU_CAN_BGR, gate_bit | rst_bit, 0);
  up_udelay(20);

  /* 2. Enable gate, reset still asserted */

  t113_ccu_modify(T113_CCU_CAN_BGR, 0, gate_bit);
  up_udelay(1);

  /* 3. De-assert reset */

  t113_ccu_modify(T113_CCU_CAN_BGR, 0, rst_bit);
  up_udelay(20);
}

/****************************************************************************
 * Name: t113_can_gpio_config
 ****************************************************************************/

static void t113_can_gpio_config(int port)
{
  if (port == 0)
    {
      t113_gpio_config(T113_CAN0_TX);
      t113_gpio_config(T113_CAN0_RX);
    }
  else
    {
      t113_gpio_config(T113_CAN1_TX);
      t113_gpio_config(T113_CAN1_RX);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_can_initialize
 ****************************************************************************/

int t113_can_initialize(int port)
{
  struct t113_can_priv_s *priv;
  char devpath[16];

  switch (port)
    {
#ifdef CONFIG_T113_CAN0
      case 0:
        priv = &g_can0priv;
        break;
#endif

#ifdef CONFIG_T113_CAN1
      case 1:
        priv = &g_can1priv;
        break;
#endif

      default:
        canerr("ERROR: Invalid CAN port %d\n", port);
        return -EINVAL;
    }

  t113_can_ccu_enable(port);
  t113_can_gpio_config(port);

  priv->dev.cd_ops  = &g_t113_can_ops;
  priv->dev.cd_priv = priv;

  snprintf(devpath, sizeof(devpath), "/dev/can%d", port);

  caninfo("CAN%d: registering %s (base=0x%08lx irq=%d)\n",
          port, devpath, (unsigned long)priv->base, priv->irq);

  return can_register(devpath, &priv->dev);
}

#endif /* CONFIG_T113_CAN0 || CONFIG_T113_CAN1 */
