/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_i2s.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>

#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_i2s.h"
#include "hardware/ls2k0300_lsia_dma.h"

#ifdef CONFIG_LS2K0300_I2S

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Physical address of the I2S controller.  The DMA engine addresses the
 * physical address space, so the data registers must be referenced with
 * the physical (not the uncached window) address.
 */

#define LS2K0300_I2S_PHYS_BASE       0x16114000

/* DMA channels used by the I2S controller */

#define LS2K0300_I2S_RX_CHANNEL      0
#define LS2K0300_I2S_TX_CHANNEL      1

/* Depth of the per direction transfer queue.  The audio upper half keeps
 * up to CONFIG_ES8388_INFLIGHT buffers in flight (default 2) and nxplayer
 * enqueues a final zero length buffer to mark the end of the stream, so
 * a couple of slots more are provided.
 */

#define LS2K0300_I2S_QUEUE_DEPTH     4

/* Default stream format */

#define LS2K0300_I2S_DEFAULT_RATE    48000
#define LS2K0300_I2S_DEFAULT_WIDTH   16

/* Number of master clock periods the controller is given to settle before
 * the transfer is considered failed.
 */

#define LS2K0300_I2S_CLOCK_TIMEOUT   20000

/* Cache maintenance for DMA buffers.  Provided by ls2k0300_dma_cache.S */

extern void ls2k0300_dma_flush(FAR void *addr, size_t len);
extern void ls2k0300_dma_invalidate(FAR void *addr, size_t len);

/* DMA driver interface */

typedef void (*lsia_dma_callback_t)(FAR void *arg, int result);

extern int ls2k0300_lsia_dma_initialize(void);
extern int ls2k0300_lsia_dma_start(int chn, uint32_t periph, uint32_t mem,
                                   uint32_t nbytes, bool tomem,
                                   lsia_dma_callback_t cb, FAR void *arg);
extern void ls2k0300_lsia_dma_stop(int chn);

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* One queued transfer */

struct ls2k0300_i2s_xfer_s
{
  FAR struct ap_buffer_s *apb;    /* Audio buffer */
  i2s_callback_t callback;        /* Completion callback */
  FAR void *arg;                  /* Callback argument */
};

/* Per direction transfer queue state */

struct ls2k0300_i2s_dir_s
{
  struct ls2k0300_i2s_xfer_s
    queue[LS2K0300_I2S_QUEUE_DEPTH];
  uint8_t head;                   /* Next slot to fill */
  uint8_t tail;                   /* Next slot to start */
  uint8_t count;                  /* Number of queued transfers */
  bool active;                    /* A transfer is in flight */
};

struct ls2k0300_i2s_s
{
  struct i2s_dev_s dev;          /* Externally visible I2S interface */
  mutex_t lock;                  /* Protects the configuration below */

  uint32_t samprate;             /* Configured sample rate */
  uint32_t mclk;                 /* Configured master clock frequency */
  uint8_t width;                 /* Bits per sample */
  uint8_t channels;              /* Number of channels (always stereo) */

  bool configdirty;              /* The configuration registers need an update */

  /* Transfer queues.  Protected by interrupt state: the DMA completion
   * callback runs in interrupt context.
   */

  struct ls2k0300_i2s_dir_s tx;
  struct ls2k0300_i2s_dir_s rx;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void ls2k0300_i2s_configure(FAR struct ls2k0300_i2s_s *priv);
static void ls2k0300_i2s_applyconfig(FAR struct ls2k0300_i2s_s *priv);
static void ls2k0300_i2s_txdone(FAR void *arg, int result);
static void ls2k0300_i2s_rxdone(FAR void *arg, int result);

static int ls2k0300_i2s_txchannels(FAR struct i2s_dev_s *dev,
                                   uint8_t channels);
static uint32_t ls2k0300_i2s_txsamplerate(FAR struct i2s_dev_s *dev,
                                          uint32_t rate);
static uint32_t ls2k0300_i2s_txdatawidth(FAR struct i2s_dev_s *dev, int bits);
static int ls2k0300_i2s_send(FAR struct i2s_dev_s *dev,
                             FAR struct ap_buffer_s *apb,
                             i2s_callback_t callback, FAR void *arg,
                             uint32_t timeout);
static int ls2k0300_i2s_rxchannels(FAR struct i2s_dev_s *dev,
                                   uint8_t channels);
static uint32_t ls2k0300_i2s_rxsamplerate(FAR struct i2s_dev_s *dev,
                                          uint32_t rate);
static uint32_t ls2k0300_i2s_rxdatawidth(FAR struct i2s_dev_s *dev, int bits);
static int ls2k0300_i2s_receive(FAR struct i2s_dev_s *dev,
                                FAR struct ap_buffer_s *apb,
                                i2s_callback_t callback, FAR void *arg,
                                uint32_t timeout);
static uint32_t ls2k0300_i2s_getmclkfrequency(FAR struct i2s_dev_s *dev);
static uint32_t ls2k0300_i2s_setmclkfrequency(FAR struct i2s_dev_s *dev,
                                               uint32_t frequency);
static int ls2k0300_i2s_ioctl(FAR struct i2s_dev_s *dev, int cmd,
                              unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_i2sops =
{
  .i2s_txchannels      = ls2k0300_i2s_txchannels,
  .i2s_txsamplerate    = ls2k0300_i2s_txsamplerate,
  .i2s_txdatawidth     = ls2k0300_i2s_txdatawidth,
  .i2s_send            = ls2k0300_i2s_send,
  .i2s_rxchannels      = ls2k0300_i2s_rxchannels,
  .i2s_rxsamplerate    = ls2k0300_i2s_rxsamplerate,
  .i2s_rxdatawidth     = ls2k0300_i2s_rxdatawidth,
  .i2s_receive         = ls2k0300_i2s_receive,
  .i2s_getmclkfrequency = ls2k0300_i2s_getmclkfrequency,
  .i2s_setmclkfrequency = ls2k0300_i2s_setmclkfrequency,
  .i2s_ioctl           = ls2k0300_i2s_ioctl,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2s_read / i2s_write
 *
 * Description:
 *   Access the I2S registers through the uncached window.
 *
 ****************************************************************************/

static inline uint32_t i2s_read(uint32_t offset)
{
  return getreg32(LS2K0300_I2S_BASE + offset);
}

static inline void i2s_write(uint32_t offset, uint32_t value)
{
  putreg32(value, LS2K0300_I2S_BASE + offset);
}

/****************************************************************************
 * Name: ls2k0300_i2s_enable_master
 *
 * Description:
 *   Enable the master clock output and select master mode.  The I2S
 *   controller then provides MCLK, BCLK and LRCLK to the codec.
 *
 ****************************************************************************/

static void ls2k0300_i2s_enable_master(void)
{
  int timeout;

  /* Release the controller soft reset (RESETn is active low, the power
   * on value 0 holds the core in reset) and enable the master clock
   * output.  Then wait until it is stable.
   */

  i2s_write(IIS_CONTROL_OFFSET,
            IIS_CONTROL_RESETN | IIS_CONTROL_MCLK_EN);

  for (timeout = LS2K0300_I2S_CLOCK_TIMEOUT; timeout > 0; timeout--)
    {
      if (i2s_read(IIS_CONTROL_OFFSET) & IIS_CONTROL_MCLK_READY)
        {
          break;
        }

      up_udelay(5);
    }

  if (timeout <= 0)
    {
      aerr("ERROR: MCLK did not become ready, IISControl=%08lx\n",
           (unsigned long)i2s_read(IIS_CONTROL_OFFSET));
    }

  /* Select master mode (MCLK/BCLK/LRCLK are generated by the I2S) */

  i2s_write(IIS_CONTROL_OFFSET,
            IIS_CONTROL_RESETN | IIS_CONTROL_MASTER |
            IIS_CONTROL_MCLK_EN);

  for (timeout = LS2K0300_I2S_CLOCK_TIMEOUT; timeout > 0; timeout--)
    {
      if (i2s_read(IIS_CONTROL_OFFSET) & IIS_CONTROL_CLK_READY)
        {
          break;
        }

      up_udelay(5);
    }

  if (timeout <= 0)
    {
      aerr("ERROR: BCLK/LRCLK did not become ready, IISControl=%08lx\n",
           (unsigned long)i2s_read(IIS_CONTROL_OFFSET));
    }
}

/****************************************************************************
 * Name: ls2k0300_i2s_configure
 *
 * Description:
 *   Program the I2S configuration registers from the current stream
 *   settings.
 *
 *   The controller derives MCLK from its input clock with an integer plus
 *   a 16 bit fractional divider, and derives BCLK from MCLK:
 *
 *     MCLK = refclk / (mclk_ratio + mclk_frac / 65536)
 *     BCLK = MCLK / (2 * (bclk_ratio + 1))
 *
 *   The BCLK ratio is selected so that one 32 bit word carries one
 *   stereo frame (left channel in the lower half, right channel in the
 *   upper half), i.e. BCLK = 2 * width * samplerate.
 *
 ****************************************************************************/

static void ls2k0300_i2s_configure(FAR struct ls2k0300_i2s_s *priv)
{
  uint32_t refclk = CONFIG_LS2K0300_I2S_REFCLK;
  uint32_t mclk = priv->mclk;
  uint32_t fs = priv->samprate;
  uint32_t depth = priv->width;
  uint32_t bclk;
  uint32_t mclk_ratio;
  uint32_t mclk_frac;
  uint32_t bclk_ratio;
  uint32_t val;

  if (mclk == 0)
    {
      /* The codec did not request a specific master clock.  Use the
       * conventional 256 * samplerate.
       */

      mclk = 256 * fs;
    }

  /* MCLK divider: integer part plus 16 bit fraction */

  mclk_ratio = refclk / mclk;
  mclk_frac  = ((uint64_t)(refclk - mclk_ratio * mclk) << 16) / mclk;

  /* BCLK divider: one 32 bit word per stereo frame */

  bclk = depth * 2 * fs;
  bclk_ratio = mclk / (2 * bclk);
  if (bclk_ratio == 0)
    {
      bclk_ratio = 1;
    }

  bclk_ratio--;

  val = (depth << IIS_CONFIG0_LR_LEN_SHIFT) |
        (depth << IIS_CONFIG0_TX_DEPTH_SHIFT) |
        (bclk_ratio << IIS_CONFIG0_BCLK_RATIO_SHIFT) |
        (depth << IIS_CONFIG0_RX_DEPTH_SHIFT);

  i2s_write(IIS_CONFIG0_OFFSET, val);
  i2s_write(IIS_CONFIG1_OFFSET,
            (mclk_frac << IIS_CONFIG1_MCLK_FRAC_SHIFT) |
            (mclk_ratio << IIS_CONFIG1_MCLK_RATIO_SHIFT));

  audinfo("I2S: rate=%u width=%u mclk=%u (ratio=%u frac=%u bclk_ratio=%u)\n",
          fs, depth, mclk, mclk_ratio, mclk_frac, bclk_ratio);
}

/****************************************************************************
 * Name: ls2k0300_i2s_applyconfig
 *
 * Description:
 *   Program the configuration registers if the stream settings changed
 *   since the last update.
 *
 ****************************************************************************/

static void ls2k0300_i2s_applyconfig(FAR struct ls2k0300_i2s_s *priv)
{
  if (priv->configdirty)
    {
      ls2k0300_i2s_configure(priv);
      priv->configdirty = false;
    }
}

/****************************************************************************
 * Name: ls2k0300_i2s_dir_start / ls2k0300_i2s_dir_done
 *
 * Description:
 *   Queue helpers for one transfer direction.  The queue is protected by
 *   the caller (interrupt state).
 *
 ****************************************************************************/

static int ls2k0300_i2s_dir_start(FAR struct ls2k0300_i2s_s *priv,
                                  FAR struct ls2k0300_i2s_dir_s *dir,
                                  int channel, bool tomem)
{
  FAR struct ls2k0300_i2s_xfer_s *xfer = &dir->queue[dir->tail];
  FAR uint8_t *samp = &xfer->apb->samp[xfer->apb->curbyte];
  size_t nbytes = tomem
    ? (xfer->apb->nmaxbytes - xfer->apb->curbyte)
    : (xfer->apb->nbytes - xfer->apb->curbyte);
  uint32_t ctl;
  int ret;

  /* A queued transfer is never zero length: zero length buffers are
   * completed immediately at submit time.
   */

  dir->tail = (dir->tail + 1) % LS2K0300_I2S_QUEUE_DEPTH;
  dir->count--;
  dir->active = true;

  /* Sync the buffer with the DMA engine.  For TX the CPU wrote the
   * samples, so dirty cache lines must be pushed to memory.  For RX the
   * device overwrites memory and the stale CPU-side lines are dropped
   * when the buffer is submitted (see ls2k0300_i2s_receive): doing it
   * here would walk 32KB with interrupts disabled in the completion
   * path, while the 2-frame RX FIFO is already filling up.
   */

  if (!tomem)
    {
      ls2k0300_dma_flush(samp, nbytes);
    }

  /* Enable the data path and its DMA request before starting the
   * transfer.
   */

  ctl = i2s_read(IIS_CONTROL_OFFSET);
  ctl |= IIS_CONTROL_RESETN | IIS_CONTROL_MASTER | IIS_CONTROL_MSB_LSB;
  ctl |= tomem ? (IIS_CONTROL_RX_EN | IIS_CONTROL_RX_DMA_EN)
               : (IIS_CONTROL_TX_EN | IIS_CONTROL_TX_DMA_EN);
  i2s_write(IIS_CONTROL_OFFSET, ctl);

  ret = ls2k0300_lsia_dma_start(channel,
                                LS2K0300_I2S_PHYS_BASE +
                                (tomem ? IIS_RXDATA_OFFSET
                                       : IIS_TXDATA_OFFSET),
                                (uint32_t)(uintptr_t)samp, nbytes, tomem,
                                tomem ? ls2k0300_i2s_rxdone
                                      : ls2k0300_i2s_txdone, priv);
  if (ret < 0)
    {
      dir->active = false;
    }

  return ret;
}

/****************************************************************************
 * Name: ls2k0300_i2s_txdone / ls2k0300_i2s_rxdone
 *
 * Description:
 *   DMA completion callbacks.  These run in the context of the DMA
 *   interrupt handler.  The ES8388 upper half accepts completion
 *   callbacks from interrupt context.
 *
 ****************************************************************************/

static void ls2k0300_i2s_txdone(FAR void *arg, int result)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)arg;
  FAR struct ls2k0300_i2s_dir_s *dir = &priv->tx;
  FAR struct ls2k0300_i2s_xfer_s done;
  irqstate_t flags;

  flags = up_irq_save();

  /* Start the next queued transfer first so that the I2S FIFO does not
   * underrun between two buffers.
   */

  dir->active = false;
  done = dir->queue[(dir->tail + LS2K0300_I2S_QUEUE_DEPTH - 1) %
                    LS2K0300_I2S_QUEUE_DEPTH];

  if (dir->count > 0)
    {
      ls2k0300_i2s_dir_start(priv, dir, LS2K0300_I2S_TX_CHANNEL, false);
    }

  up_irq_restore(flags);

  if (done.callback != NULL)
    {
      done.callback(&priv->dev, done.apb, done.arg, result);
    }
}

static void ls2k0300_i2s_rxdone(FAR void *arg, int result)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)arg;
  FAR struct ls2k0300_i2s_dir_s *dir = &priv->rx;
  FAR struct ls2k0300_i2s_xfer_s done;
  irqstate_t flags;

  flags = up_irq_save();

  dir->active = false;
  done = dir->queue[(dir->tail + LS2K0300_I2S_QUEUE_DEPTH - 1) %
                    LS2K0300_I2S_QUEUE_DEPTH];

  /* Arm the next transfer before doing anything else.  The RX FIFO holds
   * only two 32-bit frames, so at 48kHz the controller has about 42us of
   * slack between the completed buffer and the next DMA start; the cache
   * maintenance below used to eat most of it.
   */

  if (dir->count > 0)
    {
      ls2k0300_i2s_dir_start(priv, dir, LS2K0300_I2S_RX_CHANNEL, true);
    }

  up_irq_restore(flags);

  if (result == OK)
    {
      done.apb->nbytes = done.apb->nmaxbytes;

      /* The DMA wrote samples behind the CPU's back.  Drop the stale
       * cache lines WITHOUT writing them back: a writeback here would
       * push speculative CPU-side data over the fresh samples, which is
       * exactly what garbled earlier recordings with noise bursts.
       */

      ls2k0300_dma_invalidate(&done.apb->samp[done.apb->curbyte],
                              done.apb->nbytes);
    }

  if (done.callback != NULL)
    {
      done.callback(&priv->dev, done.apb, done.arg, result);
    }
}

/****************************************************************************
 * Name: ls2k0300_i2s_txchannels
 *
 * Description:
 *   The I2S datapath carries a packed stereo frame in every 32 bit word,
 *   so only two channels are supported.
 *
 ****************************************************************************/

static int ls2k0300_i2s_txchannels(FAR struct i2s_dev_s *dev,
                                    uint8_t channels)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;

  if (channels != 2)
    {
      return -EINVAL;
    }

  priv->channels = channels;
  return OK;
}

/****************************************************************************
 * Name: ls2k0300_i2s_txsamplerate
 *
 ****************************************************************************/

static uint32_t ls2k0300_i2s_txsamplerate(FAR struct i2s_dev_s *dev,
                                           uint32_t rate)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;
  nxmutex_lock(&priv->lock);
  priv->samprate = rate;
  priv->configdirty = true;
  nxmutex_unlock(&priv->lock);
  return rate;
}

/****************************************************************************
 * Name: ls2k0300_i2s_txdatawidth
 *
 ****************************************************************************/

static uint32_t ls2k0300_i2s_txdatawidth(FAR struct i2s_dev_s *dev, int bits)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;

  if (bits != 16 && bits != 24 && bits != 32)
    {
      return 0;
    }

  nxmutex_lock(&priv->lock);
  priv->width = (uint8_t)bits;
  priv->configdirty = true;
  nxmutex_unlock(&priv->lock);
  return bits;
}

/****************************************************************************
 * Name: ls2k0300_i2s_send
 *
 * Description:
 *   Send one audio buffer through the I2S transmitter.
 *
 ****************************************************************************/

static int ls2k0300_i2s_send(FAR struct i2s_dev_s *dev,
                             FAR struct ap_buffer_s *apb,
                             i2s_callback_t callback, FAR void *arg,
                             uint32_t timeout)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;
  FAR struct ls2k0300_i2s_dir_s *dir = &priv->tx;
  irqstate_t flags;
  size_t nbytes;
  int ret = OK;

  if (apb == NULL || callback == NULL)
    {
      return -EINVAL;
    }

  nbytes = apb->nbytes - apb->curbyte;

  /* A zero length buffer marks the end of the stream.  The audio upper
   * half (nxplayer) enqueues one on close; complete it immediately.
   */

  if (nbytes == 0)
    {
      callback(&priv->dev, apb, arg, OK);
      return OK;
    }

  if ((nbytes & 3) != 0)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);

  /* Make sure the controller is programmed for the current stream
   * configuration.
   */

  ls2k0300_i2s_applyconfig(priv);

  flags = up_irq_save();

  if (dir->count >= LS2K0300_I2S_QUEUE_DEPTH)
    {
      ret = -EBUSY;
    }
  else
    {
      FAR struct ls2k0300_i2s_xfer_s *xfer = &dir->queue[dir->head];

      xfer->apb = apb;
      xfer->callback = callback;
      xfer->arg = arg;
      dir->head = (dir->head + 1) % LS2K0300_I2S_QUEUE_DEPTH;
      dir->count++;

      /* If the transmitter is idle, this buffer starts immediately */

      if (!dir->active)
        {
          ret = ls2k0300_i2s_dir_start(priv, dir,
                                       LS2K0300_I2S_TX_CHANNEL, false);
        }
    }

  up_irq_restore(flags);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: ls2k0300_i2s_rxchannels
 *
 ****************************************************************************/

static int ls2k0300_i2s_rxchannels(FAR struct i2s_dev_s *dev,
                                   uint8_t channels)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;

  if (channels != 2)
    {
      return -EINVAL;
    }

  priv->channels = channels;
  return OK;
}

/****************************************************************************
 * Name: ls2k0300_i2s_rxsamplerate
 *
 ****************************************************************************/

static uint32_t ls2k0300_i2s_rxsamplerate(FAR struct i2s_dev_s *dev,
                                           uint32_t rate)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;
  nxmutex_lock(&priv->lock);
  priv->samprate = rate;
  priv->configdirty = true;
  nxmutex_unlock(&priv->lock);
  return rate;
}

/****************************************************************************
 * Name: ls2k0300_i2s_rxdatawidth
 *
 ****************************************************************************/

static uint32_t ls2k0300_i2s_rxdatawidth(FAR struct i2s_dev_s *dev, int bits)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;

  if (bits != 16 && bits != 24 && bits != 32)
    {
      return 0;
    }

  nxmutex_lock(&priv->lock);
  priv->width = (uint8_t)bits;
  priv->configdirty = true;
  nxmutex_unlock(&priv->lock);
  return bits;
}

/****************************************************************************
 * Name: ls2k0300_i2s_receive
 *
 * Description:
 *   Receive one audio buffer from the I2S receiver.
 *
 ****************************************************************************/

static int ls2k0300_i2s_receive(FAR struct i2s_dev_s *dev,
                                FAR struct ap_buffer_s *apb,
                                i2s_callback_t callback, FAR void *arg,
                                uint32_t timeout)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;
  FAR struct ls2k0300_i2s_dir_s *dir = &priv->rx;
  irqstate_t flags;
  size_t nbytes;
  int ret = OK;

  if (apb == NULL || callback == NULL)
    {
      return -EINVAL;
    }

  nbytes = apb->nmaxbytes - apb->curbyte;

  /* A zero length buffer is a no-op; complete it immediately */

  if (nbytes == 0)
    {
      callback(&priv->dev, apb, arg, OK);
      return OK;
    }

  if ((nbytes & 3) != 0)
    {
      return -EINVAL;
    }

  /* Drop the stale CPU-side cache lines now, in thread context.  The
   * completion ISR arms the next transfer and must not spend time walking
   * 32KB of cache while the 2-frame RX FIFO is draining.
   */

  ls2k0300_dma_invalidate(&apb->samp[apb->curbyte], nbytes);

  nxmutex_lock(&priv->lock);

  ls2k0300_i2s_applyconfig(priv);

  flags = up_irq_save();

  if (dir->count >= LS2K0300_I2S_QUEUE_DEPTH)
    {
      ret = -EBUSY;
    }
  else
    {
      FAR struct ls2k0300_i2s_xfer_s *xfer = &dir->queue[dir->head];

      xfer->apb = apb;
      xfer->callback = callback;
      xfer->arg = arg;
      dir->head = (dir->head + 1) % LS2K0300_I2S_QUEUE_DEPTH;
      dir->count++;

      /* If the receiver is idle, this buffer starts immediately */

      if (!dir->active)
        {
          ret = ls2k0300_i2s_dir_start(priv, dir,
                                       LS2K0300_I2S_RX_CHANNEL, true);
        }
    }

  up_irq_restore(flags);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: ls2k0300_i2s_getmclkfrequency
 *
 ****************************************************************************/

static uint32_t ls2k0300_i2s_getmclkfrequency(FAR struct i2s_dev_s *dev)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;
  return priv->mclk;
}

/****************************************************************************
 * Name: ls2k0300_i2s_setmclkfrequency
 *
 ****************************************************************************/

static uint32_t ls2k0300_i2s_setmclkfrequency(FAR struct i2s_dev_s *dev,
                                               uint32_t frequency)
{
  FAR struct ls2k0300_i2s_s *priv = (FAR struct ls2k0300_i2s_s *)dev;

  if (frequency == 0)
    {
      return 0;
    }

  nxmutex_lock(&priv->lock);
  priv->mclk = frequency;
  priv->configdirty = true;
  nxmutex_unlock(&priv->lock);
  return frequency;
}

/****************************************************************************
 * Name: ls2k0300_i2s_ioctl
 *
 ****************************************************************************/

static int ls2k0300_i2s_ioctl(FAR struct i2s_dev_s *dev, int cmd,
                              unsigned long arg)
{
  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ls2k0300_i2s_initialize
 *
 * Description:
 *   Initialize the I2S controller and return the lower half interface.
 *
 * Returned Value:
 *   A pointer to the I2S lower half interface on success; NULL on
 *   failure.
 *
 ****************************************************************************/

FAR struct i2s_dev_s *ls2k0300_i2s_initialize(void)
{
  FAR struct ls2k0300_i2s_s *priv;
  int ret;

  priv = kmm_zalloc(sizeof(struct ls2k0300_i2s_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->dev.ops = &g_i2sops;
  nxmutex_init(&priv->lock);

  priv->samprate = LS2K0300_I2S_DEFAULT_RATE;
  priv->width = LS2K0300_I2S_DEFAULT_WIDTH;
  priv->channels = 2;
  priv->mclk = 0;

  ret = ls2k0300_lsia_dma_initialize();
  if (ret < 0)
    {
      auderr("ERROR: Failed to initialize the LSIA DMA: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  /* The codec is a slave: the I2S controller provides MCLK, BCLK and
   * LRCLK.
   */

  ls2k0300_i2s_enable_master();
  ls2k0300_i2s_configure(priv);

  return &priv->dev;
}

#endif /* CONFIG_LS2K0300_I2S */
