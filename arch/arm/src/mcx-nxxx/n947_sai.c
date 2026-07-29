/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_sai.c
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
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/audio/i2s.h>
#include <nuttx/cache.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/queue.h>
#include <nuttx/spinlock.h>
#include <nuttx/wqueue.h>

#include <arch/irq.h>

#include "arm_internal.h"
#include "hardware/nxxx_clock.h"
#ifdef CONFIG_N947_SAI_TXDMA
#  include "hardware/n947/n947_dmamux.h"
#  include "n947_edma.h"
#endif
#include "hardware/n947/n947_sai.h"
#include "nxxx_clockconfig.h"
#include "n947_sai.h"

#ifdef CONFIG_N947_SAI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error "MCX-Nxxx SAI requires CONFIG_SCHED_HPWORK"
#endif

#ifndef CONFIG_N947_SAI_MAXINFLIGHT
#  define CONFIG_N947_SAI_MAXINFLIGHT 4
#endif

#define N947_SAI_SOURCE_CLOCK          150000000u
#define N947_SAI_DEFAULT_RATE          16000u
#define N947_SAI_DEFAULT_WIDTH         16
#define N947_SAI_DEFAULT_CHANNELS      1
#define N947_SAI_FIFO_WATERMARK        4
#define N947_SAI_FIFO_SERVICE_WORDS    \
  (N947_SAI_FIFO_DEPTH - N947_SAI_FIFO_WATERMARK)

#ifdef CONFIG_N947_SAI_TXDMA
/* Keep two 2 kB DMA blocks in one 4 kB circular transfer.  eDMA advances
 * from one half to the other in hardware; half/major interrupts only refill
 * the half that has already reached the FIFO.  This is the compact
 * equivalent of the MCUXpresso SAI driver's scatter-gather queue and avoids
 * disabling/reprogramming the DMA channel at every audio block boundary.
 */

#  define N947_SAI_TXDMA_WORDS          1024
#  define N947_SAI_TXDMA_BLOCKS         2
#  define N947_SAI_TXDMA_BLOCK_WORDS    \
  (N947_SAI_TXDMA_WORDS / N947_SAI_TXDMA_BLOCKS)
#  define N947_SAI_TXDMA_ALIGN          32
#  define N947_SAI_TXDMA_MINOR_WORDS    N947_SAI_FIFO_SERVICE_WORDS
#endif

#define N947_SAI_REG(priv, offset)     ((priv)->base + (offset))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct n947_sai_s;

struct n947_sai_buffer_s
{
  sq_entry_t entry;
  FAR struct ap_buffer_s *apb;
  i2s_callback_t callback;
  FAR void *arg;
  int result;
  bool inuse;
};

struct n947_sai_transport_s
{
  FAR struct n947_sai_s *priv;
  sq_queue_t pending;
  sq_queue_t done;
  FAR struct n947_sai_buffer_s *active;
  struct work_s work;
  bool work_scheduled;
  bool receive;
  struct n947_sai_buffer_s containers[CONFIG_N947_SAI_MAXINFLIGHT];
};

struct n947_sai_s
{
  struct i2s_dev_s dev;
  uintptr_t base;
  int irq;

  struct n947_sai_transport_s tx;
  struct n947_sai_transport_s rx;

  uint32_t requested_rate;
  uint32_t actual_rate;
  uint32_t bit_clock;
  uint32_t tx_underruns;
  uint32_t tx_underruns_busy;
  uint32_t tx_underruns_idle;
  uint32_t tx_boundary_empty;
  uint32_t tx_queue_max;
  uint32_t tx_dma_chunks;
  uint32_t tx_dma_errors;
  uint32_t tx_dma_recoveries;
  uint32_t rx_overruns;
  uint32_t tx_buffers;
  uint32_t rx_buffers;

  uint32_t tx_mono_sample;
  uint8_t tx_width;
  uint8_t rx_width;
  uint8_t tx_channels;
  uint8_t rx_channels;
  uint8_t tx_slot;
  uint8_t rx_slot;

#ifdef CONFIG_N947_SAI_TXDMA
  DMACH_HANDLE txdma;
  FAR uint32_t *txdma_buffer;
  FAR struct n947_edma_tcd_s *txdma_tcd;
  FAR struct n947_sai_buffer_s *txdma_completion[N947_SAI_TXDMA_BLOCKS];
  size_t txdma_words[N947_SAI_TXDMA_BLOCKS];
  bool txdma_ready[N947_SAI_TXDMA_BLOCKS];
  uint8_t txdma_index;
  bool txdma_active;
  bool txdma_draining;
  bool txdma_failed;
#endif

  bool tx_started;
  bool rx_started;
  bool tx_paused;
  bool rx_paused;
  bool tx_running;
  bool rx_running;
  bool initialized;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int n947_sai_rxchannels(FAR struct i2s_dev_s *dev, uint8_t channels);
static uint32_t n947_sai_rxsamplerate(FAR struct i2s_dev_s *dev,
                                      uint32_t rate);
static uint32_t n947_sai_rxdatawidth(FAR struct i2s_dev_s *dev, int bits);
static int n947_sai_receive(FAR struct i2s_dev_s *dev,
                            FAR struct ap_buffer_s *apb,
                            i2s_callback_t callback, FAR void *arg,
                            uint32_t timeout);
static int n947_sai_txchannels(FAR struct i2s_dev_s *dev, uint8_t channels);
static uint32_t n947_sai_txsamplerate(FAR struct i2s_dev_s *dev,
                                      uint32_t rate);
static uint32_t n947_sai_txdatawidth(FAR struct i2s_dev_s *dev, int bits);
static int n947_sai_send(FAR struct i2s_dev_s *dev,
                         FAR struct ap_buffer_s *apb,
                         i2s_callback_t callback, FAR void *arg,
                         uint32_t timeout);
static uint32_t n947_sai_getmclkfrequency(FAR struct i2s_dev_s *dev);
static uint32_t n947_sai_setmclkfrequency(FAR struct i2s_dev_s *dev,
                                          uint32_t frequency);
static int n947_sai_ioctl(FAR struct i2s_dev_s *dev, int cmd,
                          unsigned long arg);
#ifdef CONFIG_N947_SAI_TXDMA
static void n947_sai_txdma_callback(DMACH_HANDLE handle, FAR void *arg,
                                    bool done, int result);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct i2s_ops_s g_n947_sai_ops =
{
  .i2s_rxchannels       = n947_sai_rxchannels,
  .i2s_rxsamplerate     = n947_sai_rxsamplerate,
  .i2s_rxdatawidth      = n947_sai_rxdatawidth,
  .i2s_receive          = n947_sai_receive,
  .i2s_txchannels       = n947_sai_txchannels,
  .i2s_txsamplerate     = n947_sai_txsamplerate,
  .i2s_txdatawidth      = n947_sai_txdatawidth,
  .i2s_send             = n947_sai_send,
  .i2s_getmclkfrequency = n947_sai_getmclkfrequency,
  .i2s_setmclkfrequency = n947_sai_setmclkfrequency,
  .i2s_ioctl            = n947_sai_ioctl,
};

#ifdef CONFIG_N947_SAI1
static struct n947_sai_s g_sai1 =
{
  .dev =
    {
      .ops = &g_n947_sai_ops,
    },
  .base           = N947_SAI1_BASE,
  .irq            = NXXX_IRQ_SAI1,
  .requested_rate = N947_SAI_DEFAULT_RATE,
  .tx_width       = N947_SAI_DEFAULT_WIDTH,
  .rx_width       = N947_SAI_DEFAULT_WIDTH,
  .tx_channels    = N947_SAI_DEFAULT_CHANNELS,
  .rx_channels    = N947_SAI_DEFAULT_CHANNELS,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t n947_sai_getreg(FAR struct n947_sai_s *priv,
                                       unsigned int offset)
{
  return getreg32(N947_SAI_REG(priv, offset));
}

static inline void n947_sai_putreg(FAR struct n947_sai_s *priv,
                                   unsigned int offset, uint32_t value)
{
  putreg32(value, N947_SAI_REG(priv, offset));
}

static inline void n947_sai_modifyreg(FAR struct n947_sai_s *priv,
                                      unsigned int offset,
                                      uint32_t clearbits,
                                      uint32_t setbits)
{
  modifyreg32(N947_SAI_REG(priv, offset), clearbits, setbits);
}

/* CSR registers combine ordinary control bits with write-one-to-clear
 * status bits.  A generic read-modify-write can therefore acknowledge a
 * status flag unintentionally.  Keep every status bit zero unless the
 * caller explicitly uses n947_sai_clear_status().
 */

static void n947_sai_modifycsr(FAR struct n947_sai_s *priv,
                               unsigned int offset,
                               uint32_t clearbits,
                               uint32_t setbits)
{
  uint32_t regval = n947_sai_getreg(priv, offset);

  regval &= ~(SAI_CSR_W1C_MASK | clearbits);
  n947_sai_putreg(priv, offset, regval | setbits);
}

/* Clear only the requested write-one-to-clear status flags. */

static void n947_sai_clear_status(FAR struct n947_sai_s *priv,
                                  unsigned int offset, uint32_t flags)
{
  uint32_t regval = n947_sai_getreg(priv, offset);

  regval &= ~SAI_CSR_W1C_MASK;
  n947_sai_putreg(priv, offset, regval | flags);
}

static bool n947_sai_transport_busy(
  FAR struct n947_sai_transport_s *transport)
{
  return transport->active != NULL || !sq_empty(&transport->pending);
}

static FAR struct n947_sai_buffer_s *
n947_sai_buffer_alloc(FAR struct n947_sai_transport_s *transport)
{
  FAR struct n947_sai_buffer_s *container = NULL;
  irqstate_t flags;
  int i;

  flags = enter_critical_section();
  for (i = 0; i < CONFIG_N947_SAI_MAXINFLIGHT; i++)
    {
      if (!transport->containers[i].inuse)
        {
          container = &transport->containers[i];
          container->inuse = true;
          break;
        }
    }

  leave_critical_section(flags);
  return container;
}

static void n947_sai_buffer_free(
  FAR struct n947_sai_buffer_s *container)
{
  irqstate_t flags = enter_critical_section();

  container->apb = NULL;
  container->callback = NULL;
  container->arg = NULL;
  container->inuse = false;
  leave_critical_section(flags);
}

static void n947_sai_worker(FAR void *arg)
{
  FAR struct n947_sai_transport_s *transport = arg;
  FAR struct n947_sai_buffer_s *container;
  irqstate_t flags;

  for (; ; )
    {
      flags = enter_critical_section();
      container = (FAR struct n947_sai_buffer_s *)
                  sq_remfirst(&transport->done);

      if (container == NULL)
        {
          transport->work_scheduled = false;
          leave_critical_section(flags);
          return;
        }

      leave_critical_section(flags);

      container->callback(&transport->priv->dev, container->apb,
                          container->arg, container->result);
      apb_free(container->apb);
      n947_sai_buffer_free(container);
    }
}

/* Caller holds the critical section (or is the SAI ISR). */

static void n947_sai_schedule_worker(
  FAR struct n947_sai_transport_s *transport)
{
  int ret;

  if (!transport->work_scheduled)
    {
      transport->work_scheduled = true;
      ret = work_queue(HPWORK, &transport->work, n947_sai_worker,
                       transport, 0);
      if (ret < 0)
        {
          transport->work_scheduled = false;
          i2serr("ERROR: failed to queue SAI completion work: %d\n", ret);
        }
    }
}

/* Caller holds the critical section (or is the SAI ISR). */

static FAR struct n947_sai_buffer_s *
n947_sai_next_buffer(FAR struct n947_sai_transport_s *transport)
{
  if (transport->active == NULL)
    {
      transport->active = (FAR struct n947_sai_buffer_s *)
                          sq_remfirst(&transport->pending);
    }

  return transport->active;
}

/* Caller holds the critical section (or is the SAI ISR). */

static void n947_sai_complete_buffer(
  FAR struct n947_sai_transport_s *transport, int result)
{
  FAR struct n947_sai_buffer_s *container = transport->active;

  if (container != NULL)
    {
      transport->active = NULL;
      container->result = result;
      sq_addlast(&container->entry, &transport->done);

      if (transport->receive)
        {
          transport->priv->rx_buffers++;
        }
      else
        {
          transport->priv->tx_buffers++;
        }

      n947_sai_schedule_worker(transport);
    }
}

#ifdef CONFIG_N947_SAI_TXDMA
/* Complete an APB that was detached after all of its bytes had been copied
 * into a DMA staging block.  The callback is delayed until that particular
 * block has actually reached the SAI FIFO, preserving normal I2S completion
 * semantics while allowing the other staging block to copy the next APB.
 *
 * Caller holds the critical section (or is an eDMA ISR).
 */

static void n947_sai_complete_detached(
  FAR struct n947_sai_transport_s *transport,
  FAR struct n947_sai_buffer_s *container, int result)
{
  if (container == NULL)
    {
      return;
    }

  container->result = result;
  sq_addlast(&container->entry, &transport->done);
  transport->priv->tx_buffers++;
  n947_sai_schedule_worker(transport);
}
#endif

static unsigned int n947_sai_sample_bytes(uint8_t width)
{
  return width == 16 ? 2 : 4;
}

/* Pull one application sample from the active TX buffer.  Application PCM
 * is little-endian signed 16- or 32-bit.  The SAI wire always uses 32-bit
 * slots, so 16-bit samples are left-aligned for MAX98357.
 *
 * Caller holds the critical section (or is the SAI ISR).
 */

static bool n947_sai_tx_sample(FAR struct n947_sai_s *priv,
                               FAR uint32_t *sample)
{
  FAR struct n947_sai_buffer_s *container;
  FAR struct ap_buffer_s *apb;

  container = n947_sai_next_buffer(&priv->tx);
  if (container == NULL)
    {
      *sample = 0;
      return false;
    }

  apb = container->apb;
  if (priv->tx_width == 16)
    {
      int16_t value;

      memcpy(&value, &apb->samp[apb->curbyte], sizeof(value));
      apb->curbyte += sizeof(value);
      *sample = (uint32_t)(uint16_t)value << 16;
    }
  else
    {
      memcpy(sample, &apb->samp[apb->curbyte], sizeof(*sample));
      apb->curbyte += sizeof(*sample);
    }

  if (apb->curbyte >= apb->nbytes)
    {
      if (sq_empty(&priv->tx.pending))
        {
          priv->tx_boundary_empty++;
        }

      n947_sai_complete_buffer(&priv->tx, OK);
    }

  return true;
}

/* Generate the next physical I2S slot.  Mono playback is copied into both
 * slots because MAX98357 modules may select either left or right internally.
 */

static uint32_t n947_sai_tx_word(FAR struct n947_sai_s *priv)
{
  uint32_t sample = 0;

  if (priv->tx_paused)
    {
      priv->tx_mono_sample = 0;
    }
  else if (priv->tx_channels == 1)
    {
      if (priv->tx_slot == 0)
        {
          n947_sai_tx_sample(priv, &priv->tx_mono_sample);
        }

      sample = priv->tx_mono_sample;
    }
  else
    {
      n947_sai_tx_sample(priv, &sample);
    }

  priv->tx_slot ^= 1;
  return sample;
}

#ifdef CONFIG_N947_SAI_TXDMA
/* Convert one bounded portion of the active application APB into the exact
 * 32-bit words consumed by the SAI FIFO.  A mono sample is copied to both
 * left and right slots so MAX98357 modules strapped for either channel work
 * identically.  APB completion is deliberately deferred until the DMA major
 * loop has reached the peripheral FIFO.
 *
 * Caller holds the critical section (or is an eDMA ISR).
 */

static size_t n947_sai_txdma_fill(FAR struct n947_sai_s *priv,
                                  unsigned int index)
{
  FAR struct n947_sai_buffer_s *container;
  FAR struct ap_buffer_s *apb;
  FAR uint32_t *buffer;
  size_t words = 0;
  unsigned int slots_per_sample;

  DEBUGASSERT(index < N947_SAI_TXDMA_BLOCKS);
  DEBUGASSERT(priv->txdma_completion[index] == NULL);

  buffer = priv->txdma_buffer +
           index * N947_SAI_TXDMA_BLOCK_WORDS;
  container = n947_sai_next_buffer(&priv->tx);
  if (container == NULL)
    {
      /* While the playback node is started, keep both cyclic TCDs valid
       * with real digital silence.  A later APB is picked up at the next
       * half boundary without stopping/restarting SAI or eDMA.  This both
       * masks short cloud jitter and removes the FIFO race caused by dozens
       * of data/silence transitions during a long spoken response.
       */

      memset(buffer, 0,
             N947_SAI_TXDMA_BLOCK_WORDS * sizeof(uint32_t));
      if (priv->tx_started && !priv->tx_paused)
        {
          priv->txdma_words[index] = N947_SAI_TXDMA_BLOCK_WORDS;
          priv->txdma_ready[index] = true;
        }
      else
        {
          priv->txdma_words[index] = 0;
          priv->txdma_ready[index] = false;
        }

      up_clean_dcache((uintptr_t)buffer,
                      (uintptr_t)buffer +
                      N947_SAI_TXDMA_BLOCK_WORDS * sizeof(uint32_t));
      return priv->txdma_ready[index] ?
             N947_SAI_TXDMA_BLOCK_WORDS : 0;
    }

  apb = container->apb;
  slots_per_sample = priv->tx_channels == 1 ? 2 : 1;

  while (words + slots_per_sample <= N947_SAI_TXDMA_BLOCK_WORDS &&
         apb->curbyte < apb->nbytes)
    {
      uint32_t sample;

      if (priv->tx_width == 16)
        {
          int16_t value;

          memcpy(&value, &apb->samp[apb->curbyte], sizeof(value));
          apb->curbyte += sizeof(value);
          sample = (uint32_t)(uint16_t)value << 16;
        }
      else
        {
          memcpy(&sample, &apb->samp[apb->curbyte], sizeof(sample));
          apb->curbyte += sizeof(sample);
        }

      buffer[words++] = sample;
      if (slots_per_sample == 2)
        {
          buffer[words++] = sample;
        }
    }

  if (apb->curbyte >= apb->nbytes)
    {
      if (sq_empty(&priv->tx.pending))
        {
          priv->tx_boundary_empty++;
        }

      /* Detach now so the other staging block can begin copying the next
       * queued APB.  Complete it only when this block's DMA callback runs.
       */

      priv->tx.active = NULL;
      priv->txdma_completion[index] = container;
    }

  /* The circular TCD always consumes a complete half.  Pad a short final
   * APB with digital silence; the APB callback is still delayed until this
   * half has actually reached the peripheral FIFO.
   */

  if (words > 0)
    {
      size_t valid_words = words;

      while (words < N947_SAI_TXDMA_BLOCK_WORDS)
        {
          buffer[words++] = 0;
        }

      priv->txdma_words[index] = valid_words;
      priv->txdma_ready[index] = true;
      up_clean_dcache((uintptr_t)buffer,
                      (uintptr_t)buffer +
                      N947_SAI_TXDMA_BLOCK_WORDS * sizeof(uint32_t));
    }

  return words;
}

/* Start one continuous two-TCD transfer.  Each descriptor links to the
 * other, so eDMA loads the alternate half in hardware before raising the
 * major-loop interrupt.  The channel is stopped only when the next half
 * contains no application data.
 *
 * Caller holds the critical section.
 */

static int n947_sai_txdma_start(FAR struct n947_sai_s *priv)
{
  struct n947_edma_xfrconfig_s config[N947_SAI_TXDMA_BLOCKS];
  unsigned int index;
  int ret;

  if (!priv->txdma_ready[0] || priv->txdma_words[0] == 0)
    {
      return 0;
    }

  memset(config, 0, sizeof(config));
  for (index = 0; index < N947_SAI_TXDMA_BLOCKS; index++)
    {
      config[index].saddr =
        (uintptr_t)(priv->txdma_buffer +
                    index * N947_SAI_TXDMA_BLOCK_WORDS);
      config[index].daddr =
        N947_SAI_REG(priv, N947_SAI_TDR_OFFSET(0));
      config[index].soff   = sizeof(uint32_t);
      config[index].doff   = 0;
      config[index].iter   = N947_SAI_TXDMA_BLOCK_WORDS /
                             N947_SAI_TXDMA_MINOR_WORDS;
      config[index].flags  = EDMA_CONFIG_LINKTYPE_LINKNONE;
      config[index].ssize  = EDMA_32BIT;
      config[index].dsize  = EDMA_32BIT;
      config[index].nbytes =
        N947_SAI_TXDMA_MINOR_WORDS * sizeof(uint32_t);
    }

  ret = n947_dmach_sgsetup(priv->txdma, config,
                            N947_SAI_TXDMA_BLOCKS,
                            priv->txdma_tcd);
  if (ret >= 0)
    {
      priv->txdma_index = 0;
      priv->txdma_active = true;
      ret = n947_dmach_start(priv->txdma, n947_sai_txdma_callback, priv);
      if (ret >= 0)
        {
          return 1;
        }

      priv->txdma_active = false;
      n947_dmach_stop(priv->txdma);
    }

  return ret;
}

/* Prime both halves and start the circular transfer. */

static int n947_sai_txdma_kick(FAR struct n947_sai_s *priv)
{
  int ret;

  if (priv->txdma == NULL || priv->txdma_buffer == NULL ||
      priv->txdma_active || priv->txdma_failed || priv->tx_paused)
    {
      return 0;
    }

  if (!priv->txdma_ready[0])
    {
      n947_sai_txdma_fill(priv, 0);
    }

  if (!priv->txdma_ready[0])
    {
      return 0;
    }

  if (!priv->txdma_ready[1])
    {
      n947_sai_txdma_fill(priv, 1);
    }

  ret = n947_sai_txdma_start(priv);
  return ret;
}

static void n947_sai_txdma_abort(FAR struct n947_sai_s *priv)
{
  unsigned int index;

  if (priv->txdma_active)
    {
      n947_dmach_stop(priv->txdma);
    }

  priv->txdma_active = false;
  priv->txdma_draining = false;
  for (index = 0; index < N947_SAI_TXDMA_BLOCKS; index++)
    {
      priv->txdma_ready[index] = false;
      priv->txdma_words[index] = 0;
      if (priv->txdma_completion[index] != NULL)
        {
          n947_sai_complete_detached(&priv->tx,
                                     priv->txdma_completion[index],
                                     -ECANCELED);
          priv->txdma_completion[index] = NULL;
        }
    }

  n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET,
                     SAI_CSR_FRDE | SAI_CSR_FWIE, 0);
}

/* eDMA calls this at the half and major boundaries.  Hardware has already
 * begun consuming the alternate half before this ISR runs, so callback
 * latency cannot starve the SAI FIFO.  Refill only the completed half.
 */

static void n947_sai_txdma_callback(DMACH_HANDLE handle, FAR void *arg,
                                    bool done, int result)
{
  FAR struct n947_sai_s *priv = arg;
  irqstate_t flags;
  unsigned int completed;
  unsigned int next;

  (void)done;
  flags = enter_critical_section();

  if (handle != priv->txdma || !priv->txdma_active)
    {
      leave_critical_section(flags);
      return;
    }

  completed = priv->txdma_index;
  next = completed ^ 1;
  if (result < 0)
    {
      priv->txdma_active = false;
      priv->txdma_ready[completed] = false;
      priv->txdma_words[completed] = 0;
      priv->tx_dma_errors++;
      priv->txdma_failed = true;
      if (priv->txdma_completion[completed] != NULL)
        {
          n947_sai_complete_detached(&priv->tx,
                                     priv->txdma_completion[completed],
                                     result);
          priv->txdma_completion[completed] = NULL;
        }

      if (priv->tx.active != NULL)
        {
          n947_sai_complete_buffer(&priv->tx, result);
        }
    }
  else
    {
      priv->tx_dma_chunks++;
      priv->txdma_index = next;
      priv->txdma_ready[completed] = false;
      priv->txdma_words[completed] = 0;
      if (priv->txdma_completion[completed] != NULL)
        {
          n947_sai_complete_detached(&priv->tx,
                                     priv->txdma_completion[completed],
                                     OK);
          priv->txdma_completion[completed] = NULL;
        }

      /* The alternate block is already active in hardware.  If it contains
       * no application data, stop immediately; its memory is pre-zeroed so
       * any words accepted before ERQ clears are harmless silence.
       */

      if (priv->tx_paused || !priv->txdma_ready[next])
        {
          n947_dmach_stop(priv->txdma);
          priv->txdma_active = false;
        }
      else
        {
          /* Refill has one complete alternate-half duration to finish. */

          n947_sai_txdma_fill(priv, completed);
        }
    }

  if (!priv->txdma_active)
    {
      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET,
                         SAI_CSR_FRDE | SAI_CSR_FWIE | SAI_CSR_FRIE, 0);

      if (priv->tx_started || priv->rx_running || priv->txdma_failed)
        {
          /* A started stream or synchronous microphone capture still needs
           * clocks.  PIO supplies silence (or any later pending buffer).
           */

          n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0,
                             SAI_CSR_FRIE);
        }
      else
        {
          /* Do not stop TE as soon as the last DMA request completes: up to
           * one FIFO of valid audio remains.  The FIFO-warning interrupt
           * performs the final stop after those words reach the wire.
           */

          priv->txdma_draining = true;
          n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0,
                             SAI_CSR_FWIE);
        }
    }

  leave_critical_section(flags);
}
#endif /* CONFIG_N947_SAI_TXDMA */

/* Store one physical RX slot into the application buffer.  INMP441 with
 * L/R tied low drives the left slot.  In mono mode the right slot is
 * deliberately drained but discarded.  A 16-bit capture keeps the most
 * significant 16 bits of the microphone's left-aligned 24-bit sample.
 *
 * Caller holds the critical section (or is the SAI ISR).
 */

static void n947_sai_rx_word(FAR struct n947_sai_s *priv, uint32_t word)
{
  FAR struct n947_sai_buffer_s *container;
  FAR struct ap_buffer_s *apb;
  bool keep;

  keep = priv->rx_channels == 2 || priv->rx_slot == 0;
  priv->rx_slot ^= 1;

  if (!keep || priv->rx_paused)
    {
      return;
    }

  container = n947_sai_next_buffer(&priv->rx);
  if (container == NULL)
    {
      return;
    }

  apb = container->apb;
  if (priv->rx_width == 16)
    {
      int16_t value = (int16_t)((int32_t)word >> 16);

      memcpy(&apb->samp[apb->curbyte], &value, sizeof(value));
      apb->curbyte += sizeof(value);
    }
  else
    {
      memcpy(&apb->samp[apb->curbyte], &word, sizeof(word));
      apb->curbyte += sizeof(word);
    }

  apb->nbytes = apb->curbyte;
  if (apb->curbyte >= apb->nmaxbytes)
    {
      n947_sai_complete_buffer(&priv->rx, OK);
    }
}

/* Choose the closest even source-clock divider.  The hardware equation is
 *
 *   BCLK = source / (2 * (DIV + 1))
 *
 * and the physical I2S frame is always two 32-bit slots.  PLL0 at 150 MHz
 * yields about 16.053 kHz for a 16 kHz request (roughly +0.33 percent).
 */

static uint32_t n947_sai_set_rate_locked(FAR struct n947_sai_s *priv,
                                         uint32_t rate)
{
  uint64_t target_bclk;
  uint64_t err_low;
  uint64_t err_high;
  uint32_t ratio;
  uint32_t low;
  uint32_t high;

  if (rate < 8000 || rate > 96000)
    {
      return 0;
    }

  target_bclk = (uint64_t)rate * N947_SAI_PHYSICAL_SLOTS *
                N947_SAI_SLOT_WIDTH;
  ratio = (uint32_t)(((uint64_t)N947_SAI_SOURCE_CLOCK +
                      target_bclk / 2) / target_bclk);
  low = ratio & ~1u;
  high = low + 2;

  if (low < 2)
    {
      low = 2;
    }

  if (high > 512)
    {
      high = 512;
    }

  err_low = N947_SAI_SOURCE_CLOCK > target_bclk * low ?
            N947_SAI_SOURCE_CLOCK - target_bclk * low :
            target_bclk * low - N947_SAI_SOURCE_CLOCK;
  err_high = N947_SAI_SOURCE_CLOCK > target_bclk * high ?
             N947_SAI_SOURCE_CLOCK - target_bclk * high :
             target_bclk * high - N947_SAI_SOURCE_CLOCK;
  ratio = err_low <= err_high ? low : high;

  priv->requested_rate = rate;
  priv->bit_clock = N947_SAI_SOURCE_CLOCK / ratio;
  priv->actual_rate = priv->bit_clock /
                      (N947_SAI_PHYSICAL_SLOTS * N947_SAI_SLOT_WIDTH);

  n947_sai_modifyreg(priv, N947_SAI_TCR2_OFFSET, SAI_CR2_DIV_MASK,
                     SAI_CR2_DIV(ratio / 2 - 1));
  return priv->actual_rate;
}

static void n947_sai_configure_hardware(FAR struct n947_sai_s *priv)
{
  uint32_t cr4;
  uint32_t cr5;

  /* Reset both halves.  SR resets the internal state and FR resets FIFO
   * pointers; clearing SR afterwards is required by the reference manual.
   */

  n947_sai_putreg(priv, N947_SAI_TCSR_OFFSET, SAI_CSR_SR | SAI_CSR_FR);
  n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, SAI_CSR_SR, 0);
  n947_sai_putreg(priv, N947_SAI_RCSR_OFFSET, SAI_CSR_SR | SAI_CSR_FR);
  n947_sai_modifycsr(priv, N947_SAI_RCSR_OFFSET, SAI_CSR_SR, 0);

  /* Classic I2S: MSB first, frame sync asserted one bit before data,
   * active-low left slot, two 32-bit words.  TX is the controller; RX is
   * synchronous with TX so one BCLK/WS pair serves speaker and microphone.
   */

  cr4 = SAI_CR4_FSD | SAI_CR4_FSP | SAI_CR4_FSE | SAI_CR4_MF |
        SAI_CR4_SYWD(31) | SAI_CR4_FRSZ(1) | SAI_CR4_FCONT;
  cr5 = SAI_CR5_FBT(31) | SAI_CR5_W0W(31) | SAI_CR5_WNW(31);

  n947_sai_putreg(priv, N947_SAI_TCR1_OFFSET,
                  SAI_CR1_FW(N947_SAI_FIFO_WATERMARK));
  /* MCXN947 routes the SAI peripheral clock through SYSCON, but the bit
   * clock generator selects that clock as its bus-clock input (MSEL=0).
   * MSEL=1 selects the separate MCLK option; without an MCLK divider/output
   * configured, the FIFO never advances.
   */

  n947_sai_putreg(priv, N947_SAI_TCR2_OFFSET,
                  SAI_CR2_BCD | SAI_CR2_BCP | SAI_CR2_MSEL(0));
  n947_sai_putreg(priv, N947_SAI_TCR3_OFFSET, SAI_TCR3_TCE(1));
  n947_sai_putreg(priv, N947_SAI_TCR4_OFFSET, cr4);
  n947_sai_putreg(priv, N947_SAI_TCR5_OFFSET, cr5);
  n947_sai_putreg(priv, N947_SAI_TMR_OFFSET, 0);

  n947_sai_putreg(priv, N947_SAI_RCR1_OFFSET,
                  SAI_CR1_FW(N947_SAI_FIFO_WATERMARK));
  n947_sai_putreg(priv, N947_SAI_RCR2_OFFSET,
                  SAI_CR2_BCP | SAI_CR2_MSEL(0) | SAI_CR2_SYNC(1));
  n947_sai_putreg(priv, N947_SAI_RCR3_OFFSET, SAI_RCR3_RCE(1));
  n947_sai_putreg(priv, N947_SAI_RCR4_OFFSET, cr4);
  n947_sai_putreg(priv, N947_SAI_RCR5_OFFSET, cr5);
  n947_sai_putreg(priv, N947_SAI_RMR_OFFSET, 0);
  n947_sai_putreg(priv, N947_SAI_MCR_OFFSET, 0);

  n947_sai_set_rate_locked(priv, priv->requested_rate);
}

/* Update hardware run state after a queue or AUDIOIOC state transition.
 * Caller holds the critical section (or is the SAI ISR).
 */

static void n947_sai_update_running(FAR struct n947_sai_s *priv)
{
  bool need_rx;
  bool need_tx_data;
  bool need_tx_clock;
#ifdef CONFIG_N947_SAI_TXDMA
  int dma_ret;
#endif
  unsigned int timeout;
  int i;

  need_rx = !priv->rx_paused &&
            (priv->rx_started || n947_sai_transport_busy(&priv->rx));
  need_tx_data = !priv->tx_paused &&
                 (priv->tx_started || n947_sai_transport_busy(&priv->tx));
  need_tx_clock = need_tx_data || need_rx;

  /* Stop RX before its synchronous TX clock. */

  if (!need_rx && priv->rx_running)
    {
      n947_sai_modifycsr(priv, N947_SAI_RCSR_OFFSET,
                         SAI_CSR_FRIE | SAI_CSR_FEIE | SAI_RCSR_RE, 0);

      /* RE/TE remain set in the readback until the current frame ends.
       * Keep BCE running while that happens; stopping BCLK first leaves the
       * state machine stranded in a half-frame.  The final write repeats the
       * enable-bit clear so a pathological timeout cannot re-enable it.
       */

      for (timeout = 100000;
           timeout > 0 &&
           (n947_sai_getreg(priv, N947_SAI_RCSR_OFFSET) &
            SAI_RCSR_RE) != 0;
           timeout--)
        {
        }

      n947_sai_modifycsr(priv, N947_SAI_RCSR_OFFSET,
                         SAI_CSR_BCE | SAI_RCSR_RE, SAI_CSR_FR);
      priv->rx_slot = 0;
      priv->rx_running = false;
    }

  if (!need_tx_clock && priv->tx_running
#ifdef CONFIG_N947_SAI_TXDMA
      && !priv->txdma_active && !priv->txdma_draining
#endif
     )
    {
      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET,
                         SAI_CSR_FRDE | SAI_CSR_FWIE | SAI_CSR_FRIE |
                         SAI_CSR_FEIE | SAI_TCSR_TE, 0);

      for (timeout = 100000;
           timeout > 0 &&
           (n947_sai_getreg(priv, N947_SAI_TCSR_OFFSET) &
            SAI_TCSR_TE) != 0;
           timeout--)
        {
        }

      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET,
                         SAI_CSR_BCE | SAI_TCSR_TE, SAI_CSR_FR);
      priv->tx_slot = 0;
      priv->tx_mono_sample = 0;
      priv->tx_running = false;
    }

#ifdef CONFIG_N947_SAI_TXDMA
  /* Playback may arrive while microphone capture is already keeping the TX
   * clock alive with PIO silence, or while a preceding DMA block is
   * draining.
   * Switch that live clock to DMA without resetting the FIFO.
   */

  if (need_tx_data && priv->tx_running && priv->txdma != NULL &&
      !priv->txdma_failed && !priv->txdma_active && !priv->tx_paused)
    {
      priv->txdma_draining = false;
      dma_ret = n947_sai_txdma_kick(priv);
      if (dma_ret > 0)
        {
          n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET,
                             SAI_CSR_FWIE | SAI_CSR_FRIE, SAI_CSR_FRDE);
        }
      else if (dma_ret < 0)
        {
          priv->tx_dma_errors++;
          priv->txdma_failed = true;
        }
    }
#endif

  /* Prime TX before enabling it.  Silent words keep BCLK/WS continuous for
   * microphone-only capture and avoid an immediate FIFO underrun.
   */

  if (need_tx_clock && !priv->tx_running)
    {
      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0, SAI_CSR_FR);
      priv->tx_slot = 0;

      for (i = 0; i < N947_SAI_FIFO_DEPTH; i++)
        {
          n947_sai_putreg(priv, N947_SAI_TDR_OFFSET(0),
                          need_tx_data ? n947_sai_tx_word(priv) : 0);
        }

      n947_sai_clear_status(priv, N947_SAI_TCSR_OFFSET,
                            SAI_CSR_FEF | SAI_CSR_SEF);

#ifdef CONFIG_N947_SAI_TXDMA
      dma_ret = 0;
      if (need_tx_data && priv->txdma != NULL && !priv->txdma_failed)
        {
          dma_ret = n947_sai_txdma_kick(priv);
          if (dma_ret < 0)
            {
              priv->tx_dma_errors++;
              priv->txdma_failed = true;
            }
        }

      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET,
                        SAI_CSR_FRDE | SAI_CSR_FWIE | SAI_CSR_FRIE, 0);
      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0,
                        (dma_ret > 0 ? SAI_CSR_FRDE : SAI_CSR_FRIE) |
                        SAI_CSR_FEIE | SAI_CSR_BCE | SAI_TCSR_TE);
#else
      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0,
                        SAI_CSR_FRIE | SAI_CSR_FEIE | SAI_CSR_BCE |
                        SAI_TCSR_TE);
#endif
      priv->tx_running = true;
    }

  if (need_rx && !priv->rx_running)
    {
      n947_sai_modifycsr(priv, N947_SAI_RCSR_OFFSET, 0, SAI_CSR_FR);
      priv->rx_slot = 0;
      n947_sai_clear_status(priv, N947_SAI_RCSR_OFFSET,
                            SAI_CSR_FEF | SAI_CSR_SEF);
      n947_sai_modifycsr(priv, N947_SAI_RCSR_OFFSET, 0,
                        SAI_CSR_FRIE | SAI_CSR_FEIE | SAI_CSR_BCE |
                        SAI_RCSR_RE);
      priv->rx_running = true;
    }
}

static int n947_sai_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct n947_sai_s *priv = arg;
  uint32_t tcsr;
  uint32_t rcsr;
#ifdef CONFIG_N947_SAI_TXDMA
  int dma_ret;
#endif
  int i;

  rcsr = n947_sai_getreg(priv, N947_SAI_RCSR_OFFSET);
  tcsr = n947_sai_getreg(priv, N947_SAI_TCSR_OFFSET);

  if ((rcsr & SAI_CSR_FEF) != 0)
    {
      priv->rx_overruns++;
      n947_sai_clear_status(priv, N947_SAI_RCSR_OFFSET, SAI_CSR_FEF);
      n947_sai_modifycsr(priv, N947_SAI_RCSR_OFFSET, 0, SAI_CSR_FR);
      priv->rx_slot = 0;
    }

  if ((tcsr & SAI_CSR_FEF) != 0)
    {
#ifdef CONFIG_N947_SAI_TXDMA
      if (!n947_sai_transport_busy(&priv->tx))
        {
          priv->tx_underruns++;
          priv->tx_underruns_idle++;
          n947_sai_clear_status(priv, N947_SAI_TCSR_OFFSET, SAI_CSR_FEF);
          n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0, SAI_CSR_FR);
          priv->tx_slot = 0;
          goto tx_fef_done;
        }

      syslog(LOG_ERR,
             "SAI1 TX FIFO error: csr=%08lx tfr=%08lx dma=%u "
             "half=%u ready=%u/%u words=%lu/%lu citer=%u "
             "active_apb=%u pending=%lu\n",
             (unsigned long)tcsr,
             (unsigned long)n947_sai_getreg(priv,
                                             N947_SAI_TFR_OFFSET(0)),
             priv->txdma_active ? 1 : 0,
             priv->txdma_index,
             priv->txdma_ready[0] ? 1 : 0,
             priv->txdma_ready[1] ? 1 : 0,
             (unsigned long)priv->txdma_words[0],
             (unsigned long)priv->txdma_words[1],
             priv->txdma != NULL ?
               n947_dmach_getcount(priv->txdma) : 0,
             priv->tx.active != NULL ? 1 : 0,
             (unsigned long)sq_count(&priv->tx.pending));
      n947_dmach_dump(priv->txdma);
#endif
      priv->tx_underruns++;
      if (n947_sai_transport_busy(&priv->tx))
        {
          priv->tx_underruns_busy++;
        }
      else
        {
          priv->tx_underruns_idle++;
        }

      n947_sai_clear_status(priv, N947_SAI_TCSR_OFFSET, SAI_CSR_FEF);
      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0, SAI_CSR_FR);
      priv->tx_slot = 0;

#ifdef CONFIG_N947_SAI_TXDMA
      if (priv->txdma_active)
        {
          /* A transient request-mux/FIFO loss must not terminate the whole
           * spoken response.  Reinstall the two cyclic TCDs and resume from
           * their already prepared blocks.  At worst this replays less than
           * one 10.7 ms half; the APB ownership remains unchanged.
           */

          n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET,
                             SAI_CSR_FRDE, 0);
          n947_dmach_stop(priv->txdma);
          priv->txdma_active = false;
          dma_ret = n947_sai_txdma_kick(priv);
          if (dma_ret > 0)
            {
              priv->tx_dma_recoveries++;
              n947_sai_clear_status(priv, N947_SAI_TCSR_OFFSET,
                                    SAI_CSR_FEF | SAI_CSR_SEF);
              n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0,
                                 SAI_CSR_FRDE);
              syslog(LOG_WARNING,
                     "SAI1 TX DMA recovered after FIFO error "
                     "(recovery=%lu)\n",
                     (unsigned long)priv->tx_dma_recoveries);
              goto tx_fef_done;
            }

          n947_sai_txdma_abort(priv);
          priv->tx_dma_errors++;
          priv->txdma_failed = true;
          if (priv->tx.active != NULL)
            {
              n947_sai_complete_buffer(&priv->tx, -EIO);
            }

          n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, 0,
                             SAI_CSR_FRIE);
        }
#endif
    }

tx_fef_done:

  if (priv->rx_running && (rcsr & SAI_CSR_FRF) != 0)
    {
      for (i = 0; i < N947_SAI_FIFO_WATERMARK; i++)
        {
          n947_sai_rx_word(priv,
                           n947_sai_getreg(priv, N947_SAI_RDR_OFFSET(0)));
        }
    }

#ifdef CONFIG_N947_SAI_TXDMA
  if (priv->txdma_draining && (tcsr & SAI_CSR_FWF) != 0)
    {
      priv->txdma_draining = false;
      n947_sai_modifycsr(priv, N947_SAI_TCSR_OFFSET, SAI_CSR_FWIE, 0);
    }
#endif

  if (priv->tx_running && (tcsr & SAI_CSR_FRF) != 0
#ifdef CONFIG_N947_SAI_TXDMA
      && !priv->txdma_active && !priv->txdma_draining
#endif
     )
    {
      for (i = 0; i < N947_SAI_FIFO_SERVICE_WORDS; i++)
        {
          n947_sai_putreg(priv, N947_SAI_TDR_OFFSET(0),
                          n947_sai_tx_word(priv));
        }
    }

  n947_sai_update_running(priv);
  return OK;
}

static int n947_sai_channels(FAR struct n947_sai_s *priv, bool receive,
                             uint8_t channels)
{
  irqstate_t flags;
  int ret = OK;

  if (channels != 1 && channels != 2)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  if (n947_sai_transport_busy(receive ? &priv->rx : &priv->tx))
    {
      ret = -EBUSY;
    }
  else if (receive)
    {
      priv->rx_channels = channels;
    }
  else
    {
      priv->tx_channels = channels;
    }

  leave_critical_section(flags);
  return ret;
}

static uint32_t n947_sai_samplerate(FAR struct n947_sai_s *priv,
                                    uint32_t rate, uint8_t width)
{
  irqstate_t flags;
  uint32_t actual;

  flags = enter_critical_section();
  if (priv->tx_running || priv->rx_running ||
      n947_sai_transport_busy(&priv->tx) ||
      n947_sai_transport_busy(&priv->rx))
    {
      actual = priv->actual_rate;
    }
  else
    {
      actual = n947_sai_set_rate_locked(priv, rate);
    }

  leave_critical_section(flags);
  return actual == 0 ? 0 : actual * width;
}

static uint32_t n947_sai_datawidth(FAR struct n947_sai_s *priv,
                                   bool receive, int bits)
{
  irqstate_t flags;
  uint32_t bitrate = 0;

  if (bits != 16 && bits != 32)
    {
      i2serr("ERROR: SAI supports application widths 16 and 32, got %d\n",
             bits);
      return 0;
    }

  flags = enter_critical_section();
  if (!n947_sai_transport_busy(receive ? &priv->rx : &priv->tx))
    {
      if (receive)
        {
          priv->rx_width = bits;
        }
      else
        {
          priv->tx_width = bits;
        }

      bitrate = priv->actual_rate * bits;
    }

  leave_critical_section(flags);
  return bitrate;
}

static int n947_sai_enqueue(FAR struct n947_sai_s *priv, bool receive,
                            FAR struct ap_buffer_s *apb,
                            i2s_callback_t callback, FAR void *arg,
                            uint32_t timeout)
{
  FAR struct n947_sai_transport_s *transport;
  FAR struct n947_sai_buffer_s *container;
  unsigned int frame_bytes;
  irqstate_t flags;

  if (apb == NULL || callback == NULL)
    {
      return -EINVAL;
    }

  /* The initial IRQ implementation has no transfer watchdog.  Production
   * audio_i2s uses zero; reject a nonzero timeout instead of silently
   * promising timeout behavior.
   */

  if (timeout != 0)
    {
      return -ENOTSUP;
    }

  transport = receive ? &priv->rx : &priv->tx;
  frame_bytes = n947_sai_sample_bytes(receive ? priv->rx_width :
                                               priv->tx_width) *
                (receive ? priv->rx_channels : priv->tx_channels);

  if (receive)
    {
      if (apb->nmaxbytes == 0 || (apb->nmaxbytes % frame_bytes) != 0)
        {
          return -EINVAL;
        }
    }
  else if (apb->nbytes <= apb->curbyte ||
           ((apb->nbytes - apb->curbyte) % frame_bytes) != 0)
    {
      return -EINVAL;
    }

  container = n947_sai_buffer_alloc(transport);
  if (container == NULL)
    {
      return -EAGAIN;
    }

  if (receive)
    {
      apb->curbyte = 0;
      apb->nbytes = 0;
    }

  apb_reference(apb);
  container->apb = apb;
  container->callback = callback;
  container->arg = arg;
  container->result = -EBUSY;

  flags = enter_critical_section();
#ifdef CONFIG_N947_SAI_TXDMA
  if (!receive && !n947_sai_transport_busy(transport) &&
      !priv->tx_running)
    {
      /* A DMA fault falls back to PIO for the rest of that stream.  Give a
       * later, independent playback a clean retry without requiring reboot.
       */

      priv->txdma_failed = false;
    }

#endif
  sq_addlast(&container->entry, &transport->pending);
  if (!receive)
    {
      uint32_t depth = sq_count(&transport->pending) +
                       (transport->active != NULL ? 1 : 0);

      if (depth > priv->tx_queue_max)
        {
          priv->tx_queue_max = depth;
        }
    }

  n947_sai_update_running(priv);
  leave_critical_section(flags);
  return OK;
}

/* Move active and pending buffers to the completion worker.  Caller holds
 * the critical section.
 */

static void n947_sai_cancel(FAR struct n947_sai_transport_s *transport)
{
  FAR struct n947_sai_buffer_s *container;

  if (transport->active != NULL)
    {
      n947_sai_complete_buffer(transport, -ECANCELED);
    }

  while ((container = (FAR struct n947_sai_buffer_s *)
                      sq_remfirst(&transport->pending)) != NULL)
    {
      container->result = -ECANCELED;
      sq_addlast(&container->entry, &transport->done);
    }

  if (!sq_empty(&transport->done))
    {
      n947_sai_schedule_worker(transport);
    }
}

static int n947_sai_rxchannels(FAR struct i2s_dev_s *dev, uint8_t channels)
{
  return n947_sai_channels((FAR struct n947_sai_s *)dev, true, channels);
}

static uint32_t n947_sai_rxsamplerate(FAR struct i2s_dev_s *dev,
                                      uint32_t rate)
{
  FAR struct n947_sai_s *priv = (FAR struct n947_sai_s *)dev;

  return n947_sai_samplerate(priv, rate, priv->rx_width);
}

static uint32_t n947_sai_rxdatawidth(FAR struct i2s_dev_s *dev, int bits)
{
  return n947_sai_datawidth((FAR struct n947_sai_s *)dev, true, bits);
}

static int n947_sai_receive(FAR struct i2s_dev_s *dev,
                            FAR struct ap_buffer_s *apb,
                            i2s_callback_t callback, FAR void *arg,
                            uint32_t timeout)
{
  return n947_sai_enqueue((FAR struct n947_sai_s *)dev, true, apb,
                          callback, arg, timeout);
}

static int n947_sai_txchannels(FAR struct i2s_dev_s *dev, uint8_t channels)
{
  return n947_sai_channels((FAR struct n947_sai_s *)dev, false, channels);
}

static uint32_t n947_sai_txsamplerate(FAR struct i2s_dev_s *dev,
                                      uint32_t rate)
{
  FAR struct n947_sai_s *priv = (FAR struct n947_sai_s *)dev;

  return n947_sai_samplerate(priv, rate, priv->tx_width);
}

static uint32_t n947_sai_txdatawidth(FAR struct i2s_dev_s *dev, int bits)
{
  return n947_sai_datawidth((FAR struct n947_sai_s *)dev, false, bits);
}

static int n947_sai_send(FAR struct i2s_dev_s *dev,
                         FAR struct ap_buffer_s *apb,
                         i2s_callback_t callback, FAR void *arg,
                         uint32_t timeout)
{
  return n947_sai_enqueue((FAR struct n947_sai_s *)dev, false, apb,
                          callback, arg, timeout);
}

static uint32_t n947_sai_getmclkfrequency(FAR struct i2s_dev_s *dev)
{
  return N947_SAI_SOURCE_CLOCK;
}

static uint32_t n947_sai_setmclkfrequency(FAR struct i2s_dev_s *dev,
                                          uint32_t frequency)
{
  return frequency == N947_SAI_SOURCE_CLOCK ? frequency : 0;
}

static int n947_sai_ioctl(FAR struct i2s_dev_s *dev, int cmd,
                          unsigned long arg)
{
  FAR struct n947_sai_s *priv = (FAR struct n947_sai_s *)dev;
  FAR struct audio_buf_desc_s *bufdesc;
  irqstate_t flags;
  bool playback;
  int ret = OK;

  playback = arg != 0;

  switch (cmd)
    {
      case AUDIOIOC_START:
        flags = enter_critical_section();
        if (playback)
          {
            priv->tx_started = true;
            priv->tx_paused = false;
          }
        else
          {
            priv->rx_started = true;
            priv->rx_paused = false;
          }

        n947_sai_update_running(priv);
        leave_critical_section(flags);
        break;

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
      case AUDIOIOC_STOP:
        flags = enter_critical_section();
        if (playback)
          {
            priv->tx_started = false;
            priv->tx_paused = false;
#ifdef CONFIG_N947_SAI_TXDMA
            n947_sai_txdma_abort(priv);
#endif
            n947_sai_cancel(&priv->tx);
          }
        else
          {
            priv->rx_started = false;
            priv->rx_paused = false;
            n947_sai_cancel(&priv->rx);
          }

        n947_sai_update_running(priv);
        leave_critical_section(flags);
        break;
#endif

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
      case AUDIOIOC_PAUSE:
        flags = enter_critical_section();
        if (playback)
          {
            priv->tx_paused = true;
          }
        else
          {
            priv->rx_paused = true;
          }

        n947_sai_update_running(priv);
        leave_critical_section(flags);
        break;

      case AUDIOIOC_RESUME:
        flags = enter_critical_section();
        if (playback)
          {
            priv->tx_paused = false;
          }
        else
          {
            priv->rx_paused = false;
          }

        n947_sai_update_running(priv);
        leave_critical_section(flags);
        break;
#endif

      case AUDIOIOC_SHUTDOWN:
        flags = enter_critical_section();
        if (playback)
          {
            priv->tx_started = false;
            priv->tx_paused = false;
#ifdef CONFIG_N947_SAI_TXDMA
            n947_sai_txdma_abort(priv);
#endif
            n947_sai_cancel(&priv->tx);
          }
        else
          {
            priv->rx_started = false;
            priv->rx_paused = false;
            n947_sai_cancel(&priv->rx);
          }

        n947_sai_update_running(priv);
        leave_critical_section(flags);
        break;

      case AUDIOIOC_ALLOCBUFFER:
        bufdesc = (FAR struct audio_buf_desc_s *)arg;
        ret = apb_alloc(bufdesc);
        break;

      case AUDIOIOC_FREEBUFFER:
        bufdesc = (FAR struct audio_buf_desc_s *)arg;
        if (bufdesc == NULL || bufdesc->u.buffer == NULL)
          {
            ret = -EINVAL;
          }
        else
          {
            apb_free(bufdesc->u.buffer);
            ret = sizeof(struct audio_buf_desc_s);
          }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct i2s_dev_s *n947_sai_initialize(int port)
{
  FAR struct n947_sai_s *priv;
  uint32_t param;
  int ret;

#ifdef CONFIG_N947_SAI1
  if (port == 1)
    {
      priv = &g_sai1;
    }
  else
#endif
    {
      return NULL;
    }

  if (priv->initialized)
    {
      return &priv->dev;
    }

  /* Route the already-running 150 MHz PLL0 to SAI1, release the peripheral
   * reset and enable its AHB clock.  No external MCLK pin is enabled.
   */

  nxxx_set_periphclock(SYSCON_SAI1CLK, PLL0_TO_SAI1, 1);
  nxxx_set_clock_gate(CLOCK_GATE_SAI1, true);
  putreg32(N947_SYSCON_SAI1_RESET, N947_SYSCON_PRESETCTRLSET2);
  putreg32(N947_SYSCON_SAI1_RESET, N947_SYSCON_PRESETCTRLCLR2);

  sq_init(&priv->tx.pending);
  sq_init(&priv->tx.done);
  sq_init(&priv->rx.pending);
  sq_init(&priv->rx.done);
  priv->tx.priv = priv;
  priv->rx.priv = priv;
  priv->tx.receive = false;
  priv->rx.receive = true;

  n947_sai_configure_hardware(priv);

  ret = irq_attach(priv->irq, n947_sai_interrupt, priv);
  if (ret < 0)
    {
      i2serr("ERROR: failed to attach SAI1 IRQ: %d\n", ret);
      nxxx_set_clock_gate(CLOCK_GATE_SAI1, false);
      return NULL;
    }

  up_enable_irq(priv->irq);

  /* Non-destructive register-level sanity check.  PARAM encodes the number
   * of data lines, FIFO depth and maximum frame size; a zero value means the
   * peripheral was not clocked or addressed correctly.
   */

  param = n947_sai_getreg(priv, N947_SAI_PARAM_OFFSET);
  if (param == 0)
    {
      i2serr("ERROR: SAI1 PARAM is zero (clock/reset/address failure)\n");
      up_disable_irq(priv->irq);
      irq_detach(priv->irq);
      nxxx_set_clock_gate(CLOCK_GATE_SAI1, false);
      return NULL;
    }

#ifdef CONFIG_N947_SAI_TXDMA
  /* SAI1 TX request source 102 is defined by the MCXN947 input-mux table in
   * n947_dmamux.h.  Allocate one channel and one reusable cache-aligned
   * conversion block.  Either allocation may fail without losing audio:
   * the existing FIFO-interrupt path remains the runtime fallback.
   */

  priv->txdma_buffer = kmm_memalign(N947_SAI_TXDMA_ALIGN,
                                    N947_SAI_TXDMA_WORDS *
                                    sizeof(uint32_t));
  if (priv->txdma_buffer != NULL)
    {
      priv->txdma_tcd = kmm_memalign(N947_SAI_TXDMA_ALIGN,
                                     N947_SAI_TXDMA_BLOCKS *
                                     sizeof(struct n947_edma_tcd_s));
      if (priv->txdma_tcd != NULL)
        {
          priv->txdma = n947_dmach_alloc(DMA_REQUEST_MUXSAI1TX, 7);
        }

      if (priv->txdma == NULL)
        {
          if (priv->txdma_tcd != NULL)
            {
              kmm_free(priv->txdma_tcd);
              priv->txdma_tcd = NULL;
            }

          kmm_free(priv->txdma_buffer);
          priv->txdma_buffer = NULL;
        }
    }

  if (priv->txdma == NULL)
    {
      i2serr("WARNING: SAI1 TX eDMA unavailable; using FIFO IRQ mode\n");
    }
  else
    {
      i2sinfo("SAI1 TX eDMA enabled: request=%u staging=%u bytes\n",
              DMA_REQUEST_MUXSAI1TX,
              N947_SAI_TXDMA_WORDS * sizeof(uint32_t));
    }
#endif

  priv->initialized = true;
  i2sinfo("SAI1 ready: PARAM=%08lx requested=%lu actual=%lu BCLK=%lu\n",
          (unsigned long)param, (unsigned long)priv->requested_rate,
          (unsigned long)priv->actual_rate,
          (unsigned long)priv->bit_clock);
  return &priv->dev;
}

int n947_sai_getstats(FAR struct i2s_dev_s *dev,
                      FAR struct n947_sai_stats_s *stats)
{
  FAR struct n947_sai_s *priv = (FAR struct n947_sai_s *)dev;
  irqstate_t flags;

  if (priv == NULL || stats == NULL || priv->dev.ops != &g_n947_sai_ops)
    {
      return -EINVAL;
    }

  flags = enter_critical_section();
  stats->requested_rate = priv->requested_rate;
  stats->actual_rate = priv->actual_rate;
  stats->bit_clock = priv->bit_clock;
  stats->tx_underruns = priv->tx_underruns;
  stats->tx_underruns_busy = priv->tx_underruns_busy;
  stats->tx_underruns_idle = priv->tx_underruns_idle;
  stats->tx_boundary_empty = priv->tx_boundary_empty;
  stats->tx_queue_max = priv->tx_queue_max;
  stats->tx_dma_chunks = priv->tx_dma_chunks;
  stats->tx_dma_errors = priv->tx_dma_errors;
  stats->tx_dma_recoveries = priv->tx_dma_recoveries;
  stats->rx_overruns = priv->rx_overruns;
  stats->tx_buffers = priv->tx_buffers;
  stats->rx_buffers = priv->rx_buffers;
  leave_critical_section(flags);
  return OK;
}

#endif /* CONFIG_N947_SAI */
