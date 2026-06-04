/****************************************************************************
 * arch/arm/src/t113/t113_dmic.c
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
 * T113-S3 PDM DMIC controller driver.
 *
 * Captures PCM from up to four PDM digital microphones (DMIC-DATA0..3)
 * and exposes /dev/audio0 as an INPUT audio_lowerhalf.  The MQ-R board
 * wires a single STMicro MP34DT06J between PD19 (DATA0) and PD20 (CLK)
 * via a flying lead; the driver defaults to 16 kHz mono S16_LE which
 * fits voice / KWS pipelines.  Stereo (two mics on the same DATA0 lane,
 * one driving the L half-period and one the R half-period) and the
 * other DMIC-DATA lanes are reachable via AUDIOIOC_CONFIGURE for future
 * boards.
 *
 * Data path (mirrors the codec ADC path in t113_audio.c so the same
 * cyclic-DMA buffer-pool pattern works for both nodes):
 *
 *   PDM mic -> DATA0/CLK -> DMIC controller -> decimator + HPF
 *     -> 21-bit RXFIFO_O -> DMIC_DATA reg (bits 15:0 = sign-extended
 *                                          16-bit sample)
 *     -> DRQ_DMIC=8 -> system DMA (16-bit width) -> linear DRAM
 *          buf_pool -> audio_lowerhalf upper half (DEQUEUE callback)
 *
 * Power / clock: t113_audio_clk_request(T113_AUDIO_CONSUMER_DMIC, rate)
 * brings up PLL_AUDIO1(DIV5) -> 24.576 MHz module clock and opens the
 * DMIC bus gate / reset.  The controller derives DMIC_CLK from the
 * module clock via its internal SR + OSR fields:
 *
 *   PDM_CLK = sample_rate * oversample
 *
 * For 16 kHz target with oversample=128x => PDM_CLK = 2.048 MHz, well
 * inside the MP34DT06J 1.2..3.25 MHz spec.  For 32/48 kHz the driver
 * switches to oversample=64x (matches mainline sun50i-dmic).
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/audio/audio.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "hardware/t113_dma.h"
#include "hardware/t113_dmic.h"
#include "hardware/t113_pinmap.h"
#include "t113_clk.h"
#include "t113_dma.h"
#include "t113_gpio.h"
#include "t113_dmic.h"

#ifdef CONFIG_T113_DMIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default sample format applied if the application does not issue
 * AUDIOIOC_CONFIGURE before AUDIOIOC_START.  Voice / KWS workloads on
 * the MQ-R reference board use 16 kHz mono S16_LE.
 */

#define T113_DMIC_DEFAULT_RATE      16000
#define T113_DMIC_DEFAULT_BITS      16
#define T113_DMIC_DEFAULT_CHANNELS  1

/* RXFIFO trigger level: half-full (64 of 128 samples).  Matches the
 * power-on-default and mainline sun50i-dmic; balances DMA wakeup rate
 * against PDM->DDR latency.  Lower values raise the IRQ rate; higher
 * values risk RXFIFO overflow under DMA stalls.
 */

#define T113_DMIC_RX_TRIG_LEVEL     0x40

/* DMA buffer pool sizing.  Re-uses the audio framework knobs; a 4 KB
 * buffer at 16 kHz mono = 128 ms / buffer.
 *
 * Buffer count is pinned to 2 (ping-pong) regardless of
 * CONFIG_AUDIO_NUM_BUFFERS.  Same constraint as the codec ADC/DAC paths
 * in t113_audio.c: t113_dma.c cyclic mode delivers exactly two IRQs per
 * full sweep (HLFDONE + PKGDONE) and dmic_dma_callback() dequeues one
 * apb per IRQ, so any count > 2 leaves surplus apbs being written by
 * the DMA but never returned to the upper half before the next pass
 * overwrites them -- the user-space byte count drops by (count-2)/count.
 * Lifting this to N > 2 requires a chained-descriptor API in t113_dma.c
 * with one IRQ per descriptor.
 */

#define T113_DMIC_BUFFER_BYTES      CONFIG_AUDIO_BUFFER_NUMBYTES
#define T113_DMIC_BUFFER_COUNT      2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct t113_dmic_dev_s
{
  struct audio_lowerhalf_s lower;     /* Must be first */

  mutex_t   lock;                     /* Protects controller state */
  bool      active;                   /* Capture stream is running */
  uint32_t  cur_rate;                 /* Active sample rate (Hz) */
  uint8_t   cur_bits;                 /* Active bit width (16 or 24) */
  uint8_t   cur_channels;             /* 1 = mono (DATA0L), 2 = stereo */

  DMA_HANDLE dma;                     /* DRQ_DMIC=8 (RX-only) */
  uint8_t  *buf_pool;                 /* Single contiguous DDR ring */
  size_t    buf_total;                /* Total bytes (count * size) */
  size_t    buf_size;                 /* Size of one apb */
  uint8_t   buf_count;                /* Number of apb in the pool */
  uint8_t   buf_alloc;                /* Index of next apb to hand out */

  spinlock_t qlock;                   /* Protects pendq + xrun */
  struct dq_queue_s pendq;            /* Buffers queued by upper */
  bool      xrun;                     /* DMA paused waiting for buffers */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  dmic_configure(struct audio_lowerhalf_s *lower,
                           void *session,
                           const struct audio_caps_s *caps);
static int  dmic_start(struct audio_lowerhalf_s *lower, void *session);
static int  dmic_stop(struct audio_lowerhalf_s *lower, void *session);
static int  dmic_reserve(struct audio_lowerhalf_s *lower, void **session);
static int  dmic_release(struct audio_lowerhalf_s *lower, void *session);
#else
static int  dmic_configure(struct audio_lowerhalf_s *lower,
                           const struct audio_caps_s *caps);
static int  dmic_start(struct audio_lowerhalf_s *lower);
static int  dmic_stop(struct audio_lowerhalf_s *lower);
static int  dmic_reserve(struct audio_lowerhalf_s *lower);
static int  dmic_release(struct audio_lowerhalf_s *lower);
#endif

static int  dmic_getcaps(struct audio_lowerhalf_s *lower, int type,
                         struct audio_caps_s *caps);
static int  dmic_shutdown(struct audio_lowerhalf_s *lower);
static int  dmic_allocbuffer(struct audio_lowerhalf_s *lower,
                             struct audio_buf_desc_s *bufdesc);
static int  dmic_freebuffer(struct audio_lowerhalf_s *lower,
                            struct audio_buf_desc_s *bufdesc);
static int  dmic_enqueuebuffer(struct audio_lowerhalf_s *lower,
                               struct ap_buffer_s *apb);
static int  dmic_ioctl(struct audio_lowerhalf_s *lower, int cmd,
                       unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_dmic_ops =
{
  .getcaps        = dmic_getcaps,
  .configure      = dmic_configure,
  .shutdown       = dmic_shutdown,
  .start          = dmic_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop           = dmic_stop,
#endif
  .allocbuffer    = dmic_allocbuffer,
  .freebuffer     = dmic_freebuffer,
  .enqueuebuffer  = dmic_enqueuebuffer,
  .ioctl          = dmic_ioctl,
  .reserve        = dmic_reserve,
  .release        = dmic_release,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dmic_validate_caps
 *
 * Description:
 *   Range-check the sample format requested through AUDIOIOC_CONFIGURE.
 *   Spec section 1 / 6.2: 8/16/32/48 kHz, 16-bit, mono or stereo.
 *
 ****************************************************************************/

static int dmic_validate_caps(uint32_t rate, uint8_t bits, uint8_t channels)
{
  if (rate != 8000 && rate != 16000 && rate != 32000 && rate != 48000)
    {
      return -EINVAL;
    }

  /* DMIC controller supports 16 bit and 24 bit.  M3 only validates the
   * 16-bit path because that is what nxrecorder advertises and the DMA
   * 16-bit width preserves precision end-to-end.  24-bit support is
   * left to a future extension.
   */

  if (bits != 16)
    {
      return -EINVAL;
    }

  if (channels != 1 && channels != 2)
    {
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: dmic_rate_to_sr
 *
 * Description:
 *   Translate a sample rate to the SR field of DMIC_SR.
 *
 ****************************************************************************/

static uint32_t dmic_rate_to_sr(uint32_t rate)
{
  switch (rate)
    {
      case 48000: return DMIC_SR_48K;
      case 32000: return DMIC_SR_32K;
      case 16000: return DMIC_SR_16K;
      case  8000: return DMIC_SR_8K;
      default:    return DMIC_SR_16K;
    }
}

/****************************************************************************
 * Name: dmic_pin_config
 *
 * Description:
 *   Mux PD19 (DATA0) and PD20 (CLK) to function 4 (DMIC).  Caller must
 *   hold the state lock so the GPIO writes are serialised against any
 *   other consumer (the t113-evb has no other lane wired but this is
 *   future-proof).  Pins are returned to their reset state on the way
 *   down -- modelled after t113_can.c which leaves CAN pins in the
 *   muxed state across stop/start cycles to avoid disturbing wires
 *   that may be shared with other peripherals.
 *
 ****************************************************************************/

static void dmic_pin_config(void)
{
  t113_gpio_config(T113_DMIC_DATA0_1);
  t113_gpio_config(T113_DMIC_CLK_1);
}

/****************************************************************************
 * Name: dmic_program_controller
 *
 * Description:
 *   Program the DMIC controller registers for the current sample
 *   format.  Caller must hold the state lock.  Mirrors the per-channel
 *   bring-up flow in mainline Linux sun50i-h6-dmic / sun50i-dmic.
 *
 *   Sequence (UM 8.2.3.3 "Operation Mode" + sun50i-dmic_startup +
 *   sun50i-dmic_hw_params):
 *     1. Disable globe + per-channel enables (DMIC_EN=0).
 *     2. Flush RXFIFO + clear sample counter.
 *     3. Program channel count (CH_NUM = N - 1) and per-channel
 *        enables (chan_en = (1 << channels) - 1).
 *     4. Enable HPF on every active channel (DC blocker).
 *     5. Program sample rate (DMIC_SR) and oversample (DMIC_CTR.OSR).
 *        OSR=0 (128x) for rate <= 24 kHz, OSR=1 (64x) for >= 32 kHz.
 *     6. Program RXFIFO control: trigger level + 16-bit / sign-extend
 *        MSB mode so a 16-bit DMA read picks up the data half of each
 *        FIFO word with sign extension preserved.
 *     7. Volume registers stay at reset default (0xa0 = 0 dB on every
 *        channel) -- there is no analog gain stage on PDM, gain is
 *        digital and applied late in the path.
 *
 *   Does NOT enable DRQ or the global enable bit; those are flipped
 *   in dmic_start after the DMA descriptor is armed.
 *
 ****************************************************************************/

static void dmic_program_controller(struct t113_dmic_dev_s *dev)
{
  uint32_t reg;
  uint32_t chan_en;

  /* Step 1: disable controller + per-channel enables.  GLOBAL is
   * mandatory off before reconfiguring per UM 8.2.3.3 step 1.
   */

  putreg32(0, T113_DMIC_EN);

  /* Step 2: flush RXFIFO and clear sample counter.  FLUSH is W1C self-
   * clearing; we OR in the trigger level + mode bits we want for the
   * subsequent capture so the post-flush register settles in the
   * desired config.  CNT cleared so A/V sync (if any) starts at zero.
   */

  reg  = DMIC_RXFIFO_CTR_FLUSH;
  reg |= DMIC_RXFIFO_CTR_MODE_MSB;
  reg |= DMIC_RXFIFO_CTR_SAMPLE_16;
  reg |= ((uint32_t)T113_DMIC_RX_TRIG_LEVEL) << DMIC_RXFIFO_CTR_TRG_SHIFT;
  putreg32(reg, T113_DMIC_RXFIFO_CTR);

  putreg32(0, T113_DMIC_CNT);

  /* Step 3: program channel count.  The hardware encodes N as (N - 1).
   * chan_en is the bit mask of active channels in DMIC_EN[7:0]; we
   * pack from LSB up: mono = DATA0L only, stereo = DATA0L + DATA0R.
   * Both halves of DATA0 share PD19; the PDM mic toggles which half
   * it presents on each PDM_CLK edge so two mics on one DATA line
   * appear as L and R inside the controller.
   */

  reg  = ((uint32_t)(dev->cur_channels - 1)) << DMIC_CH_NUM_SHIFT;
  reg &= DMIC_CH_NUM_MASK;
  putreg32(reg, T113_DMIC_CH_NUM);

  chan_en = (1u << dev->cur_channels) - 1u;

  /* Step 4: HPF on every active channel.  DC blocker -- PDM mics carry
   * a few-mV bias on their digital sample stream which would otherwise
   * appear as a fixed offset on every captured sample.  Coefficients
   * stay at reset default (cutoff ~3 Hz at 16 kHz, well below the
   * voice band).
   */

  putreg32(chan_en, T113_DMIC_HPF_EN_CTR);

  /* Step 5: sample rate + oversample.  OSR=64x for >= 32 kHz keeps the
   * PDM_CLK below the controller's 3.072 MHz max (48 kHz * 64 = 3.072
   * MHz).  At lower rates 128x improves the analog filter's stop-band
   * because each PDM bit covers more time, which the on-chip CIC has
   * more samples to integrate over before producing a PCM word.
   */

  putreg32(dmic_rate_to_sr(dev->cur_rate), T113_DMIC_SR);

  reg = getreg32(T113_DMIC_CTR);
  if (dev->cur_rate >= 32000)
    {
      reg |= DMIC_CTR_OVERSAMPLE_64X;
    }
  else
    {
      reg &= ~DMIC_CTR_OVERSAMPLE_64X;
    }

  putreg32(reg, T113_DMIC_CTR);

  /* Step 6: per-channel enables.  Global enable stays off until DMA
   * is armed (dmic_start step 5).
   */

  reg  = getreg32(T113_DMIC_EN);
  reg &= ~DMIC_EN_CHAN_MASK;
  reg |= (chan_en & DMIC_EN_CHAN_MASK);
  putreg32(reg, T113_DMIC_EN);

  /* Step 7: leave volume / channel-mapping at reset defaults.  CH_MAP
   * default 0x76543210 maps internal channel n to DATA(n/2) (L/R per
   * bit 0); since cur_channels <= 2 the controller only consumes
   * entries CH0 and CH1 which are DATA0L and DATA0R respectively.
   */
}

/****************************************************************************
 * Name: dmic_dma_callback
 *
 * Description:
 *   DMA completion callback.  Called from IRQ context.  Cyclic mode
 *   fires us at the half-buffer (HLFDONE) and full-buffer (PKGDONE)
 *   boundaries: each invocation = one apb completed.  Hand the apb
 *   back to the upper half and pause cyclic DMA if the queue drains
 *   (xrun); enqueuebuffer resumes when fresh apbs arrive.
 *
 ****************************************************************************/

static void dmic_dma_callback(DMA_HANDLE handle, uint8_t status,
                              void *arg)
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)arg;
  struct ap_buffer_s *apb;
  irqstate_t flags;

  UNUSED(handle);
  UNUSED(status);

  flags = spin_lock_irqsave(&dev->qlock);
  apb   = (struct ap_buffer_s *)dq_remfirst(&dev->pendq);
  spin_unlock_irqrestore(&dev->qlock, flags);

  if (apb == NULL)
    {
      /* xrun: DMA finished a buffer but no apb queued.  Pause; resume
       * happens in enqueuebuffer.  Per spec section 6.1 capture xrun
       * is signalled to the application via the empty DEQUEUE return.
       */

      t113_dmapause(dev->dma);
      dev->xrun = true;
      return;
    }

  /* Mark the buffer as full and invalidate cache so the application
   * read() sees fresh DRAM content.
   */

  apb->nbytes = apb->nmaxbytes;
  up_invalidate_dcache((uintptr_t)apb->samp,
                       (uintptr_t)apb->samp + apb->nbytes);

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif

  flags = spin_lock_irqsave(&dev->qlock);
  if (dq_empty(&dev->pendq))
    {
      t113_dmapause(dev->dma);
      dev->xrun = true;
    }

  spin_unlock_irqrestore(&dev->qlock, flags);
}

/****************************************************************************
 * Name: dmic_dma_start
 *
 * Description:
 *   Arm the cyclic DMA descriptor for DMIC RX.  Direction is peripheral
 *   (DMIC_DATA register) to memory (the contiguous buf_pool).
 *
 ****************************************************************************/

static int dmic_dma_start(struct t113_dmic_dev_s *dev)
{
  struct t113_dma_config_s cfg;
  int ret;

  memset(&cfg, 0, sizeof(cfg));

  /* Source: DMIC_DATA register, IO addressing.  RXFIFO_MODE = MSB sign-
   * extends RXFIFO_O[20:5] across DMIC_DATA[31:16] and places the
   * 16-bit sample in DMIC_DATA[15:0]; a 16-bit DMA read picks up the
   * data half of each FIFO word and stores 2 bytes / sample in DRAM.
   * Same convention as the codec ADC path in t113_audio.c.
   */

  cfg.src_drq    = DRQ_DMIC;
  cfg.src_width  = DMAC_WIDTH_16BIT;
  cfg.src_burst  = DMAC_BURST_4;
  cfg.src_linear = false;

  /* Destination: linear DRAM, 16-bit width.  Cache-line aligned at
   * pool allocation time so up_invalidate_dcache cannot evict adjacent
   * unrelated lines.
   */

  cfg.dst_drq    = DRQ_DRAM;
  cfg.dst_width  = DMAC_WIDTH_16BIT;
  cfg.dst_burst  = DMAC_BURST_4;
  cfg.dst_linear = true;

  cfg.mode       = DMAC_MODE_SRC_HANDSHAKE;
  cfg.circular   = true;

  ret = t113_dmasetup(dev->dma,
                      (uintptr_t)T113_DMIC_DATA,
                      (uintptr_t)dev->buf_pool,
                      dev->buf_total, &cfg);
  if (ret < 0)
    {
      return ret;
    }

  /* Drop any stale cache lines for the buffer region; the DMA write
   * path is going to overwrite them and the user will read via
   * up_invalidate_dcache after each apb completes.
   */

  up_invalidate_dcache((uintptr_t)dev->buf_pool,
                       (uintptr_t)dev->buf_pool + dev->buf_total);

  return t113_dmastart(dev->dma, dmic_dma_callback, dev);
}

/****************************************************************************
 * Lower-half ops: reserve / release
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dmic_reserve(struct audio_lowerhalf_s *lower, void **session)
#else
static int dmic_reserve(struct audio_lowerhalf_s *lower)
#endif
{
  UNUSED(lower);
#ifdef CONFIG_AUDIO_MULTI_SESSION
  if (session != NULL)
    {
      *session = NULL;
    }

#endif
  return OK;
}

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dmic_release(struct audio_lowerhalf_s *lower, void *session)
#else
static int dmic_release(struct audio_lowerhalf_s *lower)
#endif
{
  UNUSED(lower);
#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif
  return OK;
}

/****************************************************************************
 * Lower-half ops: getcaps
 ****************************************************************************/

static int dmic_getcaps(struct audio_lowerhalf_s *lower, int type,
                        struct audio_caps_s *caps)
{
  UNUSED(lower);
  UNUSED(type);

  DEBUGASSERT(caps != NULL && caps->ac_len >= sizeof(struct audio_caps_s));

  caps->ac_format.hw  = 0;
  caps->ac_controls.w = 0;

  switch (caps->ac_type)
    {
      case AUDIO_TYPE_QUERY:
        caps->ac_channels = 2;
        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            caps->ac_controls.b[0] = AUDIO_TYPE_INPUT;
            caps->ac_format.hw     = 1u << (AUDIO_FMT_PCM - 1);
          }

        break;

      case AUDIO_TYPE_INPUT:
        caps->ac_channels = 2;
        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K |
                                      AUDIO_SAMP_RATE_16K |
                                      AUDIO_SAMP_RATE_32K |
                                      AUDIO_SAMP_RATE_48K;

            /* b[2] reports primary sample width as a literal integer,
             * matching what AUDIOIOC_CONFIGURE expects.  16 bit only;
             * 24 bit hardware capability is intentionally hidden in
             * M3 (see dmic_validate_caps comment).
             */

            caps->ac_controls.b[2] = 16;
          }

        break;

      default:
        break;
    }

  return caps->ac_len;
}

/****************************************************************************
 * Lower-half ops: configure
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dmic_configure(struct audio_lowerhalf_s *lower,
                          void *session,
                          const struct audio_caps_s *caps)
#else
static int dmic_configure(struct audio_lowerhalf_s *lower,
                          const struct audio_caps_s *caps)
#endif
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)lower;
  uint32_t rate;
  uint8_t  bits;
  uint8_t  channels;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  DEBUGASSERT(caps != NULL);

  if (caps->ac_type != AUDIO_TYPE_INPUT)
    {
      return -EINVAL;
    }

  rate     = caps->ac_controls.hw[0];
  bits     = caps->ac_controls.b[2];
  channels = caps->ac_channels;

  ret = dmic_validate_caps(rate, bits, channels);
  if (ret < 0)
    {
      auderr("ERROR: invalid caps rate=%u bits=%u ch=%u\n",
             (unsigned)rate, (unsigned)bits, (unsigned)channels);
      return ret;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Spec section 6.2: configure while running is not allowed in M3. */

  if (dev->active)
    {
      nxmutex_unlock(&dev->lock);
      return -EBUSY;
    }

  dev->cur_rate     = rate;
  dev->cur_bits     = bits;
  dev->cur_channels = channels;

  nxmutex_unlock(&dev->lock);
  return OK;
}

/****************************************************************************
 * Lower-half ops: start
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dmic_start(struct audio_lowerhalf_s *lower, void *session)
#else
static int dmic_start(struct audio_lowerhalf_s *lower)
#endif
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)lower;
  uint32_t reg;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->active)
    {
      ret = -EBUSY;
      goto err_unlock;
    }

  /* Apply default config if AUDIOIOC_CONFIGURE was skipped. */

  if (dev->cur_rate == 0)
    {
      dev->cur_rate     = T113_DMIC_DEFAULT_RATE;
      dev->cur_bits     = T113_DMIC_DEFAULT_BITS;
      dev->cur_channels = T113_DMIC_DEFAULT_CHANNELS;
    }

  /* 1. Mux PD19/PD20 to DMIC F4.  Idempotent and cheap so we redo it
   * on every start() instead of caching at initialise time -- this
   * recovers from any peripheral that may have stolen the pins
   * between stop() and start() of consecutive sessions.
   */

  dmic_pin_config();

  /* 2. Request PLL_AUDIO1(DIV5) -> 24.576 MHz module clock and open
   * the DMIC bus gate / reset.  Idempotent at the provider level.
   */

  ret = t113_audio_clk_request(T113_AUDIO_CONSUMER_DMIC, dev->cur_rate);
  if (ret < 0)
    {
      auderr("ERROR: t113_audio_clk_request failed: %d\n", ret);
      goto err_unlock;
    }

  /* 3. Program controller registers (channels, SR, OSR, FIFO mode,
   * trigger level, HPF).
   */

  dmic_program_controller(dev);

  /* 4. Reset xrun tracking and arm cyclic DMA.  Buffers were enqueued
   * by the upper half before start fired (see dmic_enqueuebuffer) so
   * the queue is non-empty when the first DMA completion arrives.
   */

  dev->xrun = false;

  ret = dmic_dma_start(dev);
  if (ret < 0)
    {
      auderr("ERROR: dmic_dma_start failed: %d\n", ret);
      goto err_clk;
    }

  /* 5. Enable DRQ then global controller enable.  Order matters: DMA
   * channel is armed before the DRQ goes live so no FIFO writes are
   * lost waiting for a not-yet-ready consumer.
   */

  reg  = getreg32(T113_DMIC_INTC);
  reg |= DMIC_INTC_RXFIFO_DRQ_EN;
  putreg32(reg, T113_DMIC_INTC);

  reg  = getreg32(T113_DMIC_EN);
  reg |= DMIC_EN_GLOBAL;
  putreg32(reg, T113_DMIC_EN);

  dev->active = true;
  nxmutex_unlock(&dev->lock);
  return OK;

err_clk:
  t113_audio_clk_release(T113_AUDIO_CONSUMER_DMIC);

err_unlock:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Lower-half ops: stop
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dmic_stop(struct audio_lowerhalf_s *lower, void *session)
#else
static int dmic_stop(struct audio_lowerhalf_s *lower)
#endif
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)lower;
  struct ap_buffer_s *apb;
  uint32_t reg;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!dev->active)
    {
      nxmutex_unlock(&dev->lock);
      return OK;
    }

  /* 1. Disable global enable + DRQ first so the FIFO stops requesting
   * and the controller drains its in-flight decimator state through
   * the now-quiet DMA channel.
   */

  reg  = getreg32(T113_DMIC_EN);
  reg &= ~DMIC_EN_GLOBAL;
  putreg32(reg, T113_DMIC_EN);

  reg  = getreg32(T113_DMIC_INTC);
  reg &= ~DMIC_INTC_RXFIFO_DRQ_EN;
  putreg32(reg, T113_DMIC_INTC);

  /* 2. Tear down DMA. */

  t113_dmastop(dev->dma);

  /* 3. Drain in-flight buffers back to the upper half (spec 6.4: stop
   * must not lose track of buffers).
   */

  flags = spin_lock_irqsave(&dev->qlock);
  while (!dq_empty(&dev->pendq))
    {
      apb = (struct ap_buffer_s *)dq_remfirst(&dev->pendq);
      spin_unlock_irqrestore(&dev->qlock, flags);

#ifdef CONFIG_AUDIO_MULTI_SESSION
      dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK,
                       NULL);
#else
      dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif
      flags = spin_lock_irqsave(&dev->qlock);
    }

  dev->xrun = false;
  spin_unlock_irqrestore(&dev->qlock, flags);

  /* 4. Release the audio clock.  Other consumers (codec ADC / DAC)
   * may still hold the PLL via their own refcount; the provider
   * decides whether to power down.
   */

  t113_audio_clk_release(T113_AUDIO_CONSUMER_DMIC);

  dev->active = false;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK,
                   NULL);
#else
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  nxmutex_unlock(&dev->lock);
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

/****************************************************************************
 * Lower-half ops: shutdown
 ****************************************************************************/

static int dmic_shutdown(struct audio_lowerhalf_s *lower)
{
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  /* Force the stream down regardless of state.  Same pattern the
   * codec uses; ensures pending buffers are drained when the
   * application closes /dev/audio0 mid-stream.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dmic_stop(lower, NULL);
#else
  dmic_stop(lower);
#endif
#else
  UNUSED(lower);
#endif
  return OK;
}

/****************************************************************************
 * Lower-half ops: allocbuffer / freebuffer
 *
 * Same pre-allocated contiguous-pool scheme as the codec.  Each apb
 * indexes into a successive slot of the pool so cyclic DMA sweeps the
 * whole region with no descriptor reprogramming.
 ****************************************************************************/

static int dmic_allocbuffer(struct audio_lowerhalf_s *lower,
                            struct audio_buf_desc_s *bufdesc)
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)lower;
  struct ap_buffer_s *apb;

  if (bufdesc->numbytes != dev->buf_size)
    {
      return -EINVAL;
    }

  if (dev->buf_alloc >= dev->buf_count)
    {
      return -ENOMEM;
    }

  apb = kumm_zalloc(sizeof(struct ap_buffer_s));
  if (apb == NULL)
    {
      return -ENOMEM;
    }

  apb->i.channels = dev->cur_channels ? dev->cur_channels : 1;
  apb->crefs      = 1;
  apb->nmaxbytes  = dev->buf_size;
  apb->samp       = dev->buf_pool + dev->buf_alloc * dev->buf_size;
  nxmutex_init(&apb->lock);

  dev->buf_alloc++;
  *bufdesc->u.pbuffer = apb;
  return sizeof(struct audio_buf_desc_s);
}

static int dmic_freebuffer(struct audio_lowerhalf_s *lower,
                           struct audio_buf_desc_s *bufdesc)
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)lower;
  struct ap_buffer_s *apb;

  apb = bufdesc->u.buffer;
  if (apb != NULL)
    {
      nxmutex_destroy(&apb->lock);
      kumm_free(apb);
    }

  if (dev->buf_alloc > 0)
    {
      dev->buf_alloc--;
    }

  return sizeof(struct audio_buf_desc_s);
}

/****************************************************************************
 * Lower-half ops: enqueuebuffer
 ****************************************************************************/

static int dmic_enqueuebuffer(struct audio_lowerhalf_s *lower,
                              struct ap_buffer_s *apb)
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)lower;
  irqstate_t flags;
  bool resume;

  /* Same pattern as the codec ADC: enqueue must accept buffers BEFORE
   * start (the upper half primes the queue first, then issues
   * AUDIOIOC_START which arms cyclic DMA over the already-queued
   * buffers).  Buffers enqueued while running resume DMA from xrun
   * via t113_dmaresume().
   */

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;

  flags = spin_lock_irqsave(&dev->qlock);
  dq_addlast(&apb->dq_entry, &dev->pendq);

  resume = dev->active && dev->xrun;
  if (resume)
    {
      dev->xrun = false;
    }

  spin_unlock_irqrestore(&dev->qlock, flags);

  if (resume)
    {
      /* DMA was paused waiting for buffer space.  Clear PAU so the
       * cyclic transfer resumes; the next IRQ will drain the queue.
       */

      t113_dmaresume(dev->dma);
    }

  return OK;
}

/****************************************************************************
 * Lower-half ops: ioctl
 ****************************************************************************/

static int dmic_ioctl(struct audio_lowerhalf_s *lower, int cmd,
                      unsigned long arg)
{
  struct t113_dmic_dev_s *dev = (struct t113_dmic_dev_s *)lower;
  struct ap_buffer_info_s *bufinfo;

  switch (cmd)
    {
      case AUDIOIOC_GETBUFFERINFO:
        bufinfo              = (struct ap_buffer_info_s *)arg;
        bufinfo->buffer_size = dev->buf_size;
        bufinfo->nbuffers    = dev->buf_count;
        return OK;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_dmic_initialize
 *
 * Description:
 *   Allocate the DMIC lowerhalf wrapper, the contiguous DMA buf_pool
 *   and the system DMA channel (DRQ_DMIC=8, RX direction).  Returns
 *   NULL on any allocation failure; the caller (board bringup)
 *   interprets that as "skip audio_register so /dev/audio0 does not
 *   appear" and the rest of bringup proceeds unaffected.
 *
 ****************************************************************************/

struct audio_lowerhalf_s *t113_dmic_initialize(void)
{
  struct t113_dmic_dev_s *dev;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      auderr("ERROR: dmic alloc dev failed\n");
      return NULL;
    }

  dev->lower.ops = &g_dmic_ops;
  dev->buf_size  = T113_DMIC_BUFFER_BYTES;
  dev->buf_count = T113_DMIC_BUFFER_COUNT;
  dev->buf_total = (size_t)dev->buf_size * dev->buf_count;

  nxmutex_init(&dev->lock);
  spin_lock_init(&dev->qlock);
  dq_init(&dev->pendq);

  /* Pre-allocate one cache-line-aligned contiguous DMA pool sized for
   * the whole ring.  Cyclic DMA sweeps this region indefinitely;
   * allocbuffer hands out apb_s pointing into successive slots.
   * Aligning to a cache line keeps up_invalidate_dcache from evicting
   * adjacent unrelated lines on completion.
   */

  dev->buf_pool = kumm_memalign(64, dev->buf_total);
  if (dev->buf_pool == NULL)
    {
      auderr("ERROR: dmic alloc buf_pool (%zu bytes) failed\n",
             dev->buf_total);
      goto err_lock;
    }

  /* Allocate the DMA channel.  DRQ direction is selected at
   * t113_dmasetup time by src/dst placement; the channel itself is
   * direction-agnostic (the DMA engine programs SRC_DRQ=DRQ_DMIC and
   * DST_DRQ=DRQ_DRAM in the per-channel CFG register at that point).
   */

  dev->dma = t113_dmachannel();
  if (dev->dma == NULL)
    {
      auderr("ERROR: dmic t113_dmachannel failed\n");
      goto err_pool;
    }

  return &dev->lower;

err_pool:
  kumm_free(dev->buf_pool);

err_lock:
  nxmutex_destroy(&dev->lock);
  kmm_free(dev);
  return NULL;
}

#endif /* CONFIG_T113_DMIC */
