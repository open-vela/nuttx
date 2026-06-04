/****************************************************************************
 * arch/arm/src/t113/t113_audio.c
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
 * T113-S3 internal audio codec driver.
 *
 * M1 stage: ADC capture path (/dev/audio1).
 *   - Routes the MICIN3P/N differential pair through ADC channel 3 PGA
 *     into the digital ADC FIFO and out via DRQ_CODEC RX DMA.
 *   - Manages mic-bias VRA1/VRA2 power-up via ALDO + MMICBIAS.
 *   - Hands DMA buffers to user space through the standard audio
 *     lowerhalf framework (allocbuffer/enqueuebuffer/dequeuebuffer).
 *
 * M2 stage: DAC playback path (/dev/audio2).
 *   - Drains DRAM samples to AC_DAC_TXDATA via DRQ_CODEC TX DMA, into
 *     the digital DAC FIFO -> stereo HPOUTL/R headphone driver.
 *   - Sequences HPLDO + HPOUT bias + ramp DAC + HP driver power-up so
 *     the analog output settles before any sample reaches the pad.
 *   - Software volume ramp (200 ms) at start and stop suppresses the
 *     pop a hard mute -> unmute would otherwise inject into the PA.
 *     The mq-r reference board has the PAM8301 PA's SD pin tied high
 *     (always on), so the volume ramp is the only pop-suppression
 *     mechanism on this board.
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
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>

#include "arm_internal.h"
#include "hardware/t113_audio.h"
#include "hardware/t113_dma.h"
#include "t113_clk.h"
#include "t113_dma.h"
#include "t113_audio.h"

#ifdef CONFIG_T113_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Default sample format (used until the application issues
 * AUDIOIOC_CONFIGURE).  Spec section 1: voice / KWS, 16 kHz mono 16-bit.
 */

#define T113_CODEC_DEFAULT_RATE      16000
#define T113_CODEC_DEFAULT_BITS      16
#define T113_CODEC_DEFAULT_CHANNELS  1

/* MIC bias settling time after MMICBIAS_EN goes high.  The user manual
 * does not specify a hard number; 50 ms is the lower bound used by the
 * mainline Linux sun20i-d1-codec driver and is consistent with the mic
 * bias capacitor (1 uF typical) charging through the on-chip current
 * source.
 */

#define T113_MIC_BIAS_SETTLE_US      50000

/* ADC PGA gain for the MICIN3P/N pair.  36 dB (max) is needed on the
 * mq-r EVB to bring conversational speech up from ~-44 dBFS (raw) to
 * around -32 dBFS, leaving usable signal margin for ASR.  An electret
 * element with -44 dB/Pa sensitivity at 70 dB SPL (~0.063 Pa, normal
 * conversation) outputs ~0.4 mV; PGA 36 dB raises that to ~25 mV which
 * lands near -32 dBFS against the codec's 1 Vrms full-scale.  Tunable
 * later via AUDIOIOC_CONFIGURE.
 */

#define T113_ADC_PGA_GAIN_DEFAULT    AC_ADC_PGA_GAIN_36DB

/* ADC RXFIFO trigger level: half-full (16 of 32 samples).  Larger values
 * lower the DMA wakeup rate at the cost of ADC->DDR latency.
 */

#define T113_ADC_RX_TRIG_LEVEL       0x10

/* Sigma-delta modulator convergence window.  When EN_AD goes high the SDM
 * integrators settle from their reset state through a damped ramp that
 * emits ~80 ms of bias-recovery transient before the output reaches the
 * correct DC operating point (visible on a clap-test FFT as a half-second
 * exponential decay starting near full-scale).  We let EN_AD run for
 * 100 ms with DRQ_EN held off so the transient pours into the FIFO, then
 * write FIFO_FLUSH a second time to discard it before arming the DMA.
 * 100 ms covers both the analog SDM settling (~80 ms) and a comfortable
 * margin for any residual digital-domain HPF pre-roll.
 */

#define T113_ADC_SDM_SETTLE_US       100000

/* DMA buffer pool size.  Each ap_buffer holds nbuffers samples; the
 * apb_s share a single contiguous physical region so the DMA engine can
 * loop through them in cyclic mode.
 *
 * The buffer count is pinned to 2 (ping-pong) regardless of
 * CONFIG_AUDIO_NUM_BUFFERS.  Reason: t113_dma.c arms a single
 * self-linking descriptor covering the full buf_total region and
 * enables only HLFDONE + PKGDONE in cyclic mode (see t113_dmastart()
 * irq_bits assembly).  That delivers exactly two callbacks per cycle.
 * codec_dma_callback() / codec_dma_dac_callback() each dequeue one apb
 * per IRQ, so any buffer count > 2 means DMA writes (or reads) more
 * apbs than the callback hands back to the upper half within a single
 * cycle, and the surplus apbs get silently overwritten on the next
 * pass.  Empirically this halves the captured byte count at
 * CONFIG_AUDIO_NUM_BUFFERS = 4 (8 s record -> 128 KiB instead of
 * 256 KiB).  Pinning to 2 keeps cyclic IRQs and apb deliveries 1:1.
 *
 * Tradeoff: the upper half can only hold 2 outstanding apbs, so user-
 * space stall tolerance drops to 1 buffer (~128 ms at 16 kHz mono /
 * 4 KiB buffer).  A future fix that allows N > 2 would require
 * extending t113_dma.c with a chained-descriptor API delivering one
 * per-descriptor IRQ.
 */

#define T113_CODEC_BUFFER_BYTES      CONFIG_AUDIO_BUFFER_NUMBYTES
#define T113_CODEC_BUFFER_COUNT      2

/* DAC TXFIFO trigger level: half-full (0x40 of 128 entries).  Keeps the
 * DRQ rate roughly equal to ADC's RX_TRIG so the DMA controller bounces
 * between identical workloads in the loopback case.
 */

#define T113_DAC_TX_TRIG_LEVEL       0x40

/* HPOUT volume ramp parameters (anti-pop at start / stop).  The mainline
 * Linux sun4i-codec ramp uses the hardware ramp DAC over ~256 ms; we
 * implement a software loop driving DAC_VOL_CTRL because the hardware
 * ramp engine is tied to LINEOUT (which T113 lacks -- only HPOUT exists).
 *
 * 0x00 (mute) to 0xa0 (0 dB) over 32 steps of 5 ms each = 160 ms ramp.
 * Total pop-suppression window including PA enable settling is ~200 ms.
 */

#define T113_HPOUT_RAMP_STEPS        32
#define T113_HPOUT_RAMP_STEP_US      5000
#define T113_HPOUT_VOL_TARGET        0xa0u  /* 0 dB unity gain */

/* HPLDO settling window after enable.  The on-chip 1.8 V LDO needs a few
 * milliseconds for its output cap to charge; powering the HP driver
 * before HPVCC is stable produces a bias-rail glitch on HPOUT.  10 ms
 * matches the conservative figure used by mainline H6 codec drivers.
 */

#define T113_HPLDO_SETTLE_US         10000

/* HP driver bias settling.  After enabling HP_DRVEN + HP_DRVOUTEN the
 * output stage needs the feedback loop to converge.  20 ms is the same
 * figure mainline sun4i-codec uses on the H616 LINEOUT path; the HPOUT
 * topology is closer to the V3s but the converge time is dominated by
 * the same off-chip AC-coupling caps.
 */

#define T113_HP_DRV_SETTLE_US        20000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Direction tag for codec instances. */

enum t113_codec_dir_e
{
  T113_CODEC_DIR_ADC = 0,
  T113_CODEC_DIR_DAC = 1,
};

/* Shared codec state.  ADC and DAC instances will both reference this
 * single struct so the analog block (LDO + bias + VRA) is brought up
 * exactly once per power transition.
 */

struct t113_codec_state_s
{
  mutex_t   lock;          /* Protects register access + state */
  bool      adc_active;    /* Capture stream is running */
  bool      dac_active;    /* Playback stream is running (M2) */
  uint32_t  cur_rate;      /* Active sample rate in Hz */
  uint8_t   cur_bits;      /* Active bit width (16 or 20) */
  uint8_t   cur_channels;  /* 1 = mono ADC3, 2 = stereo ADC1+ADC3 */
};

/* Per-direction lowerhalf instance. */

struct t113_codec_dev_s
{
  struct audio_lowerhalf_s lower;     /* Must be first */
  struct t113_codec_state_s *state;
  enum t113_codec_dir_e dir;

  DMA_HANDLE dma;
  uint8_t *buf_pool;                  /* Single contiguous DDR region */
  size_t buf_total;                   /* Total bytes (count * size) */
  size_t buf_size;                    /* Size of one apb */
  uint8_t buf_count;                  /* Number of apb in the pool */
  uint8_t buf_alloc;                  /* Index of next apb to hand out */
  uint8_t cb_index;                   /* Apb the next DMA IRQ will return */

  spinlock_t qlock;                   /* Protects pendq */
  struct dq_queue_s pendq;            /* Buffers queued by upper */
  bool xrun;                          /* DMA paused waiting for buffers */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  adc_configure(struct audio_lowerhalf_s *lower,
                          void *session,
                          const struct audio_caps_s *caps);
static int  adc_start(struct audio_lowerhalf_s *lower, void *session);
static int  adc_stop(struct audio_lowerhalf_s *lower, void *session);
static int  adc_reserve(struct audio_lowerhalf_s *lower, void **session);
static int  adc_release(struct audio_lowerhalf_s *lower, void *session);
#else
static int  adc_configure(struct audio_lowerhalf_s *lower,
                          const struct audio_caps_s *caps);
static int  adc_start(struct audio_lowerhalf_s *lower);
static int  adc_stop(struct audio_lowerhalf_s *lower);
static int  adc_reserve(struct audio_lowerhalf_s *lower);
static int  adc_release(struct audio_lowerhalf_s *lower);
#endif

static int  adc_getcaps(struct audio_lowerhalf_s *lower, int type,
                        struct audio_caps_s *caps);
static int  adc_shutdown(struct audio_lowerhalf_s *lower);
static int  adc_allocbuffer(struct audio_lowerhalf_s *lower,
                            struct audio_buf_desc_s *bufdesc);
static int  adc_freebuffer(struct audio_lowerhalf_s *lower,
                           struct audio_buf_desc_s *bufdesc);
static int  adc_enqueuebuffer(struct audio_lowerhalf_s *lower,
                              struct ap_buffer_s *apb);
static int  adc_ioctl(struct audio_lowerhalf_s *lower, int cmd,
                      unsigned long arg);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int  dac_configure(struct audio_lowerhalf_s *lower,
                          void *session,
                          const struct audio_caps_s *caps);
static int  dac_start(struct audio_lowerhalf_s *lower, void *session);
static int  dac_stop(struct audio_lowerhalf_s *lower, void *session);
static int  dac_reserve(struct audio_lowerhalf_s *lower, void **session);
static int  dac_release(struct audio_lowerhalf_s *lower, void *session);
#else
static int  dac_configure(struct audio_lowerhalf_s *lower,
                          const struct audio_caps_s *caps);
static int  dac_start(struct audio_lowerhalf_s *lower);
static int  dac_stop(struct audio_lowerhalf_s *lower);
static int  dac_reserve(struct audio_lowerhalf_s *lower);
static int  dac_release(struct audio_lowerhalf_s *lower);
#endif

static int  dac_getcaps(struct audio_lowerhalf_s *lower, int type,
                        struct audio_caps_s *caps);
static int  dac_shutdown(struct audio_lowerhalf_s *lower);
static int  dac_allocbuffer(struct audio_lowerhalf_s *lower,
                            struct audio_buf_desc_s *bufdesc);
static int  dac_freebuffer(struct audio_lowerhalf_s *lower,
                           struct audio_buf_desc_s *bufdesc);
static int  dac_enqueuebuffer(struct audio_lowerhalf_s *lower,
                              struct ap_buffer_s *apb);
static int  dac_ioctl(struct audio_lowerhalf_s *lower, int cmd,
                      unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct t113_codec_state_s g_codec_state =
{
  .lock = NXMUTEX_INITIALIZER,
};

static const struct audio_ops_s g_codec_adc_ops =
{
  .getcaps        = adc_getcaps,
  .configure      = adc_configure,
  .shutdown       = adc_shutdown,
  .start          = adc_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop           = adc_stop,
#endif
  .allocbuffer    = adc_allocbuffer,
  .freebuffer     = adc_freebuffer,
  .enqueuebuffer  = adc_enqueuebuffer,
  .ioctl          = adc_ioctl,
  .reserve        = adc_reserve,
  .release        = adc_release,
};

static const struct audio_ops_s g_codec_dac_ops =
{
  .getcaps        = dac_getcaps,
  .configure      = dac_configure,
  .shutdown       = dac_shutdown,
  .start          = dac_start,
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  .stop           = dac_stop,
#endif
  .allocbuffer    = dac_allocbuffer,
  .freebuffer     = dac_freebuffer,
  .enqueuebuffer  = dac_enqueuebuffer,
  .ioctl          = dac_ioctl,
  .reserve        = dac_reserve,
  .release        = dac_release,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: codec_validate_caps
 *
 * Description:
 *   Range-check the sample format requested through AUDIOIOC_CONFIGURE.
 *   Spec section 1 / 6.2: 8/16/32/48 kHz, 16 or 20 bit, mono or stereo.
 *
 ****************************************************************************/

static int codec_validate_caps(uint32_t rate, uint8_t bits, uint8_t channels)
{
  if (rate != 8000 && rate != 16000 && rate != 32000 && rate != 48000)
    {
      return -EINVAL;
    }

  if (bits != 16 && bits != 20)
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
 * Name: codec_rate_to_adfs
 *
 * Description:
 *   Translate a sample rate to the ADFS field of AC_ADC_FIFOC.
 *
 ****************************************************************************/

static uint32_t codec_rate_to_adfs(uint32_t rate)
{
  switch (rate)
    {
      case 48000: return AC_ADC_FIFOC_ADFS_48K;
      case 32000: return AC_ADC_FIFOC_ADFS_32K;
      case 16000: return AC_ADC_FIFOC_ADFS_16K;
      case  8000: return AC_ADC_FIFOC_ADFS_8K;
      default:    return AC_ADC_FIFOC_ADFS_16K;
    }
}

/****************************************************************************
 * Name: codec_adc_power_up
 *
 * Description:
 *   Bring up the analog blocks needed for ADC capture.  Caller must hold
 *   g_codec_state.lock.  Sequence (matching mainline Linux sun20i-d1-
 *   codec):
 *
 *     1. Ensure ALDO and HPLDO are enabled in POWER_REG (the analog 1.8V
 *        rails feeding the ADC PGA and bias network).
 *     2. Ensure VRA1 speedup-down has converged (auto-clears 32 ms after
 *        bus reset release; we double-check the status bit).
 *     3. Enable the master mic bias at 2.09 V and wait for settling.
 *     4. Enable the ADC3 channel + MIC3 PGA in differential mode (single-
 *        end disabled) and program 24 dB gain.
 *     5. Optionally enable ADC1 for stereo mode (currently unused; mono
 *        is the default).
 *
 ****************************************************************************/

static int codec_adc_power_up(struct t113_codec_state_s *state)
{
  uint32_t reg;
  int retry;

  /* Step 1: power rails.  ALDO + HPLDO are gated by the system bus reset
   * (not the codec block reset) and are typically already on at chip
   * power-up.  Force them so the driver works under any boot order.
   */

  reg  = getreg32(T113_AC_POWER_REG);
  reg |= AC_POWER_REG_ALDO_EN;
  reg &= ~AC_POWER_REG_ALDO_VOL_MASK;
  reg |= AC_POWER_REG_ALDO_VOL_180V;
  putreg32(reg, T113_AC_POWER_REG);

  /* Step 2: wait for VRA1 speedup-down to settle (state bit clears once
   * the 32 ms internal timer expires).  Bound the wait so a hardware
   * fault cannot stall start() forever; 100 ms is well above the
   * 32 ms hardware spec.
   */

  for (retry = 0; retry < 100; retry++)
    {
      if ((getreg32(T113_AC_VRA1_SPEEDUP_CTRL) &
           AC_VRA1_SPEEDUP_DOWN_STATE) == 0)
        {
          break;
        }

      nxsig_usleep(1000);
    }

  /* Step 3: master mic bias.  The MICIN3P/N pair is fed by MMICBIAS via
   * the on-board RC network; HMICBIAS is for the headphone-jack mic and
   * stays off here.  Use 2.09 V which is the typical electret rated
   * voltage.
   */

  reg  = getreg32(T113_AC_MICBIAS_REG);
  reg &= ~AC_MICBIAS_MBIASSEL_MASK;
  reg |= AC_MICBIAS_MBIASSEL_2V09;
  reg |= AC_MICBIAS_MMICBIASEN;
  putreg32(reg, T113_AC_MICBIAS_REG);

  /* Wait for the bias capacitor to charge.  Mic bias uses VDD_AVCC -> RC
   * network -> mic; with a 1 uF cap a 50 ms settling time is the typical
   * lower bound used by mainline drivers (sun20i-d1-codec).  Without
   * this delay the first ~10 ms of capture clip the bias-rise transient.
   */

  nxsig_usleep(T113_MIC_BIAS_SETTLE_US);

  /* Step 4: enable ADC3 + MIC3 PGA in differential mode.  Clear
   * MIC3_SIN_EN so the channel uses MICIN3P/N as a true differential
   * pair (which is how the t113-evb mic flying-lead is wired).
   *
   * Push IOPAAF / IOPSDM1 / IOPSDM2 / IOPMIC to 0x3 (2.25 x IOPADC,
   * UM 8.4.6.114, p843).  The reset default (0x1, 1.75 x) gives the
   * analog AAF and sigma-delta integrators just enough gain-bandwidth
   * to flatly pass ~5 kHz at the 16 kHz capture rate; clap-test FFT
   * shows a strong roll-off above 5 kHz that disappears as the bias
   * is raised.  +28 % bias widens each op-amp's GBW by the same
   * factor, restoring flat response toward the digital decimator's
   * passband (~6.4 kHz) without affecting the master IOPADC current
   * (already at 4 uA = 0x3 in ADC1_REG[15:14]) or the per-channel DC
   * offset.
   */

  reg  = getreg32(T113_AC_ADC3_REG);
  reg &= ~(AC_ADC3_REG_PGA_GAIN_MASK | AC_ADC3_REG_MIC3_SIN_EN |
           AC_ADC3_REG_IOPAAF_MASK | AC_ADC3_REG_IOPSDM1_MASK |
           AC_ADC3_REG_IOPSDM2_MASK | AC_ADC3_REG_IOPMIC_MASK);
  reg |= ((uint32_t)T113_ADC_PGA_GAIN_DEFAULT)
         << AC_ADC3_REG_PGA_GAIN_SHIFT;
  reg |= AC_ADC3_REG_IOP_MAX << AC_ADC3_REG_IOPAAF_SHIFT;
  reg |= AC_ADC3_REG_IOP_MAX << AC_ADC3_REG_IOPSDM1_SHIFT;
  reg |= AC_ADC3_REG_IOP_MAX << AC_ADC3_REG_IOPSDM2_SHIFT;
  reg |= AC_ADC3_REG_IOP_MAX << AC_ADC3_REG_IOPMIC_SHIFT;
  reg |= AC_ADC3_REG_MIC3_PGA_EN;
  reg |= AC_ADC3_REG_ADC3_EN;
  putreg32(reg, T113_AC_ADC3_REG);

  /* Step 5: stereo mode brings up ADC1 (line-in left / second mic).  For
   * the t113-evb only one analog mic is wired so cur_channels == 1 is
   * the expected path.  Stereo support is kept for future hardware.
   */

  if (state->cur_channels == 2)
    {
      reg  = getreg32(T113_AC_ADC1_REG);
      reg &= ~AC_ADC1_REG_PGA_GAIN_MASK;
      reg |= ((uint32_t)T113_ADC_PGA_GAIN_DEFAULT)
             << AC_ADC1_REG_PGA_GAIN_SHIFT;
      reg |= AC_ADC1_REG_ADC1_EN;
      putreg32(reg, T113_AC_ADC1_REG);
    }

  /* Step 6: enable the digital HPF (DC blocker) in the ADC DAP.  The HPF
   * cutoff is fixed at < 1 Hz (UM 8.4.6.17, p778 "HPF Function") and is
   * the canonical fix for the DC offset that otherwise rides on every
   * captured sample -- the analog PGA + bias network injects a non-zero
   * common-mode level which the FIFO would faithfully digitise without
   * this filter.  Each HPF lives behind a DAP master gate, so we set the
   * DAP enable + HPF enable in a single RMW: DAP1/HPF1 for ADC3 (mono),
   * plus DAP0/HPF0 for ADC1 in stereo mode.  DRC bits stay 0 (bypass).
   */

  reg  = getreg32(T113_AC_ADC_DAP_CTRL);
  reg |= AC_ADC_DAP_CTR_DAP1_EN | AC_ADC_DAP_CTR_HPF1_EN;
  if (state->cur_channels == 2)
    {
      reg |= AC_ADC_DAP_CTR_DAP0_EN | AC_ADC_DAP_CTR_HPF0_EN;
    }

  putreg32(reg, T113_AC_ADC_DAP_CTRL);

  return OK;
}

/****************************************************************************
 * Name: codec_adc_power_down
 *
 * Description:
 *   Reverse of codec_adc_power_up.  Caller must hold g_codec_state.lock.
 *   Leaves ALDO and HPLDO on because the DAC path may still need them
 *   (M2 will refcount these).
 *
 ****************************************************************************/

static void codec_adc_power_down(struct t113_codec_state_s *state)
{
  uint32_t reg;

  /* Mirror codec_adc_power_up step 6: drop the DAP master enables + HPF
   * bits we set on the way up.  Bringing the gates back to 0 leaves the
   * register at its post-reset all-zero state so the next power-up RMW
   * sees a clean slate.  Done first so the HPF stops draining samples
   * before we mute the analog channel.
   */

  reg  = getreg32(T113_AC_ADC_DAP_CTRL);
  reg &= ~(AC_ADC_DAP_CTR_DAP1_EN | AC_ADC_DAP_CTR_HPF1_EN);
  if (state->cur_channels == 2)
    {
      reg &= ~(AC_ADC_DAP_CTR_DAP0_EN | AC_ADC_DAP_CTR_HPF0_EN);
    }

  putreg32(reg, T113_AC_ADC_DAP_CTRL);

  reg  = getreg32(T113_AC_ADC3_REG);
  reg &= ~(AC_ADC3_REG_ADC3_EN | AC_ADC3_REG_MIC3_PGA_EN);
  putreg32(reg, T113_AC_ADC3_REG);

  if (state->cur_channels == 2)
    {
      reg  = getreg32(T113_AC_ADC1_REG);
      reg &= ~AC_ADC1_REG_ADC1_EN;
      putreg32(reg, T113_AC_ADC1_REG);
    }

  /* Drop MMICBIAS only when the DAC path is also idle; spec section 6.4
   * requires shutdown to fully unwind ADC resources.  When DAC starts up
   * in M2 it will re-enable bias as needed.
   */

  if (!state->dac_active)
    {
      reg  = getreg32(T113_AC_MICBIAS_REG);
      reg &= ~AC_MICBIAS_MMICBIASEN;
      putreg32(reg, T113_AC_MICBIAS_REG);
    }
}

/****************************************************************************
 * Name: codec_adc_program_fifo
 *
 * Description:
 *   Program the digital ADC FIFO for the current sample format and route
 *   ADC3 (and optionally ADC1) into it.  Caller must hold the state
 *   lock.
 *
 ****************************************************************************/

static void codec_adc_program_fifo(struct t113_codec_state_s *state)
{
  uint32_t reg;
  uint32_t chan_en;

  /* Channel enable: bit 0=ADC1, bit 1=ADC2, bit 2=ADC3 */

  chan_en = AC_ADC_DIG_CHAN_EN_ADC3;
  if (state->cur_channels == 2)
    {
      chan_en |= AC_ADC_DIG_CHAN_EN_ADC1;
    }

  reg  = getreg32(T113_AC_ADC_DIG_CTRL);
  reg &= ~AC_ADC_DIG_CHAN_EN_MASK;
  reg |= (chan_en << AC_ADC_DIG_CHAN_EN_SHIFT);
  reg |= AC_ADC_DIG_ADC3_VOL_EN;        /* enable digital volume on ADC3 */
  if (state->cur_channels == 2)
    {
      reg |= AC_ADC_DIG_ADC1_2_VOL_EN;  /* + ADC1 for stereo */
    }

  putreg32(reg, T113_AC_ADC_DIG_CTRL);

  /* Digital volume: ADC_VOL_CTRL1 has 4 bytes, one per channel.  Reset
   * default 0xA0 = 0 dB; each step is 0.5 dB; 0xB8 = +12 dB.  Combined
   * with the analog PGA's +36 dB this gives ~+48 dB total on the path
   * from MICIN3P/N to FIFO.  Apply the same gain to the ADC1 byte too
   * so stereo capture is balanced when the second mic is wired.
   */

  putreg32(0xb8b8b8b8u, T113_AC_ADC_VOL_CTRL1);

  /* FIFO control: sample rate, bit width, RX trigger level, DRQ.  Do not
   * set EN_AD here -- that is done last in adc_start() after the DMA
   * descriptor is armed so the first samples don't arrive before the
   * DMA channel is ready.
   *
   * RX_FIFO_MODE = 1 (Mode 1) places the audio sample in the lower bits
   * of the 32-bit RXDATA word with sign-extension in the upper bits
   * (per UM 8.4.6.8).  Combined with a 16-bit DMA destination width in
   * codec_dma_start() this yields one 16-bit sample per FIFO read --
   * i.e. exactly two bytes per sample in DRAM, matching what nxrecorder
   * expects for S16_LE.  Without this bit the FIFO emits the sample in
   * bits 31:16 with zero LSBs and a 32-bit DMA read produces 4 bytes
   * per sample (high half = data, low half = 0).
   */

  reg  = codec_rate_to_adfs(state->cur_rate);
  reg |= ((uint32_t)T113_ADC_RX_TRIG_LEVEL) << AC_ADC_FIFOC_RX_TRIG_SHIFT;
  reg |= AC_ADC_FIFOC_RX_FIFO_MODE;
  if (state->cur_bits == 20)
    {
      reg |= AC_ADC_FIFOC_RX_BITS_20;
    }

  reg |= AC_ADC_FIFOC_FIFO_FLUSH;       /* Self-clearing */
  putreg32(reg, T113_AC_ADC_FIFOC);
}

/****************************************************************************
 * Name: codec_dma_callback
 *
 * Description:
 *   DMA completion callback.  Called from IRQ context.  In cyclic mode
 *   the T113 DMA driver fires us at the half-buffer (HLFDONE) and full-
 *   buffer (PKGDONE) boundaries: each invocation corresponds to one apb
 *   completing.  We dequeue the apb that just finished, hand it back to
 *   the upper half, and pause the channel if no further apb is queued
 *   (tracked via state->xrun).
 *
 ****************************************************************************/

static void codec_dma_callback(DMA_HANDLE handle, uint8_t status,
                               void *arg)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)arg;
  struct ap_buffer_s *apb;
  irqstate_t flags;

  UNUSED(handle);
  UNUSED(status);

  flags = spin_lock_irqsave(&dev->qlock);
  apb   = (struct ap_buffer_s *)dq_remfirst(&dev->pendq);
  spin_unlock_irqrestore(&dev->qlock, flags);

  if (apb == NULL)
    {
      /* xrun: DMA finished a buffer but the user has not enqueued a new
       * one.  Pause the channel; resume happens in enqueuebuffer.
       */

      t113_dmapause(dev->dma);
      dev->xrun = true;
      return;
    }

  /* Mark the buffer as having a full apb of capture data and invalidate
   * the cache so the application sees fresh DRAM contents.
   */

  apb->nbytes = apb->nmaxbytes;
  up_invalidate_dcache((uintptr_t)apb->samp,
                       (uintptr_t)apb->samp + apb->nbytes);

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK, NULL);
#else
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);
#endif

  /* If the queue drains, pause and flag xrun so enqueuebuffer can resume
   * cleanly when the application provides another buffer.
   */

  flags = spin_lock_irqsave(&dev->qlock);
  if (dq_empty(&dev->pendq))
    {
      t113_dmapause(dev->dma);
      dev->xrun = true;
    }

  spin_unlock_irqrestore(&dev->qlock, flags);
}

/****************************************************************************
 * Name: codec_dma_start
 *
 * Description:
 *   Arm the cyclic DMA descriptor for ADC RX.  Direction is peripheral
 *   (codec ADC RXDATA register) to memory (the contiguous buf_pool).
 *
 ****************************************************************************/

static int codec_dma_start(struct t113_codec_dev_s *dev)
{
  struct t113_dma_config_s cfg;
  int ret;

  memset(&cfg, 0, sizeof(cfg));

  /* Source: codec ADC RXDATA register, IO addressing.  The codec FIFO
   * register is physically 32 bits wide, but with RX_FIFO_MODE = 1 the
   * audio sample sits in bits 15:0 (sign-extended into 31:16 for
   * 16-bit, or in bits 19:0 for 20-bit).  We use a 16-bit DMA read so
   * the controller fetches just the data half of each FIFO word --
   * matching how Linux mainline sun4i-codec handles S16_LE capture.
   */

  cfg.src_drq    = DRQ_CODEC;
  cfg.src_width  = DMAC_WIDTH_16BIT;
  cfg.src_burst  = DMAC_BURST_4;
  cfg.src_linear = false;

  /* Destination: linear DRAM, 16-bit width to keep one sample = 2 bytes.
   * 16-bit is the canonical width advertised through ac_controls.b[2]
   * in adc_getcaps().  20-bit capture is accepted by validate_caps()
   * but at this DMA width only the lower 16 bits of each FIFO word
   * reach DRAM, which loses the bottom 4 bits of precision.  A real
   * 20-bit path would need a 32-bit dst_width and an upper-half
   * conversion -- out of scope for M1 (PCM_S16_LE only).
   */

  cfg.dst_drq    = DRQ_DRAM;
  cfg.dst_width  = DMAC_WIDTH_16BIT;
  cfg.dst_burst  = DMAC_BURST_4;
  cfg.dst_linear = true;

  cfg.mode       = DMAC_MODE_SRC_HANDSHAKE;
  cfg.circular   = true;

  ret = t113_dmasetup(dev->dma,
                      (uintptr_t)T113_AC_ADC_RXDATA,
                      (uintptr_t)dev->buf_pool,
                      dev->buf_total, &cfg);
  if (ret < 0)
    {
      return ret;
    }

  /* Make sure no stale data sits in the cache for the buffer region. */

  up_invalidate_dcache((uintptr_t)dev->buf_pool,
                       (uintptr_t)dev->buf_pool + dev->buf_total);

  return t113_dmastart(dev->dma, codec_dma_callback, dev);
}

/****************************************************************************
 * Lower-half ops: reserve / release
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int adc_reserve(struct audio_lowerhalf_s *lower, void **session)
#else
static int adc_reserve(struct audio_lowerhalf_s *lower)
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
static int adc_release(struct audio_lowerhalf_s *lower, void *session)
#else
static int adc_release(struct audio_lowerhalf_s *lower)
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

static int adc_getcaps(struct audio_lowerhalf_s *lower, int type,
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

            /* b[2] reports the primary supported sample width in the
             * same literal-integer form expected by AUDIOIOC_CONFIGURE
             * (see codec_validate_caps).  The codec also accepts 20-bit
             * but 16-bit is the canonical / default width and most
             * helpers (nxrecorder) only consume a single value.
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
static int adc_configure(struct audio_lowerhalf_s *lower,
                         void *session,
                         const struct audio_caps_s *caps)
#else
static int adc_configure(struct audio_lowerhalf_s *lower,
                         const struct audio_caps_s *caps)
#endif
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  struct t113_codec_state_s *state = dev->state;
  uint32_t rate;
  uint8_t  bits;
  uint8_t  channels;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  DEBUGASSERT(caps != NULL);

  /* Stub-acknowledge AUDIO_TYPE_FEATURE (volume / bass / treble)
   * the same way dac_configure does, in case future nxrecorder
   * versions issue setvolume on the capture path.
   */

  if (caps->ac_type == AUDIO_TYPE_FEATURE)
    {
      return OK;
    }

  if (caps->ac_type != AUDIO_TYPE_INPUT)
    {
      return -EINVAL;
    }

  /* AUDIO_FMT_PCM uses ac_controls.hw[0]=samplerate, b[2]=bits,
   * ac_channels=channel count.  See nxrecorder for the canonical
   * usage.
   */

  rate     = caps->ac_controls.hw[0];
  bits     = caps->ac_controls.b[2];
  channels = caps->ac_channels;

  ret = codec_validate_caps(rate, bits, channels);
  if (ret < 0)
    {
      auderr("ERROR: invalid caps rate=%u bits=%u ch=%u\n",
             (unsigned)rate, (unsigned)bits, (unsigned)channels);
      return ret;
    }

  ret = nxmutex_lock(&state->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Spec section 6.2: configure while a stream is running is reserved
   * for hot-reconfigure (not in scope for M1).  Reject with -EBUSY.
   */

  if (state->adc_active)
    {
      nxmutex_unlock(&state->lock);
      return -EBUSY;
    }

  state->cur_rate     = rate;
  state->cur_bits     = bits;
  state->cur_channels = channels;

  nxmutex_unlock(&state->lock);
  return OK;
}

/****************************************************************************
 * Lower-half ops: start
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int adc_start(struct audio_lowerhalf_s *lower, void *session)
#else
static int adc_start(struct audio_lowerhalf_s *lower)
#endif
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  struct t113_codec_state_s *state = dev->state;
  uint32_t reg;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  ret = nxmutex_lock(&state->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (state->adc_active)
    {
      ret = -EBUSY;
      goto err_unlock;
    }

  /* Apply default config if the application skipped AUDIOIOC_CONFIGURE. */

  if (state->cur_rate == 0)
    {
      state->cur_rate     = T113_CODEC_DEFAULT_RATE;
      state->cur_bits     = T113_CODEC_DEFAULT_BITS;
      state->cur_channels = T113_CODEC_DEFAULT_CHANNELS;
    }

  /* 1. Request PLL_AUDIO0 + program the ADC clock divider. */

  ret = t113_audio_clk_request(T113_AUDIO_CONSUMER_CODEC_ADC,
                               state->cur_rate);
  if (ret < 0)
    {
      auderr("ERROR: t113_audio_clk_request failed: %d\n", ret);
      goto err_unlock;
    }

  /* 2. Bring up the analog block (ALDO + bias + ADC3 PGA). */

  ret = codec_adc_power_up(state);
  if (ret < 0)
    {
      goto err_clk;
    }

  /* 3. Program the digital ADC FIFO (sample rate, channels, etc.). */

  codec_adc_program_fifo(state);

  /* 4. Reset the buffer queue tracking and arm cyclic DMA.  The buffers
   * themselves were enqueued by the upper half via adc_enqueuebuffer
   * before start fired.  DMA stays parked because DRQ_EN is still 0 in
   * AC_ADC_FIFOC; the controller will only fire bursts after DRQ_EN
   * goes high in step 6 below.
   */

  dev->xrun = false;

  ret = codec_dma_start(dev);
  if (ret < 0)
    {
      auderr("ERROR: codec_dma_start failed: %d\n", ret);
      goto err_pwr;
    }

  /* 5. Enable the digital ADC (EN_AD) without DRQ_EN so the sigma-delta
   * modulator starts converging.  The first ~80 ms of FIFO writes carry
   * the SDM bias-recovery transient (visible as a -0.4 dBFS spike at the
   * start of every capture); we let those drain into the FIFO and discard
   * them in step 6 instead of letting the DMA stream them to userspace.
   * Holding DRQ_EN low while EN_AD runs guarantees no FIFO write reaches
   * DRAM during this window -- the FIFO simply fills, then overflows
   * silently (RXO_INT bit asserts but no IRQ fires; we clear it later via
   * the FIFO_FLUSH).  This is cheaper than software-discarding the first
   * apb at every start and works regardless of buffer size or rate.
   */

  reg  = getreg32(T113_AC_ADC_FIFOC);
  reg |= AC_ADC_FIFOC_EN_AD;
  putreg32(reg, T113_AC_ADC_FIFOC);

  nxsig_usleep(T113_ADC_SDM_SETTLE_US);

  /* 6. Discard the SDM convergence transient by re-asserting FIFO_FLUSH,
   * then enable the DRQ so DMA starts pulling clean post-convergence
   * samples.  FIFO_FLUSH is self-clearing so we OR it into the same
   * write that sets DRQ_EN -- the order inside that 32-bit write is
   * undefined but both effects are captured in a single APB cycle.
   */

  reg  = getreg32(T113_AC_ADC_FIFOC);
  reg |= AC_ADC_FIFOC_FIFO_FLUSH | AC_ADC_FIFOC_ADC_DRQ_EN;
  putreg32(reg, T113_AC_ADC_FIFOC);

  state->adc_active = true;
  nxmutex_unlock(&state->lock);
  return OK;

err_pwr:
  codec_adc_power_down(state);

err_clk:
  t113_audio_clk_release(T113_AUDIO_CONSUMER_CODEC_ADC);

err_unlock:
  nxmutex_unlock(&state->lock);
  return ret;
}

/****************************************************************************
 * Lower-half ops: stop
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int adc_stop(struct audio_lowerhalf_s *lower, void *session)
#else
static int adc_stop(struct audio_lowerhalf_s *lower)
#endif
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  struct t113_codec_state_s *state = dev->state;
  struct ap_buffer_s *apb;
  uint32_t reg;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  ret = nxmutex_lock(&state->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!state->adc_active)
    {
      nxmutex_unlock(&state->lock);
      return OK;
    }

  /* 1. Disable digital ADC + DRQ first so the FIFO stops requesting. */

  reg  = getreg32(T113_AC_ADC_FIFOC);
  reg &= ~(AC_ADC_FIFOC_EN_AD | AC_ADC_FIFOC_ADC_DRQ_EN);
  putreg32(reg, T113_AC_ADC_FIFOC);

  /* 2. Tear down the DMA channel. */

  t113_dmastop(dev->dma);

  /* 3. Drain any in-flight buffers back to the upper half so user space
   * does not lose track of them (spec section 6.4).
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

  /* 4. Power down the analog blocks then release the audio clock. */

  codec_adc_power_down(state);
  t113_audio_clk_release(T113_AUDIO_CONSUMER_CODEC_ADC);

  state->adc_active = false;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK,
                   NULL);
#else
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  nxmutex_unlock(&state->lock);
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

/****************************************************************************
 * Lower-half ops: shutdown
 ****************************************************************************/

static int adc_shutdown(struct audio_lowerhalf_s *lower)
{
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
  /* Force the stream down regardless of state.  audio_dma.c uses the
   * same pattern; this ensures pending buffers are drained even when
   * the application closes /dev/audio1 mid-stream.
   */

#ifdef CONFIG_AUDIO_MULTI_SESSION
  adc_stop(lower, NULL);
#else
  adc_stop(lower);
#endif
#else
  UNUSED(lower);
#endif
  return OK;
}

/****************************************************************************
 * Lower-half ops: allocbuffer / freebuffer
 *
 * The codec needs DMA-coherent contiguous memory.  We pre-allocate one
 * pool at init time covering buf_count * buf_size bytes; allocbuffer
 * just hands out apb_s that point into successive slots of the pool.
 ****************************************************************************/

static int adc_allocbuffer(struct audio_lowerhalf_s *lower,
                           struct audio_buf_desc_s *bufdesc)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
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

  apb->i.channels = dev->state->cur_channels ?
                    dev->state->cur_channels : 1;
  apb->crefs      = 1;
  apb->nmaxbytes  = dev->buf_size;
  apb->samp       = dev->buf_pool + dev->buf_alloc * dev->buf_size;
  nxmutex_init(&apb->lock);

  dev->buf_alloc++;
  *bufdesc->u.pbuffer = apb;
  return sizeof(struct audio_buf_desc_s);
}

static int adc_freebuffer(struct audio_lowerhalf_s *lower,
                          struct audio_buf_desc_s *bufdesc)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
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

static int adc_enqueuebuffer(struct audio_lowerhalf_s *lower,
                             struct ap_buffer_s *apb)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  irqstate_t flags;
  bool resume;

  /* enqueuebuffer must accept buffers BEFORE start (the upper half primes
   * the queue first, then issues AUDIOIOC_START which arms cyclic DMA
   * over the already-queued buffers - see adc_start step 4).  Buffers
   * enqueued while running resume DMA from xrun via t113_dmaresume().
   */

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;

  flags = spin_lock_irqsave(&dev->qlock);
  dq_addlast(&apb->dq_entry, &dev->pendq);

  resume = dev->state->adc_active && dev->xrun;
  if (resume)
    {
      dev->xrun = false;
    }

  spin_unlock_irqrestore(&dev->qlock, flags);

  if (resume)
    {
      /* DMA was paused waiting for buffer space.  Clear PAU so the
       * cyclic transfer resumes; the next IRQ will drain through the
       * queue again.  Note: t113_dmastart() would NOT clear the PAU
       * bit, so we must call t113_dmaresume() here.
       */

      t113_dmaresume(dev->dma);
    }

  return OK;
}

/****************************************************************************
 * Lower-half ops: ioctl
 ****************************************************************************/

static int adc_ioctl(struct audio_lowerhalf_s *lower, int cmd,
                     unsigned long arg)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
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
 * DAC playback path (/dev/audio2)
 *
 * The DAC drives a 16-bit stereo stream from DRAM through the digital
 * DAC FIFO into the analog HPOUT block.  Data flow mirrors the ADC
 * path with directions reversed: DMA reads samples from a contiguous
 * buf_pool and writes them word by word into AC_DAC_TXDATA.
 *
 * Power sequencing is the load-bearing detail.  The DAC must be
 * brought up MUTED (digital volume = 0) to avoid the bias-rise step
 * appearing on HPOUT, then the volume is ramped up over ~160 ms in
 * software.  Only after the unmute ramp completes does the board
 * hook pull the external PA enable line high.  Stop reverses this:
 * PA EN low first, then the mute ramp, then power-down.  This keeps
 * the audible PA window strictly outside any voltage step.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: codec_rate_to_dac_fs
 *
 * Description:
 *   Translate a sample rate to the DAC_FS field of AC_DAC_FIFOC.  The
 *   user manual encoding for DAC differs from ADC -- 24 kHz is index 2
 *   on DAC vs index 2 on ADC, but the high-rate slots (192 kHz at 110)
 *   are DAC-only.  We accept the same set the ADC path validates.
 *
 ****************************************************************************/

static uint32_t codec_rate_to_dac_fs(uint32_t rate)
{
  switch (rate)
    {
      case 48000: return AC_DAC_FIFOC_DAC_FS_48K;
      case 32000: return AC_DAC_FIFOC_DAC_FS_32K;
      case 16000: return AC_DAC_FIFOC_DAC_FS_16K;
      case  8000: return AC_DAC_FIFOC_DAC_FS_8K;
      default:    return AC_DAC_FIFOC_DAC_FS_16K;
    }
}

/****************************************************************************
 * Name: codec_hpout_set_volume
 *
 * Description:
 *   Program AC_DAC_VOL_CTRL with a fine-step volume value applied to
 *   both L and R channels.  Caller must hold the state lock.  vol = 0
 *   is mute; vol = 0xa0 is 0 dB unity gain.
 *
 ****************************************************************************/

static void codec_hpout_set_volume(uint8_t vol)
{
  uint32_t reg;

  reg  = AC_DAC_VOL_SEL;
  reg |= ((uint32_t)vol << AC_DAC_VOL_L_SHIFT);
  reg |= ((uint32_t)vol << AC_DAC_VOL_R_SHIFT);
  putreg32(reg, T113_AC_DAC_VOL_CTRL);
}

/****************************************************************************
 * Name: codec_hpout_volume_ramp
 *
 * Description:
 *   Software volume ramp from `from` to `to` over T113_HPOUT_RAMP_STEPS
 *   stages.  Each stage applies one linear interpolation step then
 *   sleeps T113_HPOUT_RAMP_STEP_US -- total ~160 ms.  Caller must hold
 *   the state lock so a concurrent stop / shutdown cannot race the
 *   intermediate volume values.
 *
 *   We use a software loop instead of the hardware ramp DAC because
 *   the HW ramp engine is wired to LINEOUT (RAMP_REG.RD_EN gates the
 *   ramp output switch into LINEOUT), and the T113-S3 has no LINEOUT
 *   pad.  A digital-domain ramp on AC_DAC_VOL_CTRL achieves the same
 *   pop-suppression effect: each 0.75 dB step is below the speaker /
 *   PA's audible click threshold.
 *
 ****************************************************************************/

static void codec_hpout_volume_ramp(uint8_t from, uint8_t to)
{
  int delta = (int)to - (int)from;
  int step;
  uint8_t vol;

  for (step = 1; step <= T113_HPOUT_RAMP_STEPS; step++)
    {
      vol = (uint8_t)((int)from + (delta * step) / T113_HPOUT_RAMP_STEPS);
      codec_hpout_set_volume(vol);
      nxsig_usleep(T113_HPOUT_RAMP_STEP_US);
    }

  /* Final write guarantees the exact target value despite rounding. */

  codec_hpout_set_volume(to);
}

/****************************************************************************
 * Name: codec_dac_power_up
 *
 * Description:
 *   Bring up the analog blocks needed for HPOUT playback.  Caller must
 *   hold g_codec_state.lock.  The output stays at digital mute (volume
 *   0x00) until codec_hpout_volume_ramp() is invoked from dac_start.
 *
 *   Sequence (mirroring the recommended power-up in UM section 8.4.3.6
 *   + 8.4.3.8 and the H616 mainline driver):
 *     1. ALDO (analog 1.8V) -- shared with ADC; idempotent.
 *     2. HPLDO (HP analog 1.8V supply for HPVCC) + settle 10 ms.
 *     3. Headphone bias / OP / driver in HP2_REG: HPFB_BUF_EN,
 *        HP_DRVEN, HP_DRVOUTEN, ramp DAC enable, HPFB_IN_EN,
 *        RAMP_OUT_EN.  These are the analog gain stage; enabling them
 *        before the digital DAC primes the bias network so the DC
 *        operating point is settled by the time samples flow.
 *     4. Settle 20 ms for the HP feedback loop to converge.
 *     5. DACL_EN + DACR_EN in DAC_REG (analog DAC).
 *
 *   At this point the analog HPOUT chain is live but the DAC FIFO is
 *   empty / the digital domain is muted, so HPOUT sits at its DC
 *   common-mode voltage.  dac_start arms DMA, then ramps the digital
 *   volume up so the first audible sample is at near-DC.
 *
 ****************************************************************************/

static void codec_dac_power_up(struct t113_codec_state_s *state)
{
  uint32_t reg;

  UNUSED(state);

  audinfo("DAC power-up: enter (POWER=0x%08lx HP2=0x%08lx DAC=0x%08lx)\n",
          (unsigned long)getreg32(T113_AC_POWER_REG),
          (unsigned long)getreg32(T113_AC_HP2_REG),
          (unsigned long)getreg32(T113_AC_DAC_REG));

  /* Step 1: ALDO.  Already on if ADC is running; force-enable here so
   * a DAC-only consumer also gets a guaranteed analog rail.  HUB_EN
   * and the DAC digital domain stay off until codec_dac_program_fifo.
   */

  reg  = getreg32(T113_AC_POWER_REG);
  reg |= AC_POWER_REG_ALDO_EN;
  reg &= ~AC_POWER_REG_ALDO_VOL_MASK;
  reg |= AC_POWER_REG_ALDO_VOL_180V;

  /* Step 2: HPLDO at 1.8 V.  This is the dedicated HP analog supply;
   * it is independent of ALDO and must be on before any HPOUT-side
   * block enables, otherwise HP_DRVEN draws current from a rising
   * rail and the resulting bias step appears on the pad.
   */

  reg |= AC_POWER_REG_HPLDO_EN;
  reg &= ~AC_POWER_REG_HPLDO_VOL_MASK;
  reg |= AC_POWER_REG_HPLDO_VOL_180V;
  putreg32(reg, T113_AC_POWER_REG);

  nxsig_usleep(T113_HPLDO_SETTLE_US);

  audinfo("DAC power-up: ALDO+HPLDO on (POWER=0x%08lx)\n",
          (unsigned long)getreg32(T113_AC_POWER_REG));

  /* Step 3: HP2_REG -- headphone driver block.  Order within this
   * register matters less than the gross sequencing because all bits
   * land in the same write, but we keep the register-level write to
   * a single transaction so partial states never appear on HPOUT.
   *
   * RSWITCH = 1 routes the HPOUT common-mode reference to VRA1 (the
   * always-on 0.9 V reference) instead of the ramp DAC output: the
   * driver does not run the digital ramp engine (RAMP_REG.RD_EN = 0)
   * so the ramp DAC sits at silence position (AGND), which would peg
   * HPOUT VCM at the negative rail and prevent any signal swing.
   * Sourcing VCM from VRA1 gives HPOUT a stable mid-rail reference for
   * the entire stream.  HPFB_BUF_EN + HPFB_IN_EN bring up the feedback
   * buffer so the HP driver sees a well-defined ground reference at
   * HPOUTFB; HP_DRVEN + HP_DRVOUTEN gate the actual output stage.
   * HEADPHONE_GAIN stays at 0 dB (default).
   */

  reg  = getreg32(T113_AC_HP2_REG);
  reg |= AC_HP2_REG_HPFB_BUF_EN
       | AC_HP2_REG_HP_DRVEN
       | AC_HP2_REG_HP_DRVOUTEN
       | AC_HP2_REG_RSWITCH
       | AC_HP2_REG_HPFB_IN_EN;
  reg &= ~(AC_HP2_REG_HP_GAIN_MASK |
           AC_HP2_REG_RAMPEN |
           AC_HP2_REG_RAMP_OUT_EN);
  reg |= AC_HP2_REG_HP_GAIN_0DB;
  putreg32(reg, T113_AC_HP2_REG);

  /* Step 4: settle the HP driver feedback loop. */

  nxsig_usleep(T113_HP_DRV_SETTLE_US);

  audinfo("DAC power-up: HP driver on (HP2=0x%08lx)\n",
          (unsigned long)getreg32(T113_AC_HP2_REG));

  /* Step 5: enable the analog DAC stages.  Both L and R unconditionally
   * -- the digital path can still send mono via DAC_MONO_EN in FIFOC,
   * which broadcasts the same sample to both DACs.
   */

  reg  = getreg32(T113_AC_DAC_REG);
  reg |= AC_DAC_REG_DACL_EN | AC_DAC_REG_DACR_EN;
  putreg32(reg, T113_AC_DAC_REG);

  audinfo("DAC power-up: DACL/R on (DAC=0x%08lx)\n",
          (unsigned long)getreg32(T113_AC_DAC_REG));
}

/****************************************************************************
 * Name: codec_dac_power_down
 *
 * Description:
 *   Reverse of codec_dac_power_up.  Caller must hold the state lock.
 *   Volume must already be ramped to mute by the caller.  HPLDO is
 *   dropped when no other consumer needs it; the design only lists
 *   the DAC path as an HPLDO consumer so we always release.
 *
 ****************************************************************************/

static void codec_dac_power_down(struct t113_codec_state_s *state)
{
  uint32_t reg;

  UNUSED(state);

  /* Step 1: drop the analog DAC stages first so the DAC stops driving
   * the mixer summing node.  The HP driver is still live and feedback-
   * referenced, so HPOUT stays at DC common-mode through this write.
   */

  reg  = getreg32(T113_AC_DAC_REG);
  reg &= ~(AC_DAC_REG_DACL_EN | AC_DAC_REG_DACR_EN);
  putreg32(reg, T113_AC_DAC_REG);

  /* Step 2: drop the HP driver block.  HPOUT goes high-Z; with the PA
   * already disabled (caller pulls PA EN low before the mute ramp)
   * this transition is silent.  Mirror the bits we set in power_up:
   * RAMPEN / RAMP_OUT_EN are not touched on the way up either, so they
   * stay 0 here.
   */

  reg  = getreg32(T113_AC_HP2_REG);
  reg &= ~(AC_HP2_REG_HPFB_BUF_EN
         | AC_HP2_REG_HP_DRVEN
         | AC_HP2_REG_HP_DRVOUTEN
         | AC_HP2_REG_RSWITCH
         | AC_HP2_REG_HPFB_IN_EN);
  putreg32(reg, T113_AC_HP2_REG);

  /* Step 3: drop HPLDO.  ALDO stays up because the ADC path may share
   * it; codec_adc_power_down does the symmetric MMICBIAS gating only
   * if dac_active is false -- here we are inside dac_active true and
   * are about to clear it, but ADC ownership is independent.
   */

  reg  = getreg32(T113_AC_POWER_REG);
  reg &= ~AC_POWER_REG_HPLDO_EN;
  putreg32(reg, T113_AC_POWER_REG);
}

/****************************************************************************
 * Name: codec_dac_program_fifo
 *
 * Description:
 *   Program the digital DAC FIFO + DPC for the current sample format.
 *   Caller must hold the state lock.  Mirrors codec_adc_program_fifo
 *   on the playback side.  Does NOT enable EN_DA / DAC_DRQ_EN -- those
 *   land in dac_start after the DMA descriptor is armed so the FIFO
 *   does not assert DRQ before the DMA channel is ready to consume.
 *
 ****************************************************************************/

static void codec_dac_program_fifo(struct t113_codec_state_s *state)
{
  uint32_t reg;

  /* Initialise the digital volume register.  Start at mute so EN_DA
   * coming up later does not produce a step on HPOUT; dac_start ramps
   * up via codec_hpout_volume_ramp.  Setting DAC_VOL_SEL = 1 here
   * routes the per-channel fine-step volume scale (-119.25 dB to
   * +71.25 dB at 0.75 dB/step) instead of the coarse DPC.DVOL field.
   */

  codec_hpout_set_volume(0);

  /* Clear DPC's HPF / DVOL / HUB_EN / EN_DA without touching reserved
   * bits via RMW.  HPF_EN stays disabled (the playback signal is
   * already DC-blocked by the AC coupling caps at HPOUT).  HUB_EN
   * stays off (audio hub is for cross-block routing between codec /
   * I2S / OWA, none of which we use).  EN_DA is the digital master
   * enable; we set it after the DMA descriptor is armed in dac_start.
   */

  reg  = getreg32(T113_AC_DAC_DPC);
  reg &= ~(AC_DAC_DPC_EN_DA | AC_DAC_DPC_HPF_EN |
           AC_DAC_DPC_DVOL_MASK | AC_DAC_DPC_HUB_EN);
  putreg32(reg, T113_AC_DAC_DPC);

  /* DAC FIFO control: sample rate, mono / stereo, bit width, TX
   * trigger, FIFO mode, flush.  TX_FIFO_MODE = 01 places the audio
   * sample in the lower 16 bits of TXDATA (matching DMA src_width
   * = DMAC_WIDTH_16BIT in codec_dma_dac_start).  DAC_MONO_EN = 1
   * for mono; the hardware broadcasts the same sample to L and R
   * inside the FIFO so a single 16-bit DMA write feeds both DACs.
   */

  reg  = codec_rate_to_dac_fs(state->cur_rate);
  reg |= ((uint32_t)T113_DAC_TX_TRIG_LEVEL) << AC_DAC_FIFOC_TX_TRIG_SHIFT;
  reg |= AC_DAC_FIFOC_FIFO_MODE_16;

  if (state->cur_channels == 1)
    {
      reg |= AC_DAC_FIFOC_DAC_MONO_EN;
    }

  if (state->cur_bits == 20)
    {
      reg |= AC_DAC_FIFOC_TX_BITS_20;
    }

  /* SEND_LASAT = 0 (send zeros on underrun, matches mainline default).
   * FIR_VER = 0 (64-tap filter; required for sample rates >32 kHz, no
   * harm at lower rates).
   */

  reg |= AC_DAC_FIFOC_FIFO_FLUSH;       /* Self-clearing */
  putreg32(reg, T113_AC_DAC_FIFOC);

  audinfo("DAC FIFO programmed: rate=%lu ch=%u bits=%u "
          "(VOL=0x%08lx DPC=0x%08lx FIFOC=0x%08lx)\n",
          (unsigned long)state->cur_rate,
          (unsigned)state->cur_channels,
          (unsigned)state->cur_bits,
          (unsigned long)getreg32(T113_AC_DAC_VOL_CTRL),
          (unsigned long)getreg32(T113_AC_DAC_DPC),
          (unsigned long)getreg32(T113_AC_DAC_FIFOC));
}

/****************************************************************************
 * Name: codec_dma_dac_callback
 *
 * Description:
 *   DMA completion callback for the playback path.  Cyclic mode fires
 *   us at half-buffer (HLFDONE) and full-buffer (PKGDONE) boundaries;
 *   each invocation = one apb worth of data has been transmitted to
 *   the FIFO and the DMA engine has rolled onto the next slot.  We
 *   return the just-emptied apb to the upper half so the application
 *   can refill it.  When the queue drains we pause cyclic DMA -- on
 *   underrun the FIFO sends zeros (SEND_LASAT = 0) so the analog out
 *   stays at common-mode rather than glitching with stale data.
 *
 ****************************************************************************/

static void codec_dma_dac_callback(DMA_HANDLE handle, uint8_t status,
                                   void *arg)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)arg;
  struct ap_buffer_s *apb;
  irqstate_t flags;

  UNUSED(handle);
  UNUSED(status);

  flags = spin_lock_irqsave(&dev->qlock);
  apb   = (struct ap_buffer_s *)dq_remfirst(&dev->pendq);
  spin_unlock_irqrestore(&dev->qlock, flags);

  if (apb == NULL)
    {
      /* Underrun: DMA finished an apb but no more queued.  Pause the
       * channel; resume happens in dac_enqueuebuffer.  Per spec 6.1
       * playback underrun does NOT auto-resume -- the application is
       * responsible for noticing AUDIO_CALLBACK_DEQUEUE returns and
       * pushing fresh data.
       */

      t113_dmapause(dev->dma);
      dev->xrun = true;
      return;
    }

  /* Mark the apb as fully consumed and let cache reflect that the DMA
   * read out of buf_pool is complete.  dac_enqueuebuffer flushes the
   * cache before adding to pendq (so the controller fetches fresh DRAM
   * contents for the next pass), so on dequeue we only need to clear
   * the bytes-pending count.
   */

  apb->nbytes = apb->nmaxbytes;

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
 * Name: codec_dma_dac_start
 *
 * Description:
 *   Arm the cyclic DMA descriptor for DAC TX.  Direction is memory
 *   (linear DRAM buf_pool) to peripheral (AC_DAC_TXDATA, IO).
 *
 ****************************************************************************/

static int codec_dma_dac_start(struct t113_codec_dev_s *dev)
{
  struct t113_dma_config_s cfg;
  int ret;

  memset(&cfg, 0, sizeof(cfg));

  /* Source: linear DRAM, 16-bit width.  See codec_dma_start in the
   * ADC path for the equivalent comment on RX_FIFO_MODE -- the DAC
   * uses TX_FIFO_MODE (FIFO_MODE_16) so a 16-bit DMA write lands in
   * the lower half of TXDATA, hardware sign-extends to 20-bit
   * internal precision.
   */

  cfg.src_drq    = DRQ_DRAM;
  cfg.src_width  = DMAC_WIDTH_16BIT;
  cfg.src_burst  = DMAC_BURST_4;
  cfg.src_linear = true;

  /* Destination: codec DAC TXDATA register.  IO addressing -- the DMA
   * controller does not auto-increment the destination address, every
   * burst writes to the same TXDATA word.
   */

  cfg.dst_drq    = DRQ_CODEC;
  cfg.dst_width  = DMAC_WIDTH_16BIT;
  cfg.dst_burst  = DMAC_BURST_4;
  cfg.dst_linear = false;

  cfg.mode       = DMAC_MODE_DST_HANDSHAKE;
  cfg.circular   = true;

  ret = t113_dmasetup(dev->dma,
                      (uintptr_t)dev->buf_pool,
                      (uintptr_t)T113_AC_DAC_TXDATA,
                      dev->buf_total, &cfg);
  if (ret < 0)
    {
      return ret;
    }

  /* Flush the buffer pool so the DMA controller observes the most
   * recent CPU writes.  dac_enqueuebuffer also flushes per-apb but a
   * blanket flush at start covers the case where the application
   * pre-fills several apbs before the first start().
   */

  up_clean_dcache((uintptr_t)dev->buf_pool,
                  (uintptr_t)dev->buf_pool + dev->buf_total);

  return t113_dmastart(dev->dma, codec_dma_dac_callback, dev);
}

/****************************************************************************
 * Lower-half ops: reserve / release
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dac_reserve(struct audio_lowerhalf_s *lower, void **session)
#else
static int dac_reserve(struct audio_lowerhalf_s *lower)
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
static int dac_release(struct audio_lowerhalf_s *lower, void *session)
#else
static int dac_release(struct audio_lowerhalf_s *lower)
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

static int dac_getcaps(struct audio_lowerhalf_s *lower, int type,
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
            caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT;
            caps->ac_format.hw     = 1u << (AUDIO_FMT_PCM - 1);
          }

        break;

      case AUDIO_TYPE_OUTPUT:
        caps->ac_channels = 2;
        if (caps->ac_subtype == AUDIO_TYPE_QUERY)
          {
            caps->ac_controls.hw[0] = AUDIO_SAMP_RATE_8K |
                                      AUDIO_SAMP_RATE_16K |
                                      AUDIO_SAMP_RATE_32K |
                                      AUDIO_SAMP_RATE_48K;
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
static int dac_configure(struct audio_lowerhalf_s *lower,
                         void *session,
                         const struct audio_caps_s *caps)
#else
static int dac_configure(struct audio_lowerhalf_s *lower,
                         const struct audio_caps_s *caps)
#endif
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  struct t113_codec_state_s *state = dev->state;
  uint32_t rate;
  uint8_t  bits;
  uint8_t  channels;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  DEBUGASSERT(caps != NULL);

  /* AUDIO_TYPE_FEATURE arrives from nxplayer's setvolume / setbass /
   * settreble path on every play.  Acknowledge the request without
   * touching the playback state machine -- volume, bass and treble
   * are not yet wired to hardware controls (the DAC_VOL_CTRL stage
   * exists but is left at unity 0 dB).  Returning OK here lets
   * nxplayer continue past its first ioctl into the streaming path.
   */

  if (caps->ac_type == AUDIO_TYPE_FEATURE)
    {
      return OK;
    }

  if (caps->ac_type != AUDIO_TYPE_OUTPUT)
    {
      return -EINVAL;
    }

  rate     = caps->ac_controls.hw[0];
  bits     = caps->ac_controls.b[2];
  channels = caps->ac_channels;

  ret = codec_validate_caps(rate, bits, channels);
  if (ret < 0)
    {
      auderr("ERROR: invalid caps rate=%u bits=%u ch=%u\n",
             (unsigned)rate, (unsigned)bits, (unsigned)channels);
      return ret;
    }

  ret = nxmutex_lock(&state->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (state->dac_active)
    {
      nxmutex_unlock(&state->lock);
      return -EBUSY;
    }

  /* Configure shares cur_rate / cur_bits / cur_channels with the ADC
   * path.  Spec section 5.3 explicitly allows P2 + P3 to coexist (ADC
   * and DAC are independent codec subblocks), but only at the same
   * sample rate -- the codec audio clock is single-rooted at PLL_AUDIO.
   * The clock provider's request/release path takes the same target
   * rate from each consumer; if they disagree the second request wins
   * and silently retunes the first.  Userspace is expected to keep
   * them aligned.
   */

  state->cur_rate     = rate;
  state->cur_bits     = bits;
  state->cur_channels = channels;

  nxmutex_unlock(&state->lock);
  return OK;
}

/****************************************************************************
 * Lower-half ops: start
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dac_start(struct audio_lowerhalf_s *lower, void *session)
#else
static int dac_start(struct audio_lowerhalf_s *lower)
#endif
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  struct t113_codec_state_s *state = dev->state;
  uint32_t reg;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  ret = nxmutex_lock(&state->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (state->dac_active)
    {
      ret = -EBUSY;
      goto err_unlock;
    }

  if (state->cur_rate == 0)
    {
      state->cur_rate     = T113_CODEC_DEFAULT_RATE;
      state->cur_bits     = T113_CODEC_DEFAULT_BITS;
      state->cur_channels = T113_CODEC_DEFAULT_CHANNELS;
    }

  audinfo("dac_start: rate=%lu ch=%u bits=%u\n",
          (unsigned long)state->cur_rate,
          (unsigned)state->cur_channels,
          (unsigned)state->cur_bits);

  /* 1. Request the audio PLL + DAC divider. */

  ret = t113_audio_clk_request(T113_AUDIO_CONSUMER_CODEC_DAC,
                               state->cur_rate);
  if (ret < 0)
    {
      auderr("ERROR: t113_audio_clk_request failed: %d\n", ret);
      goto err_unlock;
    }

  audinfo("dac_start: PLL+divider configured\n");

  /* 2. Bring up the analog HPOUT chain (HPLDO + HP driver + DAC L/R).
   * The digital path is still off so HPOUT sits at common-mode DC.
   */

  codec_dac_power_up(state);

  /* 3. Program the DAC FIFO + DPC.  Volume is at 0 (mute) after this. */

  codec_dac_program_fifo(state);

  /* 4. Arm cyclic DMA over the buf_pool.  enqueuebuffer was called by
   * the upper half before start to seed the queue; the cyclic engine
   * will sweep through the entire pool regardless of pendq depth, but
   * the per-apb completion callback gates returning empty buffers to
   * the application until the queue actually contained that apb.
   */

  dev->xrun = false;

  ret = codec_dma_dac_start(dev);
  if (ret < 0)
    {
      auderr("ERROR: codec_dma_dac_start failed: %d\n", ret);
      goto err_pwr;
    }

  audinfo("dac_start: DMA armed (buf_pool=%p len=%zu)\n",
          dev->buf_pool, dev->buf_total);

  /* 5. Enable digital DAC + DRQ.  Order is critical: DMA channel is
   * armed before EN_DA, otherwise the FIFO would assert DRQ to a
   * channel that is not ready to consume.
   */

  reg  = getreg32(T113_AC_DAC_DPC);
  reg |= AC_DAC_DPC_EN_DA;
  putreg32(reg, T113_AC_DAC_DPC);

  reg  = getreg32(T113_AC_DAC_FIFOC);
  reg |= AC_DAC_FIFOC_DAC_DRQ_EN;
  putreg32(reg, T113_AC_DAC_FIFOC);

  audinfo("dac_start: EN_DA+DRQ_EN set "
          "(DPC=0x%08lx FIFOC=0x%08lx FIFOS=0x%08lx)\n",
          (unsigned long)getreg32(T113_AC_DAC_DPC),
          (unsigned long)getreg32(T113_AC_DAC_FIFOC),
          (unsigned long)getreg32(T113_AC_DAC_FIFOS));

  /* 6. Software volume ramp from mute to unity gain over ~160 ms.
   * The DMA is now feeding samples at full speed but the digital
   * volume is multiplied by ~0 at this point, so HPOUT is silent.
   * As the volume rises the level on HPOUT crosses up smoothly --
   * the PA on the other side of the AC-coupling cap sees a slow
   * common-mode-shift instead of a step.
   */

  codec_hpout_volume_ramp(0, T113_HPOUT_VOL_TARGET);

  audinfo("dac_start: ramp complete (VOL_CTRL=0x%08lx FIFOS=0x%08lx)\n",
          (unsigned long)getreg32(T113_AC_DAC_VOL_CTRL),
          (unsigned long)getreg32(T113_AC_DAC_FIFOS));

  /* The mq-r reference board hard-wires the PA's SD pin to VCC (always
   * on); there is no GPIO-controlled enable line.  The volume ramp
   * above is the only mechanism keeping the start-up transient out of
   * the audible band on this board.
   */

  state->dac_active = true;
  nxmutex_unlock(&state->lock);
  return OK;

err_pwr:
  codec_dac_power_down(state);
  t113_audio_clk_release(T113_AUDIO_CONSUMER_CODEC_DAC);

err_unlock:
  nxmutex_unlock(&state->lock);
  return ret;
}

/****************************************************************************
 * Lower-half ops: stop
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int dac_stop(struct audio_lowerhalf_s *lower, void *session)
#else
static int dac_stop(struct audio_lowerhalf_s *lower)
#endif
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  struct t113_codec_state_s *state = dev->state;
  struct ap_buffer_s *apb;
  uint32_t reg;
  irqstate_t flags;
  int ret;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  UNUSED(session);
#endif

  ret = nxmutex_lock(&state->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (!state->dac_active)
    {
      nxmutex_unlock(&state->lock);
      return OK;
    }

  /* 1. Volume ramp from unity gain to mute.  On mq-r the PA is always
   * on, so this ramp is the only thing keeping the teardown transient
   * out of the audible band.  Ramping to 0 over 160 ms lets the AC-
   * coupling cap discharge through HPOUT's output impedance instead
   * of slamming to the bias rail.  Without this the cap discharges
   * through the HP load on the next power_down step.
   */

  codec_hpout_volume_ramp(T113_HPOUT_VOL_TARGET, 0);

  /* 2. Disable digital DAC + DRQ.  FIFO underrun bits will assert
   * during teardown; we don't enable underrun IRQ so they are silently
   * pending and cleared by the next dac_start's FIFO_FLUSH.
   */

  reg  = getreg32(T113_AC_DAC_FIFOC);
  reg &= ~AC_DAC_FIFOC_DAC_DRQ_EN;
  putreg32(reg, T113_AC_DAC_FIFOC);

  reg  = getreg32(T113_AC_DAC_DPC);
  reg &= ~AC_DAC_DPC_EN_DA;
  putreg32(reg, T113_AC_DAC_DPC);

  /* 3. Tear down DMA. */

  t113_dmastop(dev->dma);

  /* 4. Drain in-flight buffers back to the upper half. */

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

  /* 5. Power down analog blocks then release the audio clock. */

  codec_dac_power_down(state);
  t113_audio_clk_release(T113_AUDIO_CONSUMER_CODEC_DAC);

  state->dac_active = false;

#ifdef CONFIG_AUDIO_MULTI_SESSION
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK,
                   NULL);
#else
  dev->lower.upper(dev->lower.priv, AUDIO_CALLBACK_COMPLETE, NULL, OK);
#endif

  nxmutex_unlock(&state->lock);
  return OK;
}
#endif /* CONFIG_AUDIO_EXCLUDE_STOP */

/****************************************************************************
 * Lower-half ops: shutdown
 ****************************************************************************/

static int dac_shutdown(struct audio_lowerhalf_s *lower)
{
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
  dac_stop(lower, NULL);
#else
  dac_stop(lower);
#endif
#else
  UNUSED(lower);
#endif
  return OK;
}

/****************************************************************************
 * Lower-half ops: allocbuffer / freebuffer
 *
 * The DAC uses the same buf_pool / cyclic-DMA pattern as the ADC; each
 * apb maps to a slot in the dev-private pool so the cyclic DMA engine
 * can sweep through them with no descriptor reprogramming.
 ****************************************************************************/

static int dac_allocbuffer(struct audio_lowerhalf_s *lower,
                           struct audio_buf_desc_s *bufdesc)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
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

  apb->i.channels = dev->state->cur_channels ?
                    dev->state->cur_channels : 1;
  apb->crefs      = 1;
  apb->nmaxbytes  = dev->buf_size;
  apb->samp       = dev->buf_pool + dev->buf_alloc * dev->buf_size;
  nxmutex_init(&apb->lock);

  dev->buf_alloc++;
  *bufdesc->u.pbuffer = apb;
  return sizeof(struct audio_buf_desc_s);
}

static int dac_freebuffer(struct audio_lowerhalf_s *lower,
                          struct audio_buf_desc_s *bufdesc)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
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
 *
 * For OUTPUT direction the apb arrives populated with user PCM data.
 * We must clean (write-back) the data cache so the DMA engine reads
 * the freshest samples from DRAM, then add the apb to pendq so the
 * cyclic engine has somewhere to "complete into" -- the actual DMA
 * arming is in dac_start, and this path also handles xrun resume.
 ****************************************************************************/

static int dac_enqueuebuffer(struct audio_lowerhalf_s *lower,
                             struct ap_buffer_s *apb)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
  irqstate_t flags;
  bool resume;

  apb->flags |= AUDIO_APB_OUTPUT_ENQUEUED;

  /* Push samples to DRAM so the DMA controller picks them up.  The
   * apb may have been written from a write() syscall (DDR-resident)
   * but the dirty cache lines must be flushed before the DMA fetch.
   */

  up_clean_dcache((uintptr_t)apb->samp,
                  (uintptr_t)apb->samp + apb->nmaxbytes);

  flags = spin_lock_irqsave(&dev->qlock);
  dq_addlast(&apb->dq_entry, &dev->pendq);

  resume = dev->state->dac_active && dev->xrun;
  if (resume)
    {
      dev->xrun = false;
    }

  spin_unlock_irqrestore(&dev->qlock, flags);

  if (resume)
    {
      t113_dmaresume(dev->dma);
    }

  return OK;
}

/****************************************************************************
 * Lower-half ops: ioctl
 ****************************************************************************/

static int dac_ioctl(struct audio_lowerhalf_s *lower, int cmd,
                     unsigned long arg)
{
  struct t113_codec_dev_s *dev = (struct t113_codec_dev_s *)lower;
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
 * Name: t113_codec_adc_initialize
 ****************************************************************************/

struct audio_lowerhalf_s *t113_codec_adc_initialize(void)
{
  struct t113_codec_dev_s *dev;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      auderr("ERROR: codec ADC alloc dev failed\n");
      return NULL;
    }

  dev->state     = &g_codec_state;
  dev->dir       = T113_CODEC_DIR_ADC;
  dev->lower.ops = &g_codec_adc_ops;
  dev->buf_size  = T113_CODEC_BUFFER_BYTES;
  dev->buf_count = T113_CODEC_BUFFER_COUNT;
  dev->buf_total = (size_t)dev->buf_size * dev->buf_count;

  spin_lock_init(&dev->qlock);
  dq_init(&dev->pendq);

  /* Pre-allocate a single contiguous DMA pool sized for the whole ring.
   * Cyclic DMA will sweep through this region indefinitely; allocbuffer
   * simply hands out apb_s pointing into successive slots.  Align to a
   * cache line so up_invalidate_dcache cannot evict adjacent unrelated
   * data.
   */

  dev->buf_pool = kumm_memalign(64, dev->buf_total);
  if (dev->buf_pool == NULL)
    {
      auderr("ERROR: codec ADC alloc buf_pool (%zu bytes) failed\n",
             dev->buf_total);
      goto err_dev;
    }

  /* Allocate the codec-side DMA channel (DRQ_CODEC, RX direction --
   * direction is decided at t113_dmasetup by src/dst placement, not
   * here).  Spec section 6.2: failure means /dev/audio1 must not be
   * registered, so we return NULL and bringup skips audio_register.
   */

  dev->dma = t113_dmachannel();
  if (dev->dma == NULL)
    {
      auderr("ERROR: codec ADC t113_dmachannel failed\n");
      goto err_pool;
    }

  return &dev->lower;

err_pool:
  kumm_free(dev->buf_pool);

err_dev:
  kmm_free(dev);
  return NULL;
}

/****************************************************************************
 * Name: t113_codec_dac_initialize
 *
 * Description:
 *   Allocate the DAC lowerhalf wrapper, the contiguous DMA buf_pool and
 *   the codec-side DMA channel.  Returns NULL on any allocation
 *   failure; the caller (board bringup) interprets that as "skip
 *   audio_register so /dev/audio2 does not appear".
 *
 ****************************************************************************/

struct audio_lowerhalf_s *t113_codec_dac_initialize(void)
{
  struct t113_codec_dev_s *dev;

  dev = kmm_zalloc(sizeof(*dev));
  if (dev == NULL)
    {
      auderr("ERROR: codec DAC alloc dev failed\n");
      return NULL;
    }

  dev->state     = &g_codec_state;
  dev->dir       = T113_CODEC_DIR_DAC;
  dev->lower.ops = &g_codec_dac_ops;
  dev->buf_size  = T113_CODEC_BUFFER_BYTES;
  dev->buf_count = T113_CODEC_BUFFER_COUNT;
  dev->buf_total = (size_t)dev->buf_size * dev->buf_count;

  spin_lock_init(&dev->qlock);
  dq_init(&dev->pendq);

  /* Pre-allocate one cache-line-aligned contiguous DMA pool sized for
   * the entire ring.  Identical scheme to the ADC path; the DAC pool
   * is independent so capture and playback can run concurrently.
   */

  dev->buf_pool = kumm_memalign(64, dev->buf_total);
  if (dev->buf_pool == NULL)
    {
      auderr("ERROR: codec DAC alloc buf_pool (%zu bytes) failed\n",
             dev->buf_total);
      goto err_dev;
    }

  /* Codec-side DMA channel.  TX direction is selected at t113_dmasetup
   * time (memory -> peripheral); the channel itself is direction-
   * agnostic.  DRQ_CODEC is shared by both directions inside the
   * controller -- the DRQ slot encodes "any codec" and the dst_drq /
   * src_drq fields select the actual peripheral endpoint.
   */

  dev->dma = t113_dmachannel();
  if (dev->dma == NULL)
    {
      auderr("ERROR: codec DAC t113_dmachannel failed\n");
      goto err_pool;
    }

  return &dev->lower;

err_pool:
  kumm_free(dev->buf_pool);

err_dev:
  kmm_free(dev);
  return NULL;
}

#endif /* CONFIG_T113_AUDIO */
