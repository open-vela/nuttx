/****************************************************************************
 * arch/arm/src/bk7258/bk7258_audio_diag.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <arch/board/board.h>
#include "bk7258_aud.h"
#include "bk7258_audio_diag.h"
#include "hardware/bk7258_aud.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DIAG_RATE BK7258_AUDIO_CLIP_SAMPLES
#define DIAG_SETTLE 3200u
#define DIAG_BUDGET 64u
#define DIAG_RAMP 160u

static mutex_t g_diag_lock = NXMUTEX_INITIALIZER;
static int16_t *g_clip;
static uint32_t g_recorded;
static struct bk7258_audio_report_s g_last;

struct audio_transfer_s
{
  uint32_t position;
  uint32_t settling;
  uint32_t interrupts;
  uint32_t faults;
  uint32_t stalls;
  bool failed;
  bool capture;
  bool tone;
  bool silence;
  int32_t dc;
  unsigned int replay_divisor;
  uint32_t raw_samples;
  uint32_t max_batch;
  uint32_t status_before_or;
  uint32_t status_after_or;
  uint32_t full_after;
  uint32_t empty_after;
  clock_t start_tick;
  clock_t last_irq_tick;
  clock_t max_irq_gap;
  bool started;
};

static struct audio_transfer_s g_transfer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t audio_sqrt(uint64_t value)
{
  uint64_t bit = (uint64_t)1 << 62;
  uint64_t root = 0;
  while (bit > value)
    {
      bit >>= 2;
    }
  while (bit != 0)
    {
      if (value >= root + bit)
        {
          value -= root + bit;
          root = (root >> 1) + bit;
        }
      else
        {
          root >>= 1;
        }

      bit >>= 2;
    }

  return (uint32_t)root;
}

static int16_t audio_sample(uint32_t index, bool tone, int32_t dc,
                            unsigned int divisor)
{
  static const int16_t wave[16] =
    {
      0, 122, 226, 296, 320, 296, 226, 122,
      0, -122, -226, -296, -320, -296, -226, -122
    };

  int32_t value = tone ? wave[index % 16] :
                        (g_clip[index] - dc) / (int32_t)divisor;
  uint32_t remaining = DIAG_RATE - 1 - index;

  if (value > 1024) value = 1024;
  if (value < -1024) value = -1024;
  if (index < DIAG_RAMP) value = value * (int32_t)index / (int32_t)DIAG_RAMP;
  if (remaining < DIAG_RAMP)
    value = value * (int32_t)remaining / (int32_t)DIAG_RAMP;
  return (int16_t)value;
}

static void audio_fifo_service(void *arg)
{
  struct audio_transfer_s *x = arg;
  int16_t chunk[DIAG_BUDGET];
  uint32_t status = bk7258_aud_fifo_status();
  unsigned int count;
  unsigned int i;
  uint32_t before = x->position + DIAG_SETTLE - x->settling;
  clock_t now = clock_systime_ticks();
  clock_t gap = now - x->last_irq_tick;

  x->last_irq_tick = now;
  if (gap > x->max_irq_gap) x->max_irq_gap = gap;
  x->status_before_or |= status;
  x->interrupts++;
  if (x->capture)
    {
      if ((status & BK7258_AUD_ADC_FIFO_FULL) != 0) x->faults++;
      count = bk7258_aud_adc_read(chunk, DIAG_BUDGET);
      status = bk7258_aud_fifo_status();
      if ((status & BK7258_AUD_ADC_FIFO_FULL) != 0) x->full_after++;
      if ((status & BK7258_AUD_ADC_FIFO_EMPTY) != 0) x->empty_after++;
      for (i = 0; i < count; i++)
        {
          if (x->settling > 0)
            {
              x->settling--;
            }
          else if (x->position < DIAG_RATE)
            {
              g_clip[x->position++] = chunk[i];
            }
        }

      if (x->position == DIAG_RATE) bk7258_aud_rx_int_enable(false);
    }
  else
    {
      if (x->position > 0 && x->position < DIAG_RATE &&
          (status & BK7258_AUD_DACL_FIFO_EMPTY) != 0) x->faults++;
      count = DIAG_RATE - x->position;
      if (count > DIAG_BUDGET) count = DIAG_BUDGET;
      for (i = 0; i < count; i++)
        {
          chunk[i] = x->silence ? 0 :
                     audio_sample(x->position + i, x->tone, x->dc,
                                  x->replay_divisor);
        }

      count = bk7258_aud_dac_write(chunk, count);
      x->position += count;
      status = bk7258_aud_fifo_status();
      if (x->position == DIAG_RATE) bk7258_aud_tx_int_enable(false);
    }

  x->raw_samples += count;
  if (count > x->max_batch) x->max_batch = count;
  x->status_after_or |= status;

  /* A stuck level IRQ must not starve the task that enforces the timeout. */

  if (x->position + DIAG_SETTLE - x->settling == before)
    {
      x->stalls++;
    }
  else
    {
      x->stalls = 0;
    }

  if (x->stalls >= 4 || x->interrupts > DIAG_RATE * 2)
    {
      x->failed = true;
      if (x->capture) bk7258_aud_rx_int_enable(false);
      else bk7258_aud_tx_int_enable(false);
    }
}

static void audio_statistics(struct bk7258_audio_report_s *report)
{
  int64_t sum = 0;
  uint64_t squares = 0;
  uint32_t i;
  for (i = 0; i < g_recorded; i++)
    {
      int32_t value = g_clip[i];
      uint32_t amplitude = value < 0 ? -value : value;
      sum += value;
      squares += (int64_t)value * value;
      if (amplitude > report->peak) report->peak = amplitude;
      if (value == -32768 || value == 32767) report->clipped++;
    }

  if (g_recorded != 0)
    {
      report->dc = sum / g_recorded;
      report->rms = audio_sqrt(squares / g_recorded);
    }
}

static int audio_diag_run(enum bk7258_audio_operation_e operation,
                          unsigned int divisor,
                          struct bk7258_audio_report_s *report)
{
  struct bk7258_aud_config cfg =
    {
      .samplerate = DIAG_RATE,
      .clksrc = BK7258_AUD_CLK_XTAL,
      .dac_dig_gain = BK7258_AUD_DAC_DIG_GAIN_0DB,
      .dac_ana_gain = BK7258_AUD_DAC_ANA_GAIN_DEF,
      .adc_gain = BK7258_AUD_ADC_GAIN_0DB,
      .mic_gain = BK7258_AUD_MIC_GAIN_DEF
    };

  clock_t start;
  irqstate_t flags;
  uint32_t position;
  bool failed;
  int ret;

  if (report == NULL || operation < BK7258_AUDIO_STATUS ||
      operation > BK7258_AUDIO_SILENCE) return -EINVAL;
  memset(report, 0, sizeof(*report));
  ret = nxmutex_trylock(&g_diag_lock);
  if (ret < 0) return ret;
  ret = bk7258_aud_acquire(&g_diag_lock);
  if (ret < 0)
    {
      nxmutex_unlock(&g_diag_lock);
      return ret;
    }

  bk7258_board_audio_pa(false);

  if (operation == BK7258_AUDIO_CLEAR)
    {
      if (g_clip != NULL)
        {
          explicit_bzero(g_clip, DIAG_RATE * sizeof(*g_clip));
          kmm_free(g_clip);
          g_clip = NULL;
        }

      g_recorded = 0;
      memset(&g_last, 0, sizeof(g_last));
      ret = 0;
      goto out;
    }

  if (operation == BK7258_AUDIO_STATUS)
    {
      *report = g_last;
      report->recorded = g_recorded;
      ret = 0;
      goto out;
    }

  if (operation == BK7258_AUDIO_PLAY && g_recorded != DIAG_RATE)
    {
      ret = -ENODATA;
      goto out;
    }

  if (operation == BK7258_AUDIO_RECORD)
    {
      g_recorded = 0;
      if (g_clip == NULL) g_clip = kmm_malloc(DIAG_RATE * sizeof(*g_clip));
      if (g_clip == NULL)
        {
          ret = -ENOMEM;
          goto out;
        }

      explicit_bzero(g_clip, DIAG_RATE * sizeof(*g_clip));
    }

  memset(&g_transfer, 0, sizeof(g_transfer));
  g_transfer.replay_divisor = divisor;
  g_transfer.capture = operation == BK7258_AUDIO_RECORD;
  g_transfer.tone = operation == BK7258_AUDIO_TONE;
  g_transfer.silence = operation == BK7258_AUDIO_SILENCE;
  if (operation == BK7258_AUDIO_PLAY)
    {
      audio_statistics(report);
      g_transfer.dc = report->dc;
    }

  ret = bk7258_aud_initialize();
  if (ret < 0) goto out;
  report->device_id = bk7258_aud_read_id();

  if (g_transfer.capture)
    {
      ret = bk7258_aud_adc_setup(&cfg);
      if (ret < 0) goto cleanup;
      g_transfer.settling = DIAG_SETTLE;
      bk7258_aud_set_rx_callback(audio_fifo_service, &g_transfer);
      report->adc_threshold = bk7258_aud_adc_threshold();
      g_transfer.start_tick = clock_systime_ticks();
      g_transfer.last_irq_tick = g_transfer.start_tick;
      g_transfer.started = true;
      bk7258_aud_adc_start();
      bk7258_aud_rx_int_enable(true);
    }
  else
    {
      int16_t zero[32] =
        {
          0
        };

      ret = bk7258_aud_dac_setup(&cfg);
      if (ret < 0) goto cleanup;
      bk7258_aud_dac_write(zero, 32);
      bk7258_aud_dac_start();
      ret = nxsig_usleep(20000);
      if (ret < 0) goto cleanup;
      bk7258_board_audio_pa(true);
      ret = nxsig_usleep(20000);
      if (ret < 0) goto cleanup;
      ret = bk7258_aud_dac_mute(false);
      if (ret < 0) goto cleanup;
      bk7258_aud_set_tx_callback(audio_fifo_service, &g_transfer);
      g_transfer.start_tick = clock_systime_ticks();
      g_transfer.last_irq_tick = g_transfer.start_tick;
      g_transfer.started = true;
      bk7258_aud_tx_int_enable(true);
    }

  start = clock_systime_ticks();
  for (; ; )
    {
      flags = enter_critical_section();
      position = g_transfer.position;
      failed = g_transfer.failed;
      leave_critical_section(flags);
      if (failed)
        {
          ret = -EIO;
          break;
        }

      if (position >= DIAG_RATE) break;
      if ((clock_t)(clock_systime_ticks() - start) >= MSEC2TICK(3000))
        {
          ret = -ETIMEDOUT;
          break;
        }

      ret = nxsig_usleep(10000);
      if (ret < 0) break;
    }

  if (ret >= 0 && !g_transfer.capture)
    {
      /* Last samples are queued, not yet audible: let the FIFO drain. */

      ret = nxsig_usleep(10000);
    }

cleanup:
  bk7258_board_audio_pa(false);
  bk7258_aud_tx_int_enable(false);
  bk7258_aud_rx_int_enable(false);
  bk7258_aud_set_tx_callback(NULL, NULL);
  bk7258_aud_set_rx_callback(NULL, NULL);
  bk7258_aud_dac_shutdown();
  bk7258_aud_adc_shutdown();
  report->samples = g_transfer.position;
  report->interrupts = g_transfer.interrupts;
  report->fifo_faults = g_transfer.faults;
  report->raw_samples = g_transfer.raw_samples;
  if (g_transfer.started)
    {
      /* Complete transfers end at the final FIFO service, not task wakeup.
       * For playback this measures enqueue time, excluding final drain.
       */

      clock_t end = ret >= 0 ?
                    g_transfer.last_irq_tick : clock_systime_ticks();
      report->elapsed_ms = TICK2MSEC(end - g_transfer.start_tick);
      report->max_irq_gap_ms = TICK2MSEC(g_transfer.max_irq_gap);
    }

  report->max_batch = g_transfer.max_batch;
  report->status_before_or = g_transfer.status_before_or;
  report->status_after_or = g_transfer.status_after_or;
  report->full_after = g_transfer.full_after;
  report->empty_after = g_transfer.empty_after;
  if (g_transfer.capture)
    {
      g_recorded = ret >= 0 ? g_transfer.position : 0;
      if (ret < 0) explicit_bzero(g_clip, DIAG_RATE * sizeof(*g_clip));
    }

  report->dc = 0;
  report->rms = 0;
  report->peak = 0;
  report->clipped = 0;
  audio_statistics(report);
  report->recorded = g_recorded;
  g_last = *report;
out:
  bk7258_aud_release(&g_diag_lock);
  nxmutex_unlock(&g_diag_lock);
  return ret;
}

int bk7258_audio_diag(enum bk7258_audio_operation_e operation,
                      struct bk7258_audio_report_s *report)
{
  return audio_diag_run(operation, 8, report);
}

int bk7258_audio_replay(unsigned int divisor,
                        struct bk7258_audio_report_s *report)
{
  if (report == NULL) return -EINVAL;
  if (divisor != 8 && divisor != 4 && divisor != 2 && divisor != 1)
    {
      memset(report, 0, sizeof(*report));
      return -EINVAL;
    }

  return audio_diag_run(BK7258_AUDIO_PLAY, divisor, report);
}

int bk7258_audio_copy(int16_t *samples, uint32_t capacity)
{
  int ret;
  if (samples == NULL || capacity < DIAG_RATE) return -EINVAL;
  ret = nxmutex_trylock(&g_diag_lock);
  if (ret < 0) return ret;
  if (g_recorded != DIAG_RATE)
    {
      ret = -ENODATA;
    }
  else
    {
      memcpy(samples, g_clip, DIAG_RATE * sizeof(*samples));
      ret = DIAG_RATE;
    }

  nxmutex_unlock(&g_diag_lock);
  return ret;
}
