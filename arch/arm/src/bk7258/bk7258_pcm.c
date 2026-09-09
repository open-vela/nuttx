/****************************************************************************
 * arch/arm/src/bk7258/bk7258_pcm.c
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/* Continuous PCM FIFO transport using the existing BK7258 chip driver.
 * Task-side calls serialize control; IRQs only copy bounded sample blocks.
 * No upper-half callbacks, file I/O, logging or allocation runs in the ISR.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/signal.h>
#include <nuttx/spinlock.h>
#include <arch/board/board.h>
#include "bk7258_aud.h"
#include "bk7258_pcm.h"
#include "hardware/bk7258_aud.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PCM_IRQ_BUDGET 64u
#define PCM_SETTLE 3200u
#define PCM_RAMP 160u

static mutex_t g_pcm_lock = NXMUTEX_INITIALIZER;
static uint32_t g_generation;
static struct
{
  bool active;
  bool draining;
  uint32_t session;
  uint32_t head;
  uint32_t tail;
  uint32_t settling;
  uint32_t ramp;
  unsigned int stalls;
  int16_t ring[BK7258_PCM_RING_SAMPLES];
  struct bk7258_pcm_status_s status;
} g_pcm;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pcm_mask(void)
{
  bk7258_aud_rx_int_enable(false);
  bk7258_aud_tx_int_enable(false);
}

static void pcm_cleanup(void)
{
  irqstate_t flags = enter_critical_section();
  pcm_mask();
  g_pcm.status.running = false;
  bk7258_aud_set_rx_callback(NULL, NULL);
  bk7258_aud_set_tx_callback(NULL, NULL);
  leave_critical_section(flags);
  bk7258_board_audio_pa(false);
  bk7258_aud_dac_shutdown();
  bk7258_aud_adc_shutdown();
}

static void pcm_irq(void *arg)
{
  int16_t block[PCM_IRQ_BUDGET];
  uint32_t available;
  unsigned int count;
  unsigned int i;
  uint32_t flags = bk7258_aud_fifo_status();
  (void)arg;

  if (!g_pcm.status.running)
    {
      pcm_mask();
      return;
    }

  if (g_pcm.status.capture)
    {
      if (flags & BK7258_AUD_ADC_FIFO_FULL) g_pcm.status.fifo_boundaries++;
      count = bk7258_aud_adc_read(block, PCM_IRQ_BUDGET);
      for (i = 0; i < count; i++)
        {
          if (g_pcm.settling != 0)
            {
              g_pcm.settling--;
              continue;
            }

          if (g_pcm.status.buffered == BK7258_PCM_RING_SAMPLES)
            {
              g_pcm.status.dropped_samples++;
              continue;
            }

          g_pcm.ring[g_pcm.head] = block[i];
          g_pcm.head = (g_pcm.head + 1) % BK7258_PCM_RING_SAMPLES;
          g_pcm.status.buffered++;
        }
    }
  else
    {
      if (flags & BK7258_AUD_DACL_FIFO_EMPTY) g_pcm.status.fifo_boundaries++;
      available = g_pcm.status.buffered;
      if (available > PCM_IRQ_BUDGET) available = PCM_IRQ_BUDGET;
      if (available == 0 && g_pcm.draining)
        {
          bk7258_aud_tx_int_enable(false);
          return;
        }

      for (i = 0; i < PCM_IRQ_BUDGET; i++)
        block[i] =
          i < available ?
          g_pcm.ring[(g_pcm.tail + i) % BK7258_PCM_RING_SAMPLES] : 0;

      count = bk7258_aud_dac_write(block,
                                   available ? available : PCM_IRQ_BUDGET);
      if (available != 0)
        {
          g_pcm.tail = (g_pcm.tail + count) % BK7258_PCM_RING_SAMPLES;
          g_pcm.status.buffered -= count;
        }
      else
        {
          g_pcm.status.silence_samples += count;
        }
    }

  g_pcm.status.fifo_samples += count;
  g_pcm.stalls = count ? 0 : g_pcm.stalls + 1;
  if (g_pcm.stalls >= 4)
    {
      g_pcm.status.error = -EIO;
      g_pcm.status.running = false;
      pcm_mask();
      bk7258_board_audio_pa(false);
      bk7258_aud_dac_stop();
      bk7258_aud_adc_stop();
    }
}

static int pcm_validate(uint32_t session)
{
  if (!g_pcm.active || session == 0 || session != g_pcm.session)
    {
      return -EBADF;
    }

  return 0;
}

int bk7258_pcm_start_rate(bool capture, uint32_t rate, uint32_t *session)
{
  struct bk7258_aud_config cfg =
    {
      .samplerate = rate,
      .clksrc = BK7258_AUD_CLK_XTAL,
      .dac_dig_gain = BK7258_AUD_DAC_DIG_GAIN_0DB,
      .dac_ana_gain = BK7258_AUD_DAC_ANA_GAIN_DEF,
      .adc_gain = BK7258_AUD_ADC_GAIN_0DB,
      .mic_gain = BK7258_AUD_MIC_GAIN_DEF
    };

  irqstate_t flags;
  int ret;
  if (session == NULL) return -EINVAL;
  *session = 0;
  if (rate != BK7258_PCM_RATE && (capture || rate != 24000)) return -EINVAL;
  ret = nxmutex_lock(&g_pcm_lock);
  if (ret < 0) return ret;
  ret = bk7258_aud_acquire(&g_pcm);
  if (ret < 0) goto out;
  memset(&g_pcm, 0, sizeof(g_pcm));
  g_pcm.status.capture = capture;
  bk7258_board_audio_pa(false);
  ret = bk7258_aud_initialize();
  if (ret < 0) goto release;
  ret = capture ? bk7258_aud_adc_setup(&cfg) : bk7258_aud_dac_setup(&cfg);
  if (ret < 0) goto cleanup;
  if (!capture)
    {
      int16_t zero[32] =
        {
          0
        };

      bk7258_aud_dac_write(zero, 32);
      bk7258_aud_dac_start();
      ret = nxsig_usleep(20000);
      if (ret < 0) goto cleanup;
      bk7258_board_audio_pa(true);
      ret = nxsig_usleep(20000);
      if (ret < 0) goto cleanup;
      ret = bk7258_aud_dac_mute(false);
      if (ret < 0) goto cleanup;
    }

  flags = enter_critical_section();
  g_pcm.active = true;
  if (++g_generation == 0) ++g_generation;
  g_pcm.session = g_generation;
  g_pcm.status.running = true;
  if (capture)
    {
      g_pcm.settling = PCM_SETTLE;
      bk7258_aud_set_rx_callback(pcm_irq, NULL);
      bk7258_aud_adc_start();
      bk7258_aud_rx_int_enable(true);
    }
  else
    {
      bk7258_aud_set_tx_callback(pcm_irq, NULL);
      bk7258_aud_tx_int_enable(true);
    }

  leave_critical_section(flags);
  *session = g_pcm.session;
  ret = 0;
  goto out;
cleanup:
  pcm_cleanup();
release:
  bk7258_aud_release(&g_pcm);
out:
  nxmutex_unlock(&g_pcm_lock);
  return ret;
}

int bk7258_pcm_start(bool capture, uint32_t *session)
{
  return bk7258_pcm_start_rate(capture, BK7258_PCM_RATE, session);
}

int bk7258_pcm_read(uint32_t session, int16_t *samples, uint32_t count)
{
  irqstate_t flags;
  uint32_t i;
  int ret;
  if (samples == NULL || count == 0) return -EINVAL;
  ret = nxmutex_lock(&g_pcm_lock);
  if (ret < 0) return ret;
  flags = enter_critical_section();
  ret = pcm_validate(session);
  if (ret == 0 && !g_pcm.status.capture) ret = -EPERM;
  if (ret == 0 && g_pcm.status.error) ret = g_pcm.status.error;
  if (ret == 0)
    {
      if (count > BK7258_PCM_FRAME_SAMPLES) count = BK7258_PCM_FRAME_SAMPLES;
      if (count > g_pcm.status.buffered) count = g_pcm.status.buffered;
      for (i = 0; i < count; i++)
        {
          samples[i] = g_pcm.ring[g_pcm.tail];
          g_pcm.ring[g_pcm.tail] = 0;
          g_pcm.tail = (g_pcm.tail + 1) % BK7258_PCM_RING_SAMPLES;
        }

      g_pcm.status.buffered -= count;
      ret = count ? (int)count : -EAGAIN;
    }

  leave_critical_section(flags);
  nxmutex_unlock(&g_pcm_lock);
  return ret;
}

int bk7258_pcm_write(uint32_t session, const int16_t *samples,
                     uint32_t count)
{
  int16_t block[BK7258_PCM_FRAME_SAMPLES];
  irqstate_t flags;
  uint32_t i;
  uint32_t ramp;
  uint32_t limited = 0;
  int ret;
  if (samples == NULL || count == 0) return -EINVAL;
  ret = nxmutex_lock(&g_pcm_lock);
  if (ret < 0) return ret;
  flags = enter_critical_section();
  ret = pcm_validate(session);
  if (ret == 0 && g_pcm.status.capture) ret = -EPERM;
  if (ret == 0 && g_pcm.status.error) ret = g_pcm.status.error;
  if (ret == 0 && g_pcm.draining) ret = -EPIPE;
  leave_critical_section(flags);
  if (ret < 0) goto out;
  if (count > BK7258_PCM_FRAME_SAMPLES) count = BK7258_PCM_FRAME_SAMPLES;
  flags = enter_critical_section();
  if (count > BK7258_PCM_RING_SAMPLES - g_pcm.status.buffered)
    count = BK7258_PCM_RING_SAMPLES - g_pcm.status.buffered;
  leave_critical_section(flags);
  ramp = g_pcm.ramp;
  for (i = 0; i < count; i++)
    {
      int32_t value = samples[i] / 4;
      if (value > 1024)
        {
          value = 1024;
          limited++;
        }

      if (value < -1024)
        {
          value = -1024;
          limited++;
        }

      if (ramp < PCM_RAMP)
        value = value * (int32_t)ramp++ / (int32_t)PCM_RAMP;
      block[i] = value;
    }

  flags = enter_critical_section();
  if (g_pcm.status.error)
    {
      ret = g_pcm.status.error;
      leave_critical_section(flags);
      goto out;
    }

  for (i = 0; i < count; i++)
    {
      g_pcm.ring[g_pcm.head] = block[i];
      g_pcm.head = (g_pcm.head + 1) % BK7258_PCM_RING_SAMPLES;
    }

  g_pcm.status.buffered += count;
  g_pcm.status.limited_samples += limited;
  g_pcm.ramp = ramp;
  leave_critical_section(flags);
  ret = count ? (int)count : -EAGAIN;
out:
  nxmutex_unlock(&g_pcm_lock);
  return ret;
}

int bk7258_pcm_status(uint32_t session, struct bk7258_pcm_status_s *status)
{
  irqstate_t flags;
  int ret;
  if (status == NULL) return -EINVAL;
  ret = nxmutex_lock(&g_pcm_lock);
  if (ret < 0) return ret;
  flags = enter_critical_section();
  ret = pcm_validate(session);
  if (ret == 0) *status = g_pcm.status;
  leave_critical_section(flags);
  nxmutex_unlock(&g_pcm_lock);
  return ret;
}

int bk7258_pcm_drain(uint32_t session, unsigned int timeout_ms)
{
  clock_t start;
  irqstate_t flags;
  bool empty;
  int ret;
  if (timeout_ms == 0 || timeout_ms > 2000) return -EINVAL;
  ret = nxmutex_lock(&g_pcm_lock);
  if (ret < 0) return ret;
  ret = pcm_validate(session);
  if (ret == 0 && g_pcm.status.capture) ret = -EPERM;
  if (ret < 0) goto out;
  flags = enter_critical_section();
  g_pcm.draining = true;
  leave_critical_section(flags);
  start = clock_systime_ticks();
  for (; ; )
    {
      flags = enter_critical_section();
      ret = g_pcm.status.error;
      empty = g_pcm.status.buffered == 0 &&
              (bk7258_aud_fifo_status() & BK7258_AUD_DACL_FIFO_EMPTY) != 0;
      leave_critical_section(flags);
      if (ret < 0 || empty) break;
      if (clock_systime_ticks() - start >= MSEC2TICK(timeout_ms))
        {
          ret = -ETIMEDOUT;
          break;
        }

      ret = nxsig_usleep(10000);
      if (ret < 0) break;
    }

out:
  nxmutex_unlock(&g_pcm_lock);
  return ret;
}

int bk7258_pcm_stop(uint32_t session)
{
  int ret = nxmutex_lock(&g_pcm_lock);
  if (ret < 0) return ret;
  ret = pcm_validate(session);
  if (ret == 0)
    {
      pcm_cleanup();
      explicit_bzero(&g_pcm, sizeof(g_pcm));
      bk7258_aud_release(&g_pcm);
    }

  nxmutex_unlock(&g_pcm_lock);
  return ret;
}
