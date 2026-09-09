/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_audio_diag.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef BK7258_AUDIO_DIAG_H
#define BK7258_AUDIO_DIAG_H
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_AUDIO_CLIP_SAMPLES 16000u

enum bk7258_audio_operation_e
{
  BK7258_AUDIO_STATUS,
  BK7258_AUDIO_TONE,
  BK7258_AUDIO_RECORD,
  BK7258_AUDIO_PLAY,
  BK7258_AUDIO_CLEAR,
  BK7258_AUDIO_SILENCE
};

struct bk7258_audio_report_s
{
  uint32_t device_id;
  uint32_t samples;
  uint32_t interrupts;
  uint32_t fifo_faults;
  uint32_t recorded;
  int32_t dc;
  uint32_t rms;
  uint32_t peak;
  uint32_t clipped;
  uint32_t raw_samples;
  uint32_t elapsed_ms;
  uint32_t max_irq_gap_ms;
  uint32_t max_batch;
  uint32_t status_before_or;
  uint32_t status_after_or;
  uint32_t full_after;
  uint32_t empty_after;
  uint32_t adc_threshold;
};

int bk7258_audio_diag(enum bk7258_audio_operation_e operation,
                      struct bk7258_audio_report_s *report);

/* Divisors 8, 4, 2 and 1 only; the peak clamp applies at every level. */

int bk7258_audio_replay(unsigned int divisor,
                        struct bk7258_audio_report_s *report);
/* Copy one complete, unprocessed recording while holding the diagnostic
 * lock.
 * Returns the sample count or a negative errno. Does not access hardware.
 */

int bk7258_audio_copy(int16_t *samples, uint32_t capacity);
#endif
