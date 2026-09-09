/****************************************************************************
 * arch/arm/src/bk7258/include/bk7258_pcm.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef BK7258_PCM_H
#define BK7258_PCM_H
/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_PCM_RATE 16000u
#define BK7258_PCM_FRAME_SAMPLES 320u
#define BK7258_PCM_RING_SAMPLES 4096u

struct bk7258_pcm_status_s
{
  bool capture;
  bool running;
  int error;
  uint32_t buffered;
  uint64_t fifo_samples;
  uint64_t dropped_samples;
  uint64_t silence_samples;
  uint64_t limited_samples;
  uint64_t fifo_boundaries;
};

/* Explicit, half-duplex 16 kHz signed-16 mono sessions, not /dev/audio.
 * No operation is called automatically at boot. A session survives task
 * boundaries; callers must stop it on every exit path. Read/write return a
 * partial sample count (at most 320), or -EAGAIN for an empty/full ring.
 * Playback currently divides by four and clamps to +/-1024 for safety.
 * Ring underruns transmit zeros; capture overruns discard newest samples.
 */

int bk7258_pcm_start(bool capture, uint32_t *session);
/* Cloud PCM16 TTS uses the vendor DAC's native 24 kHz mode. Capture remains
 * 16 kHz. Existing start() callers keep their original rate.
 */

int bk7258_pcm_start_rate(bool capture, uint32_t rate, uint32_t *session);
int bk7258_pcm_read(uint32_t session, int16_t *samples, uint32_t count);
int bk7258_pcm_write(uint32_t session, const int16_t *samples,
                     uint32_t count);
int bk7258_pcm_status(uint32_t session, struct bk7258_pcm_status_s *status);
/* Drain does not accept new writes concurrently; timeout is 1..2000 ms.
 * It leaves the session allocated. Call stop even if drain fails.
 */

int bk7258_pcm_drain(uint32_t session, unsigned int timeout_ms);
int bk7258_pcm_stop(uint32_t session);
#endif
