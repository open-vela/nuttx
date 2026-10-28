/****************************************************************************
 * arch/arm/src/stm32h7/stm32_sai_capture.h
 *
 * Continuous PDM capture on top of the SAI receive path.
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
 * Why this exists
 *
 * The one-shot receive path (I2S_RECEIVE) ends by clearing SAIEN in the DMA
 * completion callback, so every buffer costs a full stop and restart of the
 * block.  Two consecutive buffers therefore cannot be presented as one
 * continuous stream: the gap is whatever the worker takes to notice the
 * completion plus the time to reprogram, and at 128 KB/s even a few
 * milliseconds is a hole in the PDM bitstream.  Decimation state carried
 * across that hole produces a click, not a recording.
 *
 * This interface instead wants a region the DMA writes in circular mode and
 * starts exactly once.  The CPU then consumes what the DMA has already
 * written, tracked through the position callback, and never stops the block
 * for the whole capture.  Stopping is a separate, explicit call.
 *
 * Buffer rules, because a DMA target is not an ordinary buffer:
 *   - the memory must be reachable by DMA1/DMA2, i.e. not DTCM and not ITCM;
 *   - one DMA half-transfer and transfer-complete interrupt fire per ring
 *     wrap, so the ring length must be a whole number of DMA transfers and
 *     the DMA count register is 16 bits wide (65535 max);
 *   - the CPU must not read the half the DMA is filling.  The callback tells
 *     the client which half just finished so it can read the other one.
 *     If the client falls behind, samples are lost; that is counted, never
 *     silently papered over.
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32H7_STM32_SAI_CAPTURE_H
#define __ARCH_ARM_SRC_STM32H7_STM32_SAI_CAPTURE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <nuttx/audio/i2s.h>

#ifdef CONFIG_STM32H7_SAI_CAPTURE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The ring is divided in two halves by the DMA half-transfer interrupt, so
 * its length must be even and a whole number of transfer units.  These are
 * the bounds the driver enforces rather than assumptions the client makes.
 */

#define STM32H7_SAI_CAPTURE_MINBYTES   (256)
#define STM32H7_SAI_CAPTURE_MAXBYTES   (65534)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Called from interrupt context when the DMA finishes a half of the ring.
 * Stop may deliver a final partial region from task context.
 *
 *   half  0 = first half full, 1 = second half full
 *   data  start of the half that just finished
 *   bytes length of that half
 *
 * The handler may run in the DMA interrupt: it may count and copy into a
 * pre-allocated region, and must not print, allocate, block or take a
 * mutex that a thread may hold while waiting for it.
 */

typedef void (*stm32_sai_capture_cb_t)(void *arg, int half,
                                       const uint8_t *data, size_t bytes);

/* Capture statistics.  Every field is a counter, so a client can subtract
 * two snapshots; none of them is ever cleared by the driver.
 */

struct stm32_sai_capture_stats
{
  uint32_t tailbytes;      /* Cumulative partial bytes delivered at stop */
  uint32_t halves;         /* Half-ring completions delivered */
  uint32_t wraps;          /* Full ring wraps (transfer-complete events) */
  uint32_t dropped;        /* Unsafe or ambiguous DMA half boundaries */
  uint32_t overruns;       /* SAI OVRUDR seen at a completion */
  uint32_t clockerrors;    /* SAI WCKCFG seen at a completion */
  uint32_t dmaerrors;      /* DMA TEIF/DMEIF at a completion */
  uint32_t starts;         /* Successful starts */
  uint32_t stops;          /* Successful stops */
  uint32_t setup_failures; /* Starts refused before the DMA was enabled */
  uint32_t busy;           /* Starts refused because a capture was running */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**
 * @brief  Start continuous capture into a caller-owned ring.
 *
 * The driver programs the DMA in circular mode over the whole ring and
 * enables the block once; it does not hand the ring back until
 * stm32_sai_capture_stop() is called.
 *
 * @param  dev         SAI device from stm32_sai_initialize().
 * @param  samplerate  Output sample rate the decimation targets
 *                     (e.g. 16000).  This programs the divider.  Delivered
 *                     memory bytes were measured 3x faster at the 48 kHz
 *                     default than at 16 kHz.  It is not a wire-clock
 *                     measurement.
 * @param  ring   DMA-reachable buffer, aligned to 32 bytes.
 * @param  bytes  Ring length: multiple of 64, <= 65534; both halves must
 *                occupy whole cache lines.
 * @param  cb     Called from interrupt context per half ring.  Required.
 * @param  arg    Passed to cb.
 *
 * @retval 0 on success.
 * @retval -EINVAL bad arguments, or the ring is not on an even transfer-unit
 *         boundary.
 * @retval -EFAULT the ring is in memory the DMA cannot reach.
 * @retval -E2BIG  the ring exceeds the 16-bit transfer count.
 * @retval -EBUSY  a capture is already running on this block.
 * @retval -ENODEV dev is not a capture-capable SAI block.
 */

int stm32_sai_capture_start(struct i2s_dev_s *dev, uint32_t samplerate,
                            uint8_t *ring, size_t bytes,
                            stm32_sai_capture_cb_t cb, void *arg);

/**
 * @brief  Stop continuous capture and wait for the DMA to be released.
 *
 * Disables the DMA stream and the block, then waits a bounded time for both
 * to report disabled.  The ring belongs to the caller again only after this
 * returns; the callback will not fire afterwards.
 *
 * @param  dev      SAI device.
 * @param  wait_ms  Upper bound on the post-disable wait; 0 uses a default.
 *
 * @retval 0 capture stopped, or none was running.
 * @retval -ETIMEDOUT the DMA or block did not report disabled in time.  The
 *         caller must not reuse the ring; treat the block as unusable until
 *         it is reinitialized.
 */

int stm32_sai_capture_stop(struct i2s_dev_s *dev, unsigned int wait_ms);

/** @brief  Read the counters.  Safe from any thread. */

void stm32_sai_capture_stats(struct i2s_dev_s *dev,
                             struct stm32_sai_capture_stats *out);

/**
 * @brief  Number of bytes the DMA has written since the last wrap.
 *
 * This is the DMA's own count register, converted to elapsed bytes.  A
 * client that wants to know how far behind it is can compare this against
 * the position it has consumed.
 *
 * @param  dev  SAI device.
 * @retval >=0 elapsed bytes in the current pass, or -ENODEV.
 */

int stm32_sai_capture_position(struct i2s_dev_s *dev);

#endif /* CONFIG_STM32H7_SAI_CAPTURE */
#endif /* __ARCH_ARM_SRC_STM32H7_STM32_SAI_CAPTURE_H */
