/****************************************************************************
 * arch/risc-v/src/common/espressif/freertos/semphr.h
 *
 * Minimal FreeRTOS semaphore compatibility layer for the ESP32-P4 JPEG
 * encoder HAL.
 *
 * The JPEG encoder HAL uses a binary semaphore (codec_mutex) in classic
 * mutex fashion: xSemaphoreTake() guards the encoder pipeline, and
 * xSemaphoreGive() releases it.  We implement this directly on NuttX
 * nxsem_* primitives (see freertos_shim.c) rather than on top of
 * esp_os_wait_sem_timeout(), whose "drain then wait" semantics would
 * consume the initial give and dead-lock the mutex.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __FREERTOS_SEMPHR_H
#define __FREERTOS_SEMPHR_H

#include "FreeRTOS.h"

/* In the real FreeRTOS, semphr.h is built on top of queue.h: a semaphore is
 * a queue of one slot and SemaphoreHandle_t aliases QueueHandle_t.  The
 * jpeg_private.h header relies on this, referencing QueueHandle_t after
 * including only FreeRTOS.h + semphr.h.  Match that contract here.
 */
#include "queue.h"

/* SemaphoreHandle_t is an opaque handle to a heap-allocated NuttX
 * semaphore (see struct freertos_sem_s in freertos_shim.c).
 */
typedef void *SemaphoreHandle_t;

SemaphoreHandle_t xSemaphoreCreateBinaryWithCaps(UBaseType_t uxCaps);

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore,
                          TickType_t xBlockTime);

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore);

BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore,
                                 BaseType_t *pxHigherPriorityTaskWoken);

void vSemaphoreDeleteWithCaps(SemaphoreHandle_t xSemaphore);

#endif /* __FREERTOS_SEMPHR_H */
