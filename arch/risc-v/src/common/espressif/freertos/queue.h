/****************************************************************************
 * arch/risc-v/src/common/espressif/freertos/queue.h
 *
 * Minimal FreeRTOS queue compatibility layer for the ESP32-P4 JPEG encoder
 * HAL.
 *
 * FreeRTOS queues are mapped onto the NuttX message queue adapter provided
 * by platform/os.c (esp_os_queue_create_with_caps and friends).  The
 * functions that need FreeRTOS pdTRUE/pdFALSE return semantics are wrapped
 * here; the underlying esp_os_* helpers return esp_err_t.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __FREERTOS_QUEUE_H
#define __FREERTOS_QUEUE_H

#include "FreeRTOS.h"

/* QueueHandle_t is an opaque handle.  The underlying object is the NuttX
 * message-queue adapter (struct mq_adpt *) returned by
 * esp_os_queue_create_with_caps().
 */
typedef void *QueueHandle_t;

/****************************************************************************
 * Name: xQueueCreateWithCaps
 *
 * Description:
 *   Create a queue with a specific allocation capability.  Maps onto
 *   esp_os_queue_create_with_caps().
 *
 ****************************************************************************/

static inline QueueHandle_t xQueueCreateWithCaps(UBaseType_t uxQueueLength,
                                                 UBaseType_t uxItemSize,
                                                 UBaseType_t uxCaps)
{
  return esp_os_queue_create_with_caps((size_t)uxQueueLength,
                                       (size_t)uxItemSize,
                                       (uint32_t)uxCaps);
}

/****************************************************************************
 * Name: vQueueDeleteWithCaps
 *
 ****************************************************************************/

static inline void vQueueDeleteWithCaps(QueueHandle_t xQueue)
{
  esp_os_queue_delete_with_caps(xQueue);
}

/****************************************************************************
 * Name: xQueueSendFromISR
 *
 * Description:
 *   Post an item to the back of a queue from an ISR.  Returns pdTRUE on
 *   success (mirroring esp_os_queue_send_from_isr returning ESP_OK).
 *
 ****************************************************************************/

static inline BaseType_t xQueueSendFromISR(QueueHandle_t xQueue,
                                           const void *pvItemToQueue,
                                           BaseType_t *pxHigherPriorityTaskWoken)
{
  esp_err_t err = esp_os_queue_send_from_isr(xQueue, (FAR void *)pvItemToQueue,
                                             (FAR void *)pxHigherPriorityTaskWoken);
  if (err == ESP_OK)
    {
      return pdTRUE;
    }
  else
    {
      return pdFALSE;
    }
}

/****************************************************************************
 * Name: xQueueReceive
 *
 * Description:
 *   Receive an item from a queue, blocking up to xTicksToWait.  Returns
 *   pdTRUE on success.
 *
 ****************************************************************************/

static inline BaseType_t xQueueReceive(QueueHandle_t xQueue,
                                       void *pvBuffer,
                                       TickType_t xTicksToWait)
{
  esp_err_t err = esp_os_queue_receive(xQueue, pvBuffer,
                                       (uint32_t)xTicksToWait);
  if (err == ESP_OK)
    {
      return pdTRUE;
    }
  else
    {
      return pdFALSE;
    }
}

/****************************************************************************
 * Name: xQueueReset
 *
 * Description:
 *   Reset a queue to its empty state.  Implemented by draining all pending
 *   messages.  Always returns pdTRUE (pdPASS).
 *
 ****************************************************************************/

BaseType_t xQueueReset(QueueHandle_t xQueue);

#endif /* __FREERTOS_QUEUE_H */
