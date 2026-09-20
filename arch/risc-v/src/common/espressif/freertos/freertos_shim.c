/****************************************************************************
 * arch/risc-v/src/common/espressif/freertos/freertos_shim.c
 *
 * Implementation of the minimal FreeRTOS compatibility shim for the
 * ESP32-P4 JPEG encoder HAL.
 *
 * Semaphores are implemented directly on NuttX nxsem_* primitives.  We do
 * NOT use esp_os_wait_sem_timeout() here: that helper drains the semaphore
 * before waiting, which is correct for event-style semaphores but would
 * consume the initial xSemaphoreGive() of the JPEG codec_mutex and
 * dead-lock the first xSemaphoreTake().
 *
 * xQueueReset() drains a NuttX message queue (the underlying storage for
 * the esp_os_queue_* adapter), returning the queue to its empty state.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/kmalloc.h>
#include <nuttx/semaphore.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

/* platform/os.h pulls in struct mq_adpt, whose msgsize field we use to
 * size the drain buffer in xQueueReset().
 */
#include "platform/os.h"

/* The semaphore object stored behind a SemaphoreHandle_t. */
struct freertos_sem_s
{
  sem_t sem;
};

/****************************************************************************
 * Name: xSemaphoreCreateBinaryWithCaps
 *
 * Description:
 *   Allocate a binary semaphore (initial count 0) from the heap.
 *
 ****************************************************************************/

SemaphoreHandle_t xSemaphoreCreateBinaryWithCaps(UBaseType_t uxCaps)
{
  FAR struct freertos_sem_s *sem;

  UNUSED(uxCaps);

  sem = (FAR struct freertos_sem_s *)kmm_malloc(sizeof(*sem));
  if (sem == NULL)
    {
      return NULL;
    }

  nxsem_init(&sem->sem, 0, 0);
  return (SemaphoreHandle_t)sem;
}

/****************************************************************************
 * Name: xSemaphoreTake
 *
 * Description:
 *   Wait for a semaphore, blocking up to xBlockTime ticks.
 *
 ****************************************************************************/

BaseType_t xSemaphoreTake(SemaphoreHandle_t xSemaphore,
                          TickType_t xBlockTime)
{
  FAR struct freertos_sem_s *sem = (FAR struct freertos_sem_s *)xSemaphore;
  int ret;

  if (sem == NULL)
    {
      return pdFALSE;
    }

  if (xBlockTime == portMAX_DELAY)
    {
      ret = nxsem_wait(&sem->sem);
    }
  else
    {
      ret = nxsem_tickwait(&sem->sem, xBlockTime);
    }

  return (ret == OK) ? pdTRUE : pdFALSE;
}

/****************************************************************************
 * Name: xSemaphoreGive
 *
 * Description:
 *   Release a semaphore.  nxsem_post() is safe to call from task context
 *   and from ISR context.
 *
 ****************************************************************************/

BaseType_t xSemaphoreGive(SemaphoreHandle_t xSemaphore)
{
  FAR struct freertos_sem_s *sem = (FAR struct freertos_sem_s *)xSemaphore;
  int ret;

  if (sem == NULL)
    {
      return pdFALSE;
    }

  ret = nxsem_post(&sem->sem);
  return (ret == OK) ? pdTRUE : pdFALSE;
}

/****************************************************************************
 * Name: xSemaphoreGiveFromISR
 *
 * Description:
 *   Release a semaphore from an ISR.  NuttX nxsem_post() is ISR-safe.
 *
 ****************************************************************************/

BaseType_t xSemaphoreGiveFromISR(SemaphoreHandle_t xSemaphore,
                                 BaseType_t *pxHigherPriorityTaskWoken)
{
  BaseType_t ret = xSemaphoreGive(xSemaphore);

  if (pxHigherPriorityTaskWoken != NULL)
    {
      *pxHigherPriorityTaskWoken = pdFALSE;
    }

  return ret;
}

/****************************************************************************
 * Name: vSemaphoreDeleteWithCaps
 *
 * Description:
 *   Destroy a semaphore created with xSemaphoreCreateBinaryWithCaps().
 *
 ****************************************************************************/

void vSemaphoreDeleteWithCaps(SemaphoreHandle_t xSemaphore)
{
  FAR struct freertos_sem_s *sem = (FAR struct freertos_sem_s *)xSemaphore;

  if (sem != NULL)
    {
      nxsem_destroy(&sem->sem);
      kmm_free(sem);
    }
}

/****************************************************************************
 * Name: xQueueReset
 *
 * Description:
 *   Drain a NuttX message queue back to its empty state.  The queue handle
 *   is the esp_os_queue_handle_t adapter (struct mq_adpt *).  Non-blocking
 *   receives are attempted until the queue reports empty.
 *
 ****************************************************************************/

BaseType_t xQueueReset(QueueHandle_t xQueue)
{
  FAR struct mq_adpt *mq_adpt = (FAR struct mq_adpt *)xQueue;
  FAR uint8_t *buf;
  size_t msgsize;
  esp_err_t err;

  if (mq_adpt == NULL)
    {
      return pdFAIL;
    }

  msgsize = mq_adpt->msgsize;
  buf = (FAR uint8_t *)kmm_malloc(msgsize);
  if (buf == NULL)
    {
      return pdFAIL;
    }

  /* Drain until the queue is empty.  esp_os_queue_receive() with
   * ticks=0 performs a non-blocking receive and returns ESP_FAIL when the
   * queue is empty.
   */

  do
    {
      err = esp_os_queue_receive((esp_os_queue_handle_t)mq_adpt, buf, 0);
    }
  while (err == ESP_OK);

  kmm_free(buf);
  return pdTRUE;
}
