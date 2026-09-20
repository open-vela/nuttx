/****************************************************************************
 * arch/risc-v/src/common/espressif/freertos/FreeRTOS.h
 *
 * Minimal FreeRTOS compatibility layer for the ESP32-P4 JPEG encoder HAL.
 *
 * Provides the FreeRTOS types and configuration macros referenced by the
 * esp-hal-3rdparty upper_hal_jpeg / upper_hal_dma drivers, mapping them onto
 * NuttX primitives via platform/os.h and the freertos shim.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __FREERTOS_FREERTOS_H
#define __FREERTOS_FREERTOS_H

#include "portmacro.h"

/* TaskHandle_t is an opaque handle to a task.  NuttX uses pid_t for task
 * handles; the JPEG/DMA2D drivers only pass it around as an opaque value.
 */
typedef void *TaskHandle_t;

/* __containerof is defined by esp_attr.h in the upstream esp-hal-3rdparty,
 * but the NuttX port has trimmed that header.  The DMA2D driver uses it
 * (upper_hal_dma/src/dma2d.c).  Provide the same GCC statement-expression
 * implementation here so every driver that includes FreeRTOS.h gets it.
 */
#ifndef __containerof
#define __containerof(ptr, type, member)                                        \
  __extension__({ const __typeof__(((type *)0)->member) *__mptr = (ptr);        \
                  (type *)((char *)__mptr - offsetof(type, member)); })
#endif

#endif /* __FREERTOS_FREERTOS_H */
