/****************************************************************************
 * arch/risc-v/src/common/espressif/freertos/task.h
 *
 * Minimal FreeRTOS task compatibility layer for the ESP32-P4 JPEG encoder
 * HAL.
 *
 * The esp-hal-3rdparty drivers reference TaskHandle_t and a few task
 * helpers.  NuttX uses pid_t for task handles; the JPEG/DMA2D drivers only
 * pass handles around as opaque values, so a void pointer is sufficient.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __FREERTOS_TASK_H
#define __FREERTOS_TASK_H

#include "FreeRTOS.h"

#endif /* __FREERTOS_TASK_H */
