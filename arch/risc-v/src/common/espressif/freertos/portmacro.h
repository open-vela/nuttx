/****************************************************************************
 * arch/risc-v/src/common/espressif/freertos/portmacro.h
 *
 * Minimal FreeRTOS port-layer shim for the ESP32-P4 JPEG encoder HAL.
 *
 * The esp-hal-3rdparty upper_hal_jpeg / upper_hal_dma drivers include
 * <freertos/FreeRTOS.h> and use a small set of FreeRTOS types and macros.
 * This shim maps those onto NuttX primitives so the JPEG/DMA2D HAL can be
 * compiled into the NuttX kernel.
 *
 * Base types (BaseType_t/UBaseType_t/TickType_t), portMAX_DELAY and
 * portMUX_INITIALIZER_UNLOCKED are provided by the NuttX platform layer
 * (platform/os.h).  We reuse those definitions instead of re-defining them,
 * to avoid macro-redefinition conflicts when a driver includes both this
 * header and esp_private/critical_section.h (which pulls in platform/os.h).
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __FREERTOS_PORTMACRO_H
#define __FREERTOS_PORTMACRO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <nuttx/spinlock.h>

#include "platform/os.h"   /* BaseType_t, UBaseType_t, TickType_t, portMAX_DELAY,
                            * portMUX_INITIALIZER_UNLOCKED, OS_SPINLOCK_TYPE */

/* portMUX_TYPE must be rspinlock_t: the DMA2D driver passes the same
 * spinlock member to esp_os_enter_critical_safe(), and OS_SPINLOCK_TYPE
 * is defined as rspinlock_t by the NuttX platform layer (platform/os.h).
 */
#ifndef portMUX_TYPE
typedef OS_SPINLOCK_TYPE portMUX_TYPE;
#endif

/* FreeRTOS-style boolean values.  platform/os.h defines pdPASS as 0 (a NuttX
 * convention); the JPEG/DMA2D drivers only test pdTRUE/pdFALSE.
 */
#ifndef pdTRUE
#define pdTRUE  1
#endif
#ifndef pdFALSE
#define pdFALSE 0
#endif
#ifndef pdFAIL
#define pdFAIL  pdFALSE
#endif

/* portBASE_TYPE is a generic "signed width type" in FreeRTOS.  The JPEG
 * driver uses it for ISR "higher priority task woken" flags.
 */
#ifndef portBASE_TYPE
typedef BaseType_t portBASE_TYPE;
#endif

/* Convert milliseconds to ticks using the NuttX clock helper. */
#ifndef pdMS_TO_TICKS
#define pdMS_TO_TICKS(ms) MSEC2TICK(ms)
#endif

/* Tick rate and period, derived from the NuttX CONFIG_USEC_PER_TICK. */
#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ (1000000UL / CONFIG_USEC_PER_TICK)
#endif
#ifndef portTICK_PERIOD_MS
#define portTICK_PERIOD_MS (CONFIG_USEC_PER_TICK / 1000)
#endif

/* NuttX schedules out of nxsem_post()/file_mq_send() called from an ISR;
 * nothing else needs to happen here.  The drivers call this with no
 * argument, so accept a variable number of arguments.
 */
#ifndef portYIELD_FROM_ISR
#define portYIELD_FROM_ISR(...) do { (void)0; } while (0)
#endif

/* The HAL code uses the esp_os_enter_critical* helpers (platform/os.h) for
 * critical sections.  These classic macros are kept for defensive coverage.
 */
#ifndef portENTER_CRITICAL
#define portENTER_CRITICAL()                 enter_critical_section()
#endif
#ifndef portEXIT_CRITICAL
#define portEXIT_CRITICAL()                  leave_critical_section(0)
#endif
#ifndef portSET_INTERRUPT_MASK_FROM_ISR
#define portSET_INTERRUPT_MASK_FROM_ISR()    0
#endif
#ifndef portCLEAR_INTERRUPT_MASK_FROM_ISR
#define portCLEAR_INTERRUPT_MASK_FROM_ISR(x) do { (void)(x); } while (0)
#endif

#endif /* __FREERTOS_PORTMACRO_H */
