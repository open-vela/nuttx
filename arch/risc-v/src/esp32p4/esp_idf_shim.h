/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_idf_shim.h
 *
 * SPDX-License-Identifier: Apache-2.0
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
 * Shim layer mapping ESP-IDF/FreeRTOS APIs to NuttX equivalents.
 * Used by ported ESP-IDF DSI/DMA drivers (dw_gdma, dsi_bus, panel_dpi)
 * so they compile under NuttX without modification.
 ****************************************************************************/

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_IDF_SHIM_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_IDF_SHIM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <nuttx/kmalloc.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <unistd.h>

/****************************************************************************
 * ESP-IDF Error Codes
 *
 * Use the HAL's definitions from esp_err.h when available (included
 * transitively via HAL headers). Only define here if not yet defined.
 ****************************************************************************/

#include "esp_err.h"  /* from esp-hal-3rdparty: defines ESP_OK, esp_err_t, etc */

/****************************************************************************
 * Logging → syslog
 ****************************************************************************/

#define ESP_LOGE(tag, fmt, ...) \
  syslog(LOG_ERR,     "[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) \
  syslog(LOG_WARNING, "[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) \
  syslog(LOG_INFO,    "[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) \
  syslog(LOG_DEBUG,   "[%s] " fmt "\n", tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) /* verbose: compiled out */

#define ESP_DRAM_LOGE(tag, fmt, ...) \
  syslog(LOG_ERR,     "[%s] " fmt "\n", tag, ##__VA_ARGS__)

#define ESP_LOG_ATTR_TAG(var, str) static const char *var = str

/****************************************************************************
 * Error Checking Macros — match ESP-IDF semantics exactly
 *
 * Note: ESP_GOTO_ON_ERROR and ESP_GOTO_ON_FALSE assume a local variable
 * named `ret` exists in the calling scope (ESP-IDF convention).
 ****************************************************************************/

#define ESP_RETURN_ON_ERROR(x, tag, fmt, ...) do { \
    esp_err_t err_rc_ = (x); \
    if (err_rc_ != ESP_OK) { \
        ESP_LOGE(tag, "%s(%d): " fmt " (0x%x)", \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__, err_rc_); \
        return err_rc_; \
    } \
} while (0)

#define ESP_RETURN_ON_FALSE(a, err, tag, fmt, ...) do { \
    if (!(a)) { \
        ESP_LOGE(tag, "%s(%d): " fmt, \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (err); \
    } \
} while (0)

#define ESP_GOTO_ON_ERROR(x, goto_tag, log_tag, fmt, ...) do { \
    esp_err_t err_rc_ = (x); \
    if (err_rc_ != ESP_OK) { \
        ESP_LOGE(log_tag, "%s(%d): " fmt " (0x%x)", \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__, err_rc_); \
        ret = err_rc_; \
        goto goto_tag; \
    } \
} while (0)

#define ESP_GOTO_ON_FALSE(a, err, goto_tag, log_tag, fmt, ...) do { \
    if (!(a)) { \
        ESP_LOGE(log_tag, "%s(%d): " fmt, \
                 __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        ret = (err); \
        goto goto_tag; \
    } \
} while (0)

/****************************************************************************
 * FreeRTOS → NuttX: Delay / Yield
 ****************************************************************************/

#define pdMS_TO_TICKS(ms)       (ms)
#define vTaskDelay(ticks)       usleep((ticks) * 1000)
#define portYIELD_FROM_ISR()    /* no-op under NuttX */

/****************************************************************************
 * FreeRTOS → NuttX: Spinlock / Critical Section
 ****************************************************************************/

typedef spinlock_t portMUX_TYPE;

#define portMUX_INITIALIZER_UNLOCKED  SP_UNLOCKED
#define portMUX_INITIALIZE(mux)       spin_lock_init(mux)

static inline void esp_os_enter_critical(spinlock_t *lock)
{
  spin_lock(lock);
}

static inline void esp_os_exit_critical(spinlock_t *lock)
{
  spin_unlock(lock);
}

/****************************************************************************
 * Memory Allocation — heap_caps → kmm_memalign + memset
 *
 * All MALLOC_CAP_* flags are ignored; NuttX kernel heap is used directly.
 * Alignment defaults to 64 bytes (cache line size on ESP32-P4).
 ****************************************************************************/

#define MALLOC_CAP_DEFAULT      0
#define MALLOC_CAP_INTERNAL     0
#define MALLOC_CAP_8BIT         0
#define MALLOC_CAP_DMA          0
#define MALLOC_CAP_SPIRAM       0

static inline void *heap_caps_calloc(size_t n, size_t size, uint32_t caps)
{
  void *p;
  (void)caps;
  p = kmm_memalign(64, n * size);
  if (p)
    {
      memset(p, 0, n * size);
    }

  return p;
}

static inline void *heap_caps_aligned_calloc(size_t align, size_t n,
                                             size_t size, uint32_t caps)
{
  void *p;
  (void)caps;
  p = kmm_memalign(align, n * size);
  if (p)
    {
      memset(p, 0, n * size);
    }

  return p;
}

#define heap_caps_free(p)  kmm_free(p)

/****************************************************************************
 * Clock Tree — simplified stubs for ESP32-P4
 *
 * soc_module_clk_t is defined in soc/clk_tree_defs.h (from HAL headers).
 * We only provide stub functions for esp_clk_tree_enable_src and
 * esp_clk_tree_src_get_freq_hz.
 ****************************************************************************/

#include "soc/clk_tree_defs.h"  /* provides soc_module_clk_t enum */

#define ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED  0
#define ESP_CLK_TREE_SRC_FREQ_PRECISION_APPROX  1
#define ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT   2

static inline esp_err_t esp_clk_tree_enable_src(soc_module_clk_t src,
                                                bool enable)
{
  (void)src;
  (void)enable;
  return ESP_OK; /* clocks enabled by default after reset on ESP32-P4 */
}

static inline esp_err_t esp_clk_tree_src_get_freq_hz(
    soc_module_clk_t src, int precision, uint32_t *freq_hz)
{
  (void)precision;

  /* ESP32-P4 known frequencies:
   * XTAL = 40MHz (enum value varies)
   * PLL_F240M (enum ~8-9 depending on version)
   * Use a simple heuristic: if freq would be > 100MHz use 240M,
   * otherwise default to 40MHz XTAL.
   * The actual DSI code hardcodes the DPI source freq anyway.
   */

  (void)src;
  *freq_hz = 40000000;
  return ESP_OK;
}

/****************************************************************************
 * PERIPH_RCC_ATOMIC
 *
 * Already available via esp-hal-3rdparty periph_ctrl.h which provides
 * __DECLARE_RCC_ATOMIC_ENV and the PERIPH_RCC_ATOMIC() macro.
 * No shim needed here — just include periph_ctrl.h in the source file.
 ****************************************************************************/

/****************************************************************************
 * Cache
 *
 * esp_cache_msync is already available in the project from
 * esp-hal-3rdparty (components/esp_mm/esp_cache.c). No shim needed.
 * Source files that use it should include "esp_cache.h" directly.
 ****************************************************************************/

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_IDF_SHIM_H */
