/****************************************************************************
 * arch/risc-v/src/esp32p4/esp32p4_ppa.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP32P4_PPA_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP32P4_PPA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PPA Color Formats */

#define ESP32P4_PPA_COLOR_RGB565       0
#define ESP32P4_PPA_COLOR_RGB888       1
#define ESP32P4_PPA_COLOR_ARGB8888     2
#define ESP32P4_PPA_COLOR_GRAY8        3
#define ESP32P4_PPA_COLOR_YUV420       4
#define ESP32P4_PPA_COLOR_YUV422       5

/* PPA Rotation Angles (Counter-Clockwise) */

#define ESP32P4_PPA_ROT_0              0
#define ESP32P4_PPA_ROT_90             1
#define ESP32P4_PPA_ROT_180            2
#define ESP32P4_PPA_ROT_270            3

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct esp32p4_ppa_srm_config_s
{
  const void *src_buf;
  void       *dst_buf;
  uint16_t    src_w;
  uint16_t    src_h;
  uint16_t    dst_w;
  uint16_t    dst_h;
  int         color_fmt;
  int         rotation;
  bool        mirror_x;
  bool        mirror_y;
};

struct esp32p4_ppa_blend_config_s
{
  const void *bg_buf;
  const void *fg_buf;
  void       *dst_buf;
  uint16_t    w;
  uint16_t    h;
  uint8_t     alpha;
  int         color_fmt;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: esp32p4_ppa_init
 *
 * Description:
 *   Initialize the ESP32-P4 PPA (Pixel Processing Accelerator) hardware
 *   peripheral, enable the bus clock and release reset.
 *
 * Returned Value:
 *   OK on success, or a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_ppa_init(void);

/****************************************************************************
 * Name: esp32p4_ppa_deinit
 *
 * Description:
 *   Deinitialize the ESP32-P4 PPA hardware and disable its bus clock.
 *
 * Returned Value:
 *   OK on success.
 *
 ****************************************************************************/

int esp32p4_ppa_deinit(void);

/****************************************************************************
 * Name: esp32p4_ppa_scale
 *
 * Description:
 *   Perform hardware image scaling via the SRM engine.
 *
 * Parameters:
 *   src       - Pointer to source image buffer
 *   src_w     - Source image width
 *   src_h     - Source image height
 *   dst       - Pointer to destination image buffer
 *   dst_w     - Destination image width
 *   dst_h     - Destination image height
 *   color_fmt - Image color format (ESP32P4_PPA_COLOR_*)
 *
 * Returned Value:
 *   OK on success, or a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_ppa_scale(const void *src, uint16_t src_w, uint16_t src_h,
                      void *dst, uint16_t dst_w, uint16_t dst_h,
                      int color_fmt);

/****************************************************************************
 * Name: esp32p4_ppa_rotate
 *
 * Description:
 *   Perform hardware image rotation (0, 90, 180, 270 deg) via SRM engine.
 *
 * Parameters:
 *   src       - Pointer to source image buffer
 *   w         - Image width
 *   h         - Image height
 *   dst       - Pointer to destination image buffer
 *   rotation  - Rotation angle (ESP32P4_PPA_ROT_*)
 *   color_fmt - Image color format
 *
 * Returned Value:
 *   OK on success, or a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_ppa_rotate(const void *src, uint16_t w, uint16_t h,
                       void *dst, int rotation, int color_fmt);

/****************************************************************************
 * Name: esp32p4_ppa_blend
 *
 * Description:
 *   Perform hardware 2D Alpha blending of two images (FG over BG).
 *
 * Parameters:
 *   bg        - Pointer to background image buffer
 *   fg        - Pointer to foreground image buffer
 *   dst       - Pointer to destination output buffer
 *   w         - Blending rectangle width
 *   h         - Blending rectangle height
 *   alpha     - Alpha value (0 = 100% BG, 255 = 100% FG)
 *   color_fmt - Image color format
 *
 * Returned Value:
 *   OK on success, or a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_ppa_blend(const void *bg, const void *fg, void *dst,
                      uint16_t w, uint16_t h, uint8_t alpha,
                      int color_fmt);

/****************************************************************************
 * Name: esp32p4_ppa_fill
 *
 * Description:
 *   Perform fast hardware solid color filling into a buffer.
 *
 * Parameters:
 *   dst       - Pointer to destination buffer
 *   w         - Fill rectangle width
 *   h         - Fill rectangle height
 *   color     - 32-bit ARGB/RGB color value
 *   color_fmt - Destination color format
 *
 * Returned Value:
 *   OK on success, or a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_ppa_fill(void *dst, uint16_t w, uint16_t h,
                     uint32_t color, int color_fmt);

/****************************************************************************
 * Name: esp32p4_ppa_csc
 *
 * Description:
 *   Perform hardware Color Space Conversion (e.g. RGB565 -> RGB888 / GRAY8).
 *
 * Parameters:
 *   src       - Pointer to source image buffer
 *   dst       - Pointer to destination image buffer
 *   w         - Image width
 *   h         - Image height
 *   src_fmt   - Source color format
 *   dst_fmt   - Destination color format
 *
 * Returned Value:
 *   OK on success, or a negated errno value on failure.
 *
 ****************************************************************************/

int esp32p4_ppa_csc(const void *src, void *dst, uint16_t w, uint16_t h,
                    int src_fmt, int dst_fmt);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP32P4_PPA_H */
