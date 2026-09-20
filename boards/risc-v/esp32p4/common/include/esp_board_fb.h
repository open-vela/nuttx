/****************************************************************************
 * boards/risc-v/esp32p4/common/include/esp_board_fb.h
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

#ifndef __BOARDS_RISCV_ESP32P4_COMMON_INCLUDE_ESP_BOARD_FB_H
#define __BOARDS_RISCV_ESP32P4_COMMON_INCLUDE_ESP_BOARD_FB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: board_fb_initialize
 *
 * Description:
 *   Initialize the MIPI-DSI framebuffer and register /dev/fb0.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_fb_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOARDS_RISCV_ESP32P4_COMMON_INCLUDE_ESP_BOARD_FB_H */
