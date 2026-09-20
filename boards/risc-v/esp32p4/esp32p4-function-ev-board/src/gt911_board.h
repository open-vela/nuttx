/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/gt911_board.h
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

#ifndef __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_GT911_BOARD_H
#define __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_GT911_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A single decoded touch point.  Coordinates are raw GT911 panel values
 * (already 12-bit clamped; no rotation applied).
 */

struct gt911_touch_s
{
  uint16_t x;      /* X coordinate */
  uint16_t y;      /* Y coordinate */
  bool pressed;    /* True while the finger is down */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_ESP32P4_GT911_SHIM

/****************************************************************************
 * Name: gt911_board_init
 *
 * Description:
 *   Initialize the Goodix GT911 touch controller over I2C0 in raw polling
 *   mode.  The GT911 INT pin is not routed to the ESP32-P4 on this board,
 *   so the NuttX gt9xx interrupt driver cannot be used; this shim instead
 *   reads the status register directly from the caller's poll loop.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int gt911_board_init(void);

/****************************************************************************
 * Name: gt911_board_poll
 *
 * Description:
 *   Poll the GT911 for new touch data and decode up to max_points points.
 *   This function performs one I2C transaction: read the status register
 *   (0x814E); if the buffer-ready bit is set, read the point block and then
 *   clear the status register so the controller can fill it again.
 *
 * Input Parameters:
 *   touches    - Output array of decoded touch points.
 *   max_points - Size of the touches array (1..5).
 *
 * Returned Value:
 *   Number of points decoded (1..max_points) when at least one finger is
 *   down; 0 when the controller reported a lift (buffer ready with zero
 *   points); -EAGAIN when there is no new touch data and the caller should
 *   retain its previous state; or another negated errno value on I2C
 *   failure.
 *
 ****************************************************************************/

int gt911_board_poll(struct gt911_touch_s *touches, int max_points);

#endif /* CONFIG_ESP32P4_GT911_SHIM */

#endif /* __BOARDS_RISCV_ESP32P4_ESP32P4_FUNCTION_EV_BOARD_SRC_GT911_BOARD_H */
