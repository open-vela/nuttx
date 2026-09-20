/****************************************************************************
 * boards/risc-v/esp32p4/esp32p4-function-ev-board/src/esp32p4_touch.c
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>

#include "espressif/esp_i2c.h"
#include "esp32p4-function-ev-board.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Goodix GT911 I2C address (7-bit) */

#define GT911_I2C_ADDR  0x5d

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gt911_initialize
 *
 * Description:
 *   Initialize the Goodix GT911 touch controller on I2C0.
 *   The GT911 INT pin is NOT routed to ESP32-P4 on this board,
 *   so the driver uses polling mode (work queue reads status
 *   register every 50ms).
 *
 * Input Parameters:
 *   busno - I2C bus number (0 for I2C0 on GPIO7/8)
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_gt911_initialize(int busno)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  iinfo("Initializing GT911 on I2C%d, addr=0x%02x\n", busno, GT911_I2C_ADDR);

  /* Get I2C bus */

  i2c = esp_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      ierr("ERROR: Failed to get I2C%d bus\n", busno);
      return -ENODEV;
    }

  /* Register GT911 driver in polling mode (board_config = NULL).
   * The driver will read touch status register (0x814E) via I2C
   * every 50ms using a work queue. */

  ret = gt9xx_register("/dev/input0", i2c, GT911_I2C_ADDR, NULL);
  if (ret < 0)
    {
      ierr("ERROR: gt9xx_register failed: %d\n", ret);
    }

  return ret;
}