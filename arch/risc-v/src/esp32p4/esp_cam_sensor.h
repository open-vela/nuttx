/****************************************************************************
 * arch/risc-v/src/esp32p4/esp_cam_sensor.h
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

#ifndef __ARCH_RISCV_SRC_ESP32P4_ESP_CAM_SENSOR_H
#define __ARCH_RISCV_SRC_ESP32P4_ESP_CAM_SENSOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SC2336 I2C Configuration
 *
 * NOTE: The ESP32-P4-Function-EV-Board is populated with an SC2336
 * (SmartSens) MIPI-CSI sensor, NOT an OV5647.  Verified via ESP-IDF
 * reference firmware which detected PID 0xcb3a at SCCB address 0x30.
 */

#define SC2336_I2C_ADDR        0x30     /* 7-bit I2C address */
#define SC2336_I2C_PORT        0        /* I2C port 0 */
#define SC2336_I2C_FREQ        100000   /* 100 kHz */
#define SC2336_SCL_GPIO        8        /* GPIO8 = SCL */
#define SC2336_SDA_GPIO        7        /* GPIO7 = SDA */

/* SC2336 Chip ID */

#define SC2336_CHIP_ID_H_REG   0x3107   /* Chip ID high byte register */
#define SC2336_CHIP_ID_L_REG   0x3108   /* Chip ID low byte register */
#define SC2336_CHIP_ID         0xcb3a   /* Full chip ID (PID) */

/* SC2336 key control registers */

#define SC2336_REG_SLEEP_MODE  0x0100   /* Stream/sleep: 1=stream, 0=sleep */
#define SC2336_REG_SW_RESET    0x0103   /* Software reset (bit0) */
#define SC2336_REG_END         0xffff   /* Init table terminator */
#define SC2336_REG_DELAY       0xfffe   /* Init table delay marker */

/* SC2336 Output Configuration */

#define SC2336_WIDTH           1024     /* Output width */
#define SC2336_HEIGHT          600      /* Output height */
#define SC2336_FPS             30       /* Target frame rate */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_cam_sensor_init
 *
 * Description:
 *   Initialize the SC2336 camera sensor over I2C.
 *   - Verifies chip ID
 *   - Sends initialization register sequence
 *   - Configures 1024x600 @ 30fps RAW8 output
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int esp_cam_sensor_init(void);

/****************************************************************************
 * Name: esp_cam_sensor_start
 *
 * Description:
 *   Start the sensor output stream.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int esp_cam_sensor_start(void);

/****************************************************************************
 * Name: esp_cam_sensor_stop
 *
 * Description:
 *   Stop the sensor output stream.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.
 ****************************************************************************/

int esp_cam_sensor_stop(void);

#endif /* __ARCH_RISCV_SRC_ESP32P4_ESP_CAM_SENSOR_H */
