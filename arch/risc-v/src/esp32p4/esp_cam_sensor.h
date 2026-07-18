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

/* OV5647 I2C Configuration */

#define OV5647_I2C_ADDR        0x36     /* 7-bit I2C address */
#define OV5647_I2C_PORT        0        /* I2C port 0 */
#define OV5647_I2C_FREQ        100000   /* 100 kHz */
#define OV5647_SCL_GPIO        8        /* GPIO8 = SCL */
#define OV5647_SDA_GPIO        7        /* GPIO7 = SDA */

/* OV5647 Chip ID */

#define OV5647_CHIP_ID_H_REG   0x300a   /* Chip ID high byte register */
#define OV5647_CHIP_ID_L_REG   0x300b   /* Chip ID low byte register */
#define OV5647_CHIP_ID_H_VAL   0x56     /* Expected high byte */
#define OV5647_CHIP_ID_L_VAL   0x47     /* Expected low byte */
#define OV5647_CHIP_ID         0x5647   /* Full chip ID */

/* OV5647 Output Configuration */

#define OV5647_WIDTH           1024     /* Output width */
#define OV5647_HEIGHT          600      /* Output height */
#define OV5647_FPS             30       /* Target frame rate */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: esp_cam_sensor_init
 *
 * Description:
 *   Initialize the OV5647 camera sensor over I2C.
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
