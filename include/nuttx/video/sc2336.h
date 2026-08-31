/****************************************************************************
 * include/nuttx/video/sc2336.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_SC2336_H
#define __INCLUDE_NUTTX_VIDEO_SC2336_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The SC2336 is a SmartSens 2MP CMOS image sensor with a MIPI CSI-2
 * output.  This driver supports the following sensor modes.  The sensor
 * outputs RAW8 Bayer (BGGR) data; the receiving CSI/ISP hardware is
 * expected to convert the stream to RGB565 inline, so the driver
 * advertises RGB565 to the video framework.
 *
 *   Width  Height  FPS  Lanes  Lane rate
 *   1024   600     30   2      288 Mbps   (default mode)
 *   1280   720     30   2      336 Mbps
 */

#define SC2336_DEFAULT_WIDTH   1024
#define SC2336_DEFAULT_HEIGHT  600
#define SC2336_DEFAULT_FPS     30

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;
struct imgsensor_s;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sc2336_initialize
 *
 * Description:
 *   Initialize the SC2336 image sensor driver instance.  The sensor is
 *   accessed via SCCB (I2C compatible) at the fixed 7-bit address 0x30.
 *   The returned image sensor interface can be passed to
 *   capture_register() to create a V4L2 capture device.
 *
 * Input Parameters:
 *   i2c - An I2C master instance for the bus the sensor is wired to.
 *
 * Returned Value:
 *   A non-NULL image sensor instance on success; NULL on failure.
 *
 ****************************************************************************/

FAR struct imgsensor_s *sc2336_initialize(FAR struct i2c_master_s *i2c);

/****************************************************************************
 * Name: sc2336_get_mode_info
 *
 * Description:
 *   Look up the MIPI CSI-2 link parameters of the sensor mode with the
 *   given frame size.  Board logic uses this information to configure the
 *   CSI host controller to match the sensor output.
 *
 * Input Parameters:
 *   width          - Frame width in pixels.
 *   height         - Frame height in pixels.
 *   data_lanes     - The number of CSI-2 data lanes is returned here.
 *                    May be NULL.
 *   lane_rate_mbps - The per-lane link rate in Mbps is returned here.
 *                    May be NULL.
 *
 * Returned Value:
 *   Zero (OK) on success; -EINVAL if no mode matches the frame size.
 *
 ****************************************************************************/

int sc2336_get_mode_info(uint16_t width, uint16_t height,
                         FAR uint8_t *data_lanes,
                         FAR uint16_t *lane_rate_mbps);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_SC2336_H */
