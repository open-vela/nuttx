/****************************************************************************
 * include/nuttx/video/ov3660.h
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

#ifndef __INCLUDE_NUTTX_VIDEO_OV3660_H
#define __INCLUDE_NUTTX_VIDEO_OV3660_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum ov3660_mode_e
{
  OV3660_MODE_JPEG_QVGA = 0,
  OV3660_MODE_RGB565_QQVGA
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct i2c_master_s;

/****************************************************************************
 * Name: ov3660_initialize
 *
 * Description:
 *   Probe and initialize OV3660 with default settings and JPEG QVGA mode.
 *   The board must power the sensor, release PWDN and reset, and provide a
 *   stable 20 MHz XCLK before calling this function.
 *
 * Input Parameters:
 *   i2c - I2C/SCCB bus used by the sensor.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ov3660_initialize(FAR struct i2c_master_s *i2c);

/****************************************************************************
 * Name: ov3660_set_mode
 *
 * Description:
 *   Switch OV3660 output format and frame geometry.
 *   The caller is responsible for serializing access to the sensor.
 *
 * Input Parameters:
 *   i2c  - I2C/SCCB bus used by the sensor.
 *   mode - Requested mode.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ov3660_set_mode(FAR struct i2c_master_s *i2c, enum ov3660_mode_e mode);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_VIDEO_OV3660_H */
