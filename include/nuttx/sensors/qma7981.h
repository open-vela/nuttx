/****************************************************************************
 * include/nuttx/sensors/qma7981.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_QMA7981_H
#define __INCLUDE_NUTTX_SENSORS_QMA7981_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_QMA7981)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* QMA7981 default 7-bit I2C slave address */

#define QMA7981_I2C_ADDR    0x12

/* Sample data returned from a single read().  Each axis is a sign-extended
 * 14-bit value (range depends on the FSR programmed at register-time).
 * For the default ±8 g full-scale range used by this driver, 1 LSB is
 * approximately 0.98 mg.
 */

struct qma7981_data_s
{
  int16_t x;
  int16_t y;
  int16_t z;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

struct i2c_master_s;

/****************************************************************************
 * Name: qma7981_register
 *
 * Description:
 *   Register the QMA7981 three-axis accelerometer character device as
 *   'devpath'.  After registration each read() returns a single
 *   struct qma7981_data_s sample (six bytes) representing the latest
 *   acceleration reading on the X, Y and Z axes.
 *
 * Input Parameters:
 *   devpath - Filesystem path, e.g. "/dev/accel0"
 *   i2c     - Initialized I2C master bus
 *   addr    - 7-bit I2C address (typically QMA7981_I2C_ADDR = 0x12)
 *
 * Returned Value:
 *   Zero on success, negated errno on failure.
 *
 ****************************************************************************/

int qma7981_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_QMA7981 */
#endif /* __INCLUDE_NUTTX_SENSORS_QMA7981_H */
