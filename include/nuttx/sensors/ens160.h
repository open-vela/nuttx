/****************************************************************************
 * include/nuttx/sensors/ens160.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_ENS160_H
#define __INCLUDE_NUTTX_SENSORS_ENS160_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ENS160_I2C_ADDR_LOW   0x52
#define ENS160_I2C_ADDR_HIGH  0x53
#define ENS160_PART_ID        0x0160

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Environmental compensation value accepted through SNIOC_SET_CALIBVALUE
 * on any ENS160 Sensor Framework node.
 */

struct ens160_env_s
{
  float temperature; /* Degrees Celsius */
  float humidity;    /* Relative humidity in percent */
};

/* Complete ENS160 result returned by /dev/uorb/sensor_ens160N.  The
 * standard /dev/uorb/sensor_co2N and /dev/uorb/sensor_tvocN nodes are
 * registered as well.  eco2 is an equivalent CO2 estimate, not a direct
 * CO2 measurement.
 */

struct ens160_iaq_s
{
  uint64_t timestamp;  /* Microseconds from CLOCK_MONOTONIC */
  uint16_t eco2_ppm;   /* Equivalent CO2, ppm */
  uint16_t tvoc_ppb;   /* Total VOC, ppb */
  uint8_t  aqi;        /* UBA air quality index, 1 through 5 */
  uint8_t  validity;   /* 0 normal, 1 warm-up, 2 initial start, 3 invalid */
  uint8_t  data_ready; /* Non-zero if fresh DATA registers were available */
  uint8_t  reserved;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: ens160_register
 *
 * Description:
 *   Register one ENS160 at the standard Sensor Framework CO2/TVOC nodes and
 *   at a composite /dev/uorb/sensor_ens160N node.
 *
 * Input Parameters:
 *   devno - Sensor Framework instance number.
 *   i2c   - I2C master used to communicate with the sensor.
 *   addr  - Seven-bit address selected by the module ADD/SDO pin.
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int ens160_register(int devno, FAR struct i2c_master_s *i2c, uint8_t addr);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_SENSORS_ENS160_H */
