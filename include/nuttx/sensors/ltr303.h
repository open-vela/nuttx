/****************************************************************************
 * include/nuttx/sensors/ltr303.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_LTR303_H
#define __INCLUDE_NUTTX_SENSORS_LTR303_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_LTR303)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C address (fixed 7-bit address 0x29 for the LTR-303ALS-01) */

#define LTR303_ADDR             0x29

/* Register addresses */

#define LTR303_ALS_CTRL         0x80  /* ALS control: bit0 enable, bits[4:2] gain */
#define LTR303_MEAS_RATE        0x85  /* Measurement rate: bits[2:0] rate, bits[5:3] integration time */
#define LTR303_PART_ID_REG      0x86  /* Part ID (expected 0x86) */
#define LTR303_MANU_ID_REG      0x87  /* Manufacturer ID (LiteOn = 0x05) */
#define LTR303_CH1DATA          0x88  /* Data: burst-read 4 bytes = CH1 low/high + CH0 low/high */
#define LTR303_STATUS           0x8C  /* Status register */

/* Register bit definitions */

#define LTR303_ALS_CTRL_MODE    0x01  /* ALS enable bit (1 = active) */
#define LTR303_ALS_CTRL_GAIN_SHIFT   2
#define LTR303_ALS_CTRL_GAIN_MASK     (0x07 << LTR303_ALS_CTRL_GAIN_SHIFT)
#define LTR303_MEAS_RATE_RATE_MASK    0x07
#define LTR303_MEAS_RATE_INTEG_SHIFT  3
#define LTR303_MEAS_RATE_INTEG_MASK   (0x07 << LTR303_MEAS_RATE_INTEG_SHIFT)

/* Expected part/manufacturer IDs */

#define LTR303_PART_ID_VALUE    0x86
#define LTR303_MANU_ID_VALUE    0x05

/* Custom ioctl: read both light channels (used with read()) */

#define SNIOC_LTR303READ        _SNIOC(0x0070)     /* Arg: struct ltr303_data_s* */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Light data container (raw 16-bit ADC counts) */

struct ltr303_data_s
{
  uint16_t ch0;         /* Channel 0 (CH0): visible + IR (full spectrum) */
  uint16_t ch1;         /* Channel 1 (CH1): IR only */
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

/****************************************************************************
 * Name: ltr303_register
 *
 * Description:
 *   Register the LTR-303 ambient light sensor character device at
 *   'devpath' (e.g. "/dev/light0").  Registration performs the enable +
 *   gain / integration time / rate configuration.
 *
 * Input Parameters:
 *   devpath - Device path, for example "/dev/light0"
 *   i2c     - An initialized I2C master instance
 *   addr    - I2C address (should be LTR303_ADDR)
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int ltr303_register(FAR const char *devpath,
                    FAR struct i2c_master_s *i2c, uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_LTR303 */
#endif /* __INCLUDE_NUTTX_SENSORS_LTR303_H */
