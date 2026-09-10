/****************************************************************************
 * include/nuttx/sensors/mmc5603.h
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

#ifndef __INCLUDE_NUTTX_SENSORS_MMC5603_H
#define __INCLUDE_NUTTX_SENSORS_MMC5603_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/sensors/ioctl.h>

#if defined(CONFIG_I2C) && defined(CONFIG_SENSORS_MMC5603)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C address (fixed 7-bit address 0x30 for the MMC5603NJ) */

#define MMC5603_ADDR            0x30

/* Register addresses */

#define MMC5603_OUT_X_L         0x00  /* X-axis low byte; burst-read 9 bytes for the 3-axis 20-bit data */
#define MMC5603_OUT_TEMP        0x09  /* Temperature (1 signed byte) */
#define MMC5603_STATUS_REG      0x18  /* Status register */
#define MMC5603_ODR_REG         0x1A  /* Data rate (0-255, or the special 1000 Hz setting) */
#define MMC5603_CTRL0_REG       0x1B  /* Control register 0 */
#define MMC5603_CTRL1_REG       0x1C  /* Control register 1 */
#define MMC5603_CTRL2_REG       0x1D  /* Control register 2 */
#define MMC5603_PRODUCT_ID      0x39  /* Product ID register */

/* Expected product ID */

#define MMC5603_CHIP_ID         0x10

/* Control register bit definitions (bit meanings follow the Zephyr
 * mainline mmc56x3 driver: SET/RESET are degauss pulses; CTRL0 0x80 =
 * continuous measurement CMM_FREQ; CTRL2 0x10 = CMM_EN)
 */

#define MMC5603_CTRL0_TAKE_M    0x01  /* Trigger a single magnetic measurement (TM_M) */
#define MMC5603_CTRL0_TAKE_T    0x02  /* Trigger a single temperature measurement (TM_T) */
#define MMC5603_CTRL0_CMD_SET   0x08  /* SET pulse: set the magnetic domains to cancel offset */
#define MMC5603_CTRL0_CMD_RESET 0x10  /* RESET pulse: reset the magnetic domains to cancel offset */
#define MMC5603_CTRL0_AUTO_SR   0x20  /* Automatic SET/RESET degauss in continuous mode */
#define MMC5603_CTRL0_CMM_FREQ  0x80  /* Continuous measurement mode enable */
#define MMC5603_CTRL1_SW_RST    0x80  /* Software reset */
#define MMC5603_CTRL2_CMM_EN    0x10  /* Continuous measurement output enable */
#define MMC5603_CTRL2_HPOWER    0x40  /* High power mode */
#define MMC5603_CTRL2_ODR_1000  0x80  /* 1000 Hz special data rate bit */

/* Magnetic field units: 20-bit signed raw value (center offset 2^19
 * already subtracted); each LSB is 0.0625 mG, full scale about +/-30 G.
 */

#define MMC5603_CENTER_OFFSET   (1 << 19)          /* Center offset 2^19 = 524288 */
#define MMC5603_MAG_SCALE_MG    0.0625f            /* 1 LSB = 0.0625 mG */

/* Custom ioctl: read the 3-axis magnetic field (used with read()) */

#define SNIOC_MMC5603READ       _SNIOC(0x006f)     /* Arg: struct mmc5603_data_s* */

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_master_s;

/* Magnetic field data container (20-bit signed raw value, center
 * offset already subtracted)
 */

struct mmc5603_data_s
{
  int32_t x;            /* X-axis field, in counts (x0.0625 gives mG) */
  int32_t y;            /* Y-axis magnetic field */
  int32_t z;            /* Z-axis magnetic field */
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
 * Name: mmc5603_register
 *
 * Description:
 *   Register the MMC5603 geomagnetic sensor character device at
 *   'devpath' (e.g. "/dev/mag0").  Registration performs the software
 *   reset, continuous measurement mode and data rate configuration.
 *
 * Input Parameters:
 *   devpath - Device path, for example "/dev/mag0"
 *   i2c     - An initialized I2C master instance
 *   addr    - I2C address (should be MMC5603_ADDR)
 *
 * Returned Value:
 *   OK (0) on success; a negated errno on failure.
 *
 ****************************************************************************/

int mmc5603_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c, uint8_t addr);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_I2C && CONFIG_SENSORS_MMC5603 */
#endif /* __INCLUDE_NUTTX_SENSORS_MMC5603_H */
