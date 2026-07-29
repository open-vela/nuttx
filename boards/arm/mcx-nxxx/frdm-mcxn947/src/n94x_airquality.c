/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_airquality.c
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

#include <errno.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/aht2x.h>
#include <nuttx/sensors/ens160.h>

#include "nxxx_lpi2c.h"
#include "frdm-mcxn947.h"

#ifdef CONFIG_N94X_ENS160_AHT21

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int n94x_airquality_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ens_ret;
  int aht_ret;

  i2c = nxxx_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LPI2C0\n");
      return -ENODEV;
    }

  /* ADD is wired to GND, selecting the ENS160 low address. */

  ens_ret = ens160_register(0, i2c, ENS160_I2C_ADDR_LOW);
  if (ens_ret < 0)
    {
      syslog(LOG_WARNING, "ENS160 probe/register failed: %d\n", ens_ret);
    }

  aht_ret = aht2x_register(0, i2c, AHT2X_I2C_ADDR);
  if (aht_ret < 0)
    {
      syslog(LOG_WARNING, "AHT20/AHT21 probe/register failed: %d\n",
             aht_ret);
    }

  if (ens_ret < 0 && aht_ret < 0)
    {
      nxxx_i2cbus_uninitialize(i2c);
      return ens_ret;
    }

  if (ens_ret < 0)
    {
      return ens_ret;
    }

  if (aht_ret < 0)
    {
      return aht_ret;
    }

  syslog(LOG_INFO,
         "ENS160/AHT21: LPI2C0 ready (ENS160=0x52, AHT21=0x38)\n");
  return OK;
}

#endif /* CONFIG_N94X_ENS160_AHT21 */
