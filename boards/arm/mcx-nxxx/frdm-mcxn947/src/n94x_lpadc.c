/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_lpadc.c
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

#include <sys/param.h>
#include <stdint.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/adc.h>

#include "n947_lpadc.h"

#include "frdm-mcxn947.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n94x_adc_initialize
 *
 * Description:
 *   Register the board LPADC instances as standard NuttX ADC devices.
 *
 ****************************************************************************/

int n94x_adc_initialize(void)
{
  struct adc_dev_s *adc;
  int ret = OK;

#ifdef CONFIG_N947_LPADC0
  static const uint8_t lpadc0_channels[] =
  {
    CONFIG_N947_LPADC0_CHANNEL
  };

  adc = n947_lpadc_initialize(0, lpadc0_channels,
                              nitems(lpadc0_channels));
  if (adc == NULL)
    {
      return -ENODEV;
    }

  ret = adc_register("/dev/adc0", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register(/dev/adc0) failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_N947_LPADC1
  static const uint8_t lpadc1_channels[] =
  {
    CONFIG_N947_LPADC1_CHANNEL
  };

  adc = n947_lpadc_initialize(1, lpadc1_channels,
                              nitems(lpadc1_channels));
  if (adc == NULL)
    {
      return -ENODEV;
    }

  ret = adc_register("/dev/adc1", adc);
  if (ret < 0)
    {
      aerr("ERROR: adc_register(/dev/adc1) failed: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
