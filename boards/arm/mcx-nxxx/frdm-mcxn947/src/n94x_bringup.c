/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_bringup.c
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
#include <nuttx/fs/fs.h>
#include <sys/types.h>
#include <syslog.h>

#ifdef CONFIG_USERLED_LOWER
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_N947_EDMA_SELFTEST
#  include "n947_edma.h"
#endif

#include "frdm-mcxn947.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n94x_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int n94x_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_FS_TMPFS) && defined(CONFIG_TESTING_FSTEST)
  /* xTS 1.3.2 explicitly runs fstest on /tmp.  tmpfs being compiled in does
   * not mount it automatically, so give the dedicated test image a real RAM
   * filesystem without changing the production firmware's mount layout.
   */

  ret = nx_mount(NULL, "/tmp", "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at /tmp: %d\n", ret);
    }
#endif

#ifdef CONFIG_USERLED_LOWER
  /* Register the LED driver at /dev/userleds */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the button driver at /dev/buttons */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_N947_EDMA_SELFTEST
  ret = n947_edma_selftest();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: eDMA selftest failed: %d\n", ret);
    }
  else
    {
      syslog(LOG_INFO, "eDMA selftest: PASS\n");
    }
#endif

#ifdef CONFIG_N947_LPADC
  ret = n94x_adc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ADC init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_N947_CTIMER_PWM
  ret = n94x_pwm_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: PWM init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_N947_LPSPI0
  ret = n94x_spidev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: SPI init failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_N94X_ENS160_AHT21
  /* Register the optional combined indoor-air module on LPI2C0. */

  ret = n94x_airquality_initialize();
  if (ret < 0)
    {
      syslog(LOG_WARNING,
             "ENS160/AHT21 module not fully available: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
