/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_pwm.c
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

#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/timers/pwm.h>

#include "n947_ctimer_pwm.h"

#include "frdm-mcxn947.h"

#ifdef CONFIG_N947_CTIMER_PWM

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int n94x_pwm_register(int timer, int channel, int minor)
{
  FAR struct pwm_lowerhalf_s *pwm;
  char devpath[16];
  int ret;

  pwm = n947_ctimer_pwminitialize(timer, channel);
  if (pwm == NULL)
    {
      return -ENODEV;
    }

  snprintf(devpath, sizeof(devpath), "/dev/pwm%d", minor);

  ret = pwm_register(devpath, pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register(%s) failed: %d\n", devpath, ret);
      return ret;
    }

  pwminfo("Registered CTIMER%d MAT%d as %s\n",
          timer, channel, devpath);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: n94x_pwm_initialize
 *
 * Description:
 *   Register configured CTIMER PWM instances as standard NuttX PWM devices.
 *
 ****************************************************************************/

int n94x_pwm_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_N947_CTIMER0_PWM
  ret = n94x_pwm_register(0, CONFIG_N947_CTIMER0_PWM_MATCH, 0);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_N947_CTIMER1_PWM
  ret = n94x_pwm_register(1, CONFIG_N947_CTIMER1_PWM_MATCH, 1);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_N947_CTIMER2_PWM
  ret = n94x_pwm_register(2, CONFIG_N947_CTIMER2_PWM_MATCH, 2);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_N947_CTIMER3_PWM
  ret = n94x_pwm_register(3, CONFIG_N947_CTIMER3_PWM_MATCH, 3);
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_N947_CTIMER4_PWM
  ret = n94x_pwm_register(4, CONFIG_N947_CTIMER4_PWM_MATCH, 4);
  if (ret < 0)
    {
      return ret;
    }
#endif

  return ret;
}

#endif /* CONFIG_N947_CTIMER_PWM */
