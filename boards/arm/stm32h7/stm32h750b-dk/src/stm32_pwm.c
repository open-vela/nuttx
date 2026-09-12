/****************************************************************************
 * boards/arm/stm32h7/stm32h750b-dk/src/stm32_pwm.c
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
#include <debug.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "stm32h750b-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_STM32H7_TIM15_PWM
#  define HAVE_PWM 1
#endif

#ifndef CONFIG_PWM
#  undef HAVE_PWM
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize TIM15 CH2 PWM and register /dev/pwm0 for the expansion
 *   board buzzer.
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
#ifdef HAVE_PWM
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  if (!initialized)
    {
      pwm = stm32_pwminitialize(15);
      if (pwm == NULL)
        {
          tmrerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          tmrerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
#else
  return -ENOSYS;
#endif
}
