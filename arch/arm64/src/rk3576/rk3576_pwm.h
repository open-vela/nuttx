/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_pwm.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_PWM_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM mode definitions */

#define RK3576_PWM_MODE_CONTINUOUS   0
#define RK3576_PWM_MODE_ONESHOT      1

/* PWM clock source definitions */

#define RK3576_PWM_CLK_DIV           0
#define RK3576_PWM_CLK_DIV_BUSY      1
#define RK3576_PWM_CLK_ALWAYS        2
#define RK3576_PWM_CLK_ALWAYS_BUSY   3

/* Common PWM frequencies for RK3576 peripherals */

#define PWM_FREQ_BACKLIGHT           20000   /* 20kHz for backlight */
#define PWM_FREQ_BUZZER              1000    /* 1kHz for buzzer */
#define PWM_FREQ_SERVO               50      /* 50Hz for servo */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_pwm_init(void);

/* Configuration */

int  rk3576_pwm_configure(int controller, int channel,
                          uint32_t period_ns, uint32_t duty_ns);

int  rk3576_pwm_set_frequency(int controller, int channel,
                              uint32_t freq_hz, uint8_t duty_percent);

/* Control */

int  rk3576_pwm_enable(int controller, int channel);
int  rk3576_pwm_disable(int controller, int channel);

/* Query */

int  rk3576_pwm_is_enabled(int controller, int channel);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_PWM_H */
