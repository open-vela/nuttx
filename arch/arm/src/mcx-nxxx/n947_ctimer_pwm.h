/****************************************************************************
 * arch/arm/src/mcx-nxxx/n947_ctimer_pwm.h
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

#ifndef __ARCH_ARM_SRC_MCX_NXXX_NXXX_PWM_H
#define __ARCH_ARM_SRC_MCX_NXXX_NXXX_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_N947_CTIMER_PWM

struct pwm_lowerhalf_s;

/****************************************************************************
 * Name: n947_ctimer_pwminitialize
 *
 * Description:
 *   Initialize one MCX-Nxxx CTIMER match output as a NuttX PWM lower-half.
 *
 * Input Parameters:
 *   timer   - CTIMER instance number, 0..4.
 *   channel - CTIMER match output channel, 0..3.  This must not equal the
 *             configured period match channel.
 *
 * Returned Value:
 *   Valid PWM lower-half reference on success; NULL on failure.
 *
 ****************************************************************************/

struct pwm_lowerhalf_s *n947_ctimer_pwminitialize(int timer, int channel);

/****************************************************************************
 * Name: n947_ctimer_pwm_boardinitialize
 *
 * Description:
 *   Optional board hook for configuring the selected CTIMER MAT pin before
 *   PWM output is started.  The weak default implementation does nothing.
 *
 ****************************************************************************/

int n947_ctimer_pwm_boardinitialize(int timer, int channel);

#endif /* CONFIG_N947_CTIMER_PWM */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_MCX_NXXX_NXXX_PWM_H */
