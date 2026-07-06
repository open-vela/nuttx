/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_pwm.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_PWM_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_PWM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PWM controller count */

#define RK3576_PWM_COUNT             4   /* PWM0 ~ PWM3 */
#define RK3576_PWM_MAX_CHANNELS      8   /* Max channels per controller */

/* PWM channel count per controller (from TRM IOMUX) */

#define PWM0_CHANNELS               2
#define PWM1_CHANNELS               6
#define PWM2_CHANNELS               8
#define PWM3_CHANNELS               2

/* PWM register offsets per channel (channel spacing = 0x10) */

#define PWM_CHANNEL_STRIDE          0x10

#define PWM_CNT                     0x00
#define PWM_PERIOD                  0x04
#define PWM_DUTY                    0x08
#define PWM_CTRL                    0x0c

/* PWM_CTRL bit definitions */

#define PWM_CTRL_ENABLE             (1 << 0)
#define PWM_CTRL_MODE_SHIFT         1
#define PWM_CTRL_MODE_MASK          (3 << PWM_CTRL_MODE_SHIFT)
#define PWM_CTRL_MODE_CONTINUOUS    (0 << PWM_CTRL_MODE_SHIFT)
#define PWM_CTRL_MODE_ONESHOT       (1 << PWM_CTRL_MODE_SHIFT)
#define PWM_CTRL_MODE_CAPTURE_RISE  (2 << PWM_CTRL_MODE_SHIFT)
#define PWM_CTRL_MODE_CAPTURE_FALL  (3 << PWM_CTRL_MODE_SHIFT)

#define PWM_CTRL_CLK_SELECT_SHIFT   3
#define PWM_CTRL_CLK_SELECT_MASK    (3 << PWM_CTRL_CLK_SELECT_SHIFT)
#define PWM_CTRL_CLK_SELECT_DIV     (0 << PWM_CTRL_CLK_SELECT_SHIFT)
#define PWM_CTRL_CLK_SELECT_DIVBUSY (1 << PWM_CTRL_CLK_SELECT_SHIFT)
#define PWM_CTRL_CLK_SELECT_ALWAYS  (2 << PWM_CTRL_CLK_SELECT_SHIFT)
#define PWM_CTRL_CLK_SELECT_ALWAYSBUSY (3 << PWM_CTRL_CLK_SELECT_SHIFT)

#define PWM_CTRL_PRESCALE_SHIFT     5
#define PWM_CTRL_PRESCALE_MASK      (0xf << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV1      (0 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV2      (1 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV4      (2 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV8      (3 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV16     (4 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV32     (5 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV64     (6 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV128    (7 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV256    (8 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV512    (9 << PWM_CTRL_PRESCALE_SHIFT)
#define PWM_CTRL_PRESCALE_DIV1024   (10 << PWM_CTRL_PRESCALE_SHIFT)

/* PWM global control register (at PWM controller base + 0x00f0) */

#define PWM_GLOBAL_CTRL             0x00f0

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_pwm_base[RK3576_PWM_COUNT];
extern const int g_pwm_channels[RK3576_PWM_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_PWM_H */
