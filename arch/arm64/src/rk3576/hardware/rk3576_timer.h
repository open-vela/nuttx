/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_timer.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_TIMER_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer controller count */

#define RK3576_TIMER_COUNT          2   /* TIMER0, TIMER1 */
#define RK3576_TIMER_CHANNELS      2   /* 2 channels per controller */

/* Timer register offsets (per channel pair) */

#define TIMER_LOAD_COUNT(ch)       ((ch) * 8 + 0x00)   /* Load count */
#define TIMER_CURRENT_COUNT(ch)    ((ch) * 8 + 0x04)   /* Current count */
#define TIMER_INTSTATUS            0x10   /* Interrupt status */
#define TIMER_INTCLR               0x14   /* Interrupt clear */
#define TIMER_CTRL                 0x18   /* Control register */

/****************************************************************************
 * TIMER_CTRL bit definitions
 ****************************************************************************/

#define TIMER_CTRL_EN              (1 << 0)   /* Timer enable */
#define TIMER_CTRL_MODE_SHIFT      1
#define TIMER_CTRL_MODE_MASK       (1 << TIMER_CTRL_MODE_SHIFT)
#define TIMER_CTRL_MODE_FREE       (0 << TIMER_CTRL_MODE_SHIFT)   /* Free-running */
#define TIMER_CTRL_MODE_USER       (1 << TIMER_CTRL_MODE_SHIFT)   /* User-defined count */
#define TIMER_CTRL_INT_EN          (1 << 2)   /* Interrupt enable */

/****************************************************************************
 * TIMER_INTSTATUS bit definitions
 ****************************************************************************/

#define TIMER_INT_CH0              (1 << 0)   /* Channel 0 interrupt */
#define TIMER_INT_CH1              (1 << 1)   /* Channel 1 interrupt */

/****************************************************************************
 * Timer clock frequency (24MHz oscillator)
 ****************************************************************************/

#define TIMER_CLK_FREQ             24000000

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_timer_base[RK3576_TIMER_COUNT];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_TIMER_H */
