/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_timer.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_TIMER_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_TIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Timer modes */

#define RK3576_TIMER_MODE_FREE      0   /* Free-running */
#define RK3576_TIMER_MODE_USER      1   /* User-defined count (one-shot) */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_timer_init(int controller);

/* Control */

void rk3576_timer_start(int controller, int channel);
void rk3576_timer_stop(int controller, int channel);

/* Configuration */

int  rk3576_timer_set_mode(int controller, int channel, int mode);
int  rk3576_timer_set_count(int controller, int channel, uint32_t count);
int  rk3576_timer_set_period_us(int controller, int channel, uint32_t us);

/* Status */

uint32_t rk3576_timer_get_count(int controller, int channel);
int  rk3576_timer_is_running(int controller, int channel);

/* Interrupt */

void rk3576_timer_clear_irq(int controller, int channel);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_TIMER_H */
