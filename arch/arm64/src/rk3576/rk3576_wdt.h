/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_wdt.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_WDT_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Watchdog modes */

#define RK3576_WDT_MODE_RESET       0
#define RK3576_WDT_MODE_INTERRUPT   1

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* Initialization */

void rk3576_wdt_init(void);

/* Control */

void rk3576_wdt_start(uint32_t timeout_ms);
void rk3576_wdt_stop(void);
void rk3576_wdt_feed(void);

/* Configuration */

int  rk3576_wdt_set_timeout(uint32_t timeout_ms);
int  rk3576_wdt_set_mode(int mode);

/* Status */

int  rk3576_wdt_is_enabled(void);
uint32_t rk3576_wdt_get_remaining(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_WDT_H */
