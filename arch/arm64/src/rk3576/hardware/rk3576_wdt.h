/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_wdt.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_WDT_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Watchdog register offsets */

#define WDT_CR                     0x0000   /* Control register */
#define WDT_TORR                   0x0004   /* Timeout range register */
#define WDT_CCVR                   0x0008   /* Current counter value */
#define WDT_CRR                    0x000c   /* Counter restart register */
#define WDT_STAT                   0x0010   /* Status register */

/****************************************************************************
 * WDT_CR bit definitions
 ****************************************************************************/

#define WDT_CR_WDT_EN              (1 << 0)   /* Watchdog enable */
#define WDT_CR_RST_EN              (1 << 1)   /* Reset enable */
#define WDT_CR_IRQ_EN              (1 << 2)   /* Interrupt enable */
#define WDT_CR_RST_MODE_SHIFT      3
#define WDT_CR_RST_MODE_MASK       (3 << WDT_CR_RST_MODE_SHIFT)
#define WDT_CR_RST_MODE_PULSE      (0 << WDT_CR_RST_MODE_SHIFT)
#define WDT_CR_RST_MODE_LEVEL      (1 << WDT_CR_RST_MODE_SHIFT)

/****************************************************************************
 * WDT_CRR bit definitions
 ****************************************************************************/

#define WDT_CRR_RESTART            0x76   /* Restart magic value */

/****************************************************************************
 * WDT_STAT bit definitions
 ****************************************************************************/

#define WDT_STAT_WDT_STAT          (1 << 0)   /* Watchdog status */

/****************************************************************************
 * Watchdog timeout range (in seconds)
 ****************************************************************************/

#define WDT_TIMEOUT_MIN            1
#define WDT_TIMEOUT_MAX            128

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

extern const uint32_t g_wdt_base;

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_WDT_H */
