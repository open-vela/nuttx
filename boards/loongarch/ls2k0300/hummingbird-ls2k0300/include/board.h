/****************************************************************************
 * boards/loongarch/ls2k0300/hummingbird-ls2k0300/include/board.h
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

#ifndef __BOARDS_LOONGARCH_LS2K0300_HUMMINGBIRD_INCLUDE_BOARD_H
#define __BOARDS_LOONGARCH_LS2K0300_HUMMINGBIRD_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BOARD_SPIFLASH_BASE      0x1c000000
#define BOARD_SPIFLASH_SIZE      0x00100000

#define BOARD_L2CACHE_BASE       0x90000000
#define BOARD_L2CACHE_SIZE       0x00060000

#define BOARD_RAM_START          BOARD_L2CACHE_BASE
#define BOARD_RAM_SIZE           BOARD_L2CACHE_SIZE

#define LED_STARTED              0
#define LED_HEAPALLOCATE         1
#define LED_IRQSENABLED          2
#define LED_STACKCREATED         3
#define LED_INIRQ                4
#define LED_SIGNAL               5
#define LED_ASSERTION            6
#define LED_PANIC                7

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ls2k0300_boardinitialize(void);
int ls2k0300_bringup(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_LOONGARCH_LS2K0300_HUMMINGBIRD_INCLUDE_BOARD_H */
