/****************************************************************************
 * arch/loongarch/include/ls2k0300/chip.h
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

#ifndef __ARCH_LOONGARCH_INCLUDE_LS2K0300_CHIP_H
#define __ARCH_LOONGARCH_INCLUDE_LS2K0300_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LS2K0300_SPIFLASH_BASE    0x1c000000
#define LS2K0300_SPIFLASH_SIZE    0x00100000

#define LS2K0300_LOCK_CACHE_BASE  0x90000000
#define LS2K0300_LOCK_CACHE_SIZE  0x00060000

#define LS2K0300_UART0_BASE       0x16100000
#define LS2K0300_UART1_BASE       0x16100400

#define LS2K0300_APB_FREQ         200

#define LS2K0300_SYSCLK_FREQ      100000000

#define LS2K0300_CORE_FREQ_MHZ    800

#endif /* __ARCH_LOONGARCH_INCLUDE_LS2K0300_CHIP_H */
