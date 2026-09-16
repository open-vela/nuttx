/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_wdt.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_WDT_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_WDT_H

#define LS2K0300_WDT_RST_CTR         0x00
#define LS2K0300_WDT_CR              0x04
#define LS2K0300_WDT_TIMER           0x08

#define WDT_RST_CTR_ENABLE          (1 << 1)
#define WDT_CR_RESET                0x01

#endif
