/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_clockconfig.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_LS2K0300_CLOCKCONFIG_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_LS2K0300_CLOCKCONFIG_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void ls2k0300_clockconfig(void);
uint32_t ls2k0300_get_cpuclk(void);
uint32_t ls2k0300_get_apbclk(void);

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_LS2K0300_CLOCKCONFIG_H */
