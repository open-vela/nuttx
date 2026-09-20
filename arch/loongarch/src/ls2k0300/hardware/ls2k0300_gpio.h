/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_gpio.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_GPIO_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_GPIO_H

#define LS2K0300_GPIO_DIR            0x800
#define LS2K0300_GPIO_OUT            0x900
#define LS2K0300_GPIO_IN             0xa00
#define LS2K0300_GPIO_IRQ            0xb00
#define LS2K0300_GPIO_IRQPOL         0xc00
#define LS2K0300_GPIO_IRQEDG         0xd00
#define LS2K0300_GPIO_IRQCLR         0xe00
#define LS2K0300_GPIO_IRQSTA         0xf00
#define LS2K0300_GPIO_IRQDUL         0xf80

#define LS2K0300_GPIO_NPINS          106

#endif
