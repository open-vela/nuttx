/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_thermal.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_THERMAL_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_THERMAL_H

#define LS2K0300_TSENSOR_CTRL_HI     0x00
#define LS2K0300_TSENSOR_CTRL_LO     0x08
#define LS2K0300_TSENSOR_STATUS      0x10
#define LS2K0300_TSENSOR_OUT         0x14
#define LS2K0300_TSENSOR_CFG         0x18

#define TSENSOR_CTRL_EN_SHIFT       8

#define TSENSOR_STATUS_CLEAR        0x03

#endif
