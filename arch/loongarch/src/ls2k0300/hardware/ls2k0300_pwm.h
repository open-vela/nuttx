/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_pwm.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_PWM_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_PWM_H

#define LS2K0300_PWM_LOW_BUFFER      0x004
#define LS2K0300_PWM_FULL_BUFFER     0x008
#define LS2K0300_PWM_CTRL            0x00c

#define PWM_CTRL_EN                 (1 << 0)
#define PWM_CTRL_OE                 (1 << 3)
#define PWM_CTRL_SINGLE             (1 << 4)
#define PWM_CTRL_INTE               (1 << 5)
#define PWM_CTRL_INT                (1 << 6)
#define PWM_CTRL_RST                (1 << 7)
#define PWM_CTRL_CAPTE              (1 << 8)
#define PWM_CTRL_INVERT             (1 << 9)
#define PWM_CTRL_DZONE              (1 << 10)

#endif
