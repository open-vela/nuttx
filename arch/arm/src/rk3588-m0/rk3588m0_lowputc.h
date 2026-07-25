/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_lowputc.h
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

#ifndef __ARCH_ARM_SRC_RK3588_M0_RK3588M0_LOWPUTC_H
#define __ARCH_ARM_SRC_RK3588_M0_RK3588M0_LOWPUTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: rk3588m0_lowsetup
 *
 * Description:
 *   Early console hook.  A no-op: the shared UART is configured by its owner
 *   (u-boot/Linux) and must not be reprogrammed.
 *
 ****************************************************************************/

void rk3588m0_lowsetup(void);

/****************************************************************************
 * Name: rk3588m0_lowputc
 *
 * Description:
 *   Output one byte on the shared console UART, polled.
 *
 ****************************************************************************/

void rk3588m0_lowputc(int ch);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RK3588_M0_RK3588M0_LOWPUTC_H */
