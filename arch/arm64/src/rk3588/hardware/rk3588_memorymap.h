/****************************************************************************
 * arch/arm64/src/rk3588/hardware/rk3588_memorymap.h
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

#ifndef __ARCH_ARM64_SRC_RK3588_HARDWARE_RK3588_MEMORYMAP_H
#define __ARCH_ARM64_SRC_RK3588_HARDWARE_RK3588_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Peripheral Base Addresses */
#define RK3588_GPIO0_ADDR        0xff720000
#define RK3588_GPIO1_ADDR        0xff730000
#define RK3588_GPIO2_ADDR        0xff780000
#define RK3588_GPIO3_ADDR        0xff788000
#define RK3588_GPIO4_ADDR        0xff790000

#define RK3588_PIO_ADDR        RK3588_GPIO0_ADDR
#define RK3588_PWM_ADDR        0xff430000
#define RK3588_UART0_ADDR      0xfd890000
#define RK3588_UART1_ADDR      0xfeb40000
#define RK3588_UART2_ADDR      0xfeb50000
#define RK3588_UART3_ADDR      0xfeb60000
#define RK3588_UART4_ADDR      0xfeb70000

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM64_SRC_RK3588_HARDWARE_RK3588_MEMORYMAP_H */
