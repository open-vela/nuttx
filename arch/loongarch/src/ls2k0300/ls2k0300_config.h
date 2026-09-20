/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_config.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_LS2K0300_CONFIG_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_LS2K0300_CONFIG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef HAVE_UART_DEVICE
#if defined(CONFIG_LS2K0300_UART0) || defined(CONFIG_LS2K0300_UART1) || \
    defined(CONFIG_LS2K0300_UART2)
#  define HAVE_UART_DEVICE 1
#endif

#if defined(CONFIG_UART0_SERIAL_CONSOLE) && defined(CONFIG_LS2K0300_UART0)
#  define HAVE_SERIAL_CONSOLE 1
#  define LS2K0300_CONSOLE_BASE     LS2K0300_UART0_BASE
#  define LS2K0300_CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define LS2K0300_CONSOLE_BITS     CONFIG_UART0_BITS
#  define LS2K0300_CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define LS2K0300_CONSOLE_2STOP    CONFIG_UART0_2STOP
#  define HAVE_UART
#elif defined(CONFIG_UART1_SERIAL_CONSOLE) && defined(CONFIG_LS2K0300_UART1)
#  define HAVE_SERIAL_CONSOLE 1
#  define LS2K0300_CONSOLE_BASE     LS2K0300_UART1_BASE
#  define LS2K0300_CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define LS2K0300_CONSOLE_BITS     CONFIG_UART1_BITS
#  define LS2K0300_CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define LS2K0300_CONSOLE_2STOP    CONFIG_UART1_2STOP
#  define HAVE_UART
#else
#  undef HAVE_SERIAL_CONSOLE
#endif

#ifndef LS2K0300_CONSOLE_BASE
#  define LS2K0300_CONSOLE_BASE       LS2K0300_UART0_BASE
#endif

#ifndef LS2K0300_CONSOLE_BAUD
#  define LS2K0300_CONSOLE_BAUD       115200
#endif

#define LS2K0300_APB_FREQ             200

#define LS2K0300_UART_REF_CLK         (LS2K0300_APB_FREQ * 1000000)

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_LS2K0300_CONFIG_H */
