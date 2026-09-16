/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_lowputc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "loongarch_internal.h"
#include "ls2k0300_config.h"
#include "hardware/ls2k0300_memorymap.h"
#include "hardware/ls2k0300_uart.h"
#include "ls2k0300.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define LS2K0300_CONSOLE_BASE     LS2K0300_UART0_BASE
#    define LS2K0300_CONSOLE_BAUD     CONFIG_UART0_BAUD
#    define HAVE_UART
#  elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#    define LS2K0300_CONSOLE_BASE     LS2K0300_UART1_BASE
#    define LS2K0300_CONSOLE_BAUD     CONFIG_UART1_BAUD
#    define HAVE_UART
#  endif
#endif

#ifndef LS2K0300_CONSOLE_BASE
#  define LS2K0300_CONSOLE_BASE       LS2K0300_UART0_BASE
#  define LS2K0300_CONSOLE_BAUD       115200
#  define HAVE_UART
#endif

#define UART_REF_CLK                 (LS2K0300_APB_FREQ * 1000000)
#define UART_DIV_VAL                 ((UART_REF_CLK + (LS2K0300_CONSOLE_BAUD * 8)) / (LS2K0300_CONSOLE_BAUD * 16))
#define UART_DIV_HI                  ((UART_DIV_VAL >> 8) & 0xff)
#define UART_DIV_LO                  (UART_DIV_VAL & 0xff)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void loongarch_lowputc(char ch)
{
#ifdef HAVE_UART
  while ((getreg8(LS2K0300_CONSOLE_BASE + UART_LSR_OFFSET) &
          UART_LSR_THRE) == 0)
    {
    }

  putreg8(ch, LS2K0300_CONSOLE_BASE + UART_THR_OFFSET);
#endif
}

void ls2k0300_lowsetup(void)
{
#ifdef HAVE_UART
  putreg8(0x80, LS2K0300_CONSOLE_BASE + UART_LCR_OFFSET);

  putreg8(UART_DIV_LO, LS2K0300_CONSOLE_BASE + UART_DLL_OFFSET);
  putreg8(UART_DIV_HI, LS2K0300_CONSOLE_BASE + UART_DLH_OFFSET);

  putreg8(UART_LCR_WLEN8, LS2K0300_CONSOLE_BASE + UART_LCR_OFFSET);

  putreg8(0x47, LS2K0300_CONSOLE_BASE + UART_FCR_OFFSET);
#endif
}
