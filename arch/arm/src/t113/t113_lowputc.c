/****************************************************************************
 * arch/arm/src/t113/t113_lowputc.c
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
#include <debug.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "t113_config.h"
#include "t113_ccu.h"
#include "t113_gpio.h"
#include "hardware/t113_uart.h"
#include "hardware/t113_ccu.h"
#include "t113_clk.h"
#include "t113_lowputc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_UART0_SERIAL_CONSOLE)
#  define CONSOLE_BASE     T113_UART0_VADDR
#  define CONSOLE_BAUD     CONFIG_UART0_BAUD
#  define CONSOLE_BITS     CONFIG_UART0_BITS
#  define CONSOLE_PARITY   CONFIG_UART0_PARITY
#  define CONSOLE_2STOP    CONFIG_UART0_2STOP
#  define CONSOLE_UART_NUM 0
#  define CONSOLE_UART_TX  T113_UART0_TX
#  define CONSOLE_UART_RX  T113_UART0_RX
#elif defined(CONFIG_UART1_SERIAL_CONSOLE)
#  define CONSOLE_BASE     T113_UART1_VADDR
#  define CONSOLE_BAUD     CONFIG_UART1_BAUD
#  define CONSOLE_BITS     CONFIG_UART1_BITS
#  define CONSOLE_PARITY   CONFIG_UART1_PARITY
#  define CONSOLE_2STOP    CONFIG_UART1_2STOP
#  define CONSOLE_UART_NUM 1
#  define CONSOLE_UART_TX  T113_UART1_TX
#  define CONSOLE_UART_RX  T113_UART1_RX
#elif defined(CONFIG_UART2_SERIAL_CONSOLE)
#  define CONSOLE_BASE     T113_UART2_VADDR
#  define CONSOLE_BAUD     CONFIG_UART2_BAUD
#  define CONSOLE_BITS     CONFIG_UART2_BITS
#  define CONSOLE_PARITY   CONFIG_UART2_PARITY
#  define CONSOLE_2STOP    CONFIG_UART2_2STOP
#  define CONSOLE_UART_NUM 2
#  define CONSOLE_UART_TX  T113_UART2_TX
#  define CONSOLE_UART_RX  T113_UART2_RX
#elif defined(CONFIG_UART3_SERIAL_CONSOLE)
#  define CONSOLE_BASE     T113_UART3_VADDR
#  define CONSOLE_BAUD     CONFIG_UART3_BAUD
#  define CONSOLE_BITS     CONFIG_UART3_BITS
#  define CONSOLE_PARITY   CONFIG_UART3_PARITY
#  define CONSOLE_2STOP    CONFIG_UART3_2STOP
#  define CONSOLE_UART_NUM 3
#  define CONSOLE_UART_TX  T113_UART3_TX
#  define CONSOLE_UART_RX  T113_UART3_RX
#elif defined(CONFIG_UART4_SERIAL_CONSOLE)
#  define CONSOLE_BASE     T113_UART4_VADDR
#  define CONSOLE_BAUD     CONFIG_UART4_BAUD
#  define CONSOLE_BITS     CONFIG_UART4_BITS
#  define CONSOLE_PARITY   CONFIG_UART4_PARITY
#  define CONSOLE_2STOP    CONFIG_UART4_2STOP
#  define CONSOLE_UART_NUM 4
#  define CONSOLE_UART_TX  T113_UART4_TX
#  define CONSOLE_UART_RX  T113_UART4_RX
#elif defined(CONFIG_UART5_SERIAL_CONSOLE)
#  define CONSOLE_BASE     T113_UART5_VADDR
#  define CONSOLE_BAUD     CONFIG_UART5_BAUD
#  define CONSOLE_BITS     CONFIG_UART5_BITS
#  define CONSOLE_PARITY   CONFIG_UART5_PARITY
#  define CONSOLE_2STOP    CONFIG_UART5_2STOP
#  define CONSOLE_UART_NUM 5
#  define CONSOLE_UART_TX  T113_UART5_TX
#  define CONSOLE_UART_RX  T113_UART5_RX
#elif defined(CONFIG_ARCH_LOWPUTC)
  /* LOWPUTC without full serial driver: default to UART0 115200-8N1 */

#  define CONSOLE_BASE     T113_UART0_VADDR
#  define CONSOLE_BAUD     115200
#  define CONSOLE_BITS     8
#  define CONSOLE_PARITY   0
#  define CONSOLE_2STOP    0
#  define CONSOLE_UART_NUM 0
#  define CONSOLE_UART_TX  T113_UART0_TX
#  define CONSOLE_UART_RX  T113_UART0_RX
#elif defined(HAVE_SERIAL_CONSOLE)
#  error "No CONFIG_UARTn_SERIAL_CONSOLE setting"
#endif

#if defined(HAVE_SERIAL_CONSOLE) || defined(CONFIG_ARCH_LOWPUTC)
#  if CONSOLE_BITS == 5
#    define CONSOLE_LCR_DLS UART_LCR_DLS_5BITS
#  elif CONSOLE_BITS == 6
#    define CONSOLE_LCR_DLS UART_LCR_DLS_6BITS
#  elif CONSOLE_BITS == 7
#    define CONSOLE_LCR_DLS UART_LCR_DLS_7BITS
#  elif CONSOLE_BITS == 8
#    define CONSOLE_LCR_DLS UART_LCR_DLS_8BITS
#  else
#    error "Invalid CONFIG_UARTn_BITS setting for console"
#  endif

#  if CONSOLE_PARITY == 0
#    define CONSOLE_LCR_PAR 0
#  elif CONSOLE_PARITY == 1
#    define CONSOLE_LCR_PAR UART_LCR_PEN
#  elif CONSOLE_PARITY == 2
#    define CONSOLE_LCR_PAR (UART_LCR_PEN | UART_LCR_EPS)
#  else
#    error "Invalid CONFIG_UARTn_PARITY setting for CONSOLE"
#  endif

#  if CONSOLE_2STOP != 0
#    define CONSOLE_LCR_STOP UART_LCR_STOP
#  else
#    define CONSOLE_LCR_STOP 0
#  endif

#  define CONSOLE_LCR_VALUE \
     (CONSOLE_LCR_DLS | CONSOLE_LCR_PAR | CONSOLE_LCR_STOP)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_lowputc(char ch)
{
#if defined(HAVE_SERIAL_CONSOLE) || defined(CONFIG_ARCH_LOWPUTC)
  while (!(getreg32(CONSOLE_BASE + T113_UART_LSR_OFFSET) & UART_LSR_TEMT))
    {
    }

  putreg32((uint32_t)ch, CONSOLE_BASE + T113_UART_THR_OFFSET);
#endif
}

void t113_lowsetup(void)
{
#if (defined(HAVE_SERIAL_CONSOLE) || defined(CONFIG_ARCH_LOWPUTC)) && \
    !defined(CONFIG_SUPPRESS_UART_CONFIG)
  uint32_t console_dl;

#if CONSOLE_UART_NUM == 1
  /* UART1 console: do NOT assert reset.  Asserting reset clears FCCR
   * to a broken default (0x10003); the BROM power-on state has the
   * correct FCCR (0x4003).  Match the sequence in t113_uart1config.
   *
   * Boot-time call before t113_ccu_init(): the AMP backend falls back
   * to plain irq save/restore, which is correct on the single CPU
   * still active here.
   */

  t113_ccu_modify(T113_CCU_UART_BGR, 0,
                  (1 << (CONSOLE_UART_NUM + 16)) |
                  (1 << CONSOLE_UART_NUM));
#else
  /* Assert reset for the console UART, barrier, then deassert and
   * enable clock gate.  Same boot-time fallback note applies.
   */

  t113_ccu_modify(T113_CCU_UART_BGR,
                  1 << (CONSOLE_UART_NUM + 16), 0);
  UP_DSB();
  t113_ccu_modify(T113_CCU_UART_BGR, 0,
                  (1 << (CONSOLE_UART_NUM + 16)) |
                  (1 << CONSOLE_UART_NUM));
#endif

  /* Console UART pin mux + pull-up (per User Manual section 9.7).
   * Pin selection comes from board.h via t113_pinmap.h.
   */

  t113_gpio_config(CONSOLE_UART_TX);
  t113_gpio_config(CONSOLE_UART_RX);

  putreg32(UART_FCR_RFIFOR | UART_FCR_XFIFOR,
           CONSOLE_BASE + T113_UART_FCR_OFFSET);
  putreg32(UART_FCR_FIFOE | UART_FCR_RT_ONE,
           CONSOLE_BASE + T113_UART_FCR_OFFSET);
  putreg32(CONSOLE_LCR_VALUE | UART_LCR_DLAB,
           CONSOLE_BASE + T113_UART_LCR_OFFSET);
  /* Derive the divisor from the live APB1 rate (round-to-nearest) rather
   * than the compile-time T113_UART_DL() macro.  clk_init() programs APB1
   * before this runs on a normal boot, so the value matches the constant;
   * under a foreign master (Linux remoteproc) that owns APB1 it tracks
   * whatever rate the master left.
   */

  console_dl = T113_UART_DL_RTN(t113_apb1_freq(), CONSOLE_BAUD);
  putreg32(console_dl >> 8, CONSOLE_BASE + T113_UART_DLH_OFFSET);
  putreg32(console_dl & 0xff, CONSOLE_BASE + T113_UART_DLL_OFFSET);
  putreg32(CONSOLE_LCR_VALUE,
           CONSOLE_BASE + T113_UART_LCR_OFFSET);
  putreg32(UART_FCR_RT_ONE | UART_FCR_XFIFOR |
           UART_FCR_RFIFOR | UART_FCR_FIFOE,
           CONSOLE_BASE + T113_UART_FCR_OFFSET);

  putreg32(0, CONSOLE_BASE + T113_UART_IER_OFFSET);

  /* Clear console UART IRQ enable in GIC distributor.
   * UART IRQs are SPI 34..39 (UART0..5), which map to
   * GIC interrupt IDs 34..39 in GICD_ICENABLER1 (IDs 32..63).
   */

  putreg32(1 << (2 + CONSOLE_UART_NUM),
           T113_GIC_DIST_PADDR + 0x184);
#endif
}
