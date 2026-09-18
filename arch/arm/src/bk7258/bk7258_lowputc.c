/****************************************************************************
 * arch/arm/src/bk7258/bk7258_lowputc.c
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

#include "arm_internal.h"
#include "hardware/bk7258_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Console defaults to UART0: the on-board CH340 is UART0 (bootROM DL_UART),
 * so download and log share one cable.  NuttX runs as the CPU0 main core
 * directly attached to this port.
 */

#ifndef CONFIG_BK7258_CONSOLE_UART_BASE
#  define CONFIG_BK7258_CONSOLE_UART_BASE  BK7258_UART0_BASE
#endif

#ifndef CONFIG_BK7258_CONSOLE_BAUD
#  define CONFIG_BK7258_CONSOLE_BAUD       115200
#endif

/* UART clock source = 26MHz XTAL (ARMINO: UART_CLOCK = CONFIG_XTAL_FREQ,
 * default 26M).  Baud formula (ARMINO uart_ll):
 *   baud = UART_CLK / (clk_div + 1)  =>  clk_div = UART_CLK / baud - 1
 *   26000000 / 115200 = 225.69 -> clk_div = 225 (actual 115044, ~0.13%).
 */

#define BK7258_UART_CLK   26000000u
#define BK7258_UART_DIV \
  (((BK7258_UART_CLK + (CONFIG_BK7258_CONSOLE_BAUD / 2)) / \
    CONFIG_BK7258_CONSOLE_BAUD) - 1)

#define CONSOLE_BASE      CONFIG_BK7258_CONSOLE_UART_BASE

/* --- Peripheral clock enable + GPIO pin mux ------------------------------
 * As the CPU0 main core, NuttX must enable the UART peripheral clock itself
 * and mux the TX/RX pins to the UART function (the bootloader may have torn
 * them down at hand-off).  Registers from the ARMINO SDK:
 *   Clock:  SYS_CPU_DEVICE_CLK_ENABLE @ 0x44010030 (UART0_CKEN = bit2)
 *   Source: SYS_CLK_DIV_MODE1 @ 0x44010020 (cksel_uart0 = bit10, 0 = XTAL)
 *   Pins:   system function select pins 8-15 @ 0x440100C4 (4 bits each,
 *           value 0 = UART0)
 *   Per pin: config @ 0x44000400 + n*4, bit6 = second (peripheral) function
 */

#define BK7258_SYS_CLK_EN        (BK7258_SYS_BASE + 0x30)      /* 0x44010030 */
#  define SYS_CLK_EN_UART0       (1 << 2)
#define BK7258_SYS_GPIO_FUNC8_15 (BK7258_SYS_BASE + 0xc4)      /* 0x440100C4 */
#define BK7258_SYS_CLK_DIV_MODE1 (BK7258_SYS_BASE + 0x20)      /* 0x44010020 */
#define BK7258_GPIO_CFG(n)       (BK7258_AON_GPIO_BASE + (n) * 4)
#  define GPIO_CFG_SECOND_FUNC   (1 << 6)

static void bk7258_putc_uart(uintptr_t base, char ch);

/* Enable the UART0 clock + clock source (XTAL) and mux GPIO11/GPIO10 to
 * UART0 TXD/RXD.  As the CPU0 main core the console uses UART0 (CH340); the
 * bootloader may have torn UART0 down at hand-off, so re-enable the clock
 * and re-mux the pins here, otherwise writing the FIFO emits no character.
 * Pins: UART0_TX = GPIO11, UART0_RX = GPIO10 (function index 0).
 */

static void bk7258_uart0_hwsetup(void)
{
  /* 1) Enable the UART0 peripheral clock */

  modifyreg32(BK7258_SYS_CLK_EN, 0, SYS_CLK_EN_UART0);

  /* 2) UART0 clock source = XTAL 26M (cksel_uart0 = bit10, 0 = XTAL) */

  modifyreg32(BK7258_SYS_CLK_DIV_MODE1, (1u << 10), 0);

  /* 3) GPIO10 -> UART0_RXD [11:8], GPIO11 -> UART0_TXD [15:12], func 0 */

  modifyreg32(BK7258_SYS_GPIO_FUNC8_15, (0xfu << 8) | (0xfu << 12), 0);

  /* 4) Enable the second (peripheral) function on GPIO10/GPIO11 */

  modifyreg32(BK7258_GPIO_CFG(10), 0, GPIO_CFG_SECOND_FUNC);
  modifyreg32(BK7258_GPIO_CFG(11), 0, GPIO_CFG_SECOND_FUNC);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bk7258_lowsetup
 ****************************************************************************/

static void bk7258_uart_config(uintptr_t base)
{
  uint32_t cfg;

  /* Do not soft reset: the bootloader has already configured UART0 and is
   * transmitting normally (div=225, TX on).  A soft reset would clear its
   * working TX state so TX no longer drains -> putc times out and drops
   * characters (symptom: CH340 emits a single garbage byte).  Only ensure
   * 8 data bits / baud / TX enable on top of the existing config; do not
   * reset or override the bootloader's working clock/FIFO state.
   */

  cfg  = getreg32(BK7258_UART_CONFIG(base));
  cfg &= ~(UART_CFG_DATA_BITS_MASK | UART_CFG_CLK_DIV_MASK);
  cfg |= UART_CFG_DATA_BITS_8;
  cfg |= (BK7258_UART_DIV << UART_CFG_CLK_DIV_SHIFT) & UART_CFG_CLK_DIV_MASK;
  cfg |= UART_CFG_TX_ENABLE;

  putreg32(cfg, BK7258_UART_CONFIG(base));
}

void bk7258_lowsetup(void)
{
  /* The CPU0 main core enables the UART0 (console = CH340) clock and pin
   * mux itself (the bootloader may have torn it down at hand-off), then
   * configures data bits / baud / TX.
   */

  bk7258_uart0_hwsetup();
  bk7258_uart_config(BK7258_UART0_BASE);
}

/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Output one byte on the serial console (polled, blocking).
 *
 ****************************************************************************/

static void bk7258_putc_uart(uintptr_t base, char ch)
{
  volatile int timeout = 200000;

  while (((getreg32(BK7258_UART_FIFO_STATUS(base)) &
           UART_FIFO_WR_READY) == 0) && (--timeout > 0))
    {
    }

  putreg32((uint32_t)(uint8_t)ch, BK7258_UART_FIFO_PORT(base));
}

void arm_lowputc(char ch)
{
  bk7258_putc_uart(CONSOLE_BASE, ch);
}
