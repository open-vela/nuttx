/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_serial.c
 *
 * UART5 console glue: the generic 16550 driver handles the DW APB UART
 * (REGWIDTH=32).  Clock and pinmux are programmed by
 * rk3576_board_initialize() before this runs.
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_16550_UART

#include <nuttx/serial/uart_16550.h>

#include "arm64_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm64_earlyserialinit(void)
{
  u16550_earlyserialinit();
}

void arm64_serialinit(void)
{
  u16550_serialinit();
}

#endif /* CONFIG_16550_UART */
