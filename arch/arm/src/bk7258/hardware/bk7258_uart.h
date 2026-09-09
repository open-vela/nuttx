/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_uart.h
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#ifndef __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_UART_H
#define __VENDOR_BEKEN_CHIPS_BK7258_HARDWARE_BK7258_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bk7258_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BK7258_UART1_CONFIG       (BK7258_UART1_BASE + 0x10u)
#define BK7258_UART1_FIFO_CONFIG  (BK7258_UART1_BASE + 0x14u)
#define BK7258_UART1_FIFO_STATUS  (BK7258_UART1_BASE + 0x18u)
#define BK7258_UART1_FIFO_PORT    (BK7258_UART1_BASE + 0x1cu)
#define BK7258_UART1_INT_ENABLE   (BK7258_UART1_BASE + 0x20u)
#define BK7258_UART1_INT_STATUS   (BK7258_UART1_BASE + 0x24u)

#define BK7258_UART_TX_ENABLE     (1u << 0)
#define BK7258_UART_RX_ENABLE     (1u << 1)
#define BK7258_UART_DATA_8BIT     (3u << 3)
#define BK7258_UART_CLKDIV_SHIFT  8

#define BK7258_UART_FIFO_WR_READY (1u << 20)
#define BK7258_UART_FIFO_RD_READY (1u << 21)
#define BK7258_UART_TX_EMPTY      (1u << 17)

#define BK7258_UART_INT_TX        (1u << 0)
#define BK7258_UART_INT_RX        ((1u << 1) | (1u << 6))
#define BK7258_UART_INT_ALL       0xffu

#define BK7258_UART_CLOCK         26000000u

#endif
