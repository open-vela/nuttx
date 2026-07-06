/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_serial.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_SERIAL_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm64_internal.h"
#include "arm64_gic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RK3576 UART Register Offsets (DW APB UART, TRM Chapter 10) */

#define RK3576_UART_RBR_OFFSET      0x00  /* Receive Buffer Register (RO) */
#define RK3576_UART_THR_OFFSET      0x00  /* Transmit Holding Register (WO) */
#define RK3576_UART_DLL_OFFSET      0x00  /* Divisor Latch Low (RW, DLAB=1) */
#define RK3576_UART_DLH_OFFSET      0x04  /* Divisor Latch High (RW, DLAB=1) */
#define RK3576_UART_IER_OFFSET      0x04  /* Interrupt Enable Register (RW) */
#define RK3576_UART_IIR_OFFSET      0x08  /* Interrupt Identification Register (RO) */
#define RK3576_UART_FCR_OFFSET      0x08  /* FIFO Control Register (WO) */
#define RK3576_UART_LCR_OFFSET      0x0c  /* Line Control Register (RW) */
#define RK3576_UART_MCR_OFFSET      0x10  /* Modem Control Register (RW) */
#define RK3576_UART_LSR_OFFSET      0x14  /* Line Status Register (RO) */
#define RK3576_UART_MSR_OFFSET      0x18  /* Modem Status Register (RO) */
#define RK3576_UART_USR_OFFSET      0x7c  /* UART Status Register (RO) */

/* RK3576 UART Register Bit Definitions */

/* IER bits */

#define RK3576_UART_IER_ERBFI       (1 << 0)  /* Enable RX Data Interrupt */
#define RK3576_UART_IER_ETBEI       (1 << 1)  /* Enable TX Holding Empty Interrupt */
#define RK3576_UART_IER_ELSI        (1 << 2)  /* Enable Receiver Line Status Interrupt */
#define RK3576_UART_IER_EDSSI       (1 << 3)  /* Enable Modem Status Interrupt */

/* IIR bits */

#define RK3576_UART_IIR_IID_SHIFT   0         /* Bits 0-3: Interrupt ID */
#define RK3576_UART_IIR_IID_MASK    (15 << RK3576_UART_IIR_IID_SHIFT)
#define RK3576_UART_IIR_IID_NONE    (1 << RK3576_UART_IIR_IID_SHIFT)  /* No interrupt */
#define RK3576_UART_IIR_IID_MODEM   (0 << RK3576_UART_IIR_IID_SHIFT)  /* Modem status */
#define RK3576_UART_IIR_IID_TXEMPTY (2 << RK3576_UART_IIR_IID_SHIFT)  /* THR empty */
#define RK3576_UART_IIR_IID_RECV    (4 << RK3576_UART_IIR_IID_SHIFT)  /* RX data available */
#define RK3576_UART_IIR_IID_LINESTATUS (6 << RK3576_UART_IIR_IID_SHIFT) /* Line status */
#define RK3576_UART_IIR_IID_BUSY    (7 << RK3576_UART_IIR_IID_SHIFT)  /* Busy detect */
#define RK3576_UART_IIR_IID_TIMEOUT (12 << RK3576_UART_IIR_IID_SHIFT) /* Character timeout */

#define RK3576_UART_IIR_FEFLAG_SHIFT 6  /* Bits 6-7: FIFOs Enabled */
#define RK3576_UART_IIR_FEFLAG_MASK  (3 << RK3576_UART_IIR_FEFLAG_SHIFT)
#define RK3576_UART_IIR_FEFLAG_DISABLE (0 << RK3576_UART_IIR_FEFLAG_SHIFT)
#define RK3576_UART_IIR_FEFLAG_ENABLE  (3 << RK3576_UART_IIR_FEFLAG_SHIFT)

/* FCR bits */

#define RK3576_UART_FCR_FIFOE       (1 << 0)  /* FIFO Enable */
#define RK3576_UART_FCR_RFIFOR      (1 << 1)  /* Receiver FIFO Reset */
#define RK3576_UART_FCR_XFIFOR      (1 << 2)  /* Transmitter FIFO Reset */
#define RK3576_UART_FCR_DMAM        (1 << 3)  /* DMA Mode Select */

#define RK3576_UART_FCR_TFT_SHIFT   4         /* Bits 4-5: TX Empty Trigger */
#define RK3576_UART_FCR_TFT_MASK    (3 << RK3576_UART_FCR_TFT_SHIFT)
#define RK3576_UART_FCR_TFT_EMPTY   (0 << RK3576_UART_FCR_TFT_SHIFT)
#define RK3576_UART_FCR_TFT_TWO     (1 << RK3576_UART_FCR_TFT_SHIFT)
#define RK3576_UART_FCR_TFT_QUARTER (2 << RK3576_UART_FCR_TFT_SHIFT)
#define RK3576_UART_FCR_TFT_HALF    (3 << RK3576_UART_FCR_TFT_SHIFT)

#define RK3576_UART_FCR_RT_SHIFT    6         /* Bits 6-7: RCVR Trigger */
#define RK3576_UART_FCR_RT_MASK     (3 << RK3576_UART_FCR_RT_SHIFT)
#define RK3576_UART_FCR_RT_ONE      (0 << RK3576_UART_FCR_RT_SHIFT)
#define RK3576_UART_FCR_RT_QUARTER  (1 << RK3576_UART_FCR_RT_SHIFT)
#define RK3576_UART_FCR_RT_HALF     (2 << RK3576_UART_FCR_RT_SHIFT)
#define RK3576_UART_FCR_RT_MINUS2   (3 << RK3576_UART_FCR_RT_SHIFT)

/* LCR bits */

#define RK3576_UART_LCR_DLS_SHIFT   0         /* Bits 0-1: Data Length Select */
#define RK3576_UART_LCR_DLS_MASK    (3 << RK3576_UART_LCR_DLS_SHIFT)
#define RK3576_UART_LCR_DLS_5BITS   (0 << RK3576_UART_LCR_DLS_SHIFT)
#define RK3576_UART_LCR_DLS_6BITS   (1 << RK3576_UART_LCR_DLS_SHIFT)
#define RK3576_UART_LCR_DLS_7BITS   (2 << RK3576_UART_LCR_DLS_SHIFT)
#define RK3576_UART_LCR_DLS_8BITS   (3 << RK3576_UART_LCR_DLS_SHIFT)

#define RK3576_UART_LCR_STOP        (1 << 2)  /* Stop Bit Select */
#define RK3576_UART_LCR_PEN         (1 << 3)  /* Parity Enable */
#define RK3576_UART_LCR_EPS         (1 << 4)  /* Even Parity Select */
#define RK3576_UART_LCR_STICK_PARITY (1 << 5) /* Stick Parity */
#define RK3576_UART_LCR_BC          (1 << 6)  /* Break Control */
#define RK3576_UART_LCR_DLAB        (1 << 7)  /* Divisor Latch Access Bit */

/* LSR bits */

#define RK3576_UART_LSR_DR          (1 << 0)  /* Data Ready */
#define RK3576_UART_LSR_OE          (1 << 1)  /* Overrun Error */
#define RK3576_UART_LSR_PE          (1 << 2)  /* Parity Error */
#define RK3576_UART_LSR_FE          (1 << 3)  /* Framing Error */
#define RK3576_UART_LSR_BI          (1 << 4)  /* Break Interrupt */
#define RK3576_UART_LSR_THRE        (1 << 5)  /* TX Holding Register Empty */
#define RK3576_UART_LSR_TEMT        (1 << 6)  /* Transmitter Empty */
#define RK3576_UART_LSR_FIFOERR     (1 << 7)  /* RX FIFO Error */

/* MCR bits */

#define RK3576_UART_MCR_DTR         (1 << 0)  /* Data Terminal Ready */
#define RK3576_UART_MCR_RTS         (1 << 1)  /* Request To Send */
#define RK3576_UART_MCR_OUT1        (1 << 2)  /* Output 1 */
#define RK3576_UART_MCR_OUT2        (1 << 3)  /* Output 2 */
#define RK3576_UART_MCR_LOOP        (1 << 4)  /* Loopback Mode */
#define RK3576_UART_MCR_AFCE        (1 << 5)  /* Auto Flow Control Enable */

/* USR bits */

#define RK3576_UART_USR_BUSY        (1 << 0)  /* UART Busy */
#define RK3576_UART_USR_TFNF        (1 << 1)  /* TX FIFO Not Full */
#define RK3576_UART_USR_RFNE        (1 << 2)  /* RX FIFO Not Empty */
#define RK3576_UART_USR_TFE         (1 << 3)  /* TX FIFO Empty */
#define RK3576_UART_USR_RFEM        (1 << 4)  /* RX FIFO Empty */

/* UART SCLK — RK3576 uses 24MHz oscillator (XIN24M) */

#define RK3576_UART_SCLK            24000000

/* Timeout for UART Busy Wait, in milliseconds */

#define RK3576_UART_TIMEOUT_MS      100

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_CHIP_RK3576

/* IRQ Numbers (SPI level, from GICv3) — Rockchip RK3576 TRM */

#define RK3576_UART0_IRQ            131
#define RK3576_UART1_IRQ            130
#define RK3576_UART2_IRQ            132
#define RK3576_UART3_IRQ            133
#define RK3576_UART4_IRQ            134

#endif /* CONFIG_ARCH_CHIP_RK3576 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_SERIAL_H */
