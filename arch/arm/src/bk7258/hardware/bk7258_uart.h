/****************************************************************************
 * arch/arm/src/bk7258/hardware/bk7258_uart.h
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

/* Register layout from ARMINO SDK
 * ap/middleware/soc/bk7258_ap/soc/uart_struct.h (uart_hw_t): each register
 * is 32-bit and laid out contiguously.
 */

#ifndef __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_UART_H
#define __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_UART_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "bk7258_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define BK7258_UART_DEVID_OFFSET       0x0000  /* dev_id */
#define BK7258_UART_VERSION_OFFSET     0x0004  /* dev_version */
#define BK7258_UART_GLOBAL_CTRL_OFFSET 0x0008  /* global_ctrl */
#define BK7258_UART_DEVSTATUS_OFFSET   0x000c  /* dev_status */
#define BK7258_UART_CONFIG_OFFSET      0x0010  /* config */
#define BK7258_UART_FIFO_CFG_OFFSET    0x0014  /* fifo_config */
#define BK7258_UART_FIFO_STATUS_OFFSET 0x0018  /* fifo_status */
#define BK7258_UART_FIFO_PORT_OFFSET   0x001c  /* fifo_port (TX/RX data) */
#define BK7258_UART_INT_ENABLE_OFFSET  0x0020  /* int_enable */
#define BK7258_UART_INT_STATUS_OFFSET  0x0024  /* int_status */
#define BK7258_UART_FLOW_CFG_OFFSET    0x0028  /* flow_ctrl_config */
#define BK7258_UART_WAKE_CFG_OFFSET    0x002c  /* wake_config */

/* Register addresses (per UART instance) ***********************************/

#define BK7258_UART_GLOBAL_CTRL(b)     ((b) + BK7258_UART_GLOBAL_CTRL_OFFSET)
#define BK7258_UART_CONFIG(b)          ((b) + BK7258_UART_CONFIG_OFFSET)
#define BK7258_UART_FIFO_CFG(b)        ((b) + BK7258_UART_FIFO_CFG_OFFSET)
#define BK7258_UART_FIFO_STATUS(b)     ((b) + BK7258_UART_FIFO_STATUS_OFFSET)
#define BK7258_UART_FIFO_PORT(b)       ((b) + BK7258_UART_FIFO_PORT_OFFSET)
#define BK7258_UART_INT_ENABLE(b)      ((b) + BK7258_UART_INT_ENABLE_OFFSET)
#define BK7258_UART_INT_STATUS(b)      ((b) + BK7258_UART_INT_STATUS_OFFSET)

/* GLOBAL_CTRL (0x08) bit definitions ***************************************/

#define UART_GLOBAL_SOFT_RESET         (1 << 0)  /* bit0: uart soft reset */
#define UART_GLOBAL_CLK_GATE_BYPASS    (1 << 1)  /* bit1: bypass clock gate */

/* CONFIG (0x10) bit definitions ********************************************/

#define UART_CFG_TX_ENABLE             (1 << 0)  /* bit0: tx enable */
#define UART_CFG_RX_ENABLE             (1 << 1)  /* bit1: rx enable */
#define UART_CFG_DATA_BITS_SHIFT       3         /* bits[3:4]: data bits */
#define UART_CFG_DATA_BITS_MASK        (0x3 << UART_CFG_DATA_BITS_SHIFT)
#  define UART_CFG_DATA_BITS_5         (0x0 << UART_CFG_DATA_BITS_SHIFT)
#  define UART_CFG_DATA_BITS_6         (0x1 << UART_CFG_DATA_BITS_SHIFT)
#  define UART_CFG_DATA_BITS_7         (0x2 << UART_CFG_DATA_BITS_SHIFT)
#  define UART_CFG_DATA_BITS_8         (0x3 << UART_CFG_DATA_BITS_SHIFT)
#define UART_CFG_PARITY_EN             (1 << 5)  /* bit5: parity enable */
#define UART_CFG_PARITY_ODD            (1 << 6)  /* bit6: 0=even,1=odd */
#define UART_CFG_STOP_BITS_2           (1 << 7)  /* bit7: 0=1bit,1=2bit */
#define UART_CFG_CLK_DIV_SHIFT         8         /* bits[8:23]: clk_div */
#define UART_CFG_CLK_DIV_MASK          (0xffff << UART_CFG_CLK_DIV_SHIFT)

/* FIFO_STATUS (0x18) bit definitions ***************************************/

#define UART_FIFO_TX_COUNT_SHIFT       0         /* bits[0:7] */
#define UART_FIFO_TX_COUNT_MASK        (0xff << 0)
#define UART_FIFO_RX_COUNT_SHIFT       8         /* bits[8:15] */
#define UART_FIFO_RX_COUNT_MASK        (0xff << 8)
#define UART_FIFO_TX_FULL              (1 << 16) /* bit16 */
#define UART_FIFO_TX_EMPTY             (1 << 17) /* bit17 */
#define UART_FIFO_RX_FULL              (1 << 18) /* bit18 */
#define UART_FIFO_RX_EMPTY             (1 << 19) /* bit19 */
#define UART_FIFO_WR_READY             (1 << 20) /* bit20: can write TX FIFO */
#define UART_FIFO_RD_READY             (1 << 21) /* bit21: RX FIFO has data */

/* FIFO_PORT (0x1c): writing (v & 0xff) sends one byte; the low 8 bits of a
 * read hold the received byte.  Ref uart_ll_write_byte():
 * hw->fifo_port.v = data & 0xff;
 */

#define UART_FIFO_RX_DATA_SHIFT        8         /* rx_fifo_data_out [8:15] */
#define UART_FIFO_RX_DATA_MASK         (0xff << 8)

/* INT_ENABLE (0x20) / INT_STATUS (0x24) bit definitions ********************/

#define UART_INT_TX_NEED_WRITE         (1 << 0)  /* bit0: TX FIFO writable */
#define UART_INT_RX_NEED_READ          (1 << 1)  /* bit1: RX FIFO has data */
#define UART_INT_RX_OVERFLOW           (1 << 2)  /* bit2: RX FIFO overflow */
#define UART_INT_RX_PARITY_ERR         (1 << 3)  /* bit3 */
#define UART_INT_RX_STOP_ERR           (1 << 4)  /* bit4 */
#define UART_INT_TX_FINISH             (1 << 5)  /* bit5 */
#define UART_INT_RX_FINISH             (1 << 6)  /* bit6: RX idle (stop) */
#define UART_INT_RXD_WAKEUP            (1 << 7)  /* bit7 */

#endif /* __ARCH_ARM_SRC_BK7258_HARDWARE_BK7258_UART_H */
