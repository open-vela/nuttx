/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_uart.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_UART_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_RBR_OFFSET    0x00
#define UART_THR_OFFSET    0x00
#define UART_IER_OFFSET    0x01
#define UART_IIR_OFFSET    0x02
#define UART_FCR_OFFSET    0x02
#define UART_LCR_OFFSET    0x03
#define UART_MCR_OFFSET    0x04
#define UART_LSR_OFFSET    0x05
#define UART_MSR_OFFSET    0x06
#define UART_SCR_OFFSET    0x07
#define UART_DLL_OFFSET    0x00
#define UART_DLH_OFFSET    0x01

#define UART_LSR_DR        (1 << 0)
#define UART_LSR_OE        (1 << 1)
#define UART_LSR_PE        (1 << 2)
#define UART_LSR_FE        (1 << 3)
#define UART_LSR_BI        (1 << 4)
#define UART_LSR_THRE      (1 << 5)
#define UART_LSR_TEMT      (1 << 6)
#define UART_LSR_FIFOE     (1 << 7)

#define UART_IER_ERBFI     (1 << 0)
#define UART_IER_ETBEI     (1 << 1)
#define UART_IER_ELSI      (1 << 2)
#define UART_IER_EDSSI     (1 << 3)

#define UART_IIR_NOINT     0x01
#define UART_IIR_THRI      0x02
#define UART_IIR_RDI       0x04
#define UART_IIR_RLSI      0x06
#define UART_IIR_CTI       0x0c

#define UART_FCR_FIFOEN    (1 << 0)
#define UART_FCR_RXRST     (1 << 1)
#define UART_FCR_TXRST     (1 << 2)
#define UART_FCR_DMAM      (1 << 3)
#define UART_FCR_RXTRG_1   (0 << 6)
#define UART_FCR_RXTRG_4   (1 << 6)
#define UART_FCR_RXTRG_8   (2 << 6)
#define UART_FCR_RXTRG_14  (3 << 6)

#define UART_LCR_WLEN5     0x00
#define UART_LCR_WLEN6     0x01
#define UART_LCR_WLEN7     0x02
#define UART_LCR_WLEN8     0x03
#define UART_LCR_STOP1     0x00
#define UART_LCR_STOP2     0x04
#define UART_LCR_PNONE     0x00
#define UART_LCR_PEVEN     0x18
#define UART_LCR_PODD      0x08
#define UART_LCR_DLAB      0x80

#define UART_MCR_DTR       (1 << 0)
#define UART_MCR_RTS       (1 << 1)
#define UART_MCR_OUT1      (1 << 2)
#define UART_MCR_OUT2      (1 << 3)
#define UART_MCR_LOOP      (1 << 4)

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_UART_H */
