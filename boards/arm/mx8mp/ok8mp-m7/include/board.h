/****************************************************************************
 * boards/arm/mx8mp/ok8mp-m7/include/board.h
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

#ifndef __BOARDS_ARM_MX8MP_OK8MP_M7_INCLUDE_BOARD_H
#define __BOARDS_ARM_MX8MP_OK8MP_M7_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The OK8MP M7 console uses the dedicated UART4 pads.  The architecture
 * serial driver configures these pads and the UART4 clock root.
 */

#define IOMUX_CONSOLE_UART_RX IOMUXC_UART4_RXD_UART4_RX, 0, UART_PAD_CTRL
#define IOMUX_CONSOLE_UART_TX IOMUXC_UART4_TXD_UART4_TX, 0, UART_PAD_CTRL

#endif /* __BOARDS_ARM_MX8MP_OK8MP_M7_INCLUDE_BOARD_H */
