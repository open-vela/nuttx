/****************************************************************************
 * boards/arm/t113/t113-evb/include/board.h
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

#ifndef __BOARDS_ARM_T113_T113_EVB_INCLUDE_BOARD_H
#define __BOARDS_ARM_T113_T113_EVB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/t113_pinmap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* UART0 parameters */

#define BOARD_UART0_BAUD     115200
#define BOARD_UART0_BITS     8
#define BOARD_UART0_PARITY   0
#define BOARD_UART0_2STOP    0

/* Pin selections - choose one variant per signal from t113_pinmap.h.
 * Change these if the board routes a peripheral to different pins.
 */

/* UART0: PF2/PF4 (group 1) -- console
 *
 * When CONFIG_T113_SMHC0 is enabled, PF0-PF5 are repurposed for SDC0
 * (Func2), which collides with UART0 on PF2/PF4 (Func3). Fall back to
 * the alternate group on PE2/PE3 (Func6) -- these pins are also routed
 * to RGMII PHY on T113-EVB; user must fly-wire PE2/PE3 directly to a
 * USB-UART adapter (board RGMII traces are unused for this config).
 */

#ifdef CONFIG_T113_SMHC0
#  define T113_UART0_TX      T113_UART0_TX_2      /* PE2  */
#  define T113_UART0_RX      T113_UART0_RX_2      /* PE3  */
#else
#  define T113_UART0_TX      T113_UART0_TX_1      /* PF2  */
#  define T113_UART0_RX      T113_UART0_RX_1      /* PF4  */
#endif

/* UART1: PG6/PG7/PG8/PG9 (group 2, 4-wire) */

#define T113_UART1_TX        T113_UART1_TX_2      /* PG6  */
#define T113_UART1_RX        T113_UART1_RX_2      /* PG7  */
#define T113_UART1_RTS       T113_UART1_RTS_2     /* PG8  */
#define T113_UART1_CTS       T113_UART1_CTS_2     /* PG9  */

/* UART2: PE2/PE3/PE0/PE1 (group 2, 4-wire) */

#define T113_UART2_TX        T113_UART2_TX_2      /* PE2  */
#define T113_UART2_RX        T113_UART2_RX_2      /* PE3  */
#define T113_UART2_RTS       T113_UART2_RTS_2     /* PE0  */
#define T113_UART2_CTS       T113_UART2_CTS_2     /* PE1  */

/* UART3: PB6/PB7 (group 1, 2-wire) */

#define T113_UART3_TX        T113_UART3_TX_1      /* PB6  */
#define T113_UART3_RX        T113_UART3_RX_1      /* PB7  */

/* UART4: PE4/PE5 (group 3, 2-wire) */

#define T113_UART4_TX        T113_UART4_TX_3      /* PE4  */
#define T113_UART4_RX        T113_UART4_RX_3      /* PE5  */

/* UART5: PE6/PE7 (group 3, 2-wire) */

#define T113_UART5_TX        T113_UART5_TX_3      /* PE6  */
#define T113_UART5_RX        T113_UART5_RX_3      /* PE7  */

/* SPI0: PC2-PC7 (group 1, only option) */

#define T113_SPI0_CLK        T113_SPI0_CLK_1      /* PC2  */
#define T113_SPI0_CS         T113_SPI0_CS_1       /* PC3  */
#define T113_SPI0_MOSI       T113_SPI0_MOSI_1     /* PC4  */
#define T113_SPI0_MISO       T113_SPI0_MISO_1     /* PC5  */
#define T113_SPI0_WP         T113_SPI0_WP_1       /* PC6  */
#define T113_SPI0_HOLD       T113_SPI0_HOLD_1     /* PC7  */

/* SPI1: PD10-PD15 (group 1, only option) */

#define T113_SPI1_CS         T113_SPI1_CS_1       /* PD10 */
#define T113_SPI1_CLK        T113_SPI1_CLK_1      /* PD11 */
#define T113_SPI1_MOSI       T113_SPI1_MOSI_1     /* PD12 */
#define T113_SPI1_MISO       T113_SPI1_MISO_1     /* PD13 */
#define T113_SPI1_HOLD       T113_SPI1_HOLD_1     /* PD14 */
#define T113_SPI1_WP         T113_SPI1_WP_1       /* PD15 */

/* TWI0: PB2/PB3 (group 1) */

#define T113_TWI0_SCK        T113_TWI0_SCK_1      /* PB3  */
#define T113_TWI0_SDA        T113_TWI0_SDA_1      /* PB2  */

/* TWI1: PB4/PB5 (group 1) */

#define T113_TWI1_SCK        T113_TWI1_SCK_1      /* PB4  */
#define T113_TWI1_SDA        T113_TWI1_SDA_1      /* PB5  */

/* TWI2: PD20/PD21 (group 1) */

#define T113_TWI2_SCK        T113_TWI2_SCK_1      /* PD20 */
#define T113_TWI2_SDA        T113_TWI2_SDA_1      /* PD21 */

/* Capacitive Touch Panel (J27 MIPI FFC) - uses TWI2 defined above.
 *
 *   INT = PD19, configured INPUT + PULL_UP.  The CTP-INT line has no
 *     external pull-up on the board (CTP_NUTTX_PORT memo section 1), and
 *     the TLSC6X driver expects the line to idle high so the IC can pulse
 *     it low for touch events.  Without the internal pull-up the line
 *     floats and the IC refuses to answer on I2C.
 *   RST = PD18, configured OUTPUT, active low.
 *
 * The I2C SCK/SDA lines (PD20/PD21) have external 4.7K pull-ups on the
 * board (R239/R240); their pinmap entry already sets internal PULL_UP
 * as a harmless default.
 */

#define T113_CTP_INT_PIN \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 19, T113_GPIO_FUNC_EINT, \
                   T113_GPIO_PULL_UP, 0)
#define T113_CTP_RST_PIN \
  T113_GPIO_ENCODE(T113_GPIO_PORTD, 18, T113_GPIO_OUTPUT, \
                   T113_GPIO_PULL_NONE, 0)

/* TWI3: PB6/PB7 (group 1) */

#define T113_TWI3_SCK        T113_TWI3_SCK_1      /* PB6  */
#define T113_TWI3_SDA        T113_TWI3_SDA_1      /* PB7  */

/* CAN0: PB2/PB3 (group 1, only option on T113-S3 QFP128) */

#define T113_CAN0_TX         T113_CAN0_TX_1       /* PB2  */
#define T113_CAN0_RX         T113_CAN0_RX_1       /* PB3  */

/* CAN1: PB4/PB5 (group 1, only option) */

#define T113_CAN1_TX         T113_CAN1_TX_1       /* PB4  */
#define T113_CAN1_RX         T113_CAN1_RX_1       /* PB5  */

/* SDC0 (microSD): PF0-PF5, func 2 -- shares with JTAG/UART0 */

#define T113_SDC0_CLK        T113_SDC0_CLK_1      /* PF2  */
#define T113_SDC0_CMD        T113_SDC0_CMD_1      /* PF3  */
#define T113_SDC0_D0         T113_SDC0_D0_1       /* PF1  */
#define T113_SDC0_D1         T113_SDC0_D1_1       /* PF0  */
#define T113_SDC0_D2         T113_SDC0_D2_1       /* PF5  */
#define T113_SDC0_D3         T113_SDC0_D3_1       /* PF4  */

/* SDC0 card-detect: PF6 (alt-func PF_EINT6).  The board schematic
 * routes SDC0-DET to T113 BGA ball K2, which corresponds to GPIO PF6
 * (the next pin after PF5/SDC0-D2 in the PF-row pinout).  The socket
 * switch closes to GND when a card is inserted, so the pin is
 * configured as input with pull-up; level-low = card present.
 */

#define T113_SDC0_CD         T113_SDC0_CD_1       /* PF6  */

/* SDC1 (SDIO WiFi): PG0-PG5, func 2 -- RTL8723DS combo on R528 HMI EVB4.
 * No card-detect signal (the module is non-removable); WL-REG-ON power
 * sequencing is owned by the Phase-2 board layer.
 */

#define T113_SDC1_CLK        T113_SDC1_CLK_1      /* PG0  */
#define T113_SDC1_CMD        T113_SDC1_CMD_1      /* PG1  */
#define T113_SDC1_D0         T113_SDC1_D0_1       /* PG2  */
#define T113_SDC1_D1         T113_SDC1_D1_1       /* PG3  */
#define T113_SDC1_D2         T113_SDC1_D2_1       /* PG4  */
#define T113_SDC1_D3         T113_SDC1_D3_1       /* PG5  */

#endif /* __BOARDS_ARM_T113_T113_EVB_INCLUDE_BOARD_H */
