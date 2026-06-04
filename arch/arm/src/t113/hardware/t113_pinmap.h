/****************************************************************************
 * arch/arm/src/t113/hardware/t113_pinmap.h
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

/* T113-S3 peripheral pin alternate-function mappings.
 *
 * Each signal has numbered variants (_1, _2, ...) for every legal
 * GPIO routing.  boards/<board>/include/board.h selects one variant
 * per signal by aliasing the unsuffixed name, e.g.:
 *
 *   #define T113_UART0_TX  T113_UART0_TX_1
 *
 * Drivers reference only the unsuffixed names - they never know
 * which physical pin is used.
 *
 * Pull and drive defaults:
 *   UART  - pull-up (User Manual section 9.7 requirement)
 *   TWI   - pull-up (open-drain bus)
 *   SPI   - no pull (avoid quad-read IO0 corruption at 100 MHz)
 *   CAN   - no pull
 *
 * Source: T113-S3 Datasheet v1.6, Table 4-3 (Pin Multiplexing).
 */

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_PINMAP_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_PINMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "t113_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Helper - shorter than writing T113_GPIO_ENCODE every time */

/* _T113_PIN - kept defined (no #undef) because the pinset macros
 * below use lazy expansion; the helper must be available at every
 * point of use, not just at definition time.
 */

#define _T113_PIN(port, pin, func, pull) \
  T113_GPIO_ENCODE(T113_GPIO_PORT##port, pin, \
                   T113_GPIO_FUNC##func, T113_GPIO_PULL_##pull, 0)

/* SPI0 - PC2-PC7, func 2 (single pin group) */

#define T113_SPI0_CLK_1       _T113_PIN(C,  2, 2, NONE)  /* PC2 */
#define T113_SPI0_CS_1        _T113_PIN(C,  3, 2, NONE)  /* PC3 */
#define T113_SPI0_MOSI_1      _T113_PIN(C,  4, 2, NONE)  /* PC4 */
#define T113_SPI0_MISO_1      _T113_PIN(C,  5, 2, NONE)  /* PC5 */
#define T113_SPI0_WP_1        _T113_PIN(C,  6, 2, NONE)  /* PC6 */
#define T113_SPI0_HOLD_1      _T113_PIN(C,  7, 2, NONE)  /* PC7 */

/* SPI1 - PD10-PD15, func 4 (single pin group) */

#define T113_SPI1_CS_1        _T113_PIN(D, 10, 4, NONE)  /* PD10 */
#define T113_SPI1_CLK_1       _T113_PIN(D, 11, 4, NONE)  /* PD11 */
#define T113_SPI1_MOSI_1      _T113_PIN(D, 12, 4, NONE)  /* PD12 */
#define T113_SPI1_MISO_1      _T113_PIN(D, 13, 4, NONE)  /* PD13 */
#define T113_SPI1_HOLD_1      _T113_PIN(D, 14, 4, NONE)  /* PD14 */
#define T113_SPI1_WP_1        _T113_PIN(D, 15, 4, NONE)  /* PD15 */

/* UART0 - 2 pin groups */

#define T113_UART0_TX_1       _T113_PIN(F,  2, 3, UP)    /* PF2  */
#define T113_UART0_RX_1       _T113_PIN(F,  4, 3, UP)    /* PF4  */

#define T113_UART0_TX_2       _T113_PIN(E,  2, 6, UP)    /* PE2  */
#define T113_UART0_RX_2       _T113_PIN(E,  3, 6, UP)    /* PE3  */

/* UART1 - 3 pin groups (4-wire: TX/RX/RTS/CTS) */

#define T113_UART1_TX_1       _T113_PIN(E, 10, 3, UP)    /* PE10 */
#define T113_UART1_RX_1       _T113_PIN(E, 11, 3, UP)    /* PE11 */
#define T113_UART1_RTS_1      _T113_PIN(E,  8, 3, UP)    /* PE8  */
#define T113_UART1_CTS_1      _T113_PIN(E,  9, 3, UP)    /* PE9  */

#define T113_UART1_TX_2       _T113_PIN(G,  6, 2, UP)    /* PG6  */
#define T113_UART1_RX_2       _T113_PIN(G,  7, 2, UP)    /* PG7  */
#define T113_UART1_RTS_2      _T113_PIN(G,  8, 2, UP)    /* PG8  */
#define T113_UART1_CTS_2      _T113_PIN(G,  9, 2, UP)    /* PG9  */

#define T113_UART1_TX_3       _T113_PIN(G, 12, 7, UP)    /* PG12 */
#define T113_UART1_RX_3       _T113_PIN(G, 13, 7, UP)    /* PG13 */
#define T113_UART1_RTS_3      _T113_PIN(G, 14, 7, UP)    /* PG14 */
#define T113_UART1_CTS_3      _T113_PIN(G, 15, 7, UP)    /* PG15 */

/* UART2 - 2 pin groups (4-wire) */

#define T113_UART2_TX_1       _T113_PIN(D,  1, 5, UP)    /* PD1  */
#define T113_UART2_RX_1       _T113_PIN(D,  2, 5, UP)    /* PD2  */
#define T113_UART2_RTS_1      _T113_PIN(D,  3, 5, UP)    /* PD3  */
#define T113_UART2_CTS_1      _T113_PIN(D,  4, 5, UP)    /* PD4  */

#define T113_UART2_TX_2       _T113_PIN(E,  2, 3, UP)    /* PE2  */
#define T113_UART2_RX_2       _T113_PIN(E,  3, 3, UP)    /* PE3  */
#define T113_UART2_RTS_2      _T113_PIN(E,  0, 3, UP)    /* PE0  */
#define T113_UART2_CTS_2      _T113_PIN(E,  1, 3, UP)    /* PE1  */

/* UART3 - 4 pin groups (4-wire, RTS/CTS only on some groups) */

#define T113_UART3_TX_1       _T113_PIN(B,  6, 7, UP)    /* PB6  */
#define T113_UART3_RX_1       _T113_PIN(B,  7, 7, UP)    /* PB7  */

#define T113_UART3_TX_2       _T113_PIN(C,  6, 4, UP)    /* PC6  */
#define T113_UART3_RX_2       _T113_PIN(C,  7, 4, UP)    /* PC7  */

#define T113_UART3_TX_3       _T113_PIN(D, 10, 5, UP)    /* PD10 */
#define T113_UART3_RX_3       _T113_PIN(D, 11, 5, UP)    /* PD11 */
#define T113_UART3_RTS_3      _T113_PIN(D, 13, 5, UP)    /* PD13 */
#define T113_UART3_CTS_3      _T113_PIN(D, 14, 5, UP)    /* PD14 */

#define T113_UART3_TX_4       _T113_PIN(G,  0, 3, UP)    /* PG0  */
#define T113_UART3_RX_4       _T113_PIN(G,  1, 3, UP)    /* PG1  */
#define T113_UART3_RTS_4      _T113_PIN(G,  2, 3, UP)    /* PG2  */
#define T113_UART3_CTS_4      _T113_PIN(G,  3, 3, UP)    /* PG3  */

/* UART4 - 4 pin groups (2-wire) */

#define T113_UART4_TX_1       _T113_PIN(B,  2, 7, UP)    /* PB2  */
#define T113_UART4_RX_1       _T113_PIN(B,  3, 7, UP)    /* PB3  */

#define T113_UART4_TX_2       _T113_PIN(D,  7, 5, UP)    /* PD7  */
#define T113_UART4_RX_2       _T113_PIN(D,  8, 5, UP)    /* PD8  */

#define T113_UART4_TX_3       _T113_PIN(E,  4, 3, UP)    /* PE4  */
#define T113_UART4_RX_3       _T113_PIN(E,  5, 3, UP)    /* PE5  */

#define T113_UART4_TX_4       _T113_PIN(G,  2, 5, UP)    /* PG2  */
#define T113_UART4_RX_4       _T113_PIN(G,  3, 5, UP)    /* PG3  */

/* UART5 - 4 pin groups (2-wire) */

#define T113_UART5_TX_1       _T113_PIN(B,  4, 7, UP)    /* PB4  */
#define T113_UART5_RX_1       _T113_PIN(B,  5, 7, UP)    /* PB5  */

#define T113_UART5_TX_2       _T113_PIN(D,  5, 5, UP)    /* PD5  */
#define T113_UART5_RX_2       _T113_PIN(D,  6, 5, UP)    /* PD6  */

#define T113_UART5_TX_3       _T113_PIN(E,  6, 3, UP)    /* PE6  */
#define T113_UART5_RX_3       _T113_PIN(E,  7, 3, UP)    /* PE7  */

#define T113_UART5_TX_4       _T113_PIN(G,  4, 3, UP)    /* PG4  */
#define T113_UART5_RX_4       _T113_PIN(G,  5, 3, UP)    /* PG5  */

/* TWI0 - 5 pin groups */

#define T113_TWI0_SCK_1       _T113_PIN(B,  3, 4, UP)    /* PB3  */
#define T113_TWI0_SDA_1       _T113_PIN(B,  2, 4, UP)    /* PB2  */

#define T113_TWI0_SCK_2       _T113_PIN(D,  0, 5, UP)    /* PD0  */
#define T113_TWI0_SDA_2       _T113_PIN(D, 12, 5, UP)    /* PD12 */

#define T113_TWI0_SCK_3       _T113_PIN(E,  2, 4, UP)    /* PE2  */
#define T113_TWI0_SDA_3       _T113_PIN(E,  3, 4, UP)    /* PE3  */

#define T113_TWI0_SCK_4       _T113_PIN(F,  2, 4, UP)    /* PF2  */
#define T113_TWI0_SDA_4       _T113_PIN(F,  4, 4, UP)    /* PF4  */

#define T113_TWI0_SCK_5       _T113_PIN(G, 12, 3, UP)    /* PG12 */
#define T113_TWI0_SDA_5       _T113_PIN(G, 13, 3, UP)    /* PG13 */

/* TWI1 - 3 pin groups */

#define T113_TWI1_SCK_1       _T113_PIN(B,  4, 4, UP)    /* PB4  */
#define T113_TWI1_SDA_1       _T113_PIN(B,  5, 4, UP)    /* PB5  */

#define T113_TWI1_SCK_2       _T113_PIN(E,  0, 4, UP)    /* PE0  */
#define T113_TWI1_SDA_2       _T113_PIN(E,  1, 4, UP)    /* PE1  */

#define T113_TWI1_SCK_3       _T113_PIN(G,  8, 3, UP)    /* PG8  */
#define T113_TWI1_SDA_3       _T113_PIN(G,  9, 3, UP)    /* PG9  */

/* TWI2 - 5 pin groups */

#define T113_TWI2_SCK_1       _T113_PIN(D, 20, 3, UP)    /* PD20 */
#define T113_TWI2_SDA_1       _T113_PIN(D, 21, 3, UP)    /* PD21 */

#define T113_TWI2_SCK_2       _T113_PIN(E,  4, 4, UP)    /* PE4  */
#define T113_TWI2_SDA_2       _T113_PIN(E,  5, 4, UP)    /* PE5  */

#define T113_TWI2_SCK_3       _T113_PIN(E, 12, 2, UP)    /* PE12 */
#define T113_TWI2_SDA_3       _T113_PIN(E, 13, 2, UP)    /* PE13 */

#define T113_TWI2_SCK_4       _T113_PIN(G,  6, 3, UP)    /* PG6  */
#define T113_TWI2_SDA_4       _T113_PIN(G,  7, 3, UP)    /* PG7  */

#define T113_TWI2_SCK_5       _T113_PIN(G, 14, 3, UP)    /* PG14 */
#define T113_TWI2_SDA_5       _T113_PIN(G, 15, 3, UP)    /* PG15 */

/* TWI3 - 4 pin groups */

#define T113_TWI3_SCK_1       _T113_PIN(B,  6, 4, UP)    /* PB6  */
#define T113_TWI3_SDA_1       _T113_PIN(B,  7, 4, UP)    /* PB7  */

#define T113_TWI3_SCK_2       _T113_PIN(C,  6, 5, UP)    /* PC6  */
#define T113_TWI3_SDA_2       _T113_PIN(C,  7, 5, UP)    /* PC7  */

#define T113_TWI3_SCK_3       _T113_PIN(E,  6, 4, UP)    /* PE6  */
#define T113_TWI3_SDA_3       _T113_PIN(E,  7, 4, UP)    /* PE7  */

#define T113_TWI3_SCK_4       _T113_PIN(G, 10, 3, UP)    /* PG10 */
#define T113_TWI3_SDA_4       _T113_PIN(G, 11, 3, UP)    /* PG11 */

/* DMIC - PD19/PD20, func 4 (PDM digital microphone, 2 lanes possible
 * but only DATA0+CLK exposed on R528 HMI EVB4).  Datasheet v1.6 Table 4-3.
 * No pull on data line; CLK is host-driven output.
 */

#define T113_DMIC_DATA0_1     _T113_PIN(D, 19, 4, NONE)  /* PD19 */
#define T113_DMIC_CLK_1       _T113_PIN(D, 20, 4, NONE)  /* PD20 */

/* CAN0 - PB2/PB3, func 8 (T113-S3 QFP128: PB0/PB1 not bonded) */

#define T113_CAN0_TX_1        _T113_PIN(B,  2, 8, NONE)  /* PB2  */
#define T113_CAN0_RX_1        _T113_PIN(B,  3, 8, NONE)  /* PB3  */

/* CAN1 - PB4/PB5, func 8 */

#define T113_CAN1_TX_1        _T113_PIN(B,  4, 8, NONE)  /* PB4  */
#define T113_CAN1_RX_1        _T113_PIN(B,  5, 8, NONE)  /* PB5  */

/* MIPI DSI host (X4B 2-lane uses D0/D1/CK; D2/D3 reserved) */

#define T113_DSI_D0P_1        _T113_PIN(D,  0, 4, NONE)  /* PD0  */
#define T113_DSI_D0N_1        _T113_PIN(D,  1, 4, NONE)  /* PD1  */
#define T113_DSI_D1P_1        _T113_PIN(D,  2, 4, NONE)  /* PD2  */
#define T113_DSI_D1N_1        _T113_PIN(D,  3, 4, NONE)  /* PD3  */
#define T113_DSI_CKP_1        _T113_PIN(D,  4, 4, NONE)  /* PD4  */
#define T113_DSI_CKN_1        _T113_PIN(D,  5, 4, NONE)  /* PD5  */

/* PWM0 backlight on PD16 */

#define T113_PWM0_1           _T113_PIN(D, 16, 5, NONE)  /* PD16 */

/* SMHC0 (microSD) -- PF0-PF5, func 2.
 * Shares the PF pin group with JTAG (func 3) and UART0 (func 4);
 * enabling SDC0 is mutually exclusive with JTAG probe access.
 * Drive level 3 (highest slew) is required for reliable operation
 * at the default 25 MHz SDCLK.
 */

#define T113_SDC0_CLK_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTF, 2, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3) /* PF2 */
#define T113_SDC0_CMD_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTF, 3, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PF3 */
#define T113_SDC0_D0_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTF, 1, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PF1 */
#define T113_SDC0_D1_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTF, 0, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PF0 */
#define T113_SDC0_D2_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTF, 5, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PF5 */
#define T113_SDC0_D3_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTF, 4, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PF4 */

/* SMHC0 card-detect -- PF6 input pull-up.  T113 BGA ball K2 carries the
 * SDC0-DET signal from the microSD socket switch; the switch closes to
 * GND on insertion (active-low).  PF6 also has alt-func PF_EINT6 which
 * a future port can latch onto for IRQ-driven hotplug.
 */

#define T113_SDC0_CD_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTF, 6, T113_GPIO_INPUT, \
                   T113_GPIO_PULL_UP,  T113_GPIO_DRV_LEVEL0) /* PF6 */

/* SMHC1 (SDIO WiFi) -- PG0-PG5, func 2.
 * Reserved for Phase 2 (RTL8723DS WiFi SDIO on R528 HMI EVB4).
 */

#define T113_SDC1_CLK_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTG, 0, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_NONE, T113_GPIO_DRV_LEVEL3) /* PG0 */
#define T113_SDC1_CMD_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTG, 1, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PG1 */
#define T113_SDC1_D0_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTG, 2, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PG2 */
#define T113_SDC1_D1_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTG, 3, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PG3 */
#define T113_SDC1_D2_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTG, 4, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PG4 */
#define T113_SDC1_D3_1 \
  T113_GPIO_ENCODE(T113_GPIO_PORTG, 5, T113_GPIO_FUNC2, \
                   T113_GPIO_PULL_UP,   T113_GPIO_DRV_LEVEL3) /* PG5 */

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_PINMAP_H */
