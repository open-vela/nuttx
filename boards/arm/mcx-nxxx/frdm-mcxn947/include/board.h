/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/include/board.h
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

#ifndef __BOARDS_ARM_MCX_NXXX_FRDM_MCXN947_INCLUDE_BOARD_H
#define __BOARDS_ARM_MCX_NXXX_FRDM_MCXN947_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PORT_LPUART4_RX PORT_CFG(1, 8, PORT_PCR_MUX_ALT2 | PORT_PCR_IBE)
#define PORT_LPUART4_TX PORT_CFG(1, 9, PORT_PCR_MUX_ALT2 | PORT_PCR_IBE)

/* ENS160 + AHT21 air-quality module on LPI2C0.
 *
 *   J8.9  = SDA = P0_12 / FC0_P0 (ALT3)
 *   J8.12 = SCL = P0_13 / FC0_P1 (ALT3)
 *
 * The module supplies the required I2C pull-ups.  Its ADD pin is tied low
 * for ENS160 address 0x52; AHT21 remains at 0x38.  INT is not connected.
 */

#define PORT_LPI2C0_SCL \
  PORT_CFG(0, 13, PORT_PCR_MUX_ALT3 | PORT_PCR_IBE | PORT_PCR_ODE)
#define PORT_LPI2C0_SDA \
  PORT_CFG(0, 12, PORT_PCR_MUX_ALT3 | PORT_PCR_IBE | PORT_PCR_ODE)

/* ENET0 pins: all on PORT1, ALT9, input buffer enabled *********************/

/* SDK reference: mcuxsdk/examples/_boards/frdmmcxn947/driver_examples/enet/
 *                txrx_transfer_rxinterrupt/pin_mux.c
 */

#define PORT_ENET_TX_CLK  PORT_CFG(1,  4, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_TXEN    PORT_CFG(1,  5, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_TXD0    PORT_CFG(1,  6, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_TXD1    PORT_CFG(1,  7, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_RXDV    PORT_CFG(1, 13, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_RXD0    PORT_CFG(1, 14, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_RXD1    PORT_CFG(1, 15, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_MDC     PORT_CFG(1, 20, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)
#define PORT_ENET_MDIO    PORT_CFG(1, 21, PORT_PCR_MUX_ALT9 | PORT_PCR_IBE)

/* On-board Microchip LAN8741 PHY at MDIO address 0.  Register 0x1f uses
 * the LAN8720/LAN874x SCSR speed and duplex bit layout.
 */

#define BOARD_PHY_NAME        "LAN8741"
#define BOARD_PHYID1          0x0007
#define BOARD_PHYID2          0xc120
#define BOARD_PHY_STATUS      MII_LAN8720_SCSR
#define BOARD_PHY_ADDR        0
#define BOARD_PHY_10BASET(s)  (((s) & MII_LAN8720_SPSCR_10MBPS) != 0)
#define BOARD_PHY_100BASET(s) (((s) & MII_LAN8720_SPSCR_100MBPS) != 0)
#define BOARD_PHY_ISDUPLEX(s) (((s) & MII_LAN8720_SPSCR_DUPLEX) != 0)

/* LED definitions **********************************************************/

/* The FRDM-MCXN947 has a single RGB LED.  Each segment is active LOW
 * (the GPIO is driven low to turn the segment ON):
 *
 *   RED   - GPIO0 / PORT0_10
 *   GREEN - GPIO0 / PORT0_27
 *   BLUE  - GPIO1 / PORT1_2
 *
 * CONFIG_ARCH_LEDS is not selected, so the LEDs are available to the user
 * through the user LED driver registered at /dev/userleds.
 *
 * The PORT_LED_x macros mux the pin to GPIO (ALT0).  The GPIO_LED_x macros
 * configure the pin as an output, initially high so the LED is OFF at boot.
 */

#define PORT_LED_R PORT_CFG(0, 10, PORT_PCR_MUX_GPIO)
#define PORT_LED_G PORT_CFG(0, 27, PORT_PCR_MUX_GPIO)
#define PORT_LED_B PORT_CFG(1,  2, PORT_PCR_MUX_GPIO)

#define GPIO_LED_R (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT0 | GPIO_PIN10)
#define GPIO_LED_G (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT0 | GPIO_PIN27)
#define GPIO_LED_B (GPIO_OUTPUT | GPIO_OUTPUT_ONE | GPIO_PORT1 | GPIO_PIN2)

/* LED index values for use with board_userled() */

#define BOARD_LED_R     0
#define BOARD_LED_G     1
#define BOARD_LED_B     2
#define BOARD_NLEDS     3

/* LED bit values for use with board_userled_all() */

#define BOARD_LED_R_BIT (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT (1 << BOARD_LED_B)

/* Auto-LED states **********************************************************
 *
 * The green segment indicates that NuttX reached the idle loop.  The red
 * segment is toggled for a panic.  Transient IRQ/signal states deliberately
 * leave the LEDs unchanged.
 */

#define LED_STARTED       0 /* NuttX has been started: all off */
#define LED_HEAPALLOCATE  0 /* Heap has been allocated: all off */
#define LED_IRQSENABLED   0 /* Interrupts enabled: all off */
#define LED_STACKCREATED  1 /* Idle stack created: green on */
#define LED_INIRQ         2 /* In an interrupt: no change */
#define LED_SIGNAL        2 /* In a signal handler: no change */
#define LED_ASSERTION     2 /* An assertion failed: no change */
#define LED_PANIC         3 /* The system has crashed: red flashes */
#define LED_IDLE          0 /* MCU is idle: no change */

/* Button definitions *******************************************************/

/* The FRDM-MCXN947 has two user push buttons.  Both are active LOW (the
 * GPIO reads low while the button is pressed) and have an external pull-up;
 * the internal pull-up is also enabled for robustness:
 *
 *   SW2 - GPIO0 / PORT0_23
 *   SW3 - GPIO0 / PORT0_6
 *
 * The GPIO_SWx pinsets request an interrupt on both edges so that both press
 * and release events are reported.  board_buttons() reads the same pins as
 * inputs (the GPIO read path uses PDIR for non-output pins).
 */

#define PORT_SW2 PORT_CFG(0, 23, PORT_PCR_MUX_GPIO | PORT_PCR_IBE | \
                                 PORT_PCR_PE | PORT_PCR_PULLUP)
#define PORT_SW3 PORT_CFG(0,  6, PORT_PCR_MUX_GPIO | PORT_PCR_IBE | \
                                 PORT_PCR_PE | PORT_PCR_PULLUP)

#define GPIO_SW2 (GPIO_INTERRUPT | GPIO_INTBOTH_EDGES | GPIO_PORT0 | GPIO_PIN23)
#define GPIO_SW3 (GPIO_INTERRUPT | GPIO_INTBOTH_EDGES | GPIO_PORT0 | GPIO_PIN6)

/* Button index values for use with board_buttons() */

#define BUTTON_SW2      0
#define BUTTON_SW3      1
#define NUM_BUTTONS     2

/* Button bit values returned by board_buttons() */

#define BUTTON_SW2_BIT  (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT  (1 << BUTTON_SW3)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxxx_boardinitialize
 *
 * Description:
 *   All architectures must provide the following entry point.  This
 *   entry point is called in the initialization phase -- after
 *   imx_memory_initialize and after all memory has been configured and
 *   mapped but before any devices have been initialized.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxxx_boardinitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_MCX_NXXX_FRDM_MCXN947_INCLUDE_BOARD_H */
