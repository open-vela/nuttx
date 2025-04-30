/****************************************************************************
 * boards/arm/stm32/stm32f407-robomaster/src/stm32f407-robomaster.h
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

#ifndef __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H
#define __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <arch/chip/chip.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* LED.  User LEDR: the red LED is a user LED connected to board LED D12
 * corresponding to MCU I/O PH10.
 *       User LEDG: the green LED is a user LED connected to board LED D12
 * corresponding to MCU I/O PH11.
 *       User LEDB: the blue LED is a user LED connected to board LED D12
 * corresponding to MCU I/O PH13.
 *
 * - When the I/O is HIGH value, the LED is on.
 * - When the I/O is LOW, the LED is off.
 */

#define GPIO_LEDB \
(GPIO_PORTH | GPIO_PIN10 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PULLUP | \
 GPIO_SPEED_50MHz)

#define GPIO_LEDG \
(GPIO_PORTH | GPIO_PIN11 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PULLUP | \
 GPIO_SPEED_50MHz)

#define GPIO_LEDR \
(GPIO_PORTH | GPIO_PIN12 | GPIO_OUTPUT_SET | GPIO_OUTPUT | GPIO_PULLUP | \
 GPIO_SPEED_50MHz)

/* Buttons
 *
 * B1 USER: the user button is connected to the I/O PA0 of the STM32
 * microcontroller.
 */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  (BUTTON_USER + 1)

#define GPIO_BTN_USER \
  (GPIO_INPUT |GPIO_PULLUP |GPIO_EXTI | GPIO_PORTA | GPIO_PIN0)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global driver instances */

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

#endif /* __BOARDS_ARM_STM32_STM32F411_MINIMUM_SRC_STM32F411_MINIMUM_H */ 