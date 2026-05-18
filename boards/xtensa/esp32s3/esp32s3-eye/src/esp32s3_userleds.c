/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-eye/src/esp32s3_userleds.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "esp32s3_gpio.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#include "esp32s3-eye.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The ESP32-S3-EYE has one user-controllable green Module Power LED on
 * GPIO3.  IMPORTANT: GPIO3 must be configured as OPEN-DRAIN.  Pulling
 * GPIO3 high through a regular push-pull driver may exceed the LED forward
 * current and burn the LED (per ESP32-S3-EYE Hardware Guide section 2.3.2).
 *
 * Logic:
 *   - LED is sourced by 3.3 V through a current-limiting resistor.
 *   - Cathode is connected to GPIO3.
 *   - Drive GPIO3 LOW   -> LED ON  (current sinks into MCU)
 *   - Tri-state GPIO3   -> LED OFF (no current path)
 */

#define LED_GPIO            3
#define BOARD_NLEDS         1
#define BOARD_LED_BIT       (1 << 0)

/* LED on means we drive the pin LOW (active-low) */

#define LED_ON_LEVEL        false
#define LED_OFF_LEVEL       true

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 *
 * Description:
 *   Configure the GPIO3 pin as an open-drain output and turn the LED off.
 *
 * Returned Value:
 *   The number of LEDs supported by the board.
 *
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Configure GPIO3 as open-drain output, default high (LED off).
   * OUTPUT_OPEN_DRAIN is critical -- a regular push-pull OUTPUT mode would
   * pull GPIO3 to 3.3 V without a current-limiting path and may damage the
   * green Module Power LED.
   */

  esp32s3_configgpio(LED_GPIO, OUTPUT_OPEN_DRAIN);
  esp32s3_gpiowrite(LED_GPIO, LED_OFF_LEVEL);

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 *
 * Description:
 *   Set the state of a single LED.  led must be 0..BOARD_NLEDS-1.
 *
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if (led == 0)
    {
      /* Active-low: LED on when pin driven LOW, off when tri-stated. */

      esp32s3_gpiowrite(LED_GPIO, ledon ? LED_ON_LEVEL : LED_OFF_LEVEL);
    }
}

/****************************************************************************
 * Name: board_userled_all
 *
 * Description:
 *   Set the state of all LEDs from a bit-set.  Bit n corresponds to LED n.
 *
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  bool ledon = (ledset & BOARD_LED_BIT) != 0;

  esp32s3_gpiowrite(LED_GPIO, ledon ? LED_ON_LEVEL : LED_OFF_LEVEL);
}

#endif /* !CONFIG_ARCH_LEDS */
