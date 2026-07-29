/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_userleds.c
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

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nxxx_gpio.h"
#include "nxxx_port.h"
#include "frdm-mcxn947.h"

#ifndef CONFIG_ARCH_LEDS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The RGB LED GPIO pinsets, indexed by BOARD_LED_R/G/B */

static const gpio_pinset_t g_ledcfg[BOARD_NLEDS] =
{
  GPIO_LED_R,
  GPIO_LED_G,
  GPIO_LED_B
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_userled_initialize
 ****************************************************************************/

uint32_t board_userled_initialize(void)
{
  /* Mux the LED pins to GPIO (ALT0) */

  nxxx_port_configure(PORT_LED_R);
  nxxx_port_configure(PORT_LED_G);
  nxxx_port_configure(PORT_LED_B);

  /* Configure the LED GPIOs as outputs, initial state OFF */

  nxxx_config_gpio(GPIO_LED_R);
  nxxx_config_gpio(GPIO_LED_G);
  nxxx_config_gpio(GPIO_LED_B);

  return BOARD_NLEDS;
}

/****************************************************************************
 * Name: board_userled
 ****************************************************************************/

void board_userled(int led, bool ledon)
{
  if ((unsigned int)led < BOARD_NLEDS)
    {
      /* The LEDs are active low: drive the pin low to turn the LED on. */

      nxxx_gpio_write(g_ledcfg[led], !ledon);
    }
}

/****************************************************************************
 * Name: board_userled_all
 ****************************************************************************/

void board_userled_all(uint32_t ledset)
{
  int i;

  for (i = 0; i < BOARD_NLEDS; i++)
    {
      board_userled(i, (ledset & (1 << i)) != 0);
    }
}

#endif /* !CONFIG_ARCH_LEDS */
