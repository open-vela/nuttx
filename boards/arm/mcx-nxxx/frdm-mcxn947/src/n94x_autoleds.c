/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_autoleds.c
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

#include <nuttx/board.h>

#include <arch/board/board.h>

#include "nxxx_gpio.h"
#include "nxxx_port.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_autoled_initialize
 ****************************************************************************/

void board_autoled_initialize(void)
{
  nxxx_port_configure(PORT_LED_R);
  nxxx_port_configure(PORT_LED_G);
  nxxx_port_configure(PORT_LED_B);

  nxxx_config_gpio(GPIO_LED_R);
  nxxx_config_gpio(GPIO_LED_G);
  nxxx_config_gpio(GPIO_LED_B);
}

/****************************************************************************
 * Name: board_autoled_on
 ****************************************************************************/

void board_autoled_on(int led)
{
  if (led == LED_STACKCREATED)
    {
      nxxx_gpio_write(GPIO_LED_G, false);
    }
  else if (led == LED_PANIC)
    {
      nxxx_gpio_write(GPIO_LED_R, false);
    }
}

/****************************************************************************
 * Name: board_autoled_off
 ****************************************************************************/

void board_autoled_off(int led)
{
  if (led == LED_STACKCREATED)
    {
      nxxx_gpio_write(GPIO_LED_G, true);
    }
  else if (led == LED_PANIC)
    {
      nxxx_gpio_write(GPIO_LED_R, true);
    }
}

#endif /* CONFIG_ARCH_LEDS */
