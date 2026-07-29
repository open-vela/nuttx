/****************************************************************************
 * boards/arm/mcx-nxxx/frdm-mcxn947/src/n94x_buttons.c
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
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nxxx_gpio.h"
#include "nxxx_port.h"
#include "frdm-mcxn947.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The user button GPIO pinsets, indexed by BUTTON_SW2/SW3 */

static const gpio_pinset_t g_buttoncfg[NUM_BUTTONS] =
{
  GPIO_SW2,
  GPIO_SW3
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  /* Mux the button pins to GPIO (ALT0) with input buffer and pull-up */

  nxxx_port_configure(PORT_SW2);
  nxxx_port_configure(PORT_SW3);

  /* Configure the pins as interrupt-capable inputs (interrupts are not
   * enabled until board_button_irq() is called).
   */

  nxxx_config_gpio(GPIO_SW2);
  nxxx_config_gpio(GPIO_SW3);

  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  uint32_t ret = 0;

  /* The buttons are active low: a pressed button reads as 0. */

  if (!nxxx_gpio_read(GPIO_SW2))
    {
      ret |= BUTTON_SW2_BIT;
    }

  if (!nxxx_gpio_read(GPIO_SW3))
    {
      ret |= BUTTON_SW3_BIT;
    }

  return ret;
}

/****************************************************************************
 * Name: board_button_irq
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  gpio_pinset_t pinset;

  if ((unsigned int)id >= NUM_BUTTONS)
    {
      return -EINVAL;
    }

  pinset = g_buttoncfg[id];

  if (irqhandler != NULL)
    {
      /* Attach the handler and enable the both-edge interrupt */

      nxxx_gpioirq_attach(pinset, irqhandler, arg);
      nxxx_gpioirq_enable(pinset);
    }
  else
    {
      /* Disable the interrupt and detach the handler */

      nxxx_gpioirq_disable(pinset);
      nxxx_gpioirq_attach(pinset, NULL, NULL);
    }

  return OK;
}
#endif /* CONFIG_ARCH_IRQBUTTONS */

#endif /* CONFIG_ARCH_BUTTONS */
