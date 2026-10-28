/****************************************************************************
 * boards/arm/stm32h7/stm32h750b-dk/src/stm32_buttons.c
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

#include <stddef.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/board.h>

#include "stm32_gpio.h"
#include "stm32h750b-dk.h"
#include <arch/board/board.h>

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if NUM_BUTTONS != 1
#  error "This board declares exactly one button; update the switch below."
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   The pin is configured as an input with a pull-down, without EXTI here.
 *   stm32_gpiosetevent() adds GPIO_EXTI when it installs an interrupt, so a
 *   caller that only samples the button never arms an edge detector.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  uint32_t pinset = GPIO_BTN_USER & ~GPIO_EXTI;

  stm32_configgpio(pinset);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 *
 * Description:
 *   Return a bit set with one bit per button, set when depressed.
 *
 ****************************************************************************/

uint32_t board_buttons(void)
{
  return stm32_gpioread(GPIO_BTN_USER) ? BUTTON_USER_BIT : 0;
}

/****************************************************************************
 * Name: board_button_irq
 *
 * Description:
 *   Attach or detach the handler called on button edges.  Passing a NULL
 *   handler detaches and disables the EXTI line.
 *
 *   Both edges are requested: the upper half derives press and release from
 *   the sampled state, so a release-only or press-only edge would leave its
 *   sample stale.  The arguments follow the other STM32H7 boards, which pass
 *   the event flag set together with the interrupt.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id == BUTTON_USER)
    {
      ret = stm32_gpiosetevent(GPIO_BTN_USER, true, true, true,
                               irqhandler, arg);
    }

  return ret;
}
#endif /* CONFIG_ARCH_IRQBUTTONS */
#endif /* CONFIG_ARCH_BUTTONS */
