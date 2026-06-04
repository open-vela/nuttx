/****************************************************************************
 * arch/arm/src/t113/t113_gpio.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_GPIO_H
#define __ARCH_ARM_SRC_T113_T113_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

#include "hardware/t113_gpio.h"

void t113_gpio_init(void);
int  t113_gpio_config(uint16_t pinset);
void t113_gpio_write(uint16_t pinset, bool value);
bool t113_gpio_read(uint16_t pinset);

#ifdef CONFIG_T113_GPIO_IRQ

#include <nuttx/irq.h>

/* GPIO interrupt framework - per-pin ISR dispatch on the PD port.
 *
 * t113_gpio_irq_initialize() - Called once at boot (by up_irqinitialize
 *   or board bringup) to attach the PD port combined IRQ and clear all
 *   masks.  Safe to call multiple times.
 *
 * t113_gpio_irq_attach() - Register a per-pin ISR.  `pinset` must select
 *   a PD pin; `trigger` is one of T113_EINT_MODE_* from
 *   hardware/t113_gpio.h.  The pin's GPIO function must already be
 *   configured INPUT via t113_gpio_config().  Returns 0 on success,
 *   -EINVAL for a non-PD pin or invalid trigger, -EALREADY if the slot
 *   is occupied.
 *
 * t113_gpio_irq_detach() - Remove a previously attached ISR.  Silently
 *   no-ops if the slot is empty.
 *
 * t113_gpio_irq_enable() - Unmask (true) or mask (false) the pin in the
 *   PD_EINT_CTL register.  Pending status (if any) is cleared on unmask.
 */

void t113_gpio_irq_initialize(void);
int  t113_gpio_irq_attach(uint16_t pinset, uint8_t trigger,
                          xcpt_t isr, void *arg);
void t113_gpio_irq_detach(uint16_t pinset);
void t113_gpio_irq_enable(uint16_t pinset, bool enable);

#endif /* CONFIG_T113_GPIO_IRQ */

#endif /* __ARCH_ARM_SRC_T113_T113_GPIO_H */
