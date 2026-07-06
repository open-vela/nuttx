/****************************************************************************
 * arch/arm64/src/rk3576/rk3576_gpio.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_RK3576_GPIO_H
#define __ARCH_ARM64_SRC_RK3576_RK3576_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO pull mode definitions */

#define RK3576_GPIO_PULL_NONE      0
#define RK3576_GPIO_PULL_UP        1
#define RK3576_GPIO_PULL_DOWN      2

/* GPIO drive strength definitions */

#define RK3576_GPIO_DRIVE_2MA      0
#define RK3576_GPIO_DRIVE_4MA      1
#define RK3576_GPIO_DRIVE_8MA      2
#define RK3576_GPIO_DRIVE_12MA     3

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

/* GPIO initialization */

void rk3576_gpio_init(void);

/* Pin configuration */

void rk3576_gpio_set_function(unsigned int pin, unsigned int function);
void rk3576_gpio_set_pull(unsigned int pin, unsigned int pull);
void rk3576_gpio_set_drive(unsigned int pin, unsigned int drive);
void rk3576_gpio_set_debounce(unsigned int pin, bool enable);

/* Direction control */

void rk3576_gpio_direction_input(unsigned int pin);
void rk3576_gpio_direction_output(unsigned int pin, int value);

/* Data access */

int  rk3576_gpio_get_value(unsigned int pin);
void rk3576_gpio_set_value(unsigned int pin, int value);

/* Interrupt control */

int  rk3576_gpio_irq_attach(unsigned int pin, xcpt_t handler, void *arg);
void rk3576_gpio_irq_detach(unsigned int pin);
int  rk3576_gpio_irq_settype(unsigned int pin, unsigned int type);
int  rk3576_gpio_irq_enable(unsigned int pin);
int  rk3576_gpio_irq_disable(unsigned int pin);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM64_SRC_RK3576_RK3576_GPIO_H */
