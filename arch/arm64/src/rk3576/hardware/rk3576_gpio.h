/****************************************************************************
 * arch/arm64/src/rk3576/hardware/rk3576_gpio.h
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

#ifndef __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_GPIO_H
#define __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "rk3576_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GPIO bank count */

#define RK3576_GPIO_BANKS          5   /* GPIO0 ~ GPIO4 */
#define RK3576_PINS_PER_BANK       32  /* 4 ports x 8 pins */
#define RK3576_PORT_PINS           8   /* 8 pins per port (A/B/C/D) */
#define RK3576_PORTS_PER_BANK      4   /* 4 ports per bank (A/B/C/D) */

/* GPIO register offsets (DesignWare GPIO IP core) */

#define GPIO_SWPORTA_DR            0x0000  /* Port A data register (R/W) */
#define GPIO_SWPORTA_DDR           0x0004  /* Port A data direction (R/W) */
#define GPIO_EXT_PORTA             0x0008  /* External port A (R only) */
#define GPIO_EXT_PORTB             0x000c  /* External port B (R only) */
#define GPIO_EXT_PORTC             0x0010  /* External port C (R only) */
#define GPIO_EXT_PORTD             0x0014  /* External port D (R only) */
#define GPIO_INT_EN                0x0050  /* Interrupt enable (R/W) */
#define GPIO_INT_MASK              0x0054  /* Interrupt mask (R/W) */
#define GPIO_INT_TYPE              0x0058  /* Interrupt type: 0=level, 1=edge */
#define GPIO_INT_POLARITY          0x005c  /* Interrupt polarity: 0=low/falling, 1=high/rising */
#define GPIO_INT_STATUS            0x0060  /* Interrupt status (R/W1C) */
#define GPIO_PORT_EOI              0x0064  /* Port interrupt clear (R/W1C) */
#define GPIO_DEBOUNCE              0x0070  /* Debounce enable (R/W) */

/* Bit definitions for GPIO_INT_STATUS / GPIO_PORT_EOI */

#define GPIO_INT_BOTH              (1 << 0)  /* Global interrupt status */

/* GPIO register access helpers */

#define GPIO_REG(base, offset)    (*(volatile uint32_t *)((base) + (offset)))

/* Read/write helpers */

#define GPIO_READ(reg)            readl(reg)
#define GPIO_WRITE(reg, val)      writel(val, reg)

/* Pin number encoding: (bank * 32) + (port * 8) + offset */

#define RK3576_GPIO_PIN(bank, port, offset) \
    ((bank) * RK3576_PINS_PER_BANK + (port) * RK3576_PORT_PINS + (offset))

/* Extract bank, port, offset from pin number */

#define RK3576_PIN_BANK(pin)      ((pin) / RK3576_PINS_PER_BANK)
#define RK3576_PIN_PORT(pin)      (((pin) % RK3576_PINS_PER_BANK) / RK3576_PORT_PINS)
#define RK3576_PIN_OFFSET(pin)    ((pin) % RK3576_PORT_PINS)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Interrupt trigger types */

enum rk3576_gpio_irqtype_e
{
  GPIO_IRQTYPE_NONE = 0,       /* No interrupt */
  GPIO_IRQTYPE_RISING,         /* Rising edge */
  GPIO_IRQTYPE_FALLING,        /* Falling edge */
  GPIO_IRQTYPE_BOTHEDGE,       /* Both edges */
  GPIO_IRQTYPE_HIGHLEVEL,      /* High level */
  GPIO_IRQTYPE_LOWLEVEL,       /* Low level */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* GPIO bank base addresses */

extern const uint32_t g_gpio_base[RK3576_GPIO_BANKS];

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM64_SRC_RK3576_HARDWARE_RK3576_GPIO_H */
