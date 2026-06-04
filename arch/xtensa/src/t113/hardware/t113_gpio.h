/****************************************************************************
 * arch/xtensa/src/t113/hardware/t113_gpio.h
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

#ifndef __ARCH_XTENSA_SRC_T113_HARDWARE_T113_GPIO_H
#define __ARCH_XTENSA_SRC_T113_HARDWARE_T113_GPIO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DSP-side GPIO pinmux.
 *
 * STUB: the AP muxes the UART pins (and every other pin the DSP uses)
 * before the DSP firmware runs, so t113_gpio_config() is a no-op for now
 * and the per-UART pin descriptors are placeholders.
 *
 * When the DSP owns a peripheral whose pins it must mux itself, this grows
 * into a real PIO driver.  The PIO registers are shared with the AP; the
 * SoC hardware spinlock at 0x03005000 is reachable from the DSP, so cross-
 * core pinmux changes should be guarded by hwspin lock id 1 (the AP's
 * T113_HWLOCK_ID_GPIO).  GPIO ports B..G additionally have DSP R_INTC
 * interrupt sources (see arch/xtensa/include/t113/irq.h) for EINT.
 */

/* Placeholder pin descriptors -- no pinmux performed on the DSP side. */

#define T113_UART0_TX  0
#define T113_UART0_RX  0
#define T113_UART1_TX  0
#define T113_UART1_RX  0
#define T113_UART2_TX  0
#define T113_UART2_RX  0
#define T113_UART3_TX  0
#define T113_UART3_RX  0

#ifndef __ASSEMBLY__

static inline void t113_gpio_config(uint32_t pincfg)
{
  (void)pincfg;
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_XTENSA_SRC_T113_HARDWARE_T113_GPIO_H */
