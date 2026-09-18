/****************************************************************************
 * arch/arm/src/bk7258/chip.h
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

#ifndef __ARCH_ARM_SRC_BK7258_CHIP_H
#define __ARCH_ARM_SRC_BK7258_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Include the memory map and chip peripheral definitions.  The NVIC
 * priority macros come from <arch/chip/chip.h>.
 */

#include "hardware/bk7258_memorymap.h"

/* Number of external (peripheral) interrupts used by the armv8-m vector
 * table.  BK7258_IRQ_NEXTINT is defined in arch/arm/include/bk7258/irq.h
 * (pulled in via nuttx/irq.h).
 */

#define ARMV8M_PERIPHERAL_INTERRUPTS BK7258_IRQ_NEXTINT

#endif /* __ARCH_ARM_SRC_BK7258_CHIP_H */
