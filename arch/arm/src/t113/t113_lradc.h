/****************************************************************************
 * arch/arm/src/t113/t113_lradc.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_LRADC_H
#define __ARCH_ARM_SRC_T113_T113_LRADC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Raw-sample callback invoked from the LRADC ISR.  Used by board-level
 * consumers such as the resistor-ladder keypad driver.  The callback runs
 * in interrupt context and MUST NOT block or allocate memory.
 *
 *  status - raw INTS value before acknowledgement
 *  raw    - 6-bit DATA0 value (0..63)
 *  priv   - opaque pointer passed to t113_lradc_register_hook
 */

typedef void (*t113_lradc_hook_t)(uint32_t status, uint32_t raw,
                                  FAR void *priv);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: t113_lradc_initialize
 *
 * Description:
 *   Register /dev/lradc0 with the NuttX ADC upper half.
 *
 ****************************************************************************/

void t113_lradc_initialize(void);

/****************************************************************************
 * Name: t113_lradc_register_hook
 *
 * Description:
 *   Install a raw-sample hook called from the LRADC ISR.  Pass hook=NULL
 *   to remove a previously installed hook.  Only one hook slot is provided;
 *   the latest call wins.
 *
 ****************************************************************************/

void t113_lradc_register_hook(t113_lradc_hook_t hook, FAR void *priv);

#endif /* __ARCH_ARM_SRC_T113_T113_LRADC_H */
