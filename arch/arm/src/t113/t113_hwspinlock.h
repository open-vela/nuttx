/****************************************************************************
 * arch/arm/src/t113/t113_hwspinlock.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_HWSPINLOCK_H
#define __ARCH_ARM_SRC_T113_T113_HWSPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/hwspinlock/hwspinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Chip-wide hwspinlock id allocation table.  Both image_a and image_b in
 * AMP must see the SAME numbering, so all consumers include this header.
 * Framework locks own ids 0..7, leaving 8..31 for application use.
 */

#define T113_HWLOCK_ID_CCU         0
#define T113_HWLOCK_ID_GPIO        1
/* 2..7 reserved for future framework locks (RSC_TABLE, ...) */
#define T113_HWLOCK_ID_APP_FIRST   8
#define T113_HWLOCK_ID_APP_LAST    31

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: t113_hwspinlock_initialize
 *
 * Description:
 *   One-shot initialization of the T113-S3 hardware spinlock module.
 *   De-asserts the SPINLOCK reset and enables its clock gate via the CCU,
 *   then prepares the array of 32 hwspinlock_dev_s instances exposed via
 *   t113_hwspinlock_get().
 *
 *   Safe to call multiple times; subsequent calls are no-ops.
 *
 ****************************************************************************/

void t113_hwspinlock_initialize(void);

/****************************************************************************
 * Name: t113_hwspinlock_get
 *
 * Description:
 *   Return the hwspinlock_dev_s instance for hardware lock 'id'.
 *
 * Input Parameters:
 *   id - Lock index in the range 0..31.
 *
 * Returned Value:
 *   Pointer to the dev_s on success, NULL if id is out of range or the
 *   driver has not been initialized.
 *
 ****************************************************************************/

FAR struct hwspinlock_dev_s *t113_hwspinlock_get(int id);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_T113_T113_HWSPINLOCK_H */
