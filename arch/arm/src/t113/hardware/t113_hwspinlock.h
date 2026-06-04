/****************************************************************************
 * arch/arm/src/t113/hardware/t113_hwspinlock.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_HWSPINLOCK_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_HWSPINLOCK_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* T113-S3 Spinlock module: 32 hardware lock units on AHB0.
 * Acquire = read SPINLOCKn_LOCK_REG: returns 0 = lock granted, 1 = busy.
 * Release = write 0 to the same register.
 */

#define T113_HWSPINLOCK_BASE        0x03005000
#define T113_HWSPINLOCK_NUM_LOCKS   32

/* Per-lock register: SPINLOCKn_LOCK_REG, N = 0..31 */

#define T113_HWSPINLOCK_LOCK_OFFSET(n)  (0x0100 + (n) * 4)
#define T113_HWSPINLOCK_LOCK(n) \
  (T113_HWSPINLOCK_BASE + T113_HWSPINLOCK_LOCK_OFFSET(n))

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_HWSPINLOCK_H */
