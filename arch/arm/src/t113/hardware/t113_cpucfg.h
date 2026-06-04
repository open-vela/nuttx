/****************************************************************************
 * arch/arm/src/t113/hardware/t113_cpucfg.h
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

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_CPUCFG_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_CPUCFG_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* C0_CPUX_CFG registers (Cluster0 CPU configuration) */

#define T113_CPUXCFG_BASE       0x09010000

#define T113_C0_RST_CTRL        (T113_CPUXCFG_BASE + 0x0000)
#define T113_C0_CTRL_REG0       (T113_CPUXCFG_BASE + 0x0010)
#define T113_C0_CPU_STATUS      (T113_CPUXCFG_BASE + 0x0080)

/* C0_RST_CTRL bit definitions */

#define C0_RST_CTRL_CORE0      (1 << 0)
#define C0_RST_CTRL_CORE1      (1 << 1)

/* C0_CTRL_REG0 bit definitions (L1 reset disable) */

#define C0_CTRL_L1RST_CORE0   (1 << 0)
#define C0_CTRL_L1RST_CORE1   (1 << 1)

/* CPU_SYS_CFG registers (CPU subsystem configuration) */

#define T113_CPUSCFG_BASE       0x07000400

/* CPU soft-entry register: CPU reads this address on power-up */

#define T113_CPU_SOFT_ENT(n)    (T113_CPUSCFG_BASE + 0x01c4 + ((n) * 4))

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_CPUCFG_H */
