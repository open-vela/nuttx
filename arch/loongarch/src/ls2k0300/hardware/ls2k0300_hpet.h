/****************************************************************************
 * arch/loongarch/src/ls2k0300/hardware/ls2k0300_hpet.h
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

#ifndef __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_HPET_H
#define __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_HPET_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define HPET_TIMESTAMP_LO    0x00
#define HPET_TIMESTAMP_HI    0x04
#define HPET_PERIOD          0x08
#define HPET_CFG             0x10
#define HPET_INT_STATUS      0x20
#define HPET_INT_ENABLE      0x24
#define HPET_MAIN_COUNTER_LO 0xf0
#define HPET_MAIN_COUNTER_HI 0xf4

#define HPET_CFG_ENABLE      (1 << 0)
#define HPET_CFG_LEGACY      (1 << 1)

#define HPET_INT_T0_ENABLE   (1 << 0)
#define HPET_INT_T1_ENABLE   (1 << 1)
#define HPET_INT_T2_ENABLE   (1 << 2)
#define HPET_INT_T3_ENABLE   (1 << 3)

#define HPET_T0_CFG          0x100
#define HPET_T0_CMP_LO       0x108
#define HPET_T0_CMP_HI       0x10c

#define HPET_T1_CFG          0x120
#define HPET_T1_CMP_LO       0x128
#define HPET_T1_CMP_HI       0x12c

#define HPET_T2_CFG          0x140
#define HPET_T2_CMP_LO       0x148
#define HPET_T2_CMP_HI       0x14c

#define HPET_T3_CFG          0x160
#define HPET_T3_CMP_LO       0x168
#define HPET_T3_CMP_HI       0x16c

#define HPET_TN_CFG_ENABLE      (1ULL << 0)
#define HPET_TN_CFG_PERIODIC    (1ULL << 1)
#define HPET_TN_CFG_INT_TYPE    (1ULL << 3)
#define HPET_TN_CFG_INT_ENABLE  (1ULL << 4)
#define HPET_TN_CFG_32BIT       (1ULL << 8)
#define HPET_TN_CFG_VAL_SET     (1ULL << 6)

#endif /* __ARCH_LOONGARCH_SRC_LS2K0300_HARDWARE_LS2K0300_HPET_H */
