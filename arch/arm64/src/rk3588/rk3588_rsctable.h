/****************************************************************************
 * arch/arm64/src/rk3588/rk3588_rsctable.h
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

#ifndef __ARCH_ARM64_SRC_RK3588_RK3588_RSCTABLE_H
#define __ARCH_ARM64_SRC_RK3588_RK3588_RSCTABLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/rptun/rptun.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct rptun_rsc_s g_rk3588_rsc_table;

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Writable copy of the resource table, inside the AMP_RPMSG carveout
 * (0x07c00000, MT_NORMAL_NC, RW). The const template g_rk3588_rsc_table lives
 * in .rodata which arm64 maps READ-ONLY, but OpenAMP writes back notify ids,
 * so the table handed to rptun MUST be in writable memory.
 */

#define RK3588_RSC_TABLE_BASE   0x07c0f000ul

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Copy the const template into the writable RK3588_RSC_TABLE_BASE and return
 * that writable pointer.
 */

void *rk3588_copy_rsc_table(void);

#endif /* __ARCH_ARM64_SRC_RK3588_RK3588_RSCTABLE_H */
