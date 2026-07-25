/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_rsctable.h
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

#ifndef __ARCH_ARM_SRC_RK3588_M0_RK3588M0_RSCTABLE_H
#define __ARCH_ARM_SRC_RK3588_M0_RK3588M0_RSCTABLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/rptun/rptun.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const struct rptun_rsc_s g_rk3588m0_rsc_table;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Copy the const template into writable shared memory and return that
 * writable pointer.
 */

void *rk3588m0_copy_rsc_table(void);

#endif /* __ARCH_ARM_SRC_RK3588_M0_RK3588M0_RSCTABLE_H */
