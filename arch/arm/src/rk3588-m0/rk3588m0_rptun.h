/****************************************************************************
 * arch/arm/src/rk3588-m0/rk3588m0_rptun.h
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

#ifndef __ARCH_ARM_SRC_RK3588_M0_RK3588M0_RPTUN_H
#define __ARCH_ARM_SRC_RK3588_M0_RK3588M0_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: rk3588m0_rptun_init
 *
 * Description:
 *   Bring up the rpmsg transport towards Linux over mailbox1.
 *
 * Input Parameters:
 *   shmemname - shared memory name passed to rptun
 *   cpuname   - name of the peer CPU ("linux")
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int rk3588m0_rptun_init(const char *shmemname, const char *cpuname);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_RK3588_M0_RK3588M0_RPTUN_H */
