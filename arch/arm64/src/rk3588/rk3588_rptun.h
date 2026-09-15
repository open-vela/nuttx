/****************************************************************************
 * arch/arm64/src/rk3588/rk3588_rptun.h
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

#ifndef __ARCH_ARM64_SRC_RK3588_RK3588_RPTUN_H
#define __ARCH_ARM64_SRC_RK3588_RK3588_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: rk3588_rptun_init
 *
 * Description:
 *   Initialize the RK3588 AMP rptun device (NuttX = remote, Linux = master).
 *   Uses mailbox0 (fec60000) as the doorbell and vrings at 0x7c00000 /
 *   0x7c08000, matching the Linux rockchip rpmsg layout.
 *
 ****************************************************************************/

int rk3588_rptun_init(const char *shmemname, const char *cpuname);

#endif /* __ARCH_ARM64_SRC_RK3588_RK3588_RPTUN_H */
