/****************************************************************************
 * arch/arm/src/t113/t113_boot.h
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

#ifndef __ARCH_ARM_SRC_T113_T113_BOOT_H
#define __ARCH_ARM_SRC_T113_T113_BOOT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

void t113_boardinitialize(void);

#if !defined(CONFIG_UP) && !defined(CONFIG_ARCH_TRUSTZONE_SECURE) && \
    !defined(CONFIG_ARCH_TRUSTZONE_NONSECURE)
void t113_timer_secondary_init(void);
#endif

/* core0-NuttX cpu1-master: release CPU1 to a runtime-resolved ELF entry
 * (rproc->bootaddr) loaded by the rptun framework.  Compiled for any
 * non-slave image; see t113_cpuboot.c.
 */

void t113_release_cpu1(uintptr_t entry);

#endif /* __ARCH_ARM_SRC_T113_T113_BOOT_H */
