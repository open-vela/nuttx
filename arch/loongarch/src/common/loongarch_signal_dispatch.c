/****************************************************************************
 * arch/loongarch/src/common/loongarch_signal_dispatch.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>

#include <arch/syscall.h>

#include "loongarch_internal.h"

#if !defined(CONFIG_BUILD_FLAT) && defined(__KERNEL__)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_signal_dispatch(_sa_sigaction_t sighand, int signo,
                        siginfo_t *info, void *ucontext)
{
  sys_call4(SYS_signal_handler, (uintptr_t)sighand, (uintptr_t)signo,
            (uintptr_t)info, (uintptr_t)ucontext);
}

#endif
