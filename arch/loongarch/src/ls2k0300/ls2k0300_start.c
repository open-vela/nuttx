/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_start.c
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

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "loongarch_internal.h"
#include "ls2k0300.h"
#include "chip.h"
#include "hardware/ls2k0300_memorymap.h"

#ifdef CONFIG_BUILD_PROTECTED
#  include "ls2k0300_userspace.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_FEATURES
#  define showprogress(c) loongarch_lowputc(c)
#else
#  define showprogress(c)
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

uintptr_t g_idle_topstack =
    PHYS_TO_CACHED(LS2K0300_L2CACHE_BASE + LS2K0300_L2CACHE_SIZE);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void __ls2k0300_start(void)
{
  uint32_t *dest;
  const uint32_t *src;

  for (dest = (uint32_t *)_sbss; dest < (uint32_t *)_ebss; )
    {
      *dest++ = 0;
    }

  for (src = (const uint32_t *)_eronly,
       dest = (uint32_t *)_sdata; dest < (uint32_t *)_edata;
      )
    {
      *dest++ = *src++;
    }

  ls2k0300_lowsetup();

#ifdef USE_EARLYSERIALINIT
  loongarch_earlyserialinit();
#endif

  ls2k0300_boardinitialize();

#ifdef CONFIG_BUILD_PROTECTED
  ls2k0300_userspace();
#endif

  nx_start();

  while (true)
    {
      asm volatile("idle 0");
    }
}
