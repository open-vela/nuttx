/****************************************************************************
 * arch/loongarch/src/common/loongarch_initialize.c
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

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "loongarch_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

volatile bool g_interrupt_context[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_initialize
 *
 * Description:
 *   up_initialize will be called once during OS initialization after the
 *   basic OS services have been initialized.
 *
 ****************************************************************************/

void up_initialize(void)
{
  loongarch_addregion();

#ifdef CONFIG_PM
  loongarch_pminitialize();
#endif

#ifdef CONFIG_ARCH_DMA
#ifdef CONFIG_HAVE_WEAKFUNCTIONS
  if (loongarch_dma_initialize)
#endif
    {
      loongarch_dma_initialize();
    }
#endif

#ifdef USE_SERIALDRIVER
  loongarch_serialinit();
#endif

  loongarch_netinitialize();

  board_autoled_on(LED_IRQSENABLED);
}
