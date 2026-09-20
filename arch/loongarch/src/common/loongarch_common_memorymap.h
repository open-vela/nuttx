/****************************************************************************
 * arch/loongarch/src/common/loongarch_common_memorymap.h
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

#ifndef __ARCH_LOONGARCH_SRC_COMMON_LOONGARCH_COMMON_MEMORYMAP_H
#define __ARCH_LOONGARCH_SRC_COMMON_LOONGARCH_COMMON_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define _START_TEXT  _stext
#define _END_TEXT    _etext
#define _START_BSS   _sbss
#define _END_BSS     _ebss
#define _DATA_INIT   _eronly
#define _START_DATA  _sdata
#define _END_DATA    _edata
#define _START_TDATA _stdata
#define _END_TDATA   _etdata
#define _START_TBSS  _stbss
#define _END_TBSS   _etbss

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifndef __ASSEMBLY__

EXTERN uintptr_t g_idle_topstack;

#define g_cpux_idlestack(cpuid) \
   (g_idle_topstack - SMP_STACK_SIZE * (CONFIG_SMP_NCPUS - (cpuid)))

#if CONFIG_ARCH_INTERRUPTSTACK > 15
EXTERN uint8_t g_intstackalloc[];
EXTERN uint8_t g_intstacktop[];
#endif

EXTERN uint8_t _stext[];
EXTERN uint8_t _etext[];
EXTERN const uint8_t _eronly[];
EXTERN uint8_t _sdata[];
EXTERN uint8_t _edata[];
EXTERN uint8_t _sbss[];
EXTERN uint8_t _ebss[];
EXTERN uint8_t _stdata[];
EXTERN uint8_t _etdata[];
EXTERN uint8_t _stbss[];
EXTERN uint8_t _etbss[];

#endif /* __ASSEMBLY__ */

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ARCH_LOONGARCH_SRC_COMMON_LOONGARCH_COMMON_MEMORYMAP_H */
