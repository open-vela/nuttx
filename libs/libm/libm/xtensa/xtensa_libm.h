/****************************************************************************
 * libs/libm/libm/xtensa/xtensa_libm.h
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

#ifndef __LIBS_LIBM_LIBM_XTENSA_XTENSA_LIBM_H
#define __LIBS_LIBM_LIBM_XTENSA_XTENSA_LIBM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HiFi4 LX7 single-precision audio-TIE FP (VFPU2) on the aed0..aed15
 * register file.  Unlike ARM there is no compiler macro (__ARM_FP) that
 * advertises this datapath, so the capability is gated on the same Kconfig
 * symbol that enables the libc soft-float helper overrides: it implies the
 * CP1 audio coprocessor is present and enabled (CONFIG_XTENSA_CP_INITSET
 * bit 1), which is the prerequisite for mul.s / madd.s / abs.s / trunc.s.
 *
 * The float<->aed bridge uses ae_movda32 / ae_movad32.l (no stack
 * round-trip); GCC cannot bind a C float to an aed register, so each helper
 * moves bits explicitly inside inline asm.
 */

#if defined(__XTENSA__) && defined(CONFIG_XTENSA_LX7_HIFI4_FLOAT)
#  define XTENSA_LIBM_HAVE_VFPU2 1
#endif

#endif /* __LIBS_LIBM_LIBM_XTENSA_XTENSA_LIBM_H */
