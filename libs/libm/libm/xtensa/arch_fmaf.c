/****************************************************************************
 * libs/libm/libm/xtensa/arch_fmaf.c
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
#include <math.h>

#include "xtensa_libm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* fmaf via the HiFi4 madd.s fused multiply-add: a single silicon op with
 * one rounding, replacing the expensive double-precision soft-float
 * fallback (lib_fmaf.c).  madd.s is inout on its first operand
 * (aedD += aedA*aedB), so z is moved into the accumulator aed2, x into
 * aed0, y into aed1.  ae_movda32 / ae_movad32.l bridge the a-register
 * float bits to/from the aed FP file with no stack round-trip; aed
 * registers cannot appear in the clobber list (GCC rejects the names),
 * which is safe here as the sequence neither spans a call nor leaves live
 * compiler-allocated aed state.
 */

#if XTENSA_LIBM_HAVE_VFPU2
float fmaf(float x, float y, float z)
{
  float result;

  __asm__ volatile
  (
    "ae_movda32   aed0, %1\n"   /* aed0 = x */
    "ae_movda32   aed1, %2\n"   /* aed1 = y */
    "ae_movda32   aed2, %3\n"   /* aed2 = z (accumulator) */
    "madd.s       aed2, aed0, aed1\n"
    "ae_movad32.l %0, aed2\n"
    : "=r" (result)
    : "r" (x), "r" (y), "r" (z)
  );

  return result;
}
#endif
