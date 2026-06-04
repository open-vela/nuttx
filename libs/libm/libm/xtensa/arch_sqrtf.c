/****************************************************************************
 * libs/libm/libm/xtensa/arch_sqrtf.c
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
#include <stdint.h>
#include <math.h>

#include "xtensa_libm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* sqrtf via HiFi4 Newton-Raphson on the reciprocal square root, kept
 * register-resident in the aed FP file.  The scalar Newton sequence is the
 * one the vendor dsp.bin csqrtf (0x37b07dd0) applies to a scalar magnitude:
 * sqrt0.s gives a mantissa-only seed (the exponent is dead until nexp01.s +
 * addexp.s supply it -- doc 16 S2.3), two maddn/msubn refinements, then
 * mksadj.s + addexpm.s/addexp.s compose the exponent and divn.s rounds.
 * Input in aed7, result in aed3.
 *
 * Validated 0 ULP vs host glibc sqrtf on T113-S3 silicon (~1500 cyc vs
 * ~12400 soft, ~8x).  IEEE special cases are screened in integer code up
 * front, matching lib_sqrtf.c: x<0 -> NaN, NaN -> NaN, +inf -> +inf,
 * +-0 -> 0.
 */

#if XTENSA_LIBM_HAVE_VFPU2
float sqrtf(float x)
{
  union
  {
    float    f;
    uint32_t u;
  } ux;

  uint32_t e;
  float    r;

  ux.f = x;
  e = (ux.u >> 23) & 0xff;

  /* NaN -> quiet NaN */

  if (e == 0xff && (ux.u & 0x007fffffu) != 0)
    {
      ux.u |= 0x00400000u;
      return ux.f;
    }

  /* negative (sign set, not -0) -> NaN */

  if ((ux.u & 0x80000000u) && (ux.u << 1) != 0)
    {
      ux.u = 0x7fc00000u;
      return ux.f;
    }

  /* +inf -> +inf, +-0 -> 0 (return unchanged) */

  if (e == 0xff || (ux.u << 1) == 0)
    {
      return x;
    }

  __asm__ volatile
  (
    "ae_movda32   aed7, %1\n"
    "const.s      aed2, 0\n"
    "sqrt0.s      aed13, aed7\n"
    "ae_mov       aed3, aed2\n"
    "maddn.s      aed3, aed13, aed13\n"
    "nexp01.s     aed15, aed7\n"
    "const.s      aed1, 3\n"
    "addexp.s     aed15, aed1\n"
    "maddn.s      aed1, aed3, aed15\n"
    "maddn.s      aed13, aed1, aed13\n"
    "nexp01.s     aed9, aed7\n"
    "ae_mov       aed3, aed2\n"
    "msubn.s      aed3, aed9, aed13\n"
    "ae_mov       aed12, aed2\n"
    "const.s      aed8, 3\n"
    "ae_mov       aed6, aed2\n"
    "maddn.s      aed12, aed13, aed15\n"
    "maddn.s      aed6, aed8, aed13\n"
    "maddn.s      aed9, aed3, aed3\n"
    "maddn.s      aed8, aed12, aed13\n"
    "neg.s        aed11, aed6\n"
    "maddn.s      aed3, aed9, aed11\n"
    "nexp01.s     aed5, aed7\n"
    "maddn.s      aed6, aed8, aed6\n"
    "mksadj.s     aed7, aed7\n"
    "maddn.s      aed5, aed3, aed3\n"
    "addexpm.s    aed3, aed7\n"
    "neg.s        aed6, aed6\n"
    "addexp.s     aed6, aed7\n"
    "divn.s       aed3, aed5, aed6\n"
    "ae_movad32.l %0, aed3\n"
    : "=r" (r) : "r" (x)
  );

  return r;
}
#endif
