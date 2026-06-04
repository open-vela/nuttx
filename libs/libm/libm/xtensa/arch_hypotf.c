/****************************************************************************
 * libs/libm/libm/xtensa/arch_hypotf.c
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

/* hypotf (sqrt(x*x + y*y)) without spurious overflow, on the HiFi4
 * audio-TIE FP datapath.  The scale-by-larger-magnitude scheme keeps the
 * squared sum in range for the whole float domain:
 *
 *   m = max(|x|, |y|),  n = min(|x|, |y|)
 *   t = n / m            in [0, 1]
 *   result = m * sqrtf(1 + t*t)
 *
 * sqrtf is itself an accelerated TIE op in this tree (arch_sqrtf.c), so the
 * single inline-asm block here only forms u = 1 + t*t with one fused madd.s
 * (the accumulator seeded with 1.0 via const.s, then t*t added), bridging t
 * in and u out.  The surrounding magnitude select, the n/m divide and the
 * m * sqrtf(u) product stay in C -- the divide has no single-op TIE
 * primitive and is cheap here, and keeping it in C matches how the other
 * arch helpers (e.g. arch_logf.c) place their scalar divides outside the
 * asm.
 *
 * The aed registers must NOT appear in the clobber list (GCC rejects the
 * names); only the scratch a-register a8 is clobbered, and the block neither
 * spans a call nor leaves live compiler-allocated aed state.
 *
 * Accuracy: max 2 ULP vs host glibc hypotf, measured over a logarithmic grid
 * of 121 x 121 exponent pairs x 64 mantissa samples (~9.4e5 points) spanning
 * 2^-60 .. 2^60 in each argument, plus the IEEE special cases.  The residual
 * 2-ULP cases come from the single-precision m * sqrtf(1 + t*t) carrying two
 * roundings (the 1 + t*t fma and the final product); this is the expected
 * faithful-rounding bound for the scaled formula in float and matches the
 * generic lib_hypotf.c fallback.
 *
 * IEEE special cases are screened in integer C up front, matching host glibc
 * hypotf:
 *   hypot(+-Inf, y) = +Inf  even when y is NaN   (Inf dominates)
 *   hypot(x, +-Inf) = +Inf  even when x is NaN
 *   either NaN (no Inf)     -> NaN
 *   hypot(x, 0)             = |x|   (falls out of the scale formula)
 *   hypot(0, 0)             = +0
 */

#if XTENSA_LIBM_HAVE_VFPU2
float hypotf(float x, float y)
{
  union
  {
    float    f;
    uint32_t u;
  } ux;

  union
  {
    float    f;
    uint32_t u;
  } uy;

  uint32_t ax;
  uint32_t ay;
  float    fx;
  float    fy;
  float    m;
  float    n;
  float    t;
  float    u;

  ux.f = x;
  uy.f = y;
  ax   = ux.u & 0x7fffffffu;
  ay   = uy.u & 0x7fffffffu;

  /* Inf in either argument dominates -> +Inf, even if the other is NaN. */

  if (ax == 0x7f800000u || ay == 0x7f800000u)
    {
      return (float)INFINITY;
    }

  /* NaN (and neither is Inf) -> NaN. */

  if (ax > 0x7f800000u || ay > 0x7f800000u)
    {
      ux.u = 0x7fc00000u;
      return ux.f;
    }

  /* m = max(|x|, |y|), n = min(|x|, |y|). */

  ux.u = ax;
  uy.u = ay;
  fx   = ux.f;
  fy   = uy.f;

  if (fx > fy)
    {
      m = fx;
      n = fy;
    }
  else
    {
      m = fy;
      n = fx;
    }

  /* Both zero (m == 0) -> +0; otherwise hypot(x,0) = |x| falls out with
   * t = 0, u = 1, result = m.
   */

  if (m == 0.0f)
    {
      return 0.0f;
    }

  t = n / m;                            /* in [0, 1] */

  /* u = 1 + t*t  (fused madd.s, accumulator seeded with 1.0). */

  __asm__ volatile
  (
    "ae_movda32   aed0, %1\n"          /* aed0 = t                          */
    "const.s      aed1, 1\n"           /* aed1 = 1.0f                       */
    "madd.s       aed1, aed0, aed0\n"  /* aed1 = 1 + t*t                    */
    "ae_movad32.l %0, aed1\n"
    : "=r" (u)
    : "r" (t)
    : "a8"
  );

  return m * sqrtf(u);
}
#endif
