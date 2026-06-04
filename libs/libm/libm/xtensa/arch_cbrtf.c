/****************************************************************************
 * libs/libm/libm/xtensa/arch_cbrtf.c
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

/* cbrtf (x^(1/3)) via a bit-hack exponent seed + Newton refinement on the
 * HiFi4 audio-TIE FP datapath.  There is no cube-root silicon op, so the
 * scheme is:
 *
 *   1. seed:  divide the biased exponent by 3 directly in the bit pattern,
 *             y0_bits = a/3 + 0x2a51067f   (a = |x| bits), giving ~3%
 *             accuracy
 *   2. two coarse Newton steps in the overflow-safe division form
 *             y = (2y + a/y^2) / 3
 *      (the term a/y^2 stays the magnitude of cbrt(x), so nothing overflows
 *      -- unlike the Halley form y*(y^3+2a)/(2y^3+a), where y^3 ~ x
 *      overflows for large |x|)
 *   3. one final residual-form Newton step
 *             y = y - (y - a/y^2) / 3
 *      which rounds the last bit better than the direct form.
 *
 * Each iteration is one multiply (y^2), one divide (a / y^2) and a fused
 * multiply against the constant 1/3.  The divide is written as a plain C '/'
 * so the compiler resolves it to our register-resident HiFi4 Newton-divide
 * helper __divsf3 (a COMPLETE, correct general single-precision divide); the
 * multiplies likewise lower to mul.s.  An earlier revision open-coded the
 * divide with the inline-asm "divn.s" TIE op -- that was WRONG: divn.s is
 * only the FINISHING primitive of a full Newton-division sequence (div0.s
 * seed -> nexp01 -> maddn -> mkdadj -> addexp -> divn.s, exactly how the
 * vendor dsp.bin uses it), and standalone it does NOT compute a correct a/b
 * quotient.  Using accelerated __divsf3 + mul keeps cbrtf fast while being
 * numerically correct.
 *
 * cbrtf is an odd function (cbrt(-x) = -cbrt(x)); the sign bit is stripped
 * before the iteration and reapplied to the result, so only |x| is iterated.
 * Subnormal |x| is scaled up by 2^24 (a multiple-of-3 exponent shift) so the
 * bit-hack and the iteration run on a normal number; the cube root then gets
 * a compensating *2^-8 = cbrt(2^-24) at the end.
 *
 * Accuracy: max 1 ULP vs host glibc cbrtf, measured over a 307-stride sweep
 * of all sampled finite floats (both signs, normals and subnormals):
 * faithfully rounded (no point at >= 2 ULP).
 *
 * IEEE special cases are returned by the C screen, matching host glibc
 * cbrtf:
 *   x = NaN    -> quiet NaN
 *   x = +-Inf  -> +-Inf   (cbrt is odd, sign preserved)
 *   x = +-0    -> +-0     (sign preserved)
 */

#define CBRTF_SEED_MAGIC  0x2a51067fu      /* exponent-divide-by-3 bias       */

#if XTENSA_LIBM_HAVE_VFPU2
float cbrtf(float x)
{
  union
  {
    float    f;
    uint32_t u;
  } ux;

  uint32_t ax;
  uint32_t sign;
  uint32_t a;
  float    fa;
  float    y;
  float    w;
  int      i;
  int      scaled = 0;

  const float third = 0.3333333432674408f;   /* (float)(1.0/3.0) */

  ux.f = x;
  ax   = ux.u & 0x7fffffffu;

  /* NaN -> quiet NaN; +-Inf and +-0 returned unchanged (cbrt is odd, so the
   * sign is already correct in x).
   */

  if (ax == 0 || ax >= 0x7f800000u)
    {
      if (ax > 0x7f800000u)
        {
          ux.u |= 0x00400000u;
        }

      return ux.f;
    }

  sign = ux.u & 0x80000000u;
  a    = ax;

  /* Subnormal |x|: scale up by 2^24 so the seed and iteration run on a
   * normal number; undo with *2^-8 on the cube root at the end.
   */

  if (a < 0x00800000u)
    {
      ux.u = ax;
      fa   = ux.f * 16777216.0f;       /* 2^24 */
      ux.f = fa;
      a    = ux.u;
      scaled = 1;
    }
  else
    {
      ux.u = a;
      fa   = ux.f;
    }

  /* Initial estimate: divide the biased exponent by 3 in the bit pattern. */

  ux.u = a / 3u + CBRTF_SEED_MAGIC;
  y    = ux.f;

  /* Two coarse Newton steps  y = (2y + a/y^2)/3  then a residual-form polish
   *   y = y - (y - a/y^2)/3.
   * The '/' lowers to the accelerated __divsf3 and '*' to mul.s; a/y^2 keeps
   * the magnitude of cbrt(x) so nothing overflows for large |x|.
   */

  for (i = 0; i < 2; i++)
    {
      float yy = y * y;
      y = (2.0f * y + fa / yy) * third;
    }

  w = fa / (y * y);
  y = y - (y - w) * third;

  /* Undo the subnormal scale, then reapply the sign (odd function). */

  if (scaled)
    {
      y = y * 0.00390625f;             /* 2^-8 = cbrt(2^-24) */
    }

  ux.f = y;
  ux.u = (ux.u & 0x7fffffffu) | sign;
  return ux.f;
}
#endif
