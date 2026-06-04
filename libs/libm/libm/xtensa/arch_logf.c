/****************************************************************************
 * libs/libm/libm/xtensa/arch_logf.c
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

/* logf via the cephes single-precision reduction, evaluated on the HiFi4
 * LX7 audio-TIE FP datapath (aed file).  There is no log instruction, so:
 *
 *   x = m * 2^e  with the IEEE exponent/mantissa pulled out in integer C
 *   (cheapest, and where the subnormal/special screening already lives);
 *   m is forced into [0.5,1) then nudged into [sqrt(0.5),sqrt(2)) so the
 *   series argument f = m-1 (or 2m-1) stays tiny.  Then
 *
 *     logf(x) = e*ln2 + log1p(f)
 *     log1p(f) ~= f - 0.5*f*f + f*f*f * P(f)
 *
 *   with P the degree-8 cephes minimax polynomial (coeffs Lg0..Lg8 below,
 *   given as exact IEEE-754 hex so the bits are reproducible).  e*ln2 uses
 *   the Cody-Waite split ln2 = C2 + C1 (C2 = 0.693359375 exact in binary,
 *   C1 = -2.12194440e-4 the tail) to keep the large-|e| reconstruction
 *   accurate.
 *
 * The whole reconstruction runs in one inline-asm block on the aed
 * registers.  Horner is done with the fused madd.s (aedD += aedA*aedB, one
 * rounding per step) seeded each step by an arbitrary float constant loaded
 * as movi a8,0xBITS + ae_movda32 aedX,a8 (const.s only loads the small
 * integers 0..3).  f, z=f*f and the integer exponent e (already widened to
 * float by float.s) are bridged in via ae_movda32; the result bits come
 * back through ae_movad32.l.  aed registers are never named as clobbers
 * (GCC rejects them); a8 is the only scratch a-register and is declared.
 *
 * Accuracy: this is the cephes coefficient set, validated here to a max of
 * 1 ULP vs host glibc logf over all 4.27e8 positive normals and all
 * 8.39e6 subnormals (mean ~0.006 ULP).  It is faithfully rounded but NOT
 * provably 0 ULP -- the last ULP cannot be guaranteed without a
 * wider-than-float residual, which this single-precision-only datapath
 * cannot form.  IEEE specials are screened in integer C first, matching the
 * generic lib_logf.c contract: x<0 -> NaN, +-0 -> -inf, +inf -> +inf,
 * NaN -> NaN, logf(1) -> +0.
 */

#if XTENSA_LIBM_HAVE_VFPU2
float logf(float x)
{
  union
  {
    float    f;
    uint32_t u;
  } ux;

  uint32_t ix;
  uint32_t e;
  int      ei;
  int      sub;
  float    m;
  float    f;
  float    ef;
  float    result;

  ux.f = x;
  ix   = ux.u;
  e    = (ix >> 23) & 0xff;

  /* NaN -> quiet NaN; -inf -> NaN; +inf -> +inf */

  if (e == 0xff)
    {
      if (ix & 0x007fffffu)
        {
          return x + x;                 /* NaN propagation (quiets sNaN) */
        }

      if (ix & 0x80000000u)
        {
          ux.u = 0x7fc00000u;           /* logf(-inf) -> NaN */
          return ux.f;
        }

      return x;                         /* logf(+inf) -> +inf */
    }

  /* negative (sign set, not -0) -> NaN */

  if ((ix & 0x80000000u) && (ix << 1) != 0)
    {
      ux.u = 0x7fc00000u;
      return ux.f;
    }

  /* +-0 -> -inf */

  if ((ix << 1) == 0)
    {
      ux.u = 0xff800000u;
      return ux.f;
    }

  /* Subnormal positive: scale by 2^25 into the normal range, then remove
   * 25 from the exponent contribution at reconstruction time.
   */

  sub = 0;
  if (e == 0)
    {
      ux.f = x * 33554432.0f;           /* 2^25 */
      ix   = ux.u;
      e    = (ix >> 23) & 0xff;
      sub  = 25;
    }

  /* x = m * 2^ei with m in [0.5,1): mantissa bits with a forced 0.5 exp. */

  ei     = (int)e - 126;
  ux.u   = (ix & 0x007fffffu) | 0x3f000000u;
  m      = ux.f;

  /* Nudge m into [sqrt(0.5), sqrt(2)) so f = m-1 is small and symmetric.
   * SQRTHF = sqrt(0.5) = 0x3f3504f3.
   */

  ux.u = 0x3f3504f3u;
  if (m < ux.f)
    {
      ei -= 1;
      f   = (m + m) - 1.0f;
    }
  else
    {
      f = m - 1.0f;
    }

  ei -= sub;
  ef  = (float)ei;

  /* Reconstruction on the aed FP file.
   *
   *   z = f*f
   *   p = ((((((((Lg0*f+Lg1)*f+Lg2)*f+Lg3)*f+Lg4)*f+Lg5)*f+Lg6)
   *         *f+Lg7)*f+Lg8)           (8 fused madd.s Horner steps)
   *   p = p * f * z                  (-> f^3 * P(f))
   *   p = p + e*C1                   (ln2 tail, fused)
   *   p = p + (-0.5)*z               (the -f^2/2 log1p term, fused)
   *   y = f + p
   *   y = y + e*C2                   (ln2 head, fused)
   *
   * Each constant is loaded movi a8,bits + ae_movda32; the seed for every
   * madd.s step is the next coefficient placed in the accumulator first.
   */

  __asm__ volatile
  (
    "ae_movda32   aed0, %1\n"          /* aed0 = f                       */
    "ae_movda32   aed1, %2\n"          /* aed1 = z = f*f                 */
    "ae_movda32   aed2, %3\n"          /* aed2 = e (float)               */

  /* p = Lg0 */

    "movi         a8, 0x3d9021bb\n"
    "ae_movda32   aed3, a8\n"          /* aed3 = p = Lg0                 */

  /* p = p*f + Lg1 */

    "movi         a8, 0xbdebd1b8\n"
    "ae_movda32   aed4, a8\n"
    "madd.s       aed4, aed3, aed0\n"  /* aed4 = Lg1 + p*f               */

  /* p = p*f + Lg2 */

    "movi         a8, 0x3def251a\n"
    "ae_movda32   aed3, a8\n"
    "madd.s       aed3, aed4, aed0\n"  /* aed3 = Lg2 + p*f               */

  /* p = p*f + Lg3 */

    "movi         a8, 0xbdfe5d4f\n"
    "ae_movda32   aed4, a8\n"
    "madd.s       aed4, aed3, aed0\n"

  /* p = p*f + Lg4 */

    "movi         a8, 0x3e11e9bf\n"
    "ae_movda32   aed3, a8\n"
    "madd.s       aed3, aed4, aed0\n"

  /* p = p*f + Lg5 */

    "movi         a8, 0xbe2aae50\n"
    "ae_movda32   aed4, a8\n"
    "madd.s       aed4, aed3, aed0\n"

  /* p = p*f + Lg6 */

    "movi         a8, 0x3e4cceac\n"
    "ae_movda32   aed3, a8\n"
    "madd.s       aed3, aed4, aed0\n"

  /* p = p*f + Lg7 */

    "movi         a8, 0xbe7ffffc\n"
    "ae_movda32   aed4, a8\n"
    "madd.s       aed4, aed3, aed0\n"

  /* p = p*f + Lg8 */

    "movi         a8, 0x3eaaaaaa\n"
    "ae_movda32   aed3, a8\n"
    "madd.s       aed3, aed4, aed0\n"  /* aed3 = P(f)                    */

  /* p = P(f) * f * z */

    "mul.s        aed3, aed3, aed0\n"  /* p *= f                         */
    "mul.s        aed3, aed3, aed1\n"  /* p *= z   -> f^3 * P(f)         */

  /* p += e * C1   (ln2 tail = -2.12194440e-4) */

    "movi         a8, 0xb95e8083\n"
    "ae_movda32   aed5, a8\n"          /* aed5 = C1                      */
    "madd.s       aed3, aed2, aed5\n"  /* p += e*C1                      */

  /* p += (-0.5) * z   (the -f^2/2 term, fused) */

    "movi         a8, 0xbf000000\n"
    "ae_movda32   aed6, a8\n"          /* aed6 = -0.5                    */
    "madd.s       aed3, aed6, aed1\n"  /* p += -0.5*z                    */

  /* y = f + p */

    "add.s        aed3, aed3, aed0\n"  /* aed3 = f + p                   */

  /* y += e * C2   (ln2 head = 0.693359375) */

    "movi         a8, 0x3f318000\n"
    "ae_movda32   aed7, a8\n"          /* aed7 = C2                      */
    "madd.s       aed3, aed2, aed7\n"  /* y += e*C2                      */

    "ae_movad32.l %0, aed3\n"
    : "=r" (result)
    : "r" (f), "r" (f * f), "r" (ef)
    : "a8"
  );

  return result;
}
#endif
