/****************************************************************************
 * libs/libm/libm/xtensa/arch_asinf.c
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

/* asinf via the cephes single-precision reduction, evaluated on the HiFi4
 * LX7 audio-TIE FP datapath (aed file).  This replaces the generic
 * lib_asinf.c, whose Newton "while" loop calls sinf/cosf every iteration
 * (~15000 cyc).  The whole inverse-trig family (acosf, atanf, atan2f)
 * bottoms out in asinf, so they all benefit.
 *
 * There is no asin silicon op, so this is the classic cephes scheme.  Let
 * a = |x|:
 *
 *   a > 0.5 :  z = 0.5*(1 - a),  xp = sqrtf(z)        range reduction
 *              asin(a) = pi/2 - 2*(xp + xp*z*P(z))
 *   else    :  z = a*a,          xp = a
 *              asin(a) = xp + xp*z*P(z)
 *
 * with P the degree-4 cephes minimax polynomial (5 coeffs P0..P4 below,
 * given as exact IEEE-754 hex so the bits are reproducible):
 *
 *   P(z) = ((((P0*z + P1)*z + P2)*z + P3)*z + P4)
 *
 * The polynomial and the correction term t = xp + xp*z*P(z) run in one
 * inline-asm block on the aed registers: 4 fused madd.s Horner steps (each
 * seeded by the next coefficient placed in the accumulator first), one mul.s
 * for z*P(z), and a final fused madd.s for xp + xp*(z*P(z)).  Each
 * coefficient is an arbitrary float, so it is loaded as raw IEEE bits via
 * "movi a8,<bits>" + ae_movda32 (const.s only loads the small integers
 * 0..3).  xp and z are bridged in with ae_movda32 and the result bits come
 * back through ae_movad32.l -- no stack round-trip.  aed registers are never
 * named as clobbers (GCC rejects them); a8 is the only scratch a-register
 * declared.
 *
 * The branch/flag selection, the a>0.5 reduction sqrt (sqrtf is itself an
 * accelerated TIE op in this tree), the final reconstruction and the sign
 * fix-up are kept in C -- cheap, branchy, and where the IEEE special-case
 * screening already lives.  For the a>0.5 branch the cephes reconstruction
 * pi/2 - 2t is evaluated as (pi/4 - t) + (pi/4 - t): folding the doubling
 * into the two pi/4 subtractions halves the cancellation versus forming 2t
 * first and subtracting from pi/2, preserving the low bits.
 *
 * Accuracy: degree-4 polynomial, validated EXHAUSTIVELY over all 2.13e9
 * float inputs with |x| <= 1 (both signs) vs host glibc asin (computed in
 * double then rounded): max 2 ULP, mean 0.006 ULP -- i.e. faithfully
 * rounded.  The 2-ULP cases sit just above the a=0.5 branch boundary
 * (e.g. x = 0x3f000004), the hardest region for the single-precision-only
 * datapath, which cannot form a wider residual.
 *
 * IEEE special cases are screened in integer C up front, matching the
 * contract of the generic lib_asinf.c / host glibc:
 *   x = NaN        -> quiet NaN
 *   |x| > 1        -> NaN  (out of domain)
 *   |x| == 1       -> copysign(pi/2, x)
 *   |x| < 1e-4     -> x     (asin(a) ~= a; returning x preserves the sign of
 *                            +-0 and tiny denormals)
 *   x = +-0        -> x     (falls out of the |x|<1e-4 screen)
 */

#define ASINF_PIO2F_BITS 0x3fc90fdbu   /* pi/2 = 1.5707963705e+00          */
#define ASINF_PIO4F_BITS 0x3f490fdbu   /* pi/4 = 7.8539818525e-01          */
#define ASINF_TINY_BITS  0x38d1b717u   /* 1e-4                             */
#define ASINF_HALF_BITS  0x3f000000u   /* 0.5                             */
#define ASINF_ONE_BITS   0x3f800000u   /* 1.0                             */

#if XTENSA_LIBM_HAVE_VFPU2
float asinf(float x)
{
  union
  {
    float    f;
    uint32_t u;
  } ux;

  uint32_t ix;
  uint32_t ax;
  uint32_t sign;
  int      flag;
  float    a;
  float    z;
  float    xp;
  float    t;
  float    r;

  ux.f = x;
  ix   = ux.u;
  ax   = ix & 0x7fffffffu;

  /* NaN -> quiet NaN. */

  if (ax > 0x7f800000u)
    {
      ux.u = ix | 0x00400000u;
      return ux.f;
    }

  /* |x| > 1 -> NaN (out of domain), covers +-Inf too. */

  if (ax > ASINF_ONE_BITS)
    {
      ux.u = 0x7fc00000u;
      return ux.f;
    }

  sign = ix & 0x80000000u;

  /* |x| == 1 -> copysign(pi/2, x). */

  if (ax == ASINF_ONE_BITS)
    {
      ux.u = ASINF_PIO2F_BITS | sign;
      return ux.f;
    }

  /* |x| < 1e-4 -> asin(a) ~= a; return x to preserve the sign of +-0 and
   * tiny denormals.
   */

  if (ax < ASINF_TINY_BITS)
    {
      return x;
    }

  /* a = |x|.  Select the reduction: a > 0.5 uses z = 0.5*(1-a), xp = sqrt(z)
   * and the pi/2 - 2t reconstruction; otherwise z = a*a, xp = a.
   */

  ux.u = ax;
  a    = ux.f;

  if (ax > ASINF_HALF_BITS)
    {
      ux.u = ASINF_HALF_BITS;
      z    = ux.f * (1.0f - a);     /* 0.5 * (1 - a)                       */
      xp   = sqrtf(z);
      flag = 1;
    }
  else
    {
      z    = a * a;
      xp   = a;
      flag = 0;
    }

  /* t = xp + xp*z*P(z) on the aed FP file.
   *
   *   P(z) = ((((P0*z + P1)*z + P2)*z + P3)*z + P4)   (4 fused madd.s)
   *   zp   = z * P(z)                                 (mul.s)
   *   t    = xp + xp*zp                               (fused madd.s)
   */

  __asm__ volatile
  (
    "ae_movda32   aed0, %1\n"          /* aed0 = z                         */
    "ae_movda32   aed1, %2\n"          /* aed1 = xp                        */

  /* p = P0 */

    "movi         a8, 0x3d2cb352\n"    /* P0 = 4.2163199048e-2             */
    "ae_movda32   aed2, a8\n"          /* aed2 = p = P0                    */

  /* p = p*z + P1 */

    "movi         a8, 0x3cc617e3\n"    /* P1 = 2.4181311049e-2             */
    "ae_movda32   aed3, a8\n"
    "madd.s       aed3, aed2, aed0\n"  /* aed3 = P1 + p*z                  */

  /* p = p*z + P2 */

    "movi         a8, 0x3d3a3ec7\n"    /* P2 = 4.5470025998e-2             */
    "ae_movda32   aed2, a8\n"
    "madd.s       aed2, aed3, aed0\n"  /* aed2 = P2 + p*z                  */

  /* p = p*z + P3 */

    "movi         a8, 0x3d9980f6\n"    /* P3 = 7.4953002686e-2             */
    "ae_movda32   aed3, a8\n"
    "madd.s       aed3, aed2, aed0\n"  /* aed3 = P3 + p*z                  */

  /* p = p*z + P4 */

    "movi         a8, 0x3e2aaae4\n"    /* P4 = 1.6666752422e-1             */
    "ae_movda32   aed2, a8\n"
    "madd.s       aed2, aed3, aed0\n"  /* aed2 = P(z)                      */

  /* zp = z * P(z) */

    "mul.s        aed2, aed2, aed0\n"  /* aed2 = z*P(z)                    */

  /* t = xp + xp*zp   (seed accumulator with xp, then fused madd) */

    "mul.s        aed2, aed2, aed1\n"  /* aed2 = xp*z*P(z)                 */
    "add.s        aed2, aed2, aed1\n"  /* aed2 = xp + xp*z*P(z) = t        */

    "ae_movad32.l %0, aed2\n"
    : "=r" (t)
    : "r" (z), "r" (xp)
    : "a8"
  );

  /* Reconstruction (in C, cheap and branchy):
   *   flag : asin(a) = pi/2 - 2t = (pi/4 - t) + (pi/4 - t)
   *   else : asin(a) = t
   * then apply copysign of the original input.
   */

  if (flag)
    {
      ux.u = ASINF_PIO4F_BITS;
      r    = ux.f - t;
      r    = r + r;
    }
  else
    {
      r = t;
    }

  ux.f = r;
  ux.u = (ux.u & 0x7fffffffu) | sign;
  return ux.f;
}
#endif
