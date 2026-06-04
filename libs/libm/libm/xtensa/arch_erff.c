/****************************************************************************
 * libs/libm/libm/xtensa/arch_erff.c
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

/* erff on the HiFi4 audio-TIE FP datapath.
 *
 * WHY A REWRITE, NOT JUST A POLY PORT:  the generic lib_erff.c uses the
 * Abramowitz & Stegun 7.1.26 formula (degree-5 in t = 1/(1+P*z), times a
 * single expf).  That formula is only ~7-decimal-digit accurate -- its max
 * ABSOLUTE error is 6.6e-7 by construction -- so near x = 0, where erf(x) ->
 * 0, the relative error explodes to many thousands of ULP (host-measured:
 * > 8e8 ULP at x ~= 1.8e-7).  Re-expressing that SAME polynomial in madd.s
 * would run faster but stay just as inaccurate; the polynomial is the
 * accuracy bottleneck, not the cost.  So this arch version swaps in the
 * Sun/fdlibm single-precision rational scheme, which is faithful to <= 1 ULP
 * and still leans on the accelerated expf (twice, in the tail).
 *
 * Four domains (fdlibm s_erff layout):
 *   |x| < 0.84375 :  erf(x) = x + x * (R(z)/S(z)),  z = x*x
 *   0.84375..1.25 :  erf(x) = sign * (erx + P1(s)/Q1(s)),  s = |x| - 1
 *   1.25..6       :  erf(x) = sign * (1 - exp(-z*z-0.5625) *
 *                                          exp((z-|x|)(z+|x|)+R/S) / |x|)
 *                    with two sub-ranges for R/S and z = |x| with the low
 *                    mantissa bits cleared so z*z is computed without error
 *   |x| >= 6      :  erf(x) = sign * 1 (rounded just inside)
 *
 * Each numerator / denominator polynomial is a Horner chain evaluated in one
 * inline-asm block of fused madd.s on the aed FP file: the coefficient is
 * placed in the accumulator first, then madd.s folds in (prev * v).  The
 * rational divides (R/S, P/Q, r/|x|) and the two expf calls stay in C --
 * there is no single-op divide TIE primitive, and this matches how the other
 * arch helpers (arch_logf.c) keep their scalar divides in C.
 *
 * All polynomial coefficients are the Sun/fdlibm single-precision values
 * from s_erff.c; each is loaded inside the asm blocks below as its exact
 * IEEE-754 bit pattern via "movi a8, <bits>" (const.s only loads small
 * integers 0..3), the hex matching the (float) of the decimal in the
 * trailing comment.  The float<->aed bridge ae_movda32 / ae_movad32.l has no
 * stack round-trip; aed registers are never named as clobbers (GCC rejects
 * them), only the scratch a-register a8.
 *
 * Accuracy: max 1 ULP vs host glibc erff, measured EXHAUSTIVELY over all
 * 1.09e9 floats with |x| <= 8 (both signs): 94.2% correctly rounded (0 ULP),
 * 5.8% at 1 ULP, none at >= 2 ULP -- faithfully rounded, versus the generic
 * formula's > 8e8 ULP.  erfcf = 1 - erff (its wrapper) inherits this.
 *
 * IEEE special cases follow host glibc erff:
 *   x = NaN     -> quiet NaN
 *   x = +-Inf   -> +-1
 *   x = +-0     -> +-0   (erf is odd; falls out of the small-x branch)
 */

#if XTENSA_LIBM_HAVE_VFPU2
float erff(float x)
{
  union
  {
    float    f;
    uint32_t u;
  } ux;

  int32_t  hx;
  int32_t  ix;
  float    xa;
  float    z;
  float    r;
  float    s;
  float    num;
  float    den;

  ux.f = x;
  hx   = (int32_t)ux.u;
  ix   = hx & 0x7fffffff;

  /* Inf / NaN: erf(+-Inf) = +-1, NaN -> NaN. */

  if (ix >= 0x7f800000)
    {
      int i = ((uint32_t)hx >> 31) << 1;
      return (float)(1 - i) + 1.0f / x;
    }

  /* ---- |x| < 0.84375 : erf(x) = x + x * R(z)/S(z),  z = x*x ---- */

  if (ix < 0x3f580000)
    {
      if (ix < 0x31800000)             /* |x| < 2^-28: erf(x) ~= x + efx*x */
        {
          if (ix < 0x04000000)         /* avoid underflow in the product */
            {
              /* 0.125*(8x + efx8*x); efx8 = 1.027033329 */

              ux.u = 0x3f8375d4u;      /* efx8 */
              return 0.125f * (8.0f * x + ux.f * x);
            }

          ux.u = 0x3e0375d4u;          /* efx = 0.1283791661 */
          return x + ux.f * x;
        }

      z = x * x;

      /* R(z) = (((PP4*z + PP3)*z + PP2)*z + PP1)*z + PP0   (Horner)
       * S(z) = ((((QQ5*z + QQ4)*z + QQ3)*z + QQ2)*z + QQ1)*z + 1
       */

      __asm__ volatile
      (
        "ae_movda32   aed0, %2\n"      /* aed0 = z                          */

      /* R(z) */

        "movi         a8, 0xb7c756b1\n"/* PP4 = -2.376301745e-05            */
        "ae_movda32   aed1, a8\n"
        "movi         a8, 0xbbbd1489\n"/* PP3 = -0.005770270247             */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xbce9528f\n"/* PP2 = -0.02848174982              */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0xbea66beb\n"/* PP1 = -0.3250420988               */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0x3e0375d4\n"/* PP0 = 0.1283791661                */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"  /* aed1 = R(z)                   */
        "ae_movad32.l %0, aed1\n"

      /* S(z) */

        "movi         a8, 0xb684e21a\n"/* QQ5 = -3.960228241e-06            */
        "ae_movda32   aed2, a8\n"
        "movi         a8, 0x390aee49\n"/* QQ4 = 0.000132494737              */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "movi         a8, 0x3ba68116\n"/* QQ3 = 0.005081306212              */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed3, aed0\n"
        "movi         a8, 0x3d852a63\n"/* QQ2 = 0.0650222525                */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "movi         a8, 0x3ecbbbce\n"/* QQ1 = 0.3979172111                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed3, aed0\n"
        "const.s      aed3, 1\n"           /* 1.0f */
        "madd.s       aed3, aed2, aed0\n"  /* aed3 = S(z)                   */
        "ae_movad32.l %1, aed3\n"
        : "=r" (num), "=r" (den)
        : "r" (z)
        : "a8"
      );

      return x + x * (num / den);
    }

  /* ---- 0.84375 <= |x| < 1.25 : erf = sign*(erx + P1(s)/Q1(s)) ---- */

  if (ix < 0x3fa00000)
    {
      xa = fabsf(x);
      s  = xa - 1.0f;

      /* P1(s) = ((((((PA6*s+PA5)*s+PA4)*s+PA3)*s+PA2)*s+PA1)*s+PA0)
       * Q1(s) = (((((QA6*s+QA5)*s+QA4)*s+QA3)*s+QA2)*s+QA1)*s + 1
       */

      __asm__ volatile
      (
        "ae_movda32   aed0, %2\n"      /* aed0 = s                          */

      /* P1(s) */

        "movi         a8, 0xbb0df9c0\n"/* PA6 = -0.002166375518             */
        "ae_movda32   aed1, a8\n"
        "movi         a8, 0x3d1151b3\n"/* PA5 = 0.03547830507               */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xbde31cc2\n"/* PA4 = -0.1108946949               */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0x3ea2fe54\n"/* PA3 = 0.3183466196                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xbebe9208\n"/* PA2 = -0.37220788                 */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0x3ed46805\n"/* PA1 = 0.414856106                 */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xbb1acdc6\n"/* PA0 = -0.002362118568             */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"  /* aed1 = P1(s)                  */
        "ae_movad32.l %0, aed1\n"

      /* Q1(s) */

        "movi         a8, 0x3c445aa3\n"/* QA6 = 0.0119845001                */
        "ae_movda32   aed2, a8\n"
        "movi         a8, 0x3c5f6e13\n"/* QA5 = 0.01363708358               */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "movi         a8, 0x3e013307\n"/* QA4 = 0.1261712164                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed3, aed0\n"
        "movi         a8, 0x3d931ae7\n"/* QA3 = 0.0718286559                */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "movi         a8, 0x3f0a5785\n"/* QA2 = 0.5403979421                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed3, aed0\n"
        "movi         a8, 0x3dd9f331\n"/* QA1 = 0.106420882                 */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "const.s      aed2, 1\n"
        "madd.s       aed2, aed3, aed0\n"  /* aed2 = Q1(s)                  */
        "ae_movad32.l %1, aed2\n"
        : "=r" (num), "=r" (den)
        : "r" (s)
        : "a8"
      );

      ux.u = 0x3f58560bu;              /* erx = 0.8450629115 */
      if (hx >= 0)
        {
          return ux.f + num / den;
        }
      else
        {
          return -ux.f - num / den;
        }
    }

  /* ---- |x| >= 6 : erf = +-1 (rounded just inside) ---- */

  if (ix >= 0x40c00000)
    {
      return hx >= 0 ? 1.0f - 1e-30f : 1e-30f - 1.0f;
    }

  /* ---- 1.25 <= |x| < 6 : erfc tail ---- */

  xa = fabsf(x);
  s  = 1.0f / (xa * xa);

  if (ix < 0x4036db6e)                 /* |x| < 1/0.35 ~= 2.857 */
    {
      /* Ra(s) deg 7, Sa(s) deg 8 */

      __asm__ volatile
      (
        "ae_movda32   aed0, %2\n"      /* aed0 = s                          */

      /* Ra(s) */

        "movi         a8, 0xc11d077e\n"/* RA7 = -9.814329147                */
        "ae_movda32   aed1, a8\n"
        "movi         a8, 0xc2a2932b\n"/* RA6 = -81.28743744                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xc3389ae7\n"/* RA5 = -184.6050873                */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0xc322658c\n"/* RA4 = -162.3966675                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xc2798057\n"/* RA3 = -62.37533188                */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0xc128f022\n"/* RA2 = -10.55862617                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xbf31a0b7\n"/* RA1 = -0.6938585639               */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0xbc21a093\n"/* RA0 = -0.009864944033             */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"  /* aed2 = Ra(s)                  */
        "ae_movad32.l %0, aed2\n"

      /* Sa(s) */

        "movi         a8, 0xbd777f97\n"/* SA8 = -0.06042441353              */
        "ae_movda32   aed1, a8\n"
        "movi         a8, 0x40d23f7c\n"/* SA7 = 6.570249557                 */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed1, aed0\n"
        "movi         a8, 0x42d9451f\n"/* SA6 = 108.6350021                 */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed3, aed0\n"
        "movi         a8, 0x43d6810b\n"/* SA5 = 429.0081482                 */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed1, aed0\n"
        "movi         a8, 0x442158c9\n"/* SA4 = 645.3872681                 */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed3, aed0\n"
        "movi         a8, 0x43d9486f\n"/* SA3 = 434.5658875                 */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed1, aed0\n"
        "movi         a8, 0x4309a863\n"/* SA2 = 137.6577606                 */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed3, aed0\n"
        "movi         a8, 0x419d35ce\n"/* SA1 = 19.65127182                 */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed1, aed0\n"
        "const.s      aed1, 1\n"
        "madd.s       aed1, aed3, aed0\n"  /* aed1 = Sa(s)                  */
        "ae_movad32.l %1, aed1\n"
        : "=r" (num), "=r" (den)
        : "r" (s)
        : "a8"
      );
    }
  else                                 /* 2.857 <= |x| < 6 */
    {
      /* Rb(s) deg 6, Sb(s) deg 7 */

      __asm__ volatile
      (
        "ae_movda32   aed0, %2\n"      /* aed0 = s                          */

      /* Rb(s) */

        "movi         a8, 0xc3f1c275\n"/* RB6 = -483.5191956                */
        "ae_movda32   aed1, a8\n"
        "movi         a8, 0xc480230b\n"/* RB5 = -1025.095093                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xc41f6441\n"/* RB4 = -637.5664673                */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0xc320a2ea\n"/* RB3 = -160.6363831                */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xc18e104b\n"/* RB2 = -17.75795555                */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"
        "movi         a8, 0xbf4c9dd4\n"/* RB1 = -0.7992832661               */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed1, aed0\n"
        "movi         a8, 0xbc21a092\n"/* RB0 = -0.009864943102             */
        "ae_movda32   aed1, a8\n"
        "madd.s       aed1, aed2, aed0\n"  /* aed1 = Rb(s)                  */
        "ae_movad32.l %0, aed1\n"

      /* Sb(s) */

        "movi         a8, 0xc1b38712\n"/* SB7 = -22.4409523                 */
        "ae_movda32   aed2, a8\n"
        "movi         a8, 0x43ed43a7\n"/* SB6 = 474.5285339                 */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "movi         a8, 0x451f90ce\n"/* SB5 = 2553.050293                 */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed3, aed0\n"
        "movi         a8, 0x4547fdbb\n"/* SB4 = 3199.858154                 */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "movi         a8, 0x44c01759\n"/* SB3 = 1536.729614                 */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed3, aed0\n"
        "movi         a8, 0x43a2e571\n"/* SB2 = 325.792511                  */
        "ae_movda32   aed3, a8\n"
        "madd.s       aed3, aed2, aed0\n"
        "movi         a8, 0x41f2b459\n"/* SB1 = 30.33806038                 */
        "ae_movda32   aed2, a8\n"
        "madd.s       aed2, aed3, aed0\n"
        "const.s      aed3, 1\n"
        "madd.s       aed3, aed2, aed0\n"  /* aed3 = Sb(s)                  */
        "ae_movad32.l %1, aed3\n"
        : "=r" (num), "=r" (den)
        : "r" (s)
        : "a8"
      );
    }

  r = num / den;

  /* z = |x| with the low 13 mantissa bits cleared, so z*z is exactly
   * representable (24-bit significand split 11+13) and the leading
   * exp(-z*z-0.5625) factor is computed without rounding error.  The fdlibm
   * canonical mask is 0xffffe000 (13 bits); 0xfffff000 (12 bits) leaves z*z
   * inexact near x~3.93 and costs ~16 ULP there.
   */

  ux.f = xa;
  ux.u &= 0xffffe000u;
  z = ux.f;

  r = expf(-z * z - 0.5625f) * expf((z - xa) * (z + xa) + r);

  return hx >= 0 ? 1.0f - r / xa : r / xa - 1.0f;
}
#endif
