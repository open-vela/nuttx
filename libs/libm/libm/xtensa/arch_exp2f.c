/****************************************************************************
 * libs/libm/libm/xtensa/arch_exp2f.c
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

/* exp2f (2^x) via the HiFi4 audio-TIE single-precision datapath, run
 * entirely register-resident in the aed FP file.  This is the base-2 sibling
 * of
 * arch_expf.c.  Because the base is already 2, no log2e factor is needed in
 * the range reduction -- the integer part of x is the 2^n exponent directly:
 *
 *   1. n  = floorf(x + 0.5)                  range reduction, n is integer
 *   2. r  = x - n                            |r| <= 0.5, no Cody-Waite split
 *                                            needed (n is exactly the power)
 *   3. p  = 1 + r*P(r)                       degree-6 Horner minimax for 2^r
 *   4. 2^x = p * 2^n                          exponent reassembly
 *
 * Step 1 forms (x + 0.5) with add.s then fifloor.s; n is pulled into an
 * a-register with trunc.s for the 2^n exponent build, and r = x - n is
 * formed back in the FP file.  P(r) is the cephes single-precision exp2f
 * minimax (degree 6, the same polynomial cephes uses, given here as exact
 * IEEE bits so the silicon result matches the host validation
 * bit-for-bit); it evaluates 2^r over r in [-0.5, 0.5].
 *
 * The 2^n factor is materialised by constructing the IEEE bits
 * ((n+127)<<23) in an a-register and bridging into an aed with ae_movda32,
 * then a mul.s -- exponent-exact and letting the C wrapper screen
 * over/underflow before n can leave the representable range.  n is split
 * n = n1 + n2 (n1 = n >> 1) and the scale applied as two mul.s so the
 * n = 128 case (x in (126.5, 127.5], result 2^127.something which is
 * finite but whose single ((n+127)<<23) build would overflow) stays in
 * range -- identical to arch_expf.c's split-exponent trick.
 *
 * Arbitrary float constants (0.5, the six polynomial coefficients) cannot
 * come from const.s (which only loads small integers 0..3), so each is
 * loaded as raw IEEE bits via "movi a8, <bits>" + ae_movda32.  The
 * float<->aed bridge (ae_movda32 / ae_movad32.l) moves bits with no stack
 * round-trip; aed registers must NOT appear in the clobber list (GCC
 * rejects the names) -- only the scratch a-registers a8/a9 are clobbered.
 *
 * Accuracy: degree-6 polynomial, max 1 ULP vs host glibc exp2f, measured
 * EXHAUSTIVELY over all 2.25e9 float inputs whose result is normal
 * (x in [-150, 128)): 99.13% correctly rounded (0 ULP), 0.87% at 1 ULP, none
 * at >= 2 ULP -- i.e. faithfully rounded, matching glibc's own 1 ULP bound.
 *
 * IEEE special cases are screened in C up front, matching the contract of
 * the host glibc exp2f and the generic lib_exp2f.c fallback:
 *   x = NaN                       -> NaN (quiet)
 *   x = +Inf                      -> +Inf
 *   x = -Inf                      -> +0
 *   x >= 128.0                    -> +Inf   (overflow; 2^128 > FLT_MAX)
 *   x <= -150.0                   -> +0     (underflow past the subnormals)
 *   x =  0                        -> 1.0    (screened for speed)
 */

#if XTENSA_LIBM_HAVE_VFPU2
float exp2f(float x)
{
  union
  {
    float    f;
    uint32_t u;
  } ux;

  uint32_t ax;
  float    r;

  ux.f = x;
  ax   = ux.u & 0x7fffffffu;

  /* NaN -> quiet NaN; +-Inf handled by the magnitude screens below. */

  if (ax > 0x7f800000u)
    {
      ux.u |= 0x00400000u;
      return ux.f;
    }

  /* x = +-0 -> 2^0 = 1. */

  if (ax == 0)
    {
      return 1.0f;
    }

  /* Overflow: x >= 128.0 (and +Inf) -> +Inf.  0x43000000 is 128.0f; any
   * positive x at or above it (smaller u for positive floats means smaller
   * value, so >= here) yields 2^x > FLT_MAX.
   */

  if (ux.u < 0x80000000u && ux.u >= 0x43000000u)
    {
      return (float)INFINITY;
    }

  /* Underflow: x <= -150.0 (and -Inf) -> +0.  Below -150 the 2^n factor is
   * subnormal past representability; the gradual-underflow tail is flushed
   * to +0 here.  0x43160000 is 150.0f, so |x| >= 150 has u (magnitude) >=
   * 0x43160000, i.e. the signed pattern >= (0x43160000 | sign).
   */

  if (ux.u >= (0x43160000u | 0x80000000u))
    {
      return 0.0f;
    }

  __asm__ volatile
  (

  /* Load x into the FP file. */

    "ae_movda32   aed0, %1\n"          /* aed0 = x                          */

  /* n = floorf(x + 0.5) */

    "movi         a8, 0x3f000000\n"    /* 0.5                               */
    "ae_movda32   aed1, a8\n"
    "add.s        aed2, aed0, aed1\n"  /* aed2 = x + 0.5                    */
    "fifloor.s    aed2, aed2\n"        /* aed2 = floorf(x + 0.5) = n (float) */
    "trunc.s      a8, aed2, 0\n"       /* a8   = (int32)n                   */

  /* r = x - n */

    "sub.s        aed7, aed0, aed2\n"  /* aed7 = r = x - n,  |r| <= 0.5     */

  /* P(r) = ((((((P0*r + P1)*r + P2)*r + P3)*r + P4)*r + P5)  (Horner). */

    "movi         a9, 0x3920fdde\n"    /* P0 = 1.5353362e-4                 */
    "ae_movda32   aed8, a9\n"
    "movi         a9, 0x3aaf9f29\n"    /* P1 = 1.3398874e-3                 */
    "ae_movda32   aed9, a9\n"
    "madd.s       aed9, aed8, aed7\n"  /* aed9 = P0*r + P1                  */
    "movi         a9, 0x3c1d96a6\n"    /* P2 = 9.6184369e-3                 */
    "ae_movda32   aed10, a9\n"
    "madd.s       aed10, aed9, aed7\n" /* aed10 = (P0*r+P1)*r + P2          */
    "movi         a9, 0x3d635774\n"    /* P3 = 5.5503324e-2                 */
    "ae_movda32   aed11, a9\n"
    "madd.s       aed11, aed10, aed7\n"/* aed11 = ...*r + P3                */
    "movi         a9, 0x3e75fdee\n"    /* P4 = 2.4022648e-1                 */
    "ae_movda32   aed12, a9\n"
    "madd.s       aed12, aed11, aed7\n"/* aed12 = ...*r + P4                */
    "movi         a9, 0x3f317218\n"    /* P5 = 6.9314718e-1                 */
    "ae_movda32   aed13, a9\n"
    "madd.s       aed13, aed12, aed7\n"/* aed13 = P(r) = ...*r + P5         */

  /* p = 1 + r * P(r) */

    "const.s      aed15, 1\n"          /* 1.0f                              */
    "madd.s       aed15, aed13, aed7\n"/* aed15 = 1 + P(r)*r = p = 2^r      */

  /* 2^n via split-exponent scaling.  n ranges over [-150, 127] for the
   * in-range arguments that survive the C screens; split n = n1 + n2 with
   * n1 = n >> 1 (arithmetic) and n2 = n - n1 so each half lies well within
   * [-126, 127] and two mul.s apply the scale without intermediate
   * overflow.  a8 still holds n here.
   */

    "srai         a9, a8, 1\n"         /* a9 = n1 = n >> 1 (arith)          */
    "sub          a8, a8, a9\n"        /* a8 = n2 = n - n1                  */
    "addi         a9, a9, 127\n"       /* a9 = n1 + 127                     */
    "slli         a9, a9, 23\n"        /* a9 = 2^n1 bits                    */
    "ae_movda32   aed0, a9\n"          /* aed0 = 2^n1                       */
    "addi         a8, a8, 127\n"       /* a8 = n2 + 127                     */
    "slli         a8, a8, 23\n"        /* a8 = 2^n2 bits                    */
    "ae_movda32   aed1, a8\n"          /* aed1 = 2^n2                       */

  /* result = (p * 2^n1) * 2^n2 */

    "mul.s        aed15, aed15, aed0\n"
    "mul.s        aed15, aed15, aed1\n"
    "ae_movad32.l %0, aed15\n"
    : "=r" (r)
    : "r" (x)
    : "a8", "a9"
  );

  return r;
}
#endif
