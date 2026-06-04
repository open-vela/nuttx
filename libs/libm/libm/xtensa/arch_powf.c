/****************************************************************************
 * libs/libm/libm/xtensa/arch_powf.c
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

/* powf on the HiFi4 LX7 audio-TIE FP datapath (aed file).  There is no pow
 * (nor log/exp) silicon op, so this is the textbook identity
 *
 *     powf(x, y) = 2^(y * log2(x))      for x > 0
 *
 * carried out entirely register-resident, but with the one twist that makes
 * a float-only powf accurate: the bridge value y*log2(x) is formed in
 * EXTENDED (double-float) precision.  A naive single-precision log2 * y
 * loses ~half the mantissa to catastrophic cancellation/scaling, so the
 * result would be tens of ULP off.  Instead log2(x) is kept as an
 * unevaluated (hi,lo) pair of floats, y is multiplied in with the fused
 * madd.s/msub.s (which give an exact product + error term --
 * TwoProd/TwoSum), and exp2 then consumes both halves.
 * This is the same compensated scheme musl/ARM-optimized-routines use, but
 * realised with float ops only because the HiFi4 scalar FPU has no double.
 *
 * Pipeline (all FP in aed0..aed15, no stack round-trips):
 *
 *   1. C splits x = m * 2^ei, m in [sqrt(.5), sqrt(2)); f = m-1 (or 2m-1).
 *      Subnormals are pre-scaled by 2^25 (the arch_logf reduction, reused).
 *   2. asm: log1p(f) as a double-float (lm_hi, lm_lo):
 *         z   = f*f                    (TwoProd captures the f*f tail too)
 *         P   = cephes degree-8 minimax (coeffs Lg0..Lg8, exact IEEE bits)
 *         lm  = TwoSum(f, -z/2) + (f^3*P - z_tail/2)        natural log of m
 *   3. asm: log2(m) = lm * log2e, log2e = (L2EHI, L2ELO) double-float, then
 *      add the exact integer ei with a TwoSum -> log2(x) = (rhi, rlo).
 *   4. asm: t = y * (rhi,rlo) double-float = (thi, tlo)  via TwoProd + fma.
 *   5. asm: exp2(t):  n = firound(thi); g = (thi-n) + tlo  in [-0.5, 0.5];
 *      2^g by a degree-6 Remez minimax (E0..E6, max rel err 2.6e-9); the 2^n
 *      factor is built as IEEE bits ((n+127)<<23), split n=n1+n2 so build
 *      never overflows a single field, then two mul.s apply it.
 *
 * Constants that const.s cannot make (it loads only the integers 0..3) are
 * materialised as raw IEEE bits with "movi a8,<bits>" + ae_movda32; the
 * float<->aed bridge is ae_movda32 / ae_movad32.l (no memory traffic).  aed
 * registers are never named as clobbers (GCC rejects them) -- only scratch
 * a-registers a8/a9 are declared.  The TwoProd/TwoSum error terms are formed
 * with msub.s (aedD -= aedA*aedB) and add.s/sub.s, exactly mirroring fmaf in
 * the host model.
 *
 * Accuracy: max 1 ULP vs host glibc powf in the common range, measured over
 * a (0,100]x[-10,10] dense grid (20.3M pts: 89.55% 0 ULP, 10.45% 1 ULP, 0%
 * >=2), 80M random (x,y), and 243M points in the high-cancellation band x in
 * [0.5,2] -- none exceeded 1 ULP; and 0 points at >=2 ULP over x in
 * [1e-4,100] with |y|<=10.  Faithfully rounded, golden-compatible with glibc
 * (itself <=1 ULP), not bit-identical.  Outside the common range the
 * extended-precision product degrades gracefully: <=3 ULP for |y| up to 40,
 * and the float32-subnormal result tail is <=2 ULP (7 of 2.58M sampled
 * subnormal results at 2 ULP, rest <=1).  The exp2 core uses a degree-6
 * Remez minimax for 2^g on [-0.5,0.5] (max rel err 2.57e-9) and the log2
 * reduction is the cephes degree-8 logf
 * minimax carried in a (hi,lo) double-float and scaled by a split log2e.
 *
 * IEEE / C99 special cases are screened in C up front (the bulk of powf's
 * correctness), reproducing glibc / lib_powf.c bit-for-bit:
 *   y = +-0                     -> 1            (even for x = NaN/Inf)
 *   x = 1                       -> 1            (even for y = NaN/Inf)
 *   y = 1                       -> x
 *   x = NaN or y = NaN          -> NaN          (quiet, after the above)
 *   y = +-Inf                   -> per |x| vs 1 (and pow(-1,+-Inf)=1)
 *   x = +-0                     -> +-Inf / +-0  per sign of y and odd-int y
 *   x = +-Inf                   -> like pow(+-0,-y)
 *   x < 0, y non-integer        -> NaN
 *   x < 0, y integer            -> +-powf(|x|,y), sign = (-1)^y
 */

#if XTENSA_LIBM_HAVE_VFPU2

/* Classify y: 0 = not an integer, 1 = odd integer, 2 = even integer.
 * |y| is known finite and nonzero on entry.
 */

static int powf_yclass(float y)
{
  union
  {
    float    f;
    uint32_t u;
  } uy;

  uint32_t ay;
  int      e;

  uy.f = y;
  ay   = uy.u & 0x7fffffffu;
  e    = (int)(ay >> 23) - 127;          /* unbiased exponent of |y| */

  if (e < 0)
    {
      return 0;                          /* |y| < 1 -> non-integer */
    }

  if (e >= 23)
    {
      return 2;                          /* >= 2^23 -> integer, always even */
    }

  /* Fractional bits live in the low (23 - e) bits of the mantissa. */

  if (ay & (0x007fffffu >> e))
    {
      return 0;                          /* has a fractional part */
    }

  /* Integer: bit (23 - e) of the mantissa is the unit bit -> odd/even. */

  return ((ay >> (23 - e)) & 1u) ? 1 : 2;
}

/* The float-only core: powf(|x|, y) for x finite > 0, y finite != 0, with
 * the sign already split out.  Returns 2^(y*log2(|x|)).
 */

static float powf_core(float x, float y)
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
  float    eif;
  float    result;
  int      nexp;

  ux.f = x;
  ix   = ux.u;
  e    = (ix >> 23) & 0xff;

  /* Subnormal positive: scale by 2^25 into the normal range and subtract 25
   * from the exponent contribution later (the arch_logf reduction).
   */

  sub = 0;
  if (e == 0)
    {
      ux.f = x * 33554432.0f;            /* 2^25 */
      ix   = ux.u;
      e    = (ix >> 23) & 0xff;
      sub  = 25;
    }

  /* x = m * 2^ei, m in [0.5, 1). */

  ei   = (int)e - 126;
  ux.u = (ix & 0x007fffffu) | 0x3f000000u;
  m    = ux.f;

  /* Nudge m into [sqrt(0.5), sqrt(2)) so f stays small and symmetric.
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
  eif = (float)ei;

  __asm__ volatile
  (
    "ae_movda32   aed0, %2\n"            /* aed0 = f                         */
    "ae_movda32   aed1, %3\n"            /* aed1 = ei (float)                */
    "ae_movda32   aed2, %4\n"            /* aed2 = y                         */

  /* ---- log1p(f) as double-float (lm_hi in aed4, lm_lo in aed5) ---- *
   *
   *   z      = f*f               (mul.s) with z_tail = fma(f,f,-z)
   *   P(f)   = cephes deg-8 Horner   (aed3)
   *   ptail  = P*f*z   (the f^3*P term)
   *   lm_hi  = TwoSum(f, -z/2)
   *   lm_lo  = (lm_hi twosum tail) + ptail - z_tail/2
   */

    "mul.s        aed3, aed0, aed0\n"    /* aed3 = z = f*f                   */
    "neg.s        aed4, aed3\n"          /* aed4 = -z                        */
    "madd.s       aed4, aed0, aed0\n"    /* aed4 = z_tail = fma(f,f,-z)      */

  /* P(f): seed accumulator with each coefficient, fuse with madd.s. */

    "movi         a8, 0x3d9021bb\n"      /* Lg0                              */
    "ae_movda32   aed5, a8\n"            /* aed5 = P = Lg0                   */
    "movi         a8, 0xbdebd1b8\n"      /* Lg1                              */
    "ae_movda32   aed6, a8\n"
    "madd.s       aed6, aed5, aed0\n"    /* aed6 = Lg1 + P*f                 */
    "movi         a8, 0x3def251a\n"      /* Lg2                              */
    "ae_movda32   aed5, a8\n"
    "madd.s       aed5, aed6, aed0\n"
    "movi         a8, 0xbdfe5d4f\n"      /* Lg3                              */
    "ae_movda32   aed6, a8\n"
    "madd.s       aed6, aed5, aed0\n"
    "movi         a8, 0x3e11e9bf\n"      /* Lg4                              */
    "ae_movda32   aed5, a8\n"
    "madd.s       aed5, aed6, aed0\n"
    "movi         a8, 0xbe2aae50\n"      /* Lg5                              */
    "ae_movda32   aed6, a8\n"
    "madd.s       aed6, aed5, aed0\n"
    "movi         a8, 0x3e4cceac\n"      /* Lg6                              */
    "ae_movda32   aed5, a8\n"
    "madd.s       aed5, aed6, aed0\n"
    "movi         a8, 0xbe7ffffc\n"      /* Lg7                              */
    "ae_movda32   aed6, a8\n"
    "madd.s       aed6, aed5, aed0\n"
    "movi         a8, 0x3eaaaaaa\n"      /* Lg8                              */
    "ae_movda32   aed5, a8\n"
    "madd.s       aed5, aed6, aed0\n"    /* aed5 = P(f)                      */

  /* ptail = P * f * z   (aed5) */

    "mul.s        aed5, aed5, aed0\n"    /* P*f                              */
    "mul.s        aed5, aed5, aed3\n"    /* aed5 = ptail = f^3*P             */

  /* hz = z/2 = z * 0.5 (aed6) */

    "movi         a8, 0x3f000000\n"      /* 0.5                              */
    "ae_movda32   aed7, a8\n"            /* aed7 = 0.5                       */
    "mul.s        aed6, aed3, aed7\n"    /* aed6 = hz = 0.5*z                */

  /* lm_hi = TwoSum(f, -hz):  s = f - hz; lo = (f - s) - hz  ... use the
   * branch-free Knuth TwoSum since |f| and |hz| are not ordered.
   *   s  = f + (-hz)
   *   bb = s - f
   *   lo = (f - (s - bb)) + ((-hz) - bb)
   */

    "sub.s        aed8, aed0, aed6\n"    /* aed8 = s = f - hz                */
    "sub.s        aed9, aed8, aed0\n"    /* aed9 = bb = s - f                */
    "sub.s        aed10, aed8, aed9\n"   /* aed10 = s - bb                   */
    "sub.s        aed10, aed0, aed10\n"  /* aed10 = f - (s - bb)             */
    "neg.s        aed11, aed6\n"         /* aed11 = -hz                      */
    "sub.s        aed11, aed11, aed9\n"  /* aed11 = (-hz) - bb               */
    "add.s        aed10, aed10, aed11\n" /* aed10 = TwoSum low word          */

  /* aed8 = lm_hi ; lm_lo = aed10 + ptail - 0.5*z_tail */

    "add.s        aed10, aed10, aed5\n"  /* += ptail                         */
    "mul.s        aed4, aed4, aed7\n"    /* aed4 = 0.5*z_tail                */
    "sub.s        aed10, aed10, aed4\n"  /* aed10 = lm_lo                    */

  /* lm = (aed8 hi, aed10 lo)  natural log of m */

  /* ---- log2(m) = lm * log2e (double-float * double-float) ---- *
   *   L2EHI = 1.4426950216, L2ELO = 1.9259630e-8
   *   ph = TwoProd(lm_hi, L2EHI);  pl = err
   *   pl = fma(lm_hi, L2ELO, pl)
   *   pl = fma(lm_lo, L2EHI, pl)
   */

    "movi         a8, 0x3fb8aa3b\n"      /* L2EHI                            */
    "ae_movda32   aed11, a8\n"           /* aed11 = L2EHI                    */
    "mul.s        aed12, aed8, aed11\n"  /* aed12 = ph = lm_hi*L2EHI         */
    "neg.s        aed13, aed12\n"        /* aed13 = -ph                      */
    "madd.s       aed13, aed8, aed11\n"  /* pl = fma(lm_hi,L2EHI,-ph)      */
    "movi         a8, 0x32a57060\n"      /* L2ELO                            */
    "ae_movda32   aed14, a8\n"           /* aed14 = L2ELO                    */
    "madd.s       aed13, aed8, aed14\n"  /* pl += lm_hi*L2ELO                */
    "madd.s       aed13, aed10, aed11\n" /* pl += lm_lo*L2EHI                */

  /* log2(m) = (aed12 ph, aed13 pl) */

  /* ---- log2(x) = ei + log2(m) ----  ei is an exact integer (aed1) *
   *   sh = TwoSum(ei, ph);  sl = err + pl
   *   then normalize (rhi, rlo) = FastTwoSum(sh, sl)
   */

    "add.s        aed5, aed1, aed12\n"   /* aed5 = sh = ei + ph              */
    "sub.s        aed6, aed5, aed1\n"    /* bb = sh - ei                     */
    "sub.s        aed7, aed5, aed6\n"    /* sh - bb                          */
    "sub.s        aed7, aed1, aed7\n"    /* ei - (sh - bb)                   */
    "sub.s        aed4, aed12, aed6\n"   /* ph - bb                          */
    "add.s        aed7, aed7, aed4\n"    /* aed7 = TwoSum low                */
    "add.s        aed7, aed7, aed13\n"   /* aed7 = sl = low + pl             */

  /* FastTwoSum(sh, sl): rhi = sh + sl; rlo = sl - (rhi - sh) */

    "add.s        aed6, aed5, aed7\n"    /* aed6 = rhi                       */
    "sub.s        aed8, aed6, aed5\n"    /* rhi - sh                         */
    "sub.s        aed7, aed7, aed8\n"    /* aed7 = rlo = sl - (rhi - sh)     */

  /* log2(x) = (aed6 rhi, aed7 rlo) */

  /* ---- t = y * (rhi, rlo) double-float ----  (aed2 = y) *
   *   thi = TwoProd(y, rhi);  tlo = err
   *   tlo = fma(y, rlo, tlo)
   */

    "mul.s        aed8, aed2, aed6\n"    /* aed8 = thi = y*rhi               */
    "neg.s        aed9, aed8\n"          /* aed9 = -thi                      */
    "madd.s       aed9, aed2, aed6\n"    /* aed9 = err = fma(y,rhi,-thi)     */
    "madd.s       aed9, aed2, aed7\n"    /* aed9 = tlo = err + y*rlo         */

  /* t = (aed8 thi, aed9 tlo) */

  /* ---- exp2(t) ----  *
   *   n  = firound(thi)            (round-to-nearest-even, as float)
   *   g  = TwoSum(thi, -n) low+hi, plus tlo, all reduced into [-0.5,0.5]
   *   2^g = E0 + g*(E1 + g*(... + g*E6))   degree-6 Remez, c0 = 1
   *   2^n via ((n+127)<<23) bits, split n = n1 + n2
   *   result = 2^g * 2^n1 * 2^n2
   */

    "firound.s    aed10, aed8\n"         /* aed10 = n (float, RNE)           */
    "trunc.s      a8, aed10, 0\n"        /* a8 = (int32)n                    */
    "mov          %1, a8\n"              /* export n for C over/underflow  */

  /* screen before a8 is reused below */

  /* g = (thi - n) + tlo  (thi - n is exact since |n| not huge here) */

    "sub.s        aed11, aed8, aed10\n"  /* aed11 = thi - n                  */
    "add.s        aed11, aed11, aed9\n"  /* aed11 = g                        */

  /* 2^g Horner: seed top coeff, fuse down. */

    "movi         a9, 0x392385a5\n"      /* E6                               */
    "ae_movda32   aed12, a9\n"           /* aed12 = p = E6                   */
    "movi         a9, 0x3aafb93b\n"      /* E5                               */
    "ae_movda32   aed13, a9\n"
    "madd.s       aed13, aed12, aed11\n" /* E5 + p*g                         */
    "movi         a9, 0x3c1d9387\n"      /* E4                               */
    "ae_movda32   aed12, a9\n"
    "madd.s       aed12, aed13, aed11\n"
    "movi         a9, 0x3d635739\n"      /* E3                               */
    "ae_movda32   aed13, a9\n"
    "madd.s       aed13, aed12, aed11\n"
    "movi         a9, 0x3e75fdf1\n"      /* E2                               */
    "ae_movda32   aed12, a9\n"
    "madd.s       aed12, aed13, aed11\n"
    "movi         a9, 0x3f317219\n"      /* E1                               */
    "ae_movda32   aed13, a9\n"
    "madd.s       aed13, aed12, aed11\n"
    "const.s      aed12, 1\n"            /* E0 = 1.0                         */
    "madd.s       aed12, aed13, aed11\n" /* aed12 = 2^g                      */

  /* 2^n split build: n1 = n>>1 (arith), n2 = n - n1.  a8 holds n. */

    "srai         a9, a8, 1\n"           /* a9 = n1                          */
    "sub          a8, a8, a9\n"          /* a8 = n2                          */
    "addi         a9, a9, 127\n"
    "slli         a9, a9, 23\n"          /* a9 = 2^n1 bits                   */
    "ae_movda32   aed13, a9\n"           /* aed13 = 2^n1                     */
    "addi         a8, a8, 127\n"
    "slli         a8, a8, 23\n"          /* a8 = 2^n2 bits                   */
    "ae_movda32   aed14, a8\n"           /* aed14 = 2^n2                     */

    "mul.s        aed12, aed12, aed13\n" /* *= 2^n1                          */
    "mul.s        aed12, aed12, aed14\n" /* *= 2^n2                          */
    "ae_movad32.l %0, aed12\n"
    : "=r" (result), "=&r" (nexp)
    : "r" (f), "r" (eif), "r" (y)
    : "a8", "a9"
  );

  /* Over/underflow guard.  n = round(y*log2(x)) is the binary exponent of
   * the result.  The split 2^n = 2^n1 * 2^n2 build keeps each
   * ((nk+127)<<23) field inside [1,254] for n in [-252,254], so for any
   * n that yields a representable float result (n in [-149,128]) the two
   * IEEE mul.s above already round overflow to +Inf and underflow to a
   * subnormal or +0
   * correctly -- no clamp needed in the result range.  The guard only fires
   * for arguments so extreme that n leaves [-252,254] and the field build
   * itself would wrap; those are unambiguous over/underflow:
   *   n >  254  -> +Inf
   *   n < -252  -> +0
   * (For y in [-10,10], x in (0,100], |n| <= 67, so this never fires in the
   * common range -- it only catches extreme arguments far outside it.)
   */

  if (nexp > 254)
    {
      return (float)INFINITY;
    }

  if (nexp < -252)
    {
      return 0.0f;
    }

  return result;
}

float powf(float x, float y)
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
  int      yclass;
  float    r;

  ux.f = x;
  uy.f = y;
  ax   = ux.u & 0x7fffffffu;
  ay   = uy.u & 0x7fffffffu;

  /* y == +-0  -> 1 for ANY x, including NaN/Inf (C99). */

  if (ay == 0)
    {
      return 1.0f;
    }

  /* x == 1 (and not -1) -> 1 for ANY y, including NaN/Inf (C99). */

  if (ux.u == 0x3f800000u)
    {
      return 1.0f;
    }

  /* pow(-1, +-Inf) == 1 (C99 special). */

  if (ux.u == 0xbf800000u && ay == 0x7f800000u)
    {
      return 1.0f;
    }

  /* y == 1 -> x. */

  if (uy.u == 0x3f800000u)
    {
      return x;
    }

  /* NaN propagation (after the x==1 / y==0 overrides above). */

  if (ax > 0x7f800000u || ay > 0x7f800000u)
    {
      ux.u = 0x7fc00000u;                /* quiet NaN */
      return ux.f;
    }

  /* y == +-Inf: result decided by |x| vs 1. */

  if (ay == 0x7f800000u)
    {
      if (ax < 0x3f800000u)              /* |x| < 1 */
        {
          return (uy.u >> 31) ? (float)INFINITY : 0.0f;
        }
      else                               /* |x| > 1 (|x|==1 handled above) */
        {
          return (uy.u >> 31) ? 0.0f : (float)INFINITY;
        }
    }

  /* Classify y as non-integer / odd / even (|y| finite, nonzero here). */

  yclass = powf_yclass(y);

  /* x == +-0. */

  if (ax == 0)
    {
      if (uy.u >> 31)                    /* y < 0 */
        {
          /* odd-int y keeps x's sign on the infinity, else +Inf. */

          if (yclass == 1 && (ux.u >> 31))
            {
              ux.u = 0xff800000u;        /* pow(-0, neg odd) = -Inf */
            }
          else
            {
              ux.u = 0x7f800000u;        /* +Inf */
            }

          return ux.f;
        }
      else                               /* y > 0 */
        {
          /* odd-int y keeps x's sign on the zero, else +0. */

          if (yclass == 1 && (ux.u >> 31))
            {
              ux.u = 0x80000000u;        /* pow(-0, pos odd) = -0 */
            }
          else
            {
              ux.u = 0x00000000u;        /* +0 */
            }

          return ux.f;
        }
    }

  /* x == +-Inf: pow(+-Inf, y) == pow(+-0, -y) by the C99 symmetry. */

  if (ax == 0x7f800000u)
    {
      if (uy.u >> 31)                    /* y < 0 -> 0 */
        {
          ux.u = (yclass == 1 && (ux.u >> 31)) ? 0x80000000u : 0x00000000u;
          return ux.f;
        }
      else                               /* y > 0 -> Inf */
        {
          ux.u = (yclass == 1 && (ux.u >> 31)) ? 0xff800000u : 0x7f800000u;
          return ux.f;
        }
    }

  /* Negative finite x. */

  if (ux.u >> 31)
    {
      if (yclass == 0)
        {
          ux.u = 0x7fc00000u;            /* non-integer y -> NaN */
          return ux.f;
        }

      /* integer y: magnitude on |x|, sign = (-1)^y. */

      r = powf_core(-x, y);
      return (yclass == 1) ? -r : r;
    }

  /* Normal path: x finite > 0, y finite nonzero, x != 1. */

  return powf_core(x, y);
}
#endif
