/****************************************************************************
 * libs/libm/libm/lib_erfcf.c
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

/* The polynomial domains and coefficients below are the Sun/fdlibm
 * single-precision erf/erfc scheme (sf_erf.c), the same source the
 * accelerated arch_erff.c was ported from:
 *
 *   ====================================================
 *   Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 *   Developed at SunPro, a Sun Microsystems, Inc. business.
 *   Permission to use, copy, modify, and distribute this software is
 *   freely granted, provided that this notice is preserved.
 *   ====================================================
 *
 * WHY THIS IS NOT erfcf(x) = 1 - erff(x):  for x > ~1 that subtraction
 * loses all significance.  At x = 2, erff(2) ~= 0.9953, so 1 - 0.9953 in
 * single precision suffers catastrophic cancellation (~47 ULP); by x = 4,
 * erff rounds to 1.0f and 1 - 1 = 0, while the true erfcf(4) ~= 1.54e-8.
 * erfc must be evaluated directly via its tail, exp(-x*x - 0.5625 + R/S)/x,
 * for x in the large-argument domains -- exactly the form fdlibm uses and
 * arch_erff.c already encodes for its erf tail.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>
#include <math.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* fdlibm sf_erf.c single-precision constants. */

static const float tiny = 1e-30f;
static const float erx  = 8.4506291151e-01f;  /* 0x3f58560b */

/* Coefficients for approximation to erf on [0, 0.84375] */

static const float efx  = 1.2837916613e-01f;  /* 0x3e0375d4 */
static const float efx8 = 1.0270333290e00f;   /* 0x3f8375d4 */
static const float pp0  = 1.2837916613e-01f;  /* 0x3e0375d4 */
static const float pp1  = -3.2504209876e-01f; /* 0xbea66beb */
static const float pp2  = -2.8481749818e-02f; /* 0xbce9528f */
static const float pp3  = -5.7702702470e-03f; /* 0xbbbd1489 */
static const float pp4  = -2.3763017452e-05f; /* 0xb7c756b1 */
static const float qq1  = 3.9791721106e-01f;  /* 0x3ecbbbce */
static const float qq2  = 6.5022252500e-02f;  /* 0x3d852a63 */
static const float qq3  = 5.0813062117e-03f;  /* 0x3ba68116 */
static const float qq4  = 1.3249473704e-04f;  /* 0x390aee49 */
static const float qq5  = -3.9602282413e-06f; /* 0xb684e21a */

/* Coefficients for approximation to erf on [0.84375, 1.25] */

static const float pa0  = -2.3621185683e-03f; /* 0xbb1acdc6 */
static const float pa1  = 4.1485610604e-01f;  /* 0x3ed46805 */
static const float pa2  = -3.7220788002e-01f; /* 0xbebe9208 */
static const float pa3  = 3.1834661961e-01f;  /* 0x3ea2fe54 */
static const float pa4  = -1.1089469492e-01f; /* 0xbde31cc2 */
static const float pa5  = 3.5478305072e-02f;  /* 0x3d1151b3 */
static const float pa6  = -2.1663755178e-03f; /* 0xbb0df9c0 */
static const float qa1  = 1.0642088205e-01f;  /* 0x3dd9f331 */
static const float qa2  = 5.4039794207e-01f;  /* 0x3f0a5785 */
static const float qa3  = 7.1828655899e-02f;  /* 0x3d931ae7 */
static const float qa4  = 1.2617121637e-01f;  /* 0x3e013307 */
static const float qa5  = 1.3637083583e-02f;  /* 0x3c5f6e13 */
static const float qa6  = 1.1984500103e-02f;  /* 0x3c445aa3 */

/* Coefficients for approximation to erfc on [1.25, 1/0.35] */

static const float ra0  = -9.8649440333e-03f; /* 0xbc21a093 */
static const float ra1  = -6.9385856390e-01f; /* 0xbf31a0b7 */
static const float ra2  = -1.0558626175e01f;  /* 0xc128f022 */
static const float ra3  = -6.2375331879e01f;  /* 0xc2798057 */
static const float ra4  = -1.6239666748e02f;  /* 0xc322658c */
static const float ra5  = -1.8460508728e02f;  /* 0xc3389ae7 */
static const float ra6  = -8.1287437439e01f;  /* 0xc2a2932b */
static const float ra7  = -9.8143291473e00f;  /* 0xc11d077e */
static const float sa1  = 1.9651271820e01f;   /* 0x419d35ce */
static const float sa2  = 1.3765776062e02f;   /* 0x4309a863 */
static const float sa3  = 4.3456588745e02f;   /* 0x43d9486f */
static const float sa4  = 6.4538726807e02f;   /* 0x442158c9 */
static const float sa5  = 4.2900814819e02f;   /* 0x43d6810b */
static const float sa6  = 1.0863500214e02f;   /* 0x42d9451f */
static const float sa7  = 6.5702495575e00f;   /* 0x40d23f7c */
static const float sa8  = -6.0424413532e-02f; /* 0xbd777f97 */

/* Coefficients for approximation to erfc on [1/0.35, 28] */

static const float rb0  = -9.8649431020e-03f; /* 0xbc21a092 */
static const float rb1  = -7.9928326607e-01f; /* 0xbf4c9dd4 */
static const float rb2  = -1.7757955551e01f;  /* 0xc18e104b */
static const float rb3  = -1.6063638306e02f;  /* 0xc320a2ea */
static const float rb4  = -6.3756646729e02f;  /* 0xc41f6441 */
static const float rb5  = -1.0250950928e03f;  /* 0xc480230b */
static const float rb6  = -4.8351919556e02f;  /* 0xc3f1c275 */
static const float sb1  = 3.0338060379e01f;   /* 0x41f2b459 */
static const float sb2  = 3.2579251099e02f;   /* 0x43a2e571 */
static const float sb3  = 1.5367296143e03f;   /* 0x44c01759 */
static const float sb4  = 3.1998581543e03f;   /* 0x4547fdbb */
static const float sb5  = 2.5530502930e03f;   /* 0x451f90ce */
static const float sb6  = 4.7452853394e02f;   /* 0x43ed43a7 */
static const float sb7  = -2.2440952301e01f;  /* 0xc1b38712 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

float erfcf(float x)
{
  union
  {
    float    f;
    uint32_t u;
  } gf;

  int32_t  hx;
  int32_t  ix;
  float    r;
  float    s;
  float    z;
  float    p;
  float    q;
  float    xa;

  gf.f = x;
  hx   = (int32_t)gf.u;
  ix   = hx & 0x7fffffff;

  /* Inf / NaN: erfc(+inf) = 0, erfc(-inf) = 2, NaN -> NaN. */

  if (ix >= 0x7f800000)
    {
      /* erfc(NaN) = NaN; erfc(+-inf): (hx >> 31) is 0 for +inf, -1 for
       * -inf, so (1 - 2*sign) yields 0 for +inf and 2 for -inf, plus a
       * vanishing 1/x term that also propagates NaN unchanged.
       */

      return (float)(((uint32_t)hx >> 31) << 1) + 1.0f / x;
    }

  /* ---- |x| < 0.84375 ---- */

  if (ix < 0x3f580000)
    {
      if (ix < 0x23800000)               /* |x| < 2^-56: erfc(x) = 1 - x */
        {
          return 1.0f - x;
        }

      z = x * x;
      r = pp0 + z * (pp1 + z * (pp2 + z * (pp3 + z * pp4)));
      s = 1.0f + z * (qq1 + z * (qq2 + z * (qq3 + z * (qq4 + z * qq5))));
      r = r / s;

      if (hx < 0x3e800000)               /* |x| < 0.25 */
        {
          return 1.0f - (x + x * r);
        }
      else
        {
          r = x * r;
          r += (x - 0.5f);
          return 0.5f - r;
        }
    }

  /* ---- 0.84375 <= |x| < 1.25 ---- */

  if (ix < 0x3fa00000)
    {
      s = fabsf(x) - 1.0f;
      p = pa0 + s * (pa1 + s * (pa2 + s * (pa3 + s * (pa4 +
              s * (pa5 + s * pa6)))));
      q = 1.0f + s * (qa1 + s * (qa2 + s * (qa3 + s * (qa4 +
              s * (qa5 + s * qa6)))));

      if (hx >= 0)
        {
          z = 1.0f - erx;
          return z - p / q;
        }
      else
        {
          z = erx + p / q;
          return 1.0f + z;
        }
    }

  /* ---- |x| < 28 : erfc tail ---- */

  if (ix < 0x41e00000)
    {
      xa = fabsf(x);
      s  = 1.0f / (xa * xa);

      if (ix < 0x4036db6d)               /* |x| < 1/0.35 ~= 2.857143 */
        {
          r = ra0 + s * (ra1 + s * (ra2 + s * (ra3 + s * (ra4 +
                  s * (ra5 + s * (ra6 + s * ra7))))));
          s = 1.0f + s * (sa1 + s * (sa2 + s * (sa3 + s * (sa4 +
                  s * (sa5 + s * (sa6 + s * (sa7 + s * sa8)))))));
        }
      else                               /* |x| >= 1/0.35 */
        {
          if (hx < 0 && ix >= 0x40c00000)
            {
              return 2.0f - tiny;        /* x < -6 */
            }

          r = rb0 + s * (rb1 + s * (rb2 + s * (rb3 + s * (rb4 +
                  s * (rb5 + s * rb6)))));
          s = 1.0f + s * (sb1 + s * (sb2 + s * (sb3 + s * (sb4 +
                  s * (sb5 + s * (sb6 + s * sb7))))));
        }

      /* z = |x| with the low 13 mantissa bits cleared (mask 0xffffe000, the
       * canonical musl/fdlibm sf_erf.c value), leaving a 10-bit mantissa so
       * z*z is exact and the leading exp(-z*z - 0.5625) factor is computed
       * without rounding error.  (Clearing only 12 bits leaves z with 11
       * mantissa bits, whose square overflows 23 bits and reintroduces ~16
       * ULP of error in the tail near |x| ~= 3.93.)
       */

      gf.f  = xa;
      gf.u &= 0xffffe000u;
      z     = gf.f;

      r = expf(-z * z - 0.5625f) *
          expf((z - xa) * (z + xa) + r / s);

      if (hx >= 0)
        {
          return r / xa;
        }
      else
        {
          return 2.0f - r / xa;
        }
    }

  /* ---- |x| >= 28 : underflow / saturate ---- */

  if (hx >= 0)
    {
      return tiny * tiny;                /* +0 (rounds correctly) */
    }
  else
    {
      return 2.0f - tiny;                /* 2 */
    }
}
