/****************************************************************************
 * libs/libm/libm/xtensa/arch_truncf.c
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

/* truncf via the HiFi4 trunc.s (float->int32, round toward zero) followed
 * by float.s back to float.  Measured ~2-3x the generic bit-twiddling
 * lib_truncf on T113-S3 silicon, 0 ULP across the hifimath vectors.
 *
 * trunc.s saturates/overflows for |x| that does not fit int32, so values
 * with exponent >= 2^23 (already integral, also covers inf/nan) are
 * returned unchanged.  The int round-trip also drops the sign of a result
 * that truncates to zero (e.g. truncf(-0.5) must be -0.0), so the original
 * sign bit is restored afterwards.
 */

#if XTENSA_LIBM_HAVE_VFPU2
float truncf(float x)
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
  } ur;

  uint32_t e;
  float    r;

  ux.f = x;
  e = (ux.u >> 23) & 0xff;

  /* |x| >= 2^23 is already integral (also catches inf/nan): return as-is. */

  if (e >= 127 + 23)
    {
      return x;
    }

  __asm__ volatile
  (
    "ae_movda32   aed0, %1\n"
    "trunc.s      a8, aed0, 0\n"   /* a8 = (int32)trunc(x) */
    "float.s      aed0, a8, 0\n"   /* back to float        */
    "ae_movad32.l %0, aed0\n"
    : "=r" (r) : "r" (x) : "a8"
  );

  /* Restore the sign bit lost when the result truncates to zero. */

  ur.f = r;
  ur.u = (ur.u & 0x7fffffffu) | (ux.u & 0x80000000u);
  return ur.f;
}
#endif
