/****************************************************************************
 * libs/libm/libm/xtensa/arch_rintf.c
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

/* rintf rounds to nearest, ties to EVEN.  The HiFi4 firound.s instruction
 * rounds ties AWAY from zero (it is the roundf primitive, silicon-verified),
 * so rintf instead uses the classic add/sub-2^23 trick: adding then
 * subtracting 2^23 forces the value through the add.s round-to-nearest-even
 * datapath, snapping it to an integer.  The magnitude 2^23 is signed to
 * match x so the rounding stays symmetric, |x| >= 2^23 is already integral
 * (and also covers inf/nan), and the sign of a zero result is restored.
 */

#if XTENSA_LIBM_HAVE_VFPU2
float rintf(float x)
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

  /* |x| >= 2^23 already integral; inf/nan pass through. */

  if (e >= 127 + 23)
    {
      return x;
    }

  __asm__ volatile
  (
    "movi         a8, 0x4b000000\n"     /* 2^23 bits              */
    "ae_movda32   aed1, a8\n"           /* aed1 = 2^23            */
    "ae_movda32   aed0, %1\n"           /* aed0 = x               */
    "abs.s        aed2, aed0\n"         /* |x|                    */
    "add.s        aed2, aed2, aed1\n"   /* |x| + 2^23 (RNE)       */
    "sub.s        aed2, aed2, aed1\n"   /* - 2^23 -> rint(|x|)    */
    "ae_movad32.l %0, aed2\n"
    : "=r" (r) : "r" (x) : "a8"
  );

  /* r is the rounded magnitude; restore the sign of x (handles -0 too). */

  ur.f = r;
  ur.u = (ur.u & 0x7fffffffu) | (ux.u & 0x80000000u);
  return ur.f;
}
#endif
