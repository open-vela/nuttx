/****************************************************************************
 * libs/libm/libm/lib_hypotf.c
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

#include <math.h>

#ifndef CONFIG_LIBM_ARCH_HYPOTF

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hypotf
 *
 * Description:
 *   Compute sqrt(x*x + y*y) without undue overflow or underflow, by scaling
 *   out the larger magnitude:  m = max(|x|,|y|), t = min/m,
 *   result = m * sqrtf(1 + t*t).  sqrtf may itself be architecture
 *   accelerated.
 *
 *   Special cases follow IEEE / C99:
 *     hypot(+-Inf, y) = +Inf  even when y is NaN  (and symmetric)
 *     either NaN (no Inf)     -> NaN
 *     hypot(x, 0)             = |x|
 *
 ****************************************************************************/

float hypotf(float x, float y)
{
  float ax = fabsf(x);
  float ay = fabsf(y);
  float m;
  float n;
  float t;

  /* Inf in either argument dominates, even against a NaN. */

  if (isinf(ax) || isinf(ay))
    {
      return (float)INFINITY;
    }

  if (isnan(ax) || isnan(ay))
    {
      return (float)NAN;
    }

  m = ax > ay ? ax : ay;
  n = ax > ay ? ay : ax;

  if (m == 0.0f)
    {
      return 0.0f;
    }

  t = n / m;
  return m * sqrtf(1.0f + t * t);
}

#endif /* CONFIG_LIBM_ARCH_HYPOTF */
