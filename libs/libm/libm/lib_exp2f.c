/****************************************************************************
 * libs/libm/libm/lib_exp2f.c
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

#ifndef CONFIG_LIBM_ARCH_EXP2F

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exp2f
 *
 * Description:
 *   Compute 2 raised to the power x.  Range-reduce to 2^x = 2^n * 2^r with
 *   n = floorf(x + 0.5) an integer and r = x - n in [-0.5, 0.5]; evaluate
 *   2^r with the cephes degree-6 minimax polynomial and scale by 2^n.  This
 *   mirrors lib_expf.c's style with the base-2 reduction (no log2e factor),
 *   and is faithful to ~1 ULP versus the simpler but less accurate
 *   expf(x * ln2) form (whose argument rounding costs tens of ULP).
 *
 ****************************************************************************/

float exp2f(float x)
{
  float n;
  float r;
  float p;

  if (isnan(x))
    {
      return x;
    }

  if (x >= 128.0f)
    {
      return (float)INFINITY;
    }

  if (x <= -150.0f)
    {
      return 0.0f;
    }

  n = floorf(x + 0.5f);
  r = x - n;

  /* 2^r, cephes exp2f degree-6 minimax over r in [-0.5, 0.5]. */

  p = 1.535336188319500e-4f;
  p = p * r + 1.339887440266574e-3f;
  p = p * r + 9.618437357674640e-3f;
  p = p * r + 5.550332471162809e-2f;
  p = p * r + 2.402264791363012e-1f;
  p = p * r + 6.931472028550421e-1f;
  p = p * r + 1.0f;

  return ldexpf(p, (int)n);
}

#endif /* CONFIG_LIBM_ARCH_EXP2F */
