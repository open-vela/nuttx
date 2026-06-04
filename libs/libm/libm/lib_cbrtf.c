/****************************************************************************
 * libs/libm/libm/lib_cbrtf.c
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

#ifndef CONFIG_LIBM_ARCH_CBRTF

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cbrtf
 *
 * Description:
 *   Compute the (real) cube root of x.  cbrt is an odd function, defined for
 *   all reals: cbrtf(-x) = -cbrtf(x).  This generic fallback computes the
 *   result in double precision and rounds to float, which is correctly
 *   rounded for the float range.
 *
 ****************************************************************************/

float cbrtf(float x)
{
  return (float)cbrt((double)x);
}

#endif /* CONFIG_LIBM_ARCH_CBRTF */
