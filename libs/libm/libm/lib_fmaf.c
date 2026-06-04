/****************************************************************************
 * libs/libm/libm/lib_fmaf.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Portable single-precision fused-multiply-add fallback.
 *
 * NuttX generic libm does not otherwise provide fmaf, so this is the
 * default implementation; an architecture may override it by selecting
 * CONFIG_LIBM_ARCH_FMAF (e.g. the xtensa HiFi4 madd.s version).
 *
 * The binary64 product is exact (24+24 significant bits fit in 53), so the
 * only rounding before the final round-to-float is the double add.  This
 * matches a true single-rounded FMA for the overwhelming majority of
 * inputs (a residual double-rounding mismatch is possible only in rare
 * carry-boundary cases).  Adequate as the portable fallback; the silicon
 * madd.s override is the single-rounded fast path.
 */

#ifndef CONFIG_LIBM_ARCH_FMAF
float fmaf(float x, float y, float z)
{
  return (float)((double)x * (double)y + (double)z);
}
#endif
