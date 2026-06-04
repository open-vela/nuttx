/****************************************************************************
 * libs/libm/libm/xtensa/arch_floorf.c
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

#include "xtensa_libm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* floorf via the HiFi4 fifloor.s round-toward-negative-infinity instruction
 * (aed -> aed, float result).  fifloor.s passes inf/nan/zero through
 * unchanged and is exact for already-integral and large-magnitude values,
 * so no special-case screen is needed.
 */

#if XTENSA_LIBM_HAVE_VFPU2
float floorf(float x)
{
  float result;

  __asm__ volatile
  (
    "ae_movda32   aed0, %1\n"
    "fifloor.s    aed1, aed0\n"
    "ae_movad32.l %0, aed1\n"
    : "=r" (result) : "r" (x)
  );

  return result;
}
#endif
