/****************************************************************************
 * libs/libc/machine/xtensa/lx7hifi4_fixfloat.S
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
 * Single-precision <-> integer conversions, HiFi4 audio-TIE accelerated.
 *
 * Overrides libgcc:
 *   __fixsfsi     float -> signed int    (truncate toward zero)  trunc.s
 *   __fixunssfsi  float -> unsigned int                          utrunc.s
 *   __floatsisf   signed int -> float                            float.s
 *   __floatunsisf unsigned int -> float                          ufloat.s
 *
 * The HiFi4 conversion ops move directly between an integer a-register and
 * the aed FP file: "trunc.s aOUT, aedIN, 0" / "float.s aedOUT, aIN, 0"
 * (the trailing immediate is a power-of-two scale, 0 here).  Windowed ABI,
 * value in a2, result in a2.  Requires CP1 (CONFIG_XTENSA_CP_INITSET bit 1).
 ****************************************************************************/

	.text

	.align 4
	.global __fixsfsi
	.type   __fixsfsi, @function
__fixsfsi:
	entry        a1, 16
	ae_movda32   aed0, a2          /* aed0 = float bits */
	trunc.s      a2, aed0, 0       /* a2 = (int)trunc(float) */
	retw.n
	.size __fixsfsi, .-__fixsfsi

	.align 4
	.global __fixunssfsi
	.type   __fixunssfsi, @function
__fixunssfsi:
	entry        a1, 16
	ae_movda32   aed0, a2
	utrunc.s     a2, aed0, 0       /* a2 = (unsigned)trunc(float) */
	retw.n
	.size __fixunssfsi, .-__fixunssfsi

	.align 4
	.global __floatsisf
	.type   __floatsisf, @function
__floatsisf:
	entry        a1, 16
	float.s      aed0, a2, 0       /* aed0 = (float)(int)a2 */
	ae_movad32.l a2, aed0
	retw.n
	.size __floatsisf, .-__floatsisf

	.align 4
	.global __floatunsisf
	.type   __floatunsisf, @function
__floatunsisf:
	entry        a1, 16
	ufloat.s     aed0, a2, 0       /* aed0 = (float)(unsigned)a2 */
	ae_movad32.l a2, aed0
	retw.n
	.size __floatunsisf, .-__floatunsisf
