/****************************************************************************
 * libs/libc/machine/xtensa/lx7hifi4_mulsf3.S
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
 * Single-precision float multiply, HiFi4 audio-TIE accelerated.
 *
 * Overrides libgcc's soft-float __mulsf3 (a multi-instruction integer
 * emulation).  Resolves ahead of libgcc by link order (libc.a precedes the
 * implicit libgcc).
 *
 * The GNU soft-float ABI passes the two float operands as raw bits in the
 * integer registers a2 and a3, and returns the result bits in a2.  We move
 * the bits directly into the AE_DR (aed) FP register file with ae_movda32
 * (a single 3-byte instruction -- NO stack round-trip, unlike the GNU
 * soft-float path), run the silicon mul.s, and move the result back with
 * ae_movad32.l.  This mirrors the vendor xcc dsp.bin technique.
 *
 * Requires CP1 (AudioEngineLX) enabled: CONFIG_XTENSA_CP_INITSET bit 1.
 ****************************************************************************/

	.text
	.align 4
	.global __mulsf3
	.type   __mulsf3, @function

__mulsf3:
	entry        a1, 16            /* windowed ABI (call8), like libgcc */
	ae_movda32   aed0, a2          /* aed0 = operand 0 (float bits) */
	ae_movda32   aed1, a3          /* aed1 = operand 1 (float bits) */
	mul.s        aed2, aed0, aed1  /* aed2 = aed0 * aed1            */
	ae_movad32.l a2, aed2          /* return result bits in a2      */
	retw.n

	.size __mulsf3, .-__mulsf3
