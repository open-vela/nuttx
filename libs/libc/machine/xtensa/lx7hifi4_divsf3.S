/****************************************************************************
 * libs/libc/machine/xtensa/lx7hifi4_divsf3.S
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
 * Single-precision float divide, HiFi4 audio-TIE accelerated.
 *
 * Overrides libgcc's soft-float __divsf3.  There is no single-instruction
 * div.s on this silicon; divide is Newton-Raphson on the reciprocal, kept
 * entirely in the aed FP register file (no DDR round-trip).  The Newton
 * core is transcribed verbatim from the vendor dsp.bin routine at
 * 0x37ad5850 (xcc-compiled, the performance reference): div0.s reciprocal
 * seed, nexp01.s mantissa extraction (which also normalises subnormals),
 * three maddn.s/msubn.s refinement iterations, mkdadj.s + addexpm.s/addexp.s
 * exponent compose, and divn.s for the final IEEE round.
 *
 * The vendor routine does NO special-case handling (its caller pre-screens),
 * so this helper adds an integer IEEE screen up front, mirroring libgcc's
 * own __divsf3: the result sign is sign(a) XOR sign(b), and inf/nan/zero
 * operands short-circuit to the IEEE result before the Newton path runs.
 *
 * Windowed ABI (entry/retw.n): operands as bits in a2 (dividend) / a3
 * (divisor), result bits in a2.  Requires CP1 (CONFIG_XTENSA_CP_INITSET
 * bit 1).
 ****************************************************************************/

	.text
	.literal_position
	.literal .Lclamp, 0x2edbe6ff   /* subnormal divisor clamp threshold */

	.align 4
	.global __divsf3
	.type   __divsf3, @function

__divsf3:
	entry        a1, 32

	/* Result sign = sign(a) XOR sign(b), bit 31 of a7. */

	xor          a7, a2, a3

	/* exp fields: a8 = exp(a), a9 = exp(b). */

	extui        a8, a2, 23, 8
	extui        a9, a3, 23, 8

	/* --- Special-case screen (integer) --- */

	/* dividend a is inf/nan? (exp == 0xff) */

	movi         a10, 0xff
	beq          a8, a10, .Ldiv_a_special

	/* divisor b is inf/nan? */

	beq          a9, a10, .Ldiv_b_special

	/* divisor b == 0 (exp==0 and mantissa==0)?  -> x/0 */

	bnez         a9, .Ldiv_b_nonzero
	slli         a11, a3, 9           /* drop sign+exp, keep mantissa */
	bnez         a11, .Ldiv_normal    /* subnormal divisor: let Newton handle */

	/* b == +-0 */

	slli         a11, a2, 1           /* drop sign of a; nonzero iff |a|!=0 */
	bnez         a11, .Ldiv_inf_signed   /* finite nonzero / 0 -> signed inf */
	/* 0 / 0 -> nan */
	movi         a2, 0x7fc00000
	retw.n

.Ldiv_b_nonzero:
	/* dividend a == 0 (exp==0, mantissa==0)? -> signed 0 */

	bnez         a8, .Ldiv_normal
	slli         a11, a2, 9
	bnez         a11, .Ldiv_normal    /* subnormal dividend: Newton handles */
	srli         a2, a7, 31
	slli         a2, a2, 31           /* signed zero */
	retw.n

.Ldiv_a_special:
	/* a is inf or nan. */

	slli         a11, a2, 9
	bnez         a11, .Ldiv_a_nan     /* a is nan -> propagate quiet */
	/* a is inf: inf/inf or inf/nan -> nan; inf/finite -> signed inf */
	beq          a9, a10, .Ldiv_nan_b_check
	j            .Ldiv_inf_signed

.Ldiv_nan_b_check:
	/* a==inf and b is inf or nan -> nan */
	movi         a2, 0x7fc00000
	retw.n

.Ldiv_a_nan:
	movi         a11, 0x00400000
	or           a2, a2, a11          /* force quiet nan from a */
	retw.n

.Ldiv_b_special:
	/* b is inf or nan (a is finite here). */

	slli         a11, a3, 9
	bnez         a11, .Ldiv_b_nan     /* b nan -> propagate */
	/* b == inf, a finite -> signed 0 */
	srli         a2, a7, 31
	slli         a2, a2, 31
	retw.n

.Ldiv_b_nan:
	mov          a2, a3
	movi         a11, 0x00400000
	or           a2, a2, a11          /* quiet nan from b */
	retw.n

.Ldiv_inf_signed:
	/* signed infinity = (sign<<31) | 0x7f800000 */

	srli         a2, a7, 31
	slli         a2, a2, 31
	movi         a11, 0x7f800000
	or           a2, a2, a11
	retw.n

	/* --- Normal Newton path (verbatim dsp.bin 0x37ad5850) --- */

.Ldiv_normal:
	ae_movda32   aed0, a2             /* aed0 = dividend */
	ae_movda32   aed1, a3             /* aed1 = divisor  */

	const.s      aed11, 1             /* 1.0 */
	abs.s        aed12, aed1          /* |divisor| */
	l32r         a11, .Lclamp
	wfr          aed13, a11           /* clamp threshold */
	max.s        aed5, aed12, aed13   /* clamp |divisor| away from 0 */
	div0.s       aed10, aed5          /* reciprocal seed */
	ae_mov       aed2, aed0           /* save dividend for mkdadj */
	nexp01.s     aed6, aed5           /* divisor mantissa */
	maddn.s      aed11, aed6, aed10   /* 1 + mant*seed */
	ae_mov       aed4, aed10
	maddn.s      aed4, aed11, aed10   /* refined reciprocal */
	nexp01.s     aed3, aed0           /* dividend mantissa */
	const.s      aed9, 1
	const.s      aed0, 0
	maddn.s      aed9, aed6, aed4
	msubn.s      aed0, aed3, aed10
	ae_mov       aed8, aed3
	maddn.s      aed4, aed9, aed4
	msubn.s      aed8, aed6, aed0
	const.s      aed7, 1
	maddn.s      aed7, aed6, aed4
	msubn.s      aed0, aed8, aed4
	mkdadj.s     aed5, aed2           /* quotient exponent adjust */
	maddn.s      aed4, aed7, aed4
	neg.s        aed3, aed3
	maddn.s      aed3, aed6, aed0
	addexpm.s    aed0, aed5
	addexp.s     aed4, aed5
	divn.s       aed0, aed3, aed4     /* final rounded quotient (magnitude) */

	ae_movad32.l a2, aed0

	/* The Newton path divided by |divisor| (abs.s for the seed), so the
	 * result only carries the dividend's sign.  Force the IEEE result sign
	 * = sign(a) XOR sign(b), precomputed in a7 bit 31.
	 */

	slli         a2, a2, 1            /* drop current sign bit */
	srli         a2, a2, 1
	extui        a11, a7, 31, 1
	slli         a11, a11, 31
	or           a2, a2, a11
	retw.n

	.size __divsf3, .-__divsf3
