/****************************************************************************
 * libs/libc/machine/xtensa/lx7hifi4_addsubsf3.S
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
 * Single-precision float add / subtract, HiFi4 audio-TIE accelerated.
 *
 * Overrides libgcc's soft-float __addsf3 / __subsf3.  Same scheme as
 * lx7hifi4_mulsf3.S: windowed ABI (entry/retw.n, call site call8), operands
 * as bits in a2/a3, result bits in a2; ae_movda32 moves bits into the aed
 * FP register file with no stack round-trip, one silicon FP op, ae_movad32.l
 * back.  Requires CP1 (CONFIG_XTENSA_CP_INITSET bit 1).
 ****************************************************************************/

	.text
	.align 4
	.global __addsf3
	.type   __addsf3, @function

__addsf3:
	entry        a1, 16
	ae_movda32   aed0, a2          /* aed0 = a (float bits) */
	ae_movda32   aed1, a3          /* aed1 = b */
	add.s        aed2, aed0, aed1  /* aed2 = a + b */
	ae_movad32.l a2, aed2
	retw.n

	.size __addsf3, .-__addsf3

	.align 4
	.global __subsf3
	.type   __subsf3, @function

__subsf3:
	entry        a1, 16
	ae_movda32   aed0, a2          /* aed0 = a */
	ae_movda32   aed1, a3          /* aed1 = b */
	sub.s        aed2, aed0, aed1  /* aed2 = a - b */
	ae_movad32.l a2, aed2
	retw.n

	.size __subsf3, .-__subsf3
