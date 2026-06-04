/****************************************************************************
 * libs/libc/machine/xtensa/lx7hifi4_negsf2.S
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
 * Single-precision float negate.
 *
 * Overrides libgcc's __negsf2.  Negation is just a sign-bit flip on the
 * IEEE-754 bit pattern, so this is pure integer (no TIE/aed needed) and
 * matches libgcc's own approach (xor with 0x80000000).  Windowed ABI:
 * operand bits in a2, result in a2.
 ****************************************************************************/

	.text
	.align 4
	.global __negsf2
	.type   __negsf2, @function

__negsf2:
	entry a1, 16
	movi  a3, 1
	slli  a3, a3, 31          /* a3 = 0x80000000 (sign bit) */
	xor   a2, a2, a3          /* flip sign */
	retw.n

	.size __negsf2, .-__negsf2
