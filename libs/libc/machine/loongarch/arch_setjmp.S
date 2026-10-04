############################################################################
# libs/libc/machine/loongarch/arch_setjmp.S
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

/* setjmp_buf_s has 11 registers:
 *   regs[0]  = ra   ($r1)
 *   regs[1]  = sp   ($r3)
 *   regs[2]  = s0   ($r22 / fp)
 *   regs[3]  = s1   ($r23)
 *   regs[4]  = s2   ($r24)
 *   regs[5]  = s3   ($r25)
 *   regs[6]  = s4   ($r26)
 *   regs[7]  = s5   ($r27)
 *   regs[8]  = s6   ($r28)
 *   regs[9]  = s7   ($r29)
 *   regs[10] = s8   ($r30)
 */

#define SZREG 8

.section .text
.globl  setjmp
.globl  _setjmp
.type   setjmp, @function
.type   _setjmp, @function
.align  2

setjmp:
_setjmp:
    st.d    $ra,  $a0, (0  * SZREG)
    st.d    $sp,  $a0, (1  * SZREG)
    st.d    $r22, $a0, (2  * SZREG)
    st.d    $r23, $a0, (3  * SZREG)
    st.d    $r24, $a0, (4  * SZREG)
    st.d    $r25, $a0, (5  * SZREG)
    st.d    $r26, $a0, (6  * SZREG)
    st.d    $r27, $a0, (7  * SZREG)
    st.d    $r28, $a0, (8  * SZREG)
    st.d    $r29, $a0, (9  * SZREG)
    st.d    $r30, $a0, (10 * SZREG)

    move    $a0, $zero
    jr      $ra
.size   setjmp, .-setjmp
.size   _setjmp, .-_setjmp

/* volatile void longjmp (jmp_buf env, int val);  */

.section .text
.globl  longjmp
.globl  _longjmp
.type   longjmp, @function
.type   _longjmp, @function
.align  2

longjmp:
_longjmp:
    ld.d    $ra,  $a0, (0  * SZREG)
    ld.d    $sp,  $a0, (1  * SZREG)
    ld.d    $r22, $a0, (2  * SZREG)
    ld.d    $r23, $a0, (3  * SZREG)
    ld.d    $r24, $a0, (4  * SZREG)
    ld.d    $r25, $a0, (5  * SZREG)
    ld.d    $r26, $a0, (6  * SZREG)
    ld.d    $r27, $a0, (7  * SZREG)
    ld.d    $r28, $a0, (8  * SZREG)
    ld.d    $r29, $a0, (9  * SZREG)
    ld.d    $r30, $a0, (10 * SZREG)

    /* a0 = (a1 == 0) ? 1 : a1 */

    beq     $a1, $zero, 1f
    move    $a0, $a1
    jr      $ra
1:
    addi.d  $a0, $zero, 1
    jr      $ra
.size   longjmp, .-longjmp
.size   _longjmp, .-_longjmp
