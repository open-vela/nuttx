/****************************************************************************
 * arch/loongarch/src/common/loongarch_fork_func.S
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
#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .globl    up_fork
    .type     up_fork, function

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fork
 *
 * Description:
 *   The fork() function has the same effect as posix fork(), except that the
 *   behavior is undefined if the process created by fork() either modifies
 *   any data other than a variable of type pid_t used to store the return
 *   value from fork(), or returns from the function in which fork() was
 *   called, or calls any other function before successfully calling _exit()
 *   or one of the exec family of functions.
 *
 *   The overall sequence is:
 *
 *   1) User code calls fork().
 *   2) This bare-metal fork() implementation collects context information
 *      and transfers control to loongarch_fork().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Upon successful completion, fork() returns 0 to the child process and
 *   returns the process ID of the child process to the parent process.
 *   Otherwise, -1 is returned to the parent, no child process is created,
 *   and errno is set to indicate the error.
 *
 ****************************************************************************/

    .section  .text
    .align    2

up_fork:
    /* Save return address and stack pointer */

    addi.d    $sp, $sp, -FORK_CONTEXT_SIZE
    st.d      $ra, $sp, 0
    st.d      $sp, $sp, 8

    /* Save callee-saved registers */

    st.d      $s0, $sp, 16
    st.d      $s1, $sp, 24
    st.d      $s2, $sp, 32
    st.d      $s3, $sp, 40
    st.d      $s4, $sp, 48
    st.d      $s5, $sp, 56
    st.d      $s6, $sp, 64
    st.d      $s7, $sp, 72
    st.d      $s8, $sp, 80
    st.d      $fp, $sp, 88

    /* Call loongarch_fork with context pointer */

    move      $a0, $sp
    bl        loongarch_fork

    /* Restore context (only parent reaches here) */

    ld.d      $ra, $sp, 0
    ld.d      $sp, $sp, 8
    ld.d      $s0, $sp, 16
    ld.d      $s1, $sp, 24
    ld.d      $s2, $sp, 32
    ld.d      $s3, $sp, 40
    ld.d      $s4, $sp, 48
    ld.d      $s5, $sp, 56
    ld.d      $s6, $sp, 64
    ld.d      $s7, $sp, 72
    ld.d      $s8, $sp, 80
    ld.d      $fp, $sp, 88

    addi.d    $sp, $sp, FORK_CONTEXT_SIZE

    /* Return to caller */

    jr        $ra

    .size     up_fork, . - up_fork
