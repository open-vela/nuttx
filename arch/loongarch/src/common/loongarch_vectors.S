/****************************************************************************
 * arch/loongarch/src/common/loongarch_vectors.S
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
#include <nuttx/irq.h>
#include <arch/irq.h>
#include <arch/csr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define XCPT_SIZE   (REGSIZE * INT_XCPT_REGS)

/****************************************************************************
 * Name: __trap_vec
 *
 * Description:
 *   All LoongArch exceptions and interrupts will be handled from here.
 *
 ****************************************************************************/

  .section .text
  .balign  64
  .global  __trap_vec

__trap_vec:
  b    exception_common

/****************************************************************************
 * Name: exception_common
 *
 * Description:
 *   Common exception handler. Save context, determine cause, dispatch.
 *
 ****************************************************************************/

  .section .text
  .global  exception_common
  .align   8

exception_common:
  addi.d  $sp, $sp, -XCPT_SIZE

  st.d    $ra,  $sp, (REGSIZE * REG_R1_NDX)
  st.d    $r4,  $sp, (REGSIZE * REG_R4_NDX)
  st.d    $r5,  $sp, (REGSIZE * REG_R5_NDX)
  st.d    $r6,  $sp, (REGSIZE * REG_R6_NDX)
  st.d    $r7,  $sp, (REGSIZE * REG_R7_NDX)
  st.d    $r8,  $sp, (REGSIZE * REG_R8_NDX)
  st.d    $r9,  $sp, (REGSIZE * REG_R9_NDX)
  st.d    $r10, $sp, (REGSIZE * REG_R10_NDX)
  st.d    $r11, $sp, (REGSIZE * REG_R11_NDX)
  st.d    $r12, $sp, (REGSIZE * REG_R12_NDX)
  st.d    $r13, $sp, (REGSIZE * REG_R13_NDX)
  st.d    $r14, $sp, (REGSIZE * REG_R14_NDX)
  st.d    $r15, $sp, (REGSIZE * REG_R15_NDX)
  st.d    $r16, $sp, (REGSIZE * REG_R16_NDX)
  st.d    $r17, $sp, (REGSIZE * REG_R17_NDX)
  st.d    $r18, $sp, (REGSIZE * REG_R18_NDX)
  st.d    $r19, $sp, (REGSIZE * REG_R19_NDX)
  st.d    $r20, $sp, (REGSIZE * REG_R20_NDX)
  st.d    $r21, $sp, (REGSIZE * REG_R21_NDX)
  st.d    $r22, $sp, (REGSIZE * REG_R22_NDX)
  st.d    $r23, $sp, (REGSIZE * REG_R23_NDX)
  st.d    $r24, $sp, (REGSIZE * REG_R24_NDX)
  st.d    $r25, $sp, (REGSIZE * REG_R25_NDX)
  st.d    $r26, $sp, (REGSIZE * REG_R26_NDX)
  st.d    $r27, $sp, (REGSIZE * REG_R27_NDX)
  st.d    $r28, $sp, (REGSIZE * REG_R28_NDX)
  st.d    $r29, $sp, (REGSIZE * REG_R29_NDX)
  st.d    $r30, $sp, (REGSIZE * REG_R30_NDX)
  st.d    $r31, $sp, (REGSIZE * REG_R31_NDX)

  addi.d  $s0, $sp, XCPT_SIZE
  st.d    $s0,  $sp, (REGSIZE * REG_R3_NDX)

  csrrd   $s0, LOONGARCH_CSR_ERA
  st.d    $s0, $sp, (REGSIZE * REG_EPC_NDX)

  csrrd   $s0, LOONGARCH_CSR_PRMD
  st.d    $s0, $sp, (REGSIZE * REG_INT_CTX_NDX)

  csrrd   $s1, LOONGARCH_CSR_ESTAT
  srli.d  $s2, $s1, CSR_ESTAT_EXC_SHIFT
  andi    $s2, $s2, 0x3f

  bnez    $s2, .Lhandle_exception

  li.d    $s3, 0x7fff
  and     $s3, $s1, $s3
  beqz    $s3, .Lspurious_interrupt

  ctz.d   $s1, $s3
  addi.d  $s1, $s1, EXCCODE_INT_START
  b       .Lhas_irq

.Lhandle_exception:
  move    $a0, $s2
  move    $a1, $sp
  bl      loongarch_dispatch_irq
  b       .Lreturn_from_exception

.Lspurious_interrupt:
  move    $a0, $sp
  b       .Lreturn_from_exception

.Lhas_irq:
  move    $a0, $s1
  move    $a1, $sp
  bl      loongarch_dispatch_irq
  b       .Lreturn_from_exception

.Lreturn_from_exception:
  move    $sp, $a0

  ld.d    $s0, $sp, (REGSIZE * REG_EPC_NDX)
  csrwr   $s0, LOONGARCH_CSR_ERA

  ld.d    $s0, $sp, (REGSIZE * REG_INT_CTX_NDX)
  csrwr   $s0, LOONGARCH_CSR_PRMD

  ld.d    $ra,  $sp, (REGSIZE * REG_R1_NDX)
  ld.d    $r4,  $sp, (REGSIZE * REG_R4_NDX)
  ld.d    $r5,  $sp, (REGSIZE * REG_R5_NDX)
  ld.d    $r6,  $sp, (REGSIZE * REG_R6_NDX)
  ld.d    $r7,  $sp, (REGSIZE * REG_R7_NDX)
  ld.d    $r8,  $sp, (REGSIZE * REG_R8_NDX)
  ld.d    $r9,  $sp, (REGSIZE * REG_R9_NDX)
  ld.d    $r10, $sp, (REGSIZE * REG_R10_NDX)
  ld.d    $r11, $sp, (REGSIZE * REG_R11_NDX)
  ld.d    $r12, $sp, (REGSIZE * REG_R12_NDX)
  ld.d    $r13, $sp, (REGSIZE * REG_R13_NDX)
  ld.d    $r14, $sp, (REGSIZE * REG_R14_NDX)
  ld.d    $r15, $sp, (REGSIZE * REG_R15_NDX)
  ld.d    $r16, $sp, (REGSIZE * REG_R16_NDX)
  ld.d    $r17, $sp, (REGSIZE * REG_R17_NDX)
  ld.d    $r18, $sp, (REGSIZE * REG_R18_NDX)
  ld.d    $r19, $sp, (REGSIZE * REG_R19_NDX)
  ld.d    $r20, $sp, (REGSIZE * REG_R20_NDX)
  ld.d    $r21, $sp, (REGSIZE * REG_R21_NDX)
  ld.d    $r22, $sp, (REGSIZE * REG_R22_NDX)
  ld.d    $r23, $sp, (REGSIZE * REG_R23_NDX)
  ld.d    $r24, $sp, (REGSIZE * REG_R24_NDX)
  ld.d    $r25, $sp, (REGSIZE * REG_R25_NDX)
  ld.d    $r26, $sp, (REGSIZE * REG_R26_NDX)
  ld.d    $r27, $sp, (REGSIZE * REG_R27_NDX)
  ld.d    $r28, $sp, (REGSIZE * REG_R28_NDX)
  ld.d    $r29, $sp, (REGSIZE * REG_R29_NDX)
  ld.d    $r30, $sp, (REGSIZE * REG_R30_NDX)
  ld.d    $r31, $sp, (REGSIZE * REG_R31_NDX)

  ld.d    $sp, $sp, (REGSIZE * REG_R3_NDX)

  ertn

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .bss
  .balign 16
  .global g_intstackalloc
  .global g_intstacktop
  .type   g_intstackalloc, object
  .type   g_intstacktop, object
g_intstackalloc:
  .skip  STACKFRAME_ALIGN_UP(INT_STACK_SIZE)
g_intstacktop:
  .size  g_intstacktop, 0
  .size  g_intstackalloc, STACKFRAME_ALIGN_DOWN(INT_STACK_SIZE)
#endif
