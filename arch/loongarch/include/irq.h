/****************************************************************************
 * arch/loongarch/include/irq.h
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

#ifndef __ARCH_LOONGARCH_INCLUDE_IRQ_H
#define __ARCH_LOONGARCH_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <arch/chip/irq.h>
#include <arch/csr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define STACKFRAME_ALIGN 16

#define STACKFRAME_ALIGN_UP(value)   (((value) + (STACKFRAME_ALIGN - 1)) & ~(STACKFRAME_ALIGN - 1))
#define STACKFRAME_ALIGN_DOWN(value) ((value) & ~(STACKFRAME_ALIGN - 1))

#define STACK_ALIGN_MASK     (STACKFRAME_ALIGN - 1)

#define SMP_STACK_MASK       (STACKFRAME_ALIGN - 1)
#define SMP_STACK_SIZE       ((CONFIG_IDLETHREAD_STACKSIZE) & ~SMP_STACK_MASK)
#define SMP_IDLE_STACK_SIZE  SMP_STACK_SIZE

#define LOONGARCH_IRQ_SIP0    (EXCCODE_INT_START + 0)
#define LOONGARCH_IRQ_SIP1    (EXCCODE_INT_START + 1)
#define LOONGARCH_IRQ_IP0     (EXCCODE_INT_START + 2)
#define LOONGARCH_IRQ_IP1     (EXCCODE_INT_START + 3)
#define LOONGARCH_IRQ_IP2     (EXCCODE_INT_START + 4)
#define LOONGARCH_IRQ_IP3     (EXCCODE_INT_START + 5)
#define LOONGARCH_IRQ_IP4     (EXCCODE_INT_START + 6)
#define LOONGARCH_IRQ_IP5     (EXCCODE_INT_START + 7)
#define LOONGARCH_IRQ_IP6     (EXCCODE_INT_START + 8)
#define LOONGARCH_IRQ_IP7     (EXCCODE_INT_START + 9)
#define LOONGARCH_IRQ_PC      (EXCCODE_INT_START + 10)
#define LOONGARCH_IRQ_TIMER   (EXCCODE_INT_START + 11)
#define LOONGARCH_IRQ_IPI     (EXCCODE_INT_START + 12)

#define LOONGARCH_MAX_IRQ     LOONGARCH_IRQ_IPI

#define LOONGARCH_IRQ_SYS     EXCCODE_SYS

#define LOONGARCH_IRQ_ASYNC   (LOONGARCH_MAX_IRQ + 1)

#define INT_STACK_SIZE       STACKFRAME_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK)

#define REG_EPC_NDX         0

#define REG_R1_NDX          1
#define REG_R3_NDX          2
#define REG_R4_NDX          3
#define REG_R5_NDX          4
#define REG_R6_NDX          5
#define REG_R7_NDX          6
#define REG_R8_NDX          7
#define REG_R9_NDX          8
#define REG_R10_NDX         9
#define REG_R11_NDX         10
#define REG_R12_NDX         11
#define REG_R13_NDX         12
#define REG_R14_NDX         13
#define REG_R15_NDX         14
#define REG_R16_NDX         15
#define REG_R17_NDX         16
#define REG_R18_NDX         17
#define REG_R19_NDX         18
#define REG_R20_NDX         19
#define REG_R21_NDX         20
#define REG_R22_NDX         21
#define REG_R23_NDX         22
#define REG_R24_NDX         23
#define REG_R25_NDX         24
#define REG_R26_NDX         25
#define REG_R27_NDX         26
#define REG_R28_NDX         27
#define REG_R29_NDX         28
#define REG_R30_NDX         29
#define REG_R31_NDX         30

#define REG_INT_CTX_NDX     31

#define INT_XCPT_REGS       (REG_INT_CTX_NDX + 1)

#define INT_REG_SIZE        8
#define REGSIZE             INT_REG_SIZE

#define INT_XCPT_SIZE       (INT_REG_SIZE * INT_XCPT_REGS)

#define XCPTCONTEXT_REGS    INT_XCPT_REGS
#define XCPTCONTEXT_SIZE    INT_XCPT_SIZE

#ifdef __ASSEMBLY__
#  define REG_EPC           (INT_REG_SIZE*REG_EPC_NDX)
#  define REG_R1            (INT_REG_SIZE*REG_R1_NDX)
#  define REG_R3            (INT_REG_SIZE*REG_R3_NDX)
#  define REG_R4            (INT_REG_SIZE*REG_R4_NDX)
#  define REG_R5            (INT_REG_SIZE*REG_R5_NDX)
#  define REG_R6            (INT_REG_SIZE*REG_R6_NDX)
#  define REG_R7            (INT_REG_SIZE*REG_R7_NDX)
#  define REG_R8            (INT_REG_SIZE*REG_R8_NDX)
#  define REG_R9            (INT_REG_SIZE*REG_R9_NDX)
#  define REG_R10           (INT_REG_SIZE*REG_R10_NDX)
#  define REG_R11           (INT_REG_SIZE*REG_R11_NDX)
#  define REG_R12           (INT_REG_SIZE*REG_R12_NDX)
#  define REG_R13           (INT_REG_SIZE*REG_R13_NDX)
#  define REG_R14           (INT_REG_SIZE*REG_R14_NDX)
#  define REG_R15           (INT_REG_SIZE*REG_R15_NDX)
#  define REG_R16           (INT_REG_SIZE*REG_R16_NDX)
#  define REG_R17           (INT_REG_SIZE*REG_R17_NDX)
#  define REG_R18           (INT_REG_SIZE*REG_R18_NDX)
#  define REG_R19           (INT_REG_SIZE*REG_R19_NDX)
#  define REG_R20           (INT_REG_SIZE*REG_R20_NDX)
#  define REG_R21           (INT_REG_SIZE*REG_R21_NDX)
#  define REG_R22           (INT_REG_SIZE*REG_R22_NDX)
#  define REG_R23           (INT_REG_SIZE*REG_R23_NDX)
#  define REG_R24           (INT_REG_SIZE*REG_R24_NDX)
#  define REG_R25           (INT_REG_SIZE*REG_R25_NDX)
#  define REG_R26           (INT_REG_SIZE*REG_R26_NDX)
#  define REG_R27           (INT_REG_SIZE*REG_R27_NDX)
#  define REG_R28           (INT_REG_SIZE*REG_R28_NDX)
#  define REG_R29           (INT_REG_SIZE*REG_R29_NDX)
#  define REG_R30           (INT_REG_SIZE*REG_R30_NDX)
#  define REG_R31           (INT_REG_SIZE*REG_R31_NDX)
#  define REG_INT_CTX       (INT_REG_SIZE*REG_INT_CTX_NDX)
#else
#  define REG_EPC           REG_EPC_NDX
#  define REG_R1            REG_R1_NDX
#  define REG_R3            REG_R3_NDX
#  define REG_R4            REG_R4_NDX
#  define REG_R5            REG_R5_NDX
#  define REG_R6            REG_R6_NDX
#  define REG_R7            REG_R7_NDX
#  define REG_R8            REG_R8_NDX
#  define REG_R9            REG_R9_NDX
#  define REG_R10           REG_R10_NDX
#  define REG_R11           REG_R11_NDX
#  define REG_R12           REG_R12_NDX
#  define REG_R13           REG_R13_NDX
#  define REG_R14           REG_R14_NDX
#  define REG_R15           REG_R15_NDX
#  define REG_R16           REG_R16_NDX
#  define REG_R17           REG_R17_NDX
#  define REG_R18           REG_R18_NDX
#  define REG_R19           REG_R19_NDX
#  define REG_R20           REG_R20_NDX
#  define REG_R21           REG_R21_NDX
#  define REG_R22           REG_R22_NDX
#  define REG_R23           REG_R23_NDX
#  define REG_R24           REG_R24_NDX
#  define REG_R25           REG_R25_NDX
#  define REG_R26           REG_R26_NDX
#  define REG_R27           REG_R27_NDX
#  define REG_R28           REG_R28_NDX
#  define REG_R29           REG_R29_NDX
#  define REG_R30           REG_R30_NDX
#  define REG_R31           REG_R31_NDX
#  define REG_INT_CTX       REG_INT_CTX_NDX
#endif

#define REG_RA              REG_R1
#define REG_SP              REG_R3

#ifndef __ASSEMBLY__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/compiler.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN volatile bool g_interrupt_context[CONFIG_SMP_NCPUS];

struct xcptcontext
{
  uintreg_t *regs;
  uintreg_t *saved_regs;
};

static inline_function void up_set_interrupt_context(bool flag)
{
  g_interrupt_context[0] = flag;
}

noinstrument_function static inline_function bool up_interrupt_context(void)
{
  return g_interrupt_context[0];
}

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  irqstate_t flags;

    __asm__ __volatile__
    (
      "csrrd   %0, %2\n\t"
      "csrxchg $zero, %1, %2\n\t"
      : "=&r"(flags)
      : "r"(CSR_CRMD_IE), "i"(LOONGARCH_CSR_CRMD)
      : "memory"
    );

  return flags & CSR_CRMD_IE;
}

noinstrument_function static inline_function void
  up_irq_restore(irqstate_t flags)
{
  __asm__ __volatile__
    (
      "csrxchg %0, %1, %2\n\t"
      : "+r"(flags)
      : "r"(CSR_CRMD_IE), "i"(LOONGARCH_CSR_CRMD)
      : "memory"
    );
}

noinstrument_function static inline_function irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  __asm__ __volatile__
    (
      "csrrd   %0, %2\n\t"
      "csrxchg %1, %1, %2\n\t"
      : "=&r"(flags)
      : "r"(CSR_CRMD_IE), "i"(LOONGARCH_CSR_CRMD)
      : "memory"
    );

  return flags & CSR_CRMD_IE;
}

#define up_getusrpc(regs) \
    (((uintptr_t *)((regs) ? (regs) : running_regs()))[REG_EPC])

#define up_getusrsp(regs) \
    (((uintptr_t *)(regs))[REG_SP])

noinstrument_function static inline_function uintptr_t up_getsp(void)
{
  register uintptr_t sp;
  __asm__ __volatile__
    (
      "move %0, $sp\n"
      : "=r"(sp)
    );
  return sp;
}

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_LOONGARCH_INCLUDE_IRQ_H */
