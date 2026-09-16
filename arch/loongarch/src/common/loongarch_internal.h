/****************************************************************************
 * arch/loongarch/src/common/loongarch_internal.h
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

#ifndef __ARCH_LOONGARCH_SRC_COMMON_LOONGARCH_INTERNAL_H
#define __ARCH_LOONGARCH_SRC_COMMON_LOONGARCH_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <nuttx/sched.h>
#  include <sys/types.h>
#  include <stdint.h>
#  include <syscall.h>
#endif

#include <nuttx/irq.h>

#include "loongarch_common_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REGLOAD   "ld.d"
#define REGSTORE  "st.d"

#define STACK_COLOR    0xdeadbeef
#define HEAP_COLOR     'h'

#define STACK_FRAME_SIZE __XSTR(STACKFRAME_ALIGN)

#ifndef CONFIG_DEV_CONSOLE
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#else
#  if defined(CONFIG_CONSOLE_SYSLOG)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  else
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
#  endif
#endif

#ifndef __ASSEMBLY__

static inline uint8_t getreg8(const volatile uintreg_t a)
{
  uint8_t v;
  __asm__ __volatile__("ld.b %0, %1"
                       : "=r"(v)
                       : "m"(*(volatile uint8_t *)a));
  return v;
}

static inline void putreg8(uint8_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("st.b %0, %1"
                       : : "r"(v), "m"(*(volatile uint8_t *)a));
}

static inline uint16_t getreg16(const volatile uintreg_t a)
{
  uint16_t v;
  __asm__ __volatile__("ld.h %0, %1"
                       : "=r"(v)
                       : "m"(*(volatile uint16_t *)a));
  return v;
}

static inline void putreg16(uint16_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("st.h %0, %1"
                       : : "r"(v), "m"(*(volatile uint16_t *)a));
}

static inline uint32_t getreg32(const volatile uintreg_t a)
{
  uint32_t v;
  __asm__ __volatile__("ld.w %0, %1"
                       : "=r"(v)
                       : "m"(*(volatile uint32_t *)a));
  return v;
}

static inline void putreg32(uint32_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("st.w %0, %1"
                       : : "r"(v), "m"(*(volatile uint32_t *)a));
}

static inline uint64_t getreg64(const volatile uintreg_t a)
{
  uint64_t v;
  __asm__ __volatile__("ld.d %0, %1"
                       : "=r"(v)
                       : "m"(*(volatile uint64_t *)a));
  return v;
}

static inline void putreg64(uint64_t v, const volatile uintreg_t a)
{
  __asm__ __volatile__("st.d %0, %1"
                       : : "r"(v), "m"(*(volatile uint64_t *)a));
}

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

void modifyreg32(uintreg_t addr, uint32_t clearbits, uint32_t setbits);

#if CONFIG_MM_REGIONS > 1
void loongarch_addregion(void);
#else
#  define loongarch_addregion()
#endif

void loongarch_ack_irq(int irq);

void loongarch_sigdeliver(void);
int loongarch_swint(int irq, void *context, void *arg);
uintptr_t loongarch_get_newintctx(void);
void loongarch_set_idleintctx(void);
void loongarch_exception_attach(void);

void loongarch_allocateheap(void);

static inline void loongarch_savecontext(struct tcb_s *tcb)
{
}

static inline void loongarch_restorecontext(struct tcb_s *tcb)
{
}

void loongarch_jump_to_user(uintptr_t entry, uintptr_t arg1, uintptr_t arg2,
                            uintptr_t arg3, uintptr_t sp, uintreg_t *regs);

#ifdef CONFIG_PM
void loongarch_pminitialize(void);
#else
#  define loongarch_pminitialize()
#endif

#ifdef CONFIG_ARCH_DMA
void weak_function loongarch_dma_initialize(void);
#endif

void loongarch_lowputc(char ch);
void loongarch_lowputs(const char *str);

#ifdef USE_SERIALDRIVER
void loongarch_serialinit(void);
#endif

#ifdef USE_EARLYSERIALINIT
void loongarch_earlyserialinit(void);
#endif

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void loongarch_netinitialize(void);
#else
#  define loongarch_netinitialize()
#endif

uintreg_t *loongarch_doirq(int irq, uintreg_t *regs);
int loongarch_exception(int excode, void *regs, void *args);

#ifdef CONFIG_STACK_COLORATION
size_t loongarch_stack_check(uintptr_t alloc, size_t size);
void loongarch_stack_color(void *stackbase, size_t nbytes);
#endif

#if defined(CONFIG_STACK_COLORATION) && \
    defined(CONFIG_ARCH_INTERRUPTSTACK) && CONFIG_ARCH_INTERRUPTSTACK > 15
void loongarch_color_intstack(void);
#else
#  define loongarch_color_intstack()
#endif

#define loongarch_fullcontextrestore()    \
  do                                      \
    {                                     \
      sys_call0(SYS_restore_context);     \
    }                                     \
  while (1)

#define loongarch_switchcontext()         \
  do                                      \
    {                                     \
      sys_call0(SYS_switch_context);      \
    }                                     \
  while (0)

#if 0
#define EARLY_DBG_PUTS(s) \
  do { \
    const char *_p = s; \
    while (*_p) \
      { \
        volatile uint8_t *__lsr = (volatile uint8_t *)(0x8000000016100000 + 5); \
        while ((*__lsr & 0x20) == 0); \
        volatile uint8_t *__thr = (volatile uint8_t *)(0x8000000016100000); \
        *__thr = *_p++; \
      } \
  } while (0)

void uart_printf(const char *format, ...);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_LOONGARCH_SRC_COMMON_LOONGARCH_INTERNAL_H */
