/****************************************************************************
 * arch/loongarch/include/syscall.h
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

#ifndef __ARCH_LOONGARCH_INCLUDE_SYSCALL_H
#define __ARCH_LOONGARCH_INCLUDE_SYSCALL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYS_syscall 0x00

#ifndef CONFIG_BUILD_FLAT
#  define CONFIG_SYS_RESERVED 6
#else
#  define CONFIG_SYS_RESERVED 4
#endif

#define SYS_restore_context       (1)
#define SYS_switch_context        (2)
#define SYS_assert_handler        (3)

#ifndef CONFIG_BUILD_FLAT

#define SYS_signal_handler        (4)
#define SYS_signal_handler_return (5)

#endif /* !CONFIG_BUILD_FLAT */

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register long r0 asm("$a0") = (long)(nbr);

  asm volatile
    (
     "syscall 0\n\t"
     :: "r"(r0)
     : "memory"
     );

  asm volatile("" : "=r"(r0));

  return r0;
}

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register long r0 asm("$a0") = (long)(nbr);
  register long r1 asm("$a1") = (long)(parm1);

  asm volatile
    (
     "syscall 0\n\t"
     :: "r"(r0), "r"(r1)
     : "memory"
     );

  asm volatile("" : "=r"(r0));

  return r0;
}

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  register long r0 asm("$a0") = (long)(nbr);
  register long r1 asm("$a1") = (long)(parm1);
  register long r2 asm("$a2") = (long)(parm2);

  asm volatile
    (
     "syscall 0\n\t"
     :: "r"(r0), "r"(r1), "r"(r2)
     : "memory"
     );

  asm volatile("" : "=r"(r0));

  return r0;
}

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  register long r0 asm("$a0") = (long)(nbr);
  register long r1 asm("$a1") = (long)(parm1);
  register long r2 asm("$a2") = (long)(parm2);
  register long r3 asm("$a3") = (long)(parm3);

  asm volatile
    (
     "syscall 0\n\t"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3)
     : "memory"
     );

  asm volatile("" : "=r"(r0));

  return r0;
}

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  register long r0 asm("$a0") = (long)(nbr);
  register long r1 asm("$a1") = (long)(parm1);
  register long r2 asm("$a2") = (long)(parm2);
  register long r3 asm("$a3") = (long)(parm3);
  register long r4 asm("$a4") = (long)(parm4);

  asm volatile
    (
     "syscall 0\n\t"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4)
     : "memory"
     );

  asm volatile("" : "=r"(r0));

  return r0;
}

static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  register long r0 asm("$a0") = (long)(nbr);
  register long r1 asm("$a1") = (long)(parm1);
  register long r2 asm("$a2") = (long)(parm2);
  register long r3 asm("$a3") = (long)(parm3);
  register long r4 asm("$a4") = (long)(parm4);
  register long r5 asm("$a5") = (long)(parm5);

  asm volatile
    (
     "syscall 0\n\t"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5)
     : "memory"
     );

  asm volatile("" : "=r"(r0));

  return r0;
}

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6)
{
  register long r0 asm("$a0") = (long)(nbr);
  register long r1 asm("$a1") = (long)(parm1);
  register long r2 asm("$a2") = (long)(parm2);
  register long r3 asm("$a3") = (long)(parm3);
  register long r4 asm("$a4") = (long)(parm4);
  register long r5 asm("$a5") = (long)(parm5);
  register long r6 asm("$a6") = (long)(parm6);

  asm volatile
    (
     "syscall 0\n\t"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5), "r"(r6)
     : "memory"
     );

  asm volatile("" : "=r"(r0));

  return r0;
}

#define up_assert(filename, linenum, msg) \
    sys_call3(SYS_assert_handler, (uintptr_t)filename, \
              (uintptr_t)linenum, (uintptr_t)msg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_LOONGARCH_INCLUDE_SYSCALL_H */
