/****************************************************************************
 * arch/risc-v/include/syscall.h
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

/* This file should never be included directly but, rather, only indirectly
 * through include/syscall.h or include/sys/sycall.h
 */

#ifndef __ARCH_RISCV_INCLUDE_SYSCALL_H
#define __ARCH_RISCV_INCLUDE_SYSCALL_H

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

/* The SYS_signal_handler_return is executed here... its value is not always
 * available in this context and so is assumed to be 7.
 */

#ifndef SYS_signal_handler_return
#  define SYS_signal_handler_return (7)
#elif SYS_signal_handler_return != 7
#  error "SYS_signal_handler_return was assumed to be 7"
#endif

/* Configuration ************************************************************/

/* SYS call 1 and 2 are defined for internal use by the RISC-V port (see
 * arch/risc-v/include/rv64gc/syscall.h). In addition, SYS call 3 is the
 * return from a SYS call in kernel mode. The first four syscall values must,
 * therefore, be reserved (0 is not used).
 */

#ifdef CONFIG_BUILD_KERNEL
#  ifndef CONFIG_SYS_RESERVED
#    error "CONFIG_SYS_RESERVED must be defined to the value 4"
#  elif CONFIG_SYS_RESERVED != 4
#    error "CONFIG_SYS_RESERVED must have the value 4"
#  endif
#endif

/* sys_call macros **********************************************************/

#ifndef __ASSEMBLY__

/* Context switching system calls *******************************************/

/* SYS call 0:
 *
 * int riscv_saveusercontext(uint64_t *saveregs);
 *
 * Return:
 * 0: Normal Return
 * 1: Context Switch Return
 */

#define SYS_save_context (0)
#define riscv_saveusercontext(saveregs) \
  (int)sys_call1(SYS_save_context, (uintptr_t)saveregs)

/* SYS call 1:
 *
 * void riscv_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
 */

#define SYS_restore_context (1)
#define riscv_fullcontextrestore(restoreregs) \
  sys_call1(SYS_restore_context, (uintptr_t)restoreregs)

/* SYS call 2:
 *
 * void riscv_switchcontext(uint32_t *saveregs, uint32_t *restoreregs);
 */

#define SYS_switch_context (2)
#define riscv_switchcontext(saveregs, restoreregs) \
  sys_call2(SYS_switch_context, (uintptr_t)saveregs, (uintptr_t)restoreregs)

#ifdef CONFIG_BUILD_KERNEL
/* SYS call 3:
 *
 * void riscv_syscall_return(void);
 */

#define SYS_syscall_return (3)
#define riscv_syscall_return() sys_call0(SYS_syscall_return)

#endif
#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: sys_call0
 *
 * Description:
 *   System call SYS_ argument and no additional parameters.
 *
 ****************************************************************************/

static inline uintptr_t sys_call0(unsigned int nbr)
{
  register long r0 asm("a0") = (long)(nbr);

  asm volatile
    (
     "ecall"
     :: "r"(r0)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: sys_call1
 *
 * Description:
 *   System call SYS_ argument and one additional parameter.
 *
 ****************************************************************************/

static inline uintptr_t sys_call1(unsigned int nbr, uintptr_t parm1)
{
  register long r0 asm("a0") = (long)(nbr);
  register long r1 asm("a1") = (long)(parm1);

  asm volatile
    (
     "ecall"
     :: "r"(r0), "r"(r1)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: sys_call2
 *
 * Description:
 *   System call SYS_ argument and two additional parameters.
 *
 ****************************************************************************/

static inline uintptr_t sys_call2(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2)
{
  register long r0 asm("a0") = (long)(nbr);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);

  asm volatile
    (
     "ecall"
     :: "r"(r0), "r"(r1), "r"(r2)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: sys_call3
 *
 * Description:
 *   System call SYS_ argument and three additional parameters.
 *
 ****************************************************************************/

static inline uintptr_t sys_call3(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3)
{
  register long r0 asm("a0") = (long)(nbr);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);

  asm volatile
    (
     "ecall"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: sys_call4
 *
 * Description:
 *   System call SYS_ argument and four additional parameters.
 *
 ****************************************************************************/

static inline uintptr_t sys_call4(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4)
{
  register long r0 asm("a0") = (long)(nbr);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);

  asm volatile
    (
     "ecall"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: sys_call5
 *
 * Description:
 *   System call SYS_ argument and five additional parameters.
 *
 ****************************************************************************/

static inline uintptr_t sys_call5(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5)
{
  register long r0 asm("a0") = (long)(nbr);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);
  register long r5 asm("a5") = (long)(parm5);

  asm volatile
    (
     "ecall"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

/****************************************************************************
 * Name: sys_call6
 *
 * Description:
 *   System call SYS_ argument and six additional parameters.
 *
 ****************************************************************************/

static inline uintptr_t sys_call6(unsigned int nbr, uintptr_t parm1,
                                  uintptr_t parm2, uintptr_t parm3,
                                  uintptr_t parm4, uintptr_t parm5,
                                  uintptr_t parm6)
{
  register long r0 asm("a0") = (long)(nbr);
  register long r1 asm("a1") = (long)(parm1);
  register long r2 asm("a2") = (long)(parm2);
  register long r3 asm("a3") = (long)(parm3);
  register long r4 asm("a4") = (long)(parm4);
  register long r5 asm("a5") = (long)(parm5);
  register long r6 asm("a6") = (long)(parm6);

  asm volatile
    (
     "ecall"
     :: "r"(r0), "r"(r1), "r"(r2), "r"(r3), "r"(r4), "r"(r5), "r"(r6)
     : "memory"
     );

  asm volatile("nop" : "=r"(r0));

  return r0;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_RISCV_INCLUDE_SYSCALL_H */
