/****************************************************************************
 * arch/arm/src/arm_m/arm_vectors.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "arm_internal.h"
#include "ram_vectors.h"
#include "nvic.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_ARMV6M
#  define ARM_PERIPHERAL_INTERRUPTS ARMV6M_PERIPHERAL_INTERRUPTS
#elif defined(CONFIG_ARCH_ARMV7M)
#  define ARM_PERIPHERAL_INTERRUPTS ARMV7M_PERIPHERAL_INTERRUPTS
#elif defined(CONFIG_ARCH_ARMV8M)
#  define ARM_PERIPHERAL_INTERRUPTS ARMV8M_PERIPHERAL_INTERRUPTS
#endif

#define IDLE_STACK      (_ebss + CONFIG_IDLETHREAD_STACKSIZE)

#ifndef ARM_PERIPHERAL_INTERRUPTS
#  error ARM_PERIPHERAL_INTERRUPTS must be defined to the number of I/O interrupts to be supported
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Chip-specific entrypoint */

extern void __start(void);

static void start(void)
{
  /* Set MSP & PSP to the value at reset */

  arm_initialize_stack();

  /* Zero lr to mark the end of backtrace */

  asm volatile ("mov lr, %0\n\t"
                "bx      %1\n\t"
                :
                : "r"(0), "r"(__start));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Common exception entrypoint */

extern void exception_common(void);
extern void exception_direct(void);

/****************************************************************************
 * Public data
 ****************************************************************************/

/* The arm_m vector table consists of an array of function pointers, with the
 * first slot (vector zero) used to hold the initial stack pointer.
 *
 * As all exceptions (interrupts) are routed via exception_common, we just
 * need to fill this array with pointers to it.
 *
 * Note that the [ ... ] designated initializer is a GCC extension.
 */

/* Some NXP boot ROMs (e.g. LPC55xx, MCX Nxxx) interpret the reserved
 * Cortex-M vector-table slots at offsets 0x20, 0x24, 0x28 and 0x34 (vector
 * indices 8, 9, 10 and 13) as an image header (image type, image length,
 * ...).  When booting a plain (unsigned) image directly from internal flash,
 * these slots must be zero or the ROM rejects the image and falls back to
 * ISP mode.  These vectors are never taken by the hardware, so forcing them
 * to zero has no functional effect on NuttX.
 */

#ifdef CONFIG_ARMV8M_NXP_BOOTROM_HEADER
#  define ARMV8M_RESERVED_VECTOR 0
#else
#  define ARMV8M_RESERVED_VECTOR &exception_common
#endif

const void * const _vectors[] locate_data(".vectors")
                              aligned_data(VECTAB_ALIGN) =
{
  /* Initial stack */

  IDLE_STACK,

  /* Reset exception handler */

  start,

  /* Vectors 2 - n point directly at the generic handler, except for the
   * architecturally reserved slots (indices 8, 9, 10 and 13) which may be
   * required to be zero by some NXP boot ROMs (see above).
   */

  [2 ... 7]               = &exception_common,
  [8 ... 10]              = ARMV8M_RESERVED_VECTOR,
  [11 ... 12]             = &exception_common,
  [13]                    = ARMV8M_RESERVED_VECTOR,
  [NVIC_IRQ_PENDSV]       = &exception_common,
  [(NVIC_IRQ_PENDSV + 1) ... (15 + ARM_PERIPHERAL_INTERRUPTS)]
                          = &exception_direct
};
