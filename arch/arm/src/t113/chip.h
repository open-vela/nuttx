/****************************************************************************
 * arch/arm/src/t113/chip.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_T113_CHIP_H
#define __ARCH_ARM_SRC_T113_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <nuttx/arch.h>
#endif

#include "hardware/t113_memorymap.h"

#include <arch/t113/irq.h>

/****************************************************************************
 * Assembly Macros
 ****************************************************************************/

#ifdef __ASSEMBLY__

#if !defined(CONFIG_UP) && CONFIG_ARCH_INTERRUPTSTACK > 7
  .macro  setirqstack, tmp1, tmp2
  mrc     p15, 0, \tmp1, c0, c0, 5
  and     \tmp1, \tmp1, #3
  ldr     \tmp2, =g_irqstack_top
  lsls    \tmp1, \tmp1, #2
  add     \tmp2, \tmp2, \tmp1
  ldr     sp, [\tmp2, #0]
  .endm

  .macro  setfiqstack, tmp1, tmp2
  mrc     p15, 0, \tmp1, c0, c0, 5
  and     \tmp1, \tmp1, #3
  ldr     \tmp2, =g_fiqstack_top
  lsls    \tmp1, \tmp1, #2
  add     \tmp2, \tmp2, \tmp1
  ldr     sp, [\tmp2, #0]
  .endm
#endif

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Allwinner T113-S3 is Cortex-A7 with a 32-byte D-cache line. */

#define ARCH_DCACHE_LINESIZE 32

/* CONFIG_GICD_BASE and CONFIG_GICC_BASE expected by armv7-a/arm_gicv2.c.
 * Guard so a Kconfig-provided value (if ever added) is not stomped.
 */

#ifndef CONFIG_GICD_BASE
#  define CONFIG_GICD_BASE  T113_GIC_DIST_PADDR
#endif

#ifndef CONFIG_GICC_BASE
#  define CONFIG_GICC_BASE  T113_GIC_CPU_PADDR
#endif

#endif /* __ARCH_ARM_SRC_T113_CHIP_H */
