/****************************************************************************
 * boards/arm/t113/t113-evb/src/t113_boot_jump.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "arm_internal.h"
#include "hardware/t113_memorymap.h"
#include "t113-evb.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* GIC v2 register offsets used by the handoff GIC reset.  Bases come from
 * arch/arm/src/t113/hardware/t113_memorymap.h (T113_GIC_DIST_PADDR /
 * T113_GIC_CPU_PADDR) so a future SoC re-spin that moves the GIC won't
 * leave stale literals here.
 */

#define T113_BOOT_GICD_IGROUP_BASE  (T113_GIC_DIST_PADDR + 0x080)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: t113_boot_jump
 *
 * Description:
 *   Prepare CPU + GIC state and jump to a loaded image.  Never returns.
 *
 *   boot0-nuttx runs with MMU off and both caches disabled (defconfig
 *   selects CONFIG_ARM_DCACHE_DISABLE=y + CONFIG_ARM_ICACHE_DISABLE=y +
 *   ARCH_USE_MMU=n), so SCTLR.M/C/I are already 0 and the kernel
 *   decompressor handles TLB / I-cache / branch-predictor maintenance
 *   on its own.  Two pieces of NuttX-specific state must still be
 *   undone before the chained image runs:
 *
 *     1) VBAR = 0.  arm_irq_initialize() set VBAR to CONFIG_RAM_START
 *        (NuttX vector table in SRAM).  ARM Linux assumes VBAR=0
 *        until head.S installs its own.  If anything triggers an
 *        exception in the handoff window - SWI from the kernel
 *        decompressor, leftover async IRQ - control jumps to the
 *        dead NuttX handler in SRAM.
 *
 *     2) GIC group config restored to power-on reset state
 *        (GICD_CTLR=0, GICC_CTLR=0, GICD_IGROUPRn=0 for every bank).
 *        This is the actual fix for "Linux boots silently to login
 *        but every IRQ count in /proc/interrupts is 0".
 *
 *        NuttX arm_gicv2_initialize() configures all SGI/PPI/SPI as
 *        Group 1 (NS-IRQ) by writing GICD_IGROUPRn = 0xFFFFFFFF and
 *        sets GICC_CTLR with both EnableGrp0 + EnableGrp1.  Linux
 *        gic_cpu_if_up() in drivers/irqchip/irq-gic.c only writes
 *        GICC_ENABLE = 0x1 = bit 0 = EnableGrp0; the bypass mask
 *        clears EnableGrp1 and Linux never re-enables it.  Linux
 *        gic_dist_init() also never writes GICD_IGROUPRn.  Without
 *        this reset every interrupt is Group 1 in the distributor
 *        and only Group 0 is enabled in the CPU interface, so timer
 *        / UART / PMU / DMA all silently drop.  printk still works
 *        because 8250 console_write is polling-based, which is why
 *        the boot looks healthy until the first sleep() hangs.
 *
 *        Restoring power-on state is exactly what the FEL bootstub
 *        leaves behind by simply not touching the GIC; boot0 has to
 *        actively undo what NuttX SPL configured.
 *
 *   boot_arg == 0  -> r2=0 = NuttX legacy raw-boot convention.
 *   boot_arg != 0  -> r2=DTB phys = ARM Linux DT calling convention.
 *
 ****************************************************************************/

void t113_boot_jump(uintptr_t entry, uintptr_t boot_arg)
{
  /* Pin entry / boot_arg to callee-saved registers (r4 / r5).
   *
   * The asm clobber list ("r0", "r1", "r2", "r3") already prevents gcc
   * from picking those for the inputs, but the binding makes the
   * sequence self-evidently correct without re-deriving the operand
   * allocator's behavior.  Defense in depth - a future maintainer
   * shouldn't have to reason about whether `mov r0, #0` clobbers an
   * operand or not.
   */

  register uintptr_t r_entry asm("r4") = entry;
  register uintptr_t r_arg   asm("r5") = boot_arg;
  int gi;

  up_irq_disable();

  /* GIC group reset - see (2) in the function comment for why.
   *
   * 5 IGROUP banks would be enough on T113-S3 (NR_IRQS=160 -> 5 banks
   * of 32), but writing the 8 banks GIC v2 always implements is one
   * extra mov-and-branch and immune to a future SoC re-spin with
   * more SPIs.  Writes past the implemented IRQ count are RAZ/WI on
   * GIC-400, so the over-provisioning is harmless.
   */

  putreg32(0, T113_GIC_DIST_PADDR);
  putreg32(0, T113_GIC_CPU_PADDR);
  for (gi = 0; gi < 8; gi++)
    {
      putreg32(0, T113_BOOT_GICD_IGROUP_BASE + (uintptr_t)gi * 4);
    }

  /* VBAR = 0, then ARM Linux DT calling convention + jump */

  __asm__ __volatile__(
    "mov r3, #0\n"
    "mcr p15, 0, r3, c12, c0, 0\n"
    "isb sy\n"
    "mov r0, #0\n"
    "mvn r1, #0\n"                   /* r1 = 0xFFFFFFFF (machine id) */
    "mov r2, %1\n"                   /* r2 = boot_arg (pinned r5) */
    "msr cpsr_c, #0xD3\n"            /* SVC, IRQ+FIQ off */
    "bx %0\n"                        /* ARM/Thumb-safe branch       */
    :: "r"(r_entry), "r"(r_arg) : "r0", "r1", "r2", "r3", "memory");

  __builtin_unreachable();
}
