/****************************************************************************
 * arch/arm/src/t113/t113_boot.c
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

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#ifdef CONFIG_LEGACY_PAGING
#  include <nuttx/page.h>
#endif

#include <nuttx/kmalloc.h>

#include "chip.h"

#include "arm.h"
#ifdef CONFIG_ARCH_USE_MMU
#include "mmu.h"
#endif
#include "arm_internal.h"
#include "t113_lowputc.h"
#include "t113_boot.h"
#include "t113_clk.h"
#include "hardware/t113_clk.h"

#ifdef CONFIG_T113_BOOT0
extern uint32_t _boot0_start;
volatile uint32_t *g_boot0_anchor = &_boot0_start;
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_USE_MMU)
#  define NMAPPINGS \
     (sizeof(section_mapping) / sizeof(struct section_mapping_s))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

extern uint8_t _vector_start[];
extern uint8_t _vector_end[];

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_USE_MMU)
static const struct section_mapping_s section_mapping[] =
{
  { T113_SRAM_PSECTION, T113_SRAM_VSECTION,
    T113_SRAM_MMUFLAGS, T113_SRAM_NSECTIONS
  },
  { T113_SP0_PSECTION,    T113_SP0_VSECTION,
    T113_SP0_MMUFLAGS,    T113_SP0_NSECTIONS
  },
  { T113_SP1_PSECTION,    T113_SP1_VSECTION,
    T113_SP1_MMUFLAGS,    T113_SP1_NSECTIONS
  },
  { T113_SH0_PSECTION,    T113_SH0_VSECTION,
    T113_SH0_MMUFLAGS,    T113_SH0_NSECTIONS
  },
  { T113_SH2_PSECTION,    T113_SH2_VSECTION,
    T113_SH2_MMUFLAGS,    T113_SH2_NSECTIONS
  },
  { T113_PERIPH_PSECTION, T113_PERIPH_VSECTION,
    T113_PERIPH_MMUFLAGS, T113_PERIPH_NSECTIONS
  },
  { T113_APBS0_PSECTION,  T113_APBS0_VSECTION,
    T113_APBS0_MMUFLAGS,  T113_APBS0_NSECTIONS
  },
  { T113_CPUX_PSECTION,   T113_CPUX_VSECTION,
    T113_CPUX_MMUFLAGS,   T113_CPUX_NSECTIONS
  },

#ifdef CONFIG_T113_RPTUN_DSP
  /* DSP CFG block: ALT_RESET_VEC, CTRL_REG0, DSP-side WDT.
   * Not covered by any existing peripheral-domain section (those start
   * at 0x02000000); this 1 MB Device mapping gives the AP access to
   * the DSP control registers.
   */

  { 0x01700000u,          0x01700000u,
    MMU_IOFLAGS,          1
  },
#endif

  /* DDR: initialized by boot0 (sys_dram_init) or by external loader. */

  { T113_DDR_MAPPADDR,    T113_DDR_MAPVADDR,
    T113_DDR_MMUFLAGS,    T113_DDR_MAPSECTIONS
  },

#ifdef CONFIG_T113_RPTUN_CORE1
  /* CPU1 carveout + slave-image window: Normal Non-Cacheable so core0 can
   * load the CPU1 ELF and share the rpmsg vrings without cache maintenance.
   */

  { T113_CPU1_SHMEM_PADDR, T113_CPU1_SHMEM_VADDR,
    T113_NONCACHE_DDR_MMUFLAGS, T113_CPU1_SHMEM_NSECTIONS
  },
#endif

#ifdef CONFIG_T113_RPTUN_DSP
  /* DSP firmware and shmem region: top 5 MB of DDR (after 1 MB ramlog
   * reserve).  Mapped Normal Non-Cacheable Shareable so the AP can
   * share data with the DSP without cache-coherency management.
   */

  { T113_DSP_SHMEM_PADDR, T113_DSP_SHMEM_VADDR,
    T113_NONCACHE_DDR_MMUFLAGS, T113_DSP_SHMEM_NSECTIONS
  },
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_USE_MMU)

static inline void t113_setupmappings(void)
{
  mmu_l1_map_regions(section_mapping, NMAPPINGS);
}
#endif

#if !defined(CONFIG_ARCH_ROMPGTABLE) && !defined(CONFIG_ARCH_LOWVECTORS)
static void t113_vectormapping(void)
{
  uint32_t vector_paddr = T113_VECTOR_PADDR & PTE_SMALL_PADDR_MASK;
  uint32_t vector_vaddr = T113_VECTOR_VADDR & PTE_SMALL_PADDR_MASK;
  uint32_t end_paddr = T113_VECTOR_PADDR +
                       (_vector_end - _vector_start);

  while (vector_paddr < end_paddr)
    {
      mmu_l2_setentry(VECTOR_L2_VBASE, vector_paddr, vector_vaddr,
                      MMU_L2_VECTORFLAGS);
      vector_paddr += 4096;
      vector_vaddr += 4096;
    }

  mmu_l1_setentry(VECTOR_L2_PBASE & PMD_PTE_PADDR_MASK,
                  T113_VECTOR_VADDR & PMD_PTE_PADDR_MASK,
                  MMU_L1_VECTORFLAGS);
}
#else
#  define t113_vectormapping()
#endif

static void t113_copyvectorblock(void)
{
  uint32_t *src  = (uint32_t *)_vector_start;
  uint32_t *end  = (uint32_t *)_vector_end;
  uint32_t *dest = (uint32_t *)(T113_VECTOR_VDDR);

  while (src < end)
    {
      *dest++ = *src++;
    }

  /* boot0-nuttx runs with both MMU and caches off (SCTLR.M=I=C=0).
   * Writes above reach RAM directly and instructions are fetched from
   * RAM without an I-cache, so no maintenance is required - only a
   * barrier pair to order this against the next instruction fetch.
   */

  __asm__ __volatile__("dsb sy");
  __asm__ __volatile__("isb sy");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_T113_BOOT0
/* NOTE: The overrides below (arm_enable_dbgmonitor, arm_dbgmonitor)
 * replace non-weak symbols from arm_hwdebug.c. This works because
 * archive linking resolves symbols from CHIP objects (t113_boot.o)
 * before CMN objects. If the build system changes to --whole-archive
 * or direct object linking, these will need weak upstream symbols or
 * a different guard.
 *
 * Skip debug monitor setup for boot0 - CP14 access causes UNDEF
 * because debug authentication is locked in the SRAM boot0 environment
 * (BROM doesn't enable external debug access).
 */

int arm_enable_dbgmonitor(void)
{
  return 0;
}

int arm_dbgmonitor(int irq, void *context, void *arg)
{
  return 0;
}
#endif

void arm_boot(void)
{
#ifdef CONFIG_T113_BOOT0
  uint32_t *p;
  uint32_t idle_top;
  uint32_t abt_sp;
  uint32_t und_sp;
  uint32_t irq_sp;

  /* BSS must be zeroed explicitly here before any C code that relies on
   * zero-initialized globals (including t113_setupmappings), because
   * arm_data_initialize() in arm_head.S does not zero BSS for this board.
   */

  p = (uint32_t *)&_sbss;
  while (p < (uint32_t *)&_ebss)
    {
      *p++ = 0;
    }

#endif

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_USE_MMU)
  t113_setupmappings();
  t113_vectormapping();
#endif

  t113_copyvectorblock();

#ifdef CONFIG_ARCH_LOWVECTORS
  __asm__ __volatile__(
    "mcr p15, 0, %0, c12, c0, 0\n"
    :
    : "r"(CONFIG_RAM_START)
    : "memory"
  );
#endif

#ifdef CONFIG_T113_BOOT0
  /* Set exception mode SPs - arm_vectordata uses srsdb sp!, #PSR_MODE_SYS
   * which writes to SYS mode SP. arm_head.S already sets SYS SP correctly.
   *
   * IMPORTANT: Do NOT set SYS SP here - arm_boot() was entered in SYS mode
   * and the compiler has pushed {r4, lr} onto the SYS stack. Resetting
   * SYS SP would lose the return address, causing pop {r4, pc} at function
   * exit to load garbage -> prefetch abort at 0xFFFFFFFE.
   *
   * Also must return to SYS mode (not SVC) since arm_boot() runs in SYS
   * mode throughout. Ending in SVC mode would make pop use the SVC stack
   * instead of the SYS stack where the return address was saved.
   */

  /* Use a small area after idle stack for exception mode stacks.
   * These overlap with heap, but exception vectors save context to the
   * SYS stack (via srsdb sp!, #PSR_MODE_SYS), so these banked SPs are
   * only used very briefly during the srsdb instruction itself.
   */

  idle_top = (uint32_t)(uintptr_t)&_ebss
             + CONFIG_IDLETHREAD_STACKSIZE;
  abt_sp = idle_top + 256;   /* 256 bytes for ABT stack */
  und_sp = abt_sp + 256;     /* 256 bytes for UND stack */
  irq_sp = und_sp + CONFIG_ARCH_INTERRUPTSTACK;

  __asm__ __volatile__(
    "cps #0x17\n"        /* ABT mode */
    "mov sp, %0\n"
    "cps #0x1B\n"        /* UND mode */
    "mov sp, %1\n"
    "cps #0x12\n"        /* IRQ mode */
    "mov sp, %2\n"
    "cps #0x1F\n"        /* back to SYS mode (NOT SVC!) */
    : : "r"(abt_sp), "r"(und_sp), "r"(irq_sp)
    : "memory"
  );
#endif

  arm_fpuconfig();

#ifdef CONFIG_BOOT_SDRAM_DATA
  arm_data_initialize();
#endif

  t113_clk_init();
  t113_lowsetup();

#ifdef CONFIG_ARCH_PERF_EVENTS
  up_perf_init((void *)(uintptr_t)T113_CPUX_FREQUENCY);
#endif

#ifdef USE_EARLYSERIALINIT
  arm_earlyserialinit();
#endif

  t113_boardinitialize();
}

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   boot0-nuttx primary heap lives in the SRAM tail above idle stack and
 *   exception-mode stacks, leaving ~50 KB of usable SRAM heap - enough
 *   for NSH and the SPI/MTD stack without touching DDR.
 *
 *   Layout (low -> high):
 *     _ebss
 *     idle stack    [CONFIG_IDLETHREAD_STACKSIZE]
 *     abt sp        [256 B]
 *     und sp        [256 B]
 *     irq sp        [CONFIG_ARCH_INTERRUPTSTACK]
 *     heap          [up to T113_SRAMA1_PADDR + 0x28000]
 *
 ****************************************************************************/

#ifdef CONFIG_T113_BOOT0

#  define BOOT0_SRAM_TOP     (T113_SRAMA1_PADDR + 0x28000)

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  uintptr_t base = (uintptr_t)&_ebss
                 + CONFIG_IDLETHREAD_STACKSIZE
                 + 256                            /* abt sp */
                 + 256                            /* und sp */
                 + CONFIG_ARCH_INTERRUPTSTACK;    /* irq sp */

  *heap_start = (FAR void *)base;
  *heap_size  = BOOT0_SRAM_TOP - base;
}

#else /* !CONFIG_T113_BOOT0 */

#  if defined(CONFIG_ARCH_HAVE_HEAP2) && CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
  kumm_addregion((FAR void *)(uintptr_t)CONFIG_HEAP2_BASE,
                 CONFIG_HEAP2_SIZE);
}
#  endif

#endif /* CONFIG_T113_BOOT0 */
