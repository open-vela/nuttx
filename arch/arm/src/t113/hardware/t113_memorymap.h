/****************************************************************************
 * arch/arm/src/t113/hardware/t113_memorymap.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * T113-S3 and R528 are the same silicon - peripheral base addresses
 * are identical on both variants.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_T113_HARDWARE_T113_MEMORYMAP_H
#define __ARCH_ARM_SRC_T113_HARDWARE_T113_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#define __CONCAT(a,b) a ## b
#define MKULONG(a) __CONCAT(a,ul)

/* T113-S3 physical section base addresses (1MB aligned) */

#define T113_SRAM_PSECTION  0x00000000
#define T113_SP0_PSECTION     0x02000000
#define T113_SP1_PSECTION     0x02500000
#define T113_SH0_PSECTION     0x03000000
#define T113_SH2_PSECTION     0x04000000
#define T113_PERIPH_PSECTION  0x05000000
#define T113_APBS0_PSECTION   0x07000000
#define T113_CPUX_PSECTION    0x08100000
#define T113_DRAM_PSECTION    0x40000000
#define T113_BROM_PSECTION    0x00000000

/* Offsets */

#define T113_SRAMA1_OFFSET    0x00020000
#define T113_CCMU_OFFSET      0x00001000
#define T113_UART0_OFFSET     0x00000000
#define T113_UART1_OFFSET     0x00000400
#define T113_UART2_OFFSET     0x00000800
#define T113_UART3_OFFSET     0x00000C00
#define T113_UART4_OFFSET     0x00001000
#define T113_UART5_OFFSET     0x00001400
#define T113_GIC_OFFSET       0x00020000

/* SMHC controllers live in the SH2 domain (0x04000000).
 * Offsets relative to T113_SH2_PSECTION.
 */

#define T113_SMHC0_OFFSET     0x00020000
#define T113_SMHC1_OFFSET     0x00021000
#define T113_SMHC2_OFFSET     0x00022000

/* Sizes */

#define T113_SRAM_SIZE      0x00100000
#define T113_SP0_SIZE         0x00100000
#define T113_SP1_SIZE         0x00100000
#define T113_SH0_SIZE         0x00400000
#define T113_SH2_SIZE         0x00600000
#define T113_PERIPH_SIZE      0x00700000
#define T113_APBS0_SIZE       0x00100000
#define T113_CPUX_SIZE        0x01000000
#define T113_BROM_SIZE        0x0000c000

#define T113_DDR_MAPSIZE      MKULONG(CONFIG_T113_DDR_MAPSIZE)

#define _NSECTIONS(b)         (((b)+0x000fffff) >> 20)

/* Section counts are independent of CONFIG_T113_BOOT0: boot0 runs from SRAM
 * but initializes DDR before jumping to NuttX, and IO sections always span
 * their full range (USB PHY, CPUX etc. cover multiple MB).
 */

#define T113_SRAM_NSECTIONS   _NSECTIONS(T113_SRAM_SIZE)
#define T113_SP0_NSECTIONS    _NSECTIONS(T113_SP0_SIZE)
#define T113_SP1_NSECTIONS    _NSECTIONS(T113_SP1_SIZE)
#define T113_SH0_NSECTIONS    _NSECTIONS(T113_SH0_SIZE)
#define T113_SH2_NSECTIONS    _NSECTIONS(T113_SH2_SIZE)
#define T113_PERIPH_NSECTIONS _NSECTIONS(T113_PERIPH_SIZE)
#define T113_APBS0_NSECTIONS  _NSECTIONS(T113_APBS0_SIZE)
#define T113_CPUX_NSECTIONS   _NSECTIONS(T113_CPUX_SIZE)
#define T113_DDR_NSECTIONS    _NSECTIONS(T113_DDR_MAPSIZE)

/* MMU flags */

#define T113_SRAM_MMUFLAGS  MMU_MEMFLAGS
#define T113_SP0_MMUFLAGS     MMU_IOFLAGS
#define T113_SP1_MMUFLAGS     MMU_IOFLAGS
#define T113_SH0_MMUFLAGS     MMU_IOFLAGS
#define T113_SH2_MMUFLAGS     MMU_IOFLAGS
#define T113_PERIPH_MMUFLAGS  MMU_IOFLAGS
#define T113_APBS0_MMUFLAGS   MMU_IOFLAGS
#define T113_CPUX_MMUFLAGS    MMU_IOFLAGS
#define T113_DDR_MMUFLAGS     MMU_MEMFLAGS
#define T113_BROM_MMUFLAGS    MMU_IOFLAGS

/* Non-cacheable Normal memory for DSP / RPMSG shared region.
 *
 * Attributes: Normal Non-Cacheable, Shareable, Privileged R/W,
 * Execute Never.  TEX=001, C=0, B=0 with S=1 gives Normal
 * Non-Cacheable Shareable per ARMv7-A architecture reference.
 * Avoids cache coherency issues between AP and DSP for any data
 * placed in this region.
 */

#define T113_NONCACHE_DDR_MMUFLAGS \
        (PMD_TYPE_SECT | PMD_SECT_AP_RW1 |          \
         (1 << PMD_SECT_TEX_SHIFT) |                \
         PMD_SECT_S | PMD_SECT_XN |                 \
         PMD_SECT_DOM(0))

/* DSP shared DDR window: fixed 5 MB at 0x47900000 (4 MB fw + 1 MB shmem).
 * The Xtensa firmware links here; the AP maps it Non-Cacheable.
 */

#define T113_DSP_RESERVED_SIZE     0x00500000   /* 5 MB DSP fw + shmem */
#define T113_DSP_SHMEM_PADDR       0x47900000u
#define T113_DSP_SHMEM_VADDR       T113_DSP_SHMEM_PADDR  /* identity-mapped */
#define T113_DSP_SHMEM_NSECTIONS \
        _NSECTIONS(T113_DSP_RESERVED_SIZE)

/* ramlog: 896 KB at the top of DDR, above the 128 KB TEE<->REE shmem
 * (0x47F00000, reserved for a future TEE).  Address-plan reservation only --
 * not consumed yet; RAMLOG still uses its default in-image BSS buffer.
 */

#define T113_RAMLOG_RESERVED_SIZE  0x000E0000   /* 896 KB ramlog (reserved) */

/* core0 <-> core1 rpmsg carveout: exactly one 1 MB MMU section (vrings +
 * buffer pool + life-sign cell).  Single source of truth for the size --
 * t113_rptun_slave.c derives its SHMEM_SIZE from this.
 */

#define T113_CPU1_CARVEOUT_SIZE      0x00100000u   /* 1 MB, one MMU section */

#ifdef CONFIG_T113_RPTUN_CORE1
/* core0 loads the core1 NuttX ELF into DRAM and shares the rpmsg vring
 * window (1 MB carveout) with core1.  Map only the carveout Normal
 * Non-Cacheable, identity (PA==VA): the vring window must be NC (no
 * inter-core cache
 * coherency in this split).  The core1 image now lives separately at its own
 * RAM_START (cacheable, far below), so it is NOT part of this NC mapping.
 */

#  define T113_CPU1_SHMEM_PADDR      CONFIG_T113_RPTUN_CORE1_CARVEOUT_BASE
#  define T113_CPU1_SHMEM_VADDR      CONFIG_T113_RPTUN_CORE1_CARVEOUT_BASE /* identity */
#  define T113_CPU1_SHMEM_NSECTIONS  _NSECTIONS(T113_CPU1_CARVEOUT_SIZE)
#endif

/* Physical base addresses */

#define T113_SRAMA1_PADDR     (T113_SRAM_PSECTION + T113_SRAMA1_OFFSET)
#define T113_UART0_PADDR      (T113_SP1_PSECTION + T113_UART0_OFFSET)
#define T113_UART1_PADDR      (T113_SP1_PSECTION + T113_UART1_OFFSET)
#define T113_UART2_PADDR      (T113_SP1_PSECTION + T113_UART2_OFFSET)
#define T113_UART3_PADDR      (T113_SP1_PSECTION + T113_UART3_OFFSET)
#define T113_UART4_PADDR      (T113_SP1_PSECTION + T113_UART4_OFFSET)
#define T113_UART5_PADDR      (T113_SP1_PSECTION + T113_UART5_OFFSET)
#define T113_GIC_PADDR        (T113_SH0_PSECTION + T113_GIC_OFFSET)
#define T113_GIC_DIST_PADDR   (T113_GIC_PADDR + 0x1000)
#define T113_GIC_CPU_PADDR    (T113_GIC_PADDR + 0x2000)

/* SMHC physical base addresses */

#define T113_SMHC0_PADDR      (T113_SH2_PSECTION + T113_SMHC0_OFFSET)
#define T113_SMHC1_PADDR      (T113_SH2_PSECTION + T113_SMHC1_OFFSET)
#define T113_SMHC2_PADDR      (T113_SH2_PSECTION + T113_SMHC2_OFFSET)

/* Virtual section base addresses (identity-mapped 1:1) */

#ifndef CONFIG_ARCH_ROMPGTABLE
#define T113_SRAM_VSECTION  0x00000000
#define T113_SP0_VSECTION     0x02000000
#define T113_SP1_VSECTION     0x02500000
#define T113_SH0_VSECTION     0x03000000
#define T113_SH2_VSECTION     0x04000000
#define T113_PERIPH_VSECTION  0x05000000
#define T113_APBS0_VSECTION   0x07000000
#define T113_CPUX_VSECTION    0x08100000
#define T113_DRAM_VSECTION    0x40000000
#define T113_BROM_VSECTION    0x00000000
#endif

/* Virtual base addresses */

#define T113_SRAMA1_VADDR     (T113_SRAM_VSECTION + T113_SRAMA1_OFFSET)
#define T113_UART0_VADDR      (T113_SP1_VSECTION + T113_UART0_OFFSET)
#define T113_UART1_VADDR      (T113_SP1_VSECTION + T113_UART1_OFFSET)
#define T113_UART2_VADDR      (T113_SP1_VSECTION + T113_UART2_OFFSET)
#define T113_UART3_VADDR      (T113_SP1_VSECTION + T113_UART3_OFFSET)
#define T113_UART4_VADDR      (T113_SP1_VSECTION + T113_UART4_OFFSET)
#define T113_UART5_VADDR      (T113_SP1_VSECTION + T113_UART5_OFFSET)
#define T113_GIC_VADDR        (T113_SH0_VSECTION + T113_GIC_OFFSET)
#define T113_UART_VADDR(n)    (T113_UART0_VADDR + (n) * 0x400)

/* DDR mapping */

#if defined(CONFIG_T113_RPTUN_MASTER)
/* AP master: it loads the remote images (core1 and/or the DSP) into DRAM
 * below the DSP window, so its cacheable map must cover that whole span up
 * to the window base (the loader memcpy's there, then flushes to PoC for the
 * cache-cold remotes).  The high-DDR NC zone (DSP shmem, carveout, TEE
 * shmem) sits above the window with no cache-attribute aliasing.  Stopping
 * the map
 * at T113_DSP_SHMEM_PADDR is independent of RAM_SIZE, which only bounds the
 * master's own heap, not the images it stages for the remotes.
 */

#  define T113_DDR_MAPPADDR     CONFIG_RAM_START
#  define T113_DDR_MAPVADDR     CONFIG_RAM_VSTART
#  define T113_DDR_MAPSECTIONS  _NSECTIONS(T113_DSP_SHMEM_PADDR - CONFIG_RAM_START)
#elif defined(CONFIG_T113_AMP)
#  define T113_DDR_MAPPADDR     CONFIG_RAM_START
#  define T113_DDR_MAPVADDR     CONFIG_RAM_VSTART
#  define T113_DDR_MAPSECTIONS  _NSECTIONS(CONFIG_RAM_SIZE)
#else
#  define T113_DDR_MAPPADDR     T113_DRAM_PSECTION
#  define T113_DDR_MAPVADDR     T113_DRAM_VSECTION
#  define T113_DDR_MAPSECTIONS  T113_DDR_NSECTIONS
#endif

/* GIC base for armv7-a layer */

#define CHIP_MPCORE_VBASE     T113_GIC_VADDR

/* NuttX virtual base address - used by arm_head.S */

#define NUTTX_TEXT_VADDR      (CONFIG_RAM_VSTART & 0xfff00000)
#define NUTTX_TEXT_PADDR      (CONFIG_RAM_START & 0xfff00000)
#define NUTTX_TEXT_PEND       ((CONFIG_RAM_END + 0x000fffff) & 0xfff00000)
#define NUTTX_TEXT_SIZE       (NUTTX_TEXT_PEND - NUTTX_TEXT_PADDR)

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_USE_MMU)
#if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)
#  if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif
#elif defined(CONFIG_T113_BOOT0)
  /* boot0 mode: NuttX runs from SRAM.  L1 page table (16KB) at end
   * of SRAM (0x44000-0x47FFF).  sram.ld limits code/data to 144KB
   * (0x20000-0x43FFF).  DDR is entirely for HEAP2.
   */

#  define PGTABLE_BASE_PADDR  (T113_SRAMA1_PADDR + 0x24000) /* 0x44000 */
#  define PGTABLE_BASE_VADDR  (T113_SRAMA1_VADDR + 0x24000) /* 0x44000 */
#elif defined(CONFIG_T113_RPTUN_SLAVE)
#  define PGTABLE_BASE_PADDR  (T113_SRAMA1_PADDR + 0x4000)
#  define PGTABLE_BASE_VADDR  (T113_SRAMA1_VADDR + 0x4000)
#  define ARMV7A_PGTABLE_MAPPING 1
#else
#  define PGTABLE_BASE_PADDR  T113_SRAMA1_PADDR
#  define PGTABLE_BASE_VADDR  T113_SRAMA1_VADDR
#  define ARMV7A_PGTABLE_MAPPING 1
#endif
#endif

/* L2 page table for high vector remapping
 * (only when not using low vectors)
 */

#ifndef CONFIG_ARCH_LOWVECTORS
#  define VECTOR_L2_OFFSET        0x00000400
#  define VECTOR_L2_SIZE          0x00000bfc
#  define VECTOR_L2_PBASE         (PGTABLE_BASE_PADDR + VECTOR_L2_OFFSET)
#  define VECTOR_L2_VBASE         (PGTABLE_BASE_VADDR + VECTOR_L2_OFFSET)
#  define VECTOR_L2_END_PADDR     (VECTOR_L2_PBASE + VECTOR_L2_SIZE)
#  define VECTOR_L2_END_VADDR     (VECTOR_L2_VBASE + VECTOR_L2_SIZE)
#endif

#define VECTOR_TABLE_SIZE         0x00010000
#define VECTOR_TABLE_OFFSET       0x00000040

/* Vector table lives at DDR start (0x40000000), NOT in SRAM */

#define T113_VECTOR_PADDR         CONFIG_RAM_START
#define T113_VECTOR_VDDR          T113_VECTOR_PADDR
#define T113_VECTOR_VADDR         0x00000000

/* Paging L2 page table */

#define PGTABLE_L2_START_PADDR    (T113_DRAM_PSECTION + T113_DDR_MAPSIZE)
#define PGTABLE_BROM_OFFSET       0x3ffc
#define PGTABLE_L2_OFFSET         ((PGTABLE_L2_START_PADDR >> 18) & ~3)
#define PGTABLE_L2_SIZE           (PGTABLE_BROM_OFFSET - PGTABLE_L2_OFFSET)
#define PGTABLE_L2_PBASE          (PGTABLE_BASE_PADDR + PGTABLE_L2_OFFSET)
#define PGTABLE_L2_VBASE          (PGTABLE_BASE_VADDR + PGTABLE_L2_OFFSET)
#define PGTABLE_L2_END_PADDR      (PGTABLE_L2_PBASE + PGTABLE_L2_SIZE)
#define PGTABLE_L2_END_VADDR      (PGTABLE_L2_VBASE + PGTABLE_L2_SIZE)

/* Display subsystem (R528 DE v2x) */

#define T113_DE_BASE              0x05000000
#define T113_DSI_BASE             0x05450000
#define T113_DISPLAY_TOP_BASE     0x05460000
#define T113_TCON_LCD0_BASE       0x05461000

#endif /* __ARCH_ARM_SRC_T113_HARDWARE_T113_MEMORYMAP_H */
