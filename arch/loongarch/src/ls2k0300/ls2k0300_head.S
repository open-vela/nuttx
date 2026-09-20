/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_head.S
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

#include <arch/irq.h>
#include <arch/csr.h>

#include "chip.h"
#include "loongarch_internal.h"
#include "hardware/ls2k0300_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 *
 * Additional constants not in ls2k0300_memorymap.h
 ****************************************************************************/

#ifndef CONFIG_LS2K0300_UBOOT_BOOT
#define BOOT_SPACE_BASE         0x1c000000
#define LS2K0300_IDLESTACK_TOP   (CACHED_MEMORY_ADDR | LS2K0300_L2CACHE_BASE | LS2K0300_L2CACHE_SIZE)
#else
#define BOOT_SPACE_BASE         0x98000000
#define LS2K0300_IDLESTACK_TOP   (CACHED_MEMORY_ADDR | BOOT_SPACE_BASE | 0x800000)
#endif

#define EARLY_UART_BASE         (UNCACHED_MEMORY_ADDR | 0x16100000)
#define EARLY_UART_THR          (EARLY_UART_BASE + 0x00)
#define EARLY_UART_LSR          (EARLY_UART_BASE + 0x05)
#define EARLY_UART_LSR_THRE     (1 << 5)

  .macro EARLY_UART_INIT
  li.d    $t2, EARLY_UART_BASE
  li.w    $t3, 0x80
  st.b    $t3, $t2, 3
  li.w    $t3, 108
  st.b    $t3, $t2, 0
  li.w    $t3, 0
  st.b    $t3, $t2, 1
  li.w    $t3, 0x03
  st.b    $t3, $t2, 3
  li.w    $t3, 0x07
  st.b    $t3, $t2, 2
  .endm

  .macro EARLY_PUTC ch
  li.d    $t2, EARLY_UART_LSR
1:
  ld.b    $t3, $t2, 0
  andi    $t3, $t3, EARLY_UART_LSR_THRE
  beqz    $t3, 1b
  li.d    $t2, EARLY_UART_THR
  li.w    $t3, \ch
  st.b    $t3, $t2, 0
  .endm

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .section .text.__start, "ax", @progbits
  .global __start
  .type  __start, function

__start:

#ifndef CONFIG_LS2K0300_UBOOT_BOOT
  /* CPU feature register configuration - only needed for direct boot,
   * skip when booting from U-Boot (U-Boot has already configured these)
   */
  li.d    $t0, (1 << (24 + 32))
  csrxchg $t0, $t0, LOONGARCH_CSR_PRID

  li.d    $t0, (0xa << (16 + 32))
  li.d    $t1, (0xff << (16 + 32))
  csrxchg $t0, $t1, LOONGARCH_CSR_MCSR9

  li.w    $t0, (0x1 << 16)
  csrxchg $t0, $t0, LOONGARCH_CSR_PERFCTRL0

  li.w    $t0, (0x1 << 9)
  csrxchg $t0, $t0, LOONGARCH_CSR_IMPCTL1

  li.w    $t0, (0x1 << 12)
  csrxchg $r0, $t0, LOONGARCH_CSR_IMPCTL1

  li.w    $t0, 0xe
  csrxchg $t0, $t0, LOONGARCH_CSR_MCSR24
#endif

  /* Configure DMWIN0 (uncached) and DMWIN1 (cached)
   * For U-Boot boot: trust U-Boot's cache cleanup, no need for
   * additional cache invalidation (avoiding self-referential paradox)
   */
  li.d    $t0, UNCACHED_MEMORY_ADDR | 0xf
  csrwr   $t0, LOONGARCH_CSR_DMWIN0
  li.d    $t0, CACHED_MEMORY_ADDR | 0x1f
  csrwr   $t0, LOONGARCH_CSR_DMWIN1

  li.d    $t0, (0x7 << 16)
  csrxchg $r0, $t0, LOONGARCH_CSR_ECFG

  li.d    $t0, PHYS_TO_CACHED(BOOT_SPACE_BASE + 0x1000)
  csrwr   $t0, LOONGARCH_CSR_EBASE

  li.d    $t0, PHYS_TO_CACHED(BOOT_SPACE_BASE + 0x2000)
  csrwr   $t0, LOONGARCH_CSR_TLBREBASE

  li.w    $t0, (1 << 3)
  csrxchg $r0, $t0, LOONGARCH_CSR_MCSR1

  li.d    $t0, (1 << 2)
  csrxchg $r0, $t0, LOONGARCH_CSR_CRMD

#ifndef CONFIG_LS2K0300_UBOOT_BOOT
  bl      ls2k0300_lowlevel_init
  nop

  bl      ls2k0300_lock_scache
  nop


  li.d    $t1, CACHED_MEMORY_ADDR
  bl      1f
1:
  addi.d  $t0, $ra, 12
  or      $t0, $t1, $t0
  jirl    $zero, $t0, 0

  li.w    $t0, 0xb0
  csrwr   $t0, LOONGARCH_CSR_CRMD
#endif

  li.d    $t0, LS2K0300_IDLESTACK_TOP
  move    $sp, $t0

#ifndef CONFIG_LS2K0300_UBOOT_BOOT
  EARLY_UART_INIT
  la      $a0, hello_str
  bl      early_print
  nop

  dbar    0
  ibar    0
#endif

  bl      __ls2k0300_start
  nop

  .global _init
  .global _fini

_init:
_fini:
  jirl    $zero, $ra, 0

  .size __start, . - __start

/****************************************************************************
 * Early print string
 ****************************************************************************/

  .section .text.early_print, "ax", @progbits
  .global early_print
  .type  early_print, function

early_print:
  move    $t4, $a0
1:
  ld.b    $t5, $t4, 0
  beqz    $t5, 2f
  li.d    $t2, EARLY_UART_LSR
3:
  ld.b    $t3, $t2, 0
  andi    $t3, $t3, EARLY_UART_LSR_THRE
  beqz    $t3, 3b
  li.d    $t2, EARLY_UART_THR
  st.b    $t5, $t2, 0
  addi.d  $t4, $t4, 1
  b       1b
2:
  jirl    $zero, $ra, 0

  .size early_print, . - early_print

hello_str:
  .ascii "\r\nHello, LoongArch!!!\r\n\0"

/****************************************************************************
 * Low-level initialization
 ****************************************************************************/

  .section .text.ls2k0300_lowlevel_init, "ax", @progbits
  .global ls2k0300_lowlevel_init
  .type  ls2k0300_lowlevel_init, function

ls2k0300_lowlevel_init:

  move    $a4, $ra

  li.d    $t0, 0x0001000107270e00
  csrwr   $t0, LOONGARCH_CSR_MCSR2

  li.d    $t0, PHYS_TO_UNCACHED(LS2K0300_GENERAL_CFG0)
  ld.w    $t1, $t0, 0x0
  li.w    $t2, (1 << 5)
  or      $t1, $t1, $t2
  st.w    $t1, $t0, 0x0

  li.d    $t0, PHYS_TO_UNCACHED(LS2K0300_SPI0_BASE)
  li.w    $t1, 0x47
  st.b    $t1, $t0, 0x4

  li.w    $t1, 0xffffff7e
  st.w    $t1, $t0, 0x1c

  ld.w    $t1, $t0, 0x1c
  ori     $t1, $t1, 0x1
  st.w    $t1, $t0, 0x1c

  li.d    $t0, PHYS_TO_UNCACHED(LS2K0300_GENERAL_CFG3)
  ld.w    $t1, $t0, 0x0
  li.w    $t2, ((0x1f << 1) | (0x1f << 9))
  or      $t1, $t1, $t2
  st.w    $t1, $t0, 0x0

  li.d    $t0, PHYS_TO_UNCACHED(LS2K0300_GPIO_32_47_MULTI_CFG)
  li.w    $t2, 0xf0000
  st.w    $t2, $t0, 0x0

  bl      ls2k0300_clk_init
  nop

  move    $ra, $a4
  jirl    $zero, $ra, 0

  .size ls2k0300_lowlevel_init, . - ls2k0300_lowlevel_init

/****************************************************************************
 * Clock initialization (PLL configuration)
 *
 * REF_FREQ = 100MHz, CORE_FREQ = 1000MHz, DDR_FREQ = 800MHz
 * REF_FREQ = 120MHz, CORE_FREQ = 1000MHz, DDR_FREQ = 800MHz
 * APB_FREQ = 200MHz, NET_FREQ = 200MHz
 *
 * NODE_PLL: REFC=6, DIV=2, LOOPC=100
 *   GMAC_DIV=16, I2S_DIV=20
 * DDR_PLL: REFC=3, DIV=2, LOOPC=40
 *   DEV_DIV=8, NETWORK_DIV=8
 ****************************************************************************/

  .section .text.ls2k0300_clk_init, "ax", @progbits
  .global ls2k0300_clk_init
  .type  ls2k0300_clk_init, function

ls2k0300_clk_init:

#define NODE_L1DIV_OUT_SHIFT    24
#define NODE_L1DIV_LOOPC_SHIFT  15
#define NODE_L1DIV_REF_SHIFT    8
#define NODE_L2DIV_OUT_I2S_SHIFT 8
#define NODE_L2DIV_OUT_GMAC_SHIFT 0
#define NODE_L2DIV_OUT_I2S_MARK  0x7f
#define NODE_L2DIV_OUT_GMAC_MARK 0x7f

#define DDR_L1DIV_OUT_SHIFT     24
#define DDR_L1DIV_LOOPC_SHIFT   15
#define DDR_L1DIV_REF_SHIFT     8
#define DDR_L2DIV_OUT_MEMDIV_SHIFT 18
#define DDR_L2DIV_OUT_DEV_SHIFT  8
#define DDR_L2DIV_OUT_NET_SHIFT  0
#define DDR_L2DIV_OUT_MEMDIV_MARK 0x3
#define DDR_L2DIV_OUT_DEV_MARK   0x7f
#define DDR_L2DIV_OUT_NET_MARK   0x7f

#define PLL_L1_ENA      (0x1 << 3)
#define PLL_L1_PD_PLL   (0x1 << 5)
#define PLL_L1_LOCKED   (0x1 << 7)
#define SEL_PLL0        (0x1)
#define SEL_PLL1        (0x2)
#define SEL_PLL2        (0x4)

  li.d    $t0, LS2K0300_NODE_PLL_L
  li.w    $t1, PLL_L1_PD_PLL
  st.w    $t1, $t0, 0
  li.w    $t4, (2 << NODE_L1DIV_OUT_SHIFT) | (100 << NODE_L1DIV_LOOPC_SHIFT) | (6 << NODE_L1DIV_REF_SHIFT)
  st.w    $t4, $t0, 0

  ld.w    $t1, $t0, 4
  li.w    $t2, ~((NODE_L2DIV_OUT_I2S_MARK << NODE_L2DIV_OUT_I2S_SHIFT) | NODE_L2DIV_OUT_GMAC_MARK)
  and     $t1, $t1, $t2
  li.w    $t2, (20 << NODE_L2DIV_OUT_I2S_SHIFT) | 16
  or      $t2, $t2, $t1
  st.w    $t2, $t0, 0x4

  ori     $t4, $t4, PLL_L1_ENA
  st.w    $t4, $t0, 0

1:
  ld.w    $a0, $t0, 0
  li.w    $a1, PLL_L1_LOCKED
  and     $a0, $a1, $a0
  beqz    $a0, 1b

  ld.w    $a0, $t0, 0
  ori     $a0, $a0, SEL_PLL0 | SEL_PLL1 | SEL_PLL2
  st.w    $a0, $t0, 0

  li.d    $t0, LS2K0300_DDR_PLL_L
  ld.w    $t1, $t0, 4
  li.w    $t2, ~(DDR_L2DIV_OUT_MEMDIV_MARK << DDR_L2DIV_OUT_MEMDIV_SHIFT)
  and     $t1, $t1, $t2
  li.w    $t2, (1 << DDR_L2DIV_OUT_MEMDIV_SHIFT)
  or      $t2, $t2, $t1
  st.w    $t2, $t0, 0x4

  ld.w    $t1, $t0, 4
  li.w    $t2, ~(1 << 16)
  and     $t2, $t1, $t2
  st.w    $t2, $t0, 0x4
  li.w    $t2, (1 << 16)
  or      $t2, $t2, $t1
  st.w    $t2, $t0, 0x4

  li.w    $t1, PLL_L1_PD_PLL
  st.w    $t1, $t0, 0
  li.w    $t4, (2 << DDR_L1DIV_OUT_SHIFT) | (40 << DDR_L1DIV_LOOPC_SHIFT) | (3 << DDR_L1DIV_REF_SHIFT)
  st.w    $t4, $t0, 0

  ld.w    $t1, $t0, 4
  li.w    $t2, ~((DDR_L2DIV_OUT_DEV_MARK << DDR_L2DIV_OUT_DEV_SHIFT) | DDR_L2DIV_OUT_NET_MARK)
  and     $t1, $t1, $t2
  li.w    $t2, (8 << DDR_L2DIV_OUT_DEV_SHIFT) | 8
  or      $t2, $t2, $t1
  st.w    $t2, $t0, 0x4

  ori     $t4, $t4, PLL_L1_ENA
  st.w    $t4, $t0, 0

2:
  ld.w    $a0, $t0, 0
  li.w    $a1, PLL_L1_LOCKED
  and     $a0, $a0, $a1
  beqz    $a0, 2b

  ld.w    $a0, $t0, 0
  ori     $a0, $a0, SEL_PLL0 | SEL_PLL1 | SEL_PLL2
  st.w    $a0, $t0, 0

  li.w    $t1, 0x100
3:
  addi.d  $t1, $t1, -1
  bnez    $t1, 3b
  nop

  jirl    $zero, $ra, 0

  .size ls2k0300_clk_init, . - ls2k0300_clk_init

/****************************************************************************
 * Lock L2 cache as memory
 ****************************************************************************/

  .section .text.ls2k0300_lock_scache, "ax", @progbits
  .global ls2k0300_lock_scache
  .type  ls2k0300_lock_scache, function

ls2k0300_lock_scache:

  li.d    $t0, PHYS_TO_UNCACHED(LS2K0300_SCACHE_LOCK_WIN0_BASE)
  li.d    $t1, ~(LS2K0300_L2CACHE_SIZE - 1)
  st.d    $t1, $t0, 0x40
  li.d    $t1, (LS2K0300_L2CACHE_BASE & 0xffffffffffff) | (1 << 63)
  st.d    $t1, $t0, 0x0

  li.w    $t1, 0x100
1:
  addi.d  $t1, $t1, -1
  bnez    $t1, 1b
  nop

  jirl    $zero, $ra, 0

  .size ls2k0300_lock_scache, . - ls2k0300_lock_scache

/****************************************************************************
 * Exception vectors
 ****************************************************************************/

  .section .text.exception_vector, "ax", @progbits
  .global exception_vector
  .type  exception_vector, function

exception_vector:

  addi.d  $sp, $sp, -XCPTCONTEXT_SIZE

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

  addi.d  $s0, $sp, XCPTCONTEXT_SIZE
  st.d    $s0,  $sp, (REGSIZE * REG_R3_NDX)

  csrrd   $s0, LOONGARCH_CSR_ERA
  st.d    $s0, $sp, (REGSIZE * REG_EPC_NDX)

  csrrd   $s0, LOONGARCH_CSR_PRMD
  st.d    $s0, $sp, (REGSIZE * REG_INT_CTX_NDX)

  csrrd   $s1, LOONGARCH_CSR_ESTAT
  srli.d  $s2, $s1, CSR_ESTAT_EXC_SHIFT
  andi    $s2, $s2, 0x3f

  bnez    $s2, .Lexc_handle_exception

  li.d    $s3, 0x7fff
  and     $s3, $s1, $s3
  beqz    $s3, .Lexc_spurious_interrupt

  ctz.d   $s1, $s3
  addi.d  $s1, $s1, EXCCODE_INT_START
  b       .Lexc_has_irq

.Lexc_handle_exception:
  move    $a0, $s2
  move    $a1, $sp
  bl      loongarch_dispatch_irq
  b       .Lexc_return

.Lexc_spurious_interrupt:
  move    $a0, $sp
  b       .Lexc_return

.Lexc_has_irq:
  move    $a0, $s1
  move    $a1, $sp
  bl      loongarch_dispatch_irq
  b       .Lexc_return

.Lexc_return:
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

  .size exception_vector, . - exception_vector

/****************************************************************************
 * TLB exception vector
 ****************************************************************************/

  .section .text.tlb_exception_vector, "ax", @progbits
  .global tlb_exception_vector
  .type  tlb_exception_vector, function

tlb_exception_vector:

  addi.d  $sp, $sp, -XCPTCONTEXT_SIZE

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

  addi.d  $t1, $sp, XCPTCONTEXT_SIZE
  st.d    $t1,  $sp, (REGSIZE * REG_R3_NDX)

  csrrd   $t1, LOONGARCH_CSR_TLBRERA
  st.d    $t1, $sp, (REGSIZE * REG_EPC_NDX)

  csrrd   $t1, LOONGARCH_CSR_PRMD
  st.d    $t1, $sp, (REGSIZE * REG_INT_CTX_NDX)

  li.w    $a0, LOONGARCH_IRQ_ASYNC
  move    $a1, $sp
  li.w    $a2, 0

  bl      loongarch_exception
  nop

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

  ld.d    $t0, $sp, (REGSIZE * REG_EPC_NDX)
  csrwr   $t0, LOONGARCH_CSR_TLBRERA

  ld.d    $t0, $sp, (REGSIZE * REG_INT_CTX_NDX)
  csrwr   $t0, LOONGARCH_CSR_PRMD

  ld.d    $sp, $sp, (REGSIZE * REG_R3_NDX)

  ertn

  .size tlb_exception_vector, . - tlb_exception_vector

  .end
