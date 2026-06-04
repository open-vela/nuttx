/****************************************************************************
 * arch/xtensa/src/t113/t113_head.S
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

#include "chip_memory.h"
#include "t113_trace.h"

    .section .ResetVector.text, "ax"
    .global _iram_start
    .align 4
_iram_start:
    j _iram_stub

    .section .iram_stub.text, "ax"
    .literal_position
    .global _iram_stub
    .align 4
_iram_stub:
    /* PS = WOE | UM (bits 18 and 5); INTLEVEL = 0.  NuttX flat builds run
     * the kernel in "user mode" (PS.UM=1) because the stock common path in
     * arch/xtensa/src/common implements the canonical handler in
     * _UserExceptionVector and only panics from _KernelExceptionVector.
     * A syscall instruction issued from kernel C code (e.g. the first
     * context switch out of the idle thread) raises a *user* exception
     * iff PS.UM=1; otherwise it raises a kernel exception which the
     * upstream code unconditionally panics on.
     *
     * Per Xtensa ISA only relevant bits here are PS.UM (5) and PS.WOE (18).
     * Set both: 0x40000 | 0x20 = 0x40020.
     */
    movi    a0, 0x40020
    wsr.ps  a0
    rsync

    /* Initialize windowed register file. */
    movi    a0, 1
    wsr.windowstart a0
    movi    a0, 0
    wsr.windowbase  a0
    rsync

    /* VECBASE = T113_IRAM_VECT_BASE (0x401000). */
    movi    a2, T113_IRAM_VECT_BASE
    wsr.vecbase a2
    isync

    /* All interrupts off. */
    movi    a0, 0
    wsr.intenable a0

    /* Enable all 8 coprocessor enable bits. */
    movi    a0, 0xff
    wsr.cpenable a0
    rsync

    /* Enable I-cache via MEMCTL bit 0. */
    rsr.memctl  a2
    movi        a3, 1
    or          a2, a2, a3
    wsr.memctl  a2
    rsync

    /* Full I-cache invalidate.
     *
     * The I-cache content at reset is undefined: power-up garbage on a
     * cold boot and stale lines from a prior firmware on a warm restart.
     * The DSP I-cache is not flushed by the AP before handoff, so stale
     * lines can cover any code about to execute -- including the IRAM
     * stub itself, the vector table at 0x401000, or the DDR .text we
     * jump into via `jx a2`.  Fetching a stale opcode causes a trap.
     *
     * Per Xtensa ISA, `iii a, imm` invalidates the I-cache line
     * containing (a + imm) regardless of way.  Walking every line
     * address (XCHAL_ICACHE_SIZE / XCHAL_ICACHE_LINESIZE iterations,
     * stepping by line size) covers all ways.  Geometry from
     * core-isa.h: 32 KB / 64-byte line / 2-way.
     *
     * Live registers at this point: a8/a9..a12 (UART scratch) not yet
     * set; we use a2 (loop addr) and a3 (remaining bytes).
     */
    movi    a2, 0                    /* base addr -- any aligned addr OK */
    movi    a3, 32768                /* XCHAL_ICACHE_SIZE */
1:  iii     a2, 0
    addi    a2, a2, 64               /* XCHAL_ICACHE_LINESIZE */
    addi    a3, a3, -64
    bnez    a3, 1b
    isync

    /* Initialize UART2 IP: LCR/DLL/DLH/FCR.  AP-side has already
     * powered UART2 (CCU + pin mux); the IP itself still needs LCR
     * baud divisor program before THRE will toggle.
     * UART2_BASE = 0x02500800; LCR offset 0x0c, DLL 0x00, DLH 0x04,
     * FCR 0x08; divisor = APB1(100M) / (16 * 115200) = 54.
     */
    movi    a8, 0x02500800           /* a8 = UART2 base */
    movi    a11, 0x83                /* LCR with DLAB=1 (8N1, dlab) */
    s32i    a11, a8, 0x0c            /* *LCR = 0x83 */
    movi    a11, 54                  /* DLL = divisor low */
    s32i    a11, a8, 0x00            /* *DLL = 54 */
    movi    a11, 0                   /* DLH = divisor high */
    s32i    a11, a8, 0x04            /* *DLH = 0 */
    movi    a11, 0x03                /* LCR with DLAB=0 (8N1, normal) */
    s32i    a11, a8, 0x0c            /* *LCR = 0x03 */
    movi    a11, 0x07                /* FCR = enable + clear RX/TX */
    s32i    a11, a8, 0x08            /* *FCR = 0x07 */

    /* -----------------------------------------------------------------
     * Early UART2 output: emit "NUTTX1\n" via direct LSR-poll so
     * the reset stub can be confirmed even before DDR-side __start
     * runs.  No .rodata reference (rodata is not in IRAM); bytes are
     * hardcoded as movi immediates.
     *
     * UART2 base = 0x02500800, LSR @ +0x14 (TX-empty bit 5),
     * RBR_THR @ +0x00.  Registers used: a8 (UART base), a9 (LSR addr),
     * a10 (LSR value), a11 (char), a12 (counter).  Avoids a0 (windowed
     * return-address holder) and a1 (will be the SP shortly).
     * ----------------------------------------------------------------- */
    movi    a8, 0x02500800
    addi    a9, a8, 0x14

    movi    a11, 'N'
1:  l32i    a10, a9, 0
    bbci    a10, 5, 1b
    s32i    a11, a8, 0
    movi    a11, 'U'
1:  l32i    a10, a9, 0
    bbci    a10, 5, 1b
    s32i    a11, a8, 0
    movi    a11, 'T'
1:  l32i    a10, a9, 0
    bbci    a10, 5, 1b
    s32i    a11, a8, 0
    movi    a11, 'T'
1:  l32i    a10, a9, 0
    bbci    a10, 5, 1b
    s32i    a11, a8, 0
    movi    a11, 'X'
1:  l32i    a10, a9, 0
    bbci    a10, 5, 1b
    s32i    a11, a8, 0
    movi    a11, '1'
1:  l32i    a10, a9, 0
    bbci    a10, 5, 1b
    s32i    a11, a8, 0
    movi    a11, 10
1:  l32i    a10, a9, 0
    bbci    a10, 5, 1b
    s32i    a11, a8, 0

    /* (a12 reserved for future counter use; unused here but keeps the
     * register allocation list stable should we extend the tracer.)
     */
    movi    a12, 0

#ifdef CONFIG_T113_TRACE
    /* Initialize the breadcrumb trace ring (TRACE_BASE = 0x17A04400)
     * directly here since DDR-side __start has not yet run and the C
     * t113_trace_init() will overwrite this immediately.  Doing it
     * inline lets a TRACE_BC fire from the IRAM stub itself.
     *
     * Header layout: [magic, head_idx, _, _].  Use a13 (base), a14
     * (value), a15 (zero) -- all dead at this point.
     */
    movi    a13, TRACE_BASE
    movi    a14, TRACE_MAGIC
    s32i    a14, a13, TRACE_MAGIC_OFF
    movi    a15, 0
    s32i    a15, a13, TRACE_HEAD_OFF

    /* Drop EV_RESET breadcrumb.  a8..a10 were UART scratches; reuse them
     * as the macro's scratches.  val1/val2 = 0.
     */
    movi    a11, 0
    TRACE_BC_ASM EV_RESET, a11, a11, a8, a9, a10
#endif /* CONFIG_T113_TRACE */

    /* SP = top of DDR. */
    .literal .Lstack_top, T113_STACK_TOP
    l32r    a1, .Lstack_top

    /* Jump to NuttX __start in DDR .text. */
    .literal .Lstart, __start
    l32r    a2, .Lstart
    jx      a2
