/****************************************************************************
 * arch/loongarch/src/ls2k0300/ls2k0300_dma_cache.S
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ls2k0300_dma_flush
 *
 * Description:
 *   Flush (writeback + invalidate) D-cache lines covering a memory range.
 *   After this call the DMA engine will see the latest data written by CPU.
 *
 * Input Parameters:
 *   a0 - Start address (need not be cache-line aligned)
 *   a1 - Length in bytes
 *
 * LoongArch cache instruction encoding:
 *   [31:26]=000000 [25:22]=0010 [21:10]=si12 [9:5]=rj [4:0]=op
 *   cache 0x15 = Hit Writeback Invalidate D-Cache (64B cache line)
 *   For rj=$a0(4), op=0x15, si12=0: .word = 0x800095
 ****************************************************************************/

    .text
    .globl  ls2k0300_dma_flush
    .type   ls2k0300_dma_flush, function

ls2k0300_dma_flush:
    /* a0 = addr, a1 = len */
    beqz    $a1, 3f              /* nothing to do if len == 0 */

    /* end = addr + len, computed BEFORE the start is rounded down.
     * Doing it the other way round (end = aligned_start + len) shortens
     * the range by the misalignment and leaves the last cache line
     * untouched, which for an audio buffer means the tail of every DMA
     * block keeps stale CPU data and clicks on playback. */
    add.d   $a1, $a0, $a1

    /* Align start down to 64-byte cache line boundary */
    li.w    $t0, 0x3f
    nor     $t0, $t0, $zero       /* t0 = ~0x3f */
    and     $a0, $a0, $t0          /* start = addr & ~63 */

1:
    /* cache 0x15, 0(a0): Hit Writeback Invalidate D-Cache */
    /* encoding for rj=$a0(4): (0x15) | (4 << 5) | (2 << 22) = 0x800095 */
    .word   0x800095

    /* next cache line (64 bytes) */
    addi.d  $a0, $a0, 0x40
    bltu    $a0, $a1, 1b

3:
    jirl    $zero, $ra, 0

    .size   ls2k0300_dma_flush, . - ls2k0300_dma_flush

/****************************************************************************
 * Name: ls2k0300_dma_invalidate
 *
 * Description:
 *   Invalidate (only) D-cache lines covering a memory range, WITHOUT
 *   writing back dirty lines.  Use this after a device has written to
 *   memory (RX path): writeback here would push stale CPU-side data over
 *   the freshly DMA'd samples.
 *
 * Input Parameters:
 *   a0 - Start address (need not be cache-line aligned)
 *   a1 - Length in bytes
 *
 *   cache 0x14 = Hit Invalidate D-Cache (64B cache line)
 *   For rj=$a0(4), op=0x14, si12=0: .word = 0x800094
 ****************************************************************************/

    .globl  ls2k0300_dma_invalidate
    .type   ls2k0300_dma_invalidate, function

ls2k0300_dma_invalidate:
    /* a0 = addr, a1 = len */
    beqz    $a1, 3f              /* nothing to do if len == 0 */

    /* end = addr + len, before the start is rounded down - see the note
     * in ls2k0300_dma_flush.  Missing the last line here is worse: it is
     * the receive path, so the tail of every captured block would keep
     * whatever the CPU had in cache and overwrite the fresh samples. */
    add.d   $a1, $a0, $a1

    /* Align start down to 64-byte cache line boundary */
    li.w    $t0, 0x3f
    nor     $t0, $t0, $zero       /* t0 = ~0x3f */
    and     $a0, $a0, $t0          /* start = addr & ~63 */

1:
    /* cache 0x14, 0(a0): Hit Invalidate D-Cache */
    .word   0x800094

    /* next cache line (64 bytes) */
    addi.d  $a0, $a0, 0x40
    bltu    $a0, $a1, 1b

3:
    jirl    $zero, $ra, 0

    .size   ls2k0300_dma_invalidate, . - ls2k0300_dma_invalidate
