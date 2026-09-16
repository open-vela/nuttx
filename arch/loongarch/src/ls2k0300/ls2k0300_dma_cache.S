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

    /* Align start down to 64-byte cache line boundary */
    li.w    $t0, 0x3f
    nor     $t0, $t0, $zero       /* t0 = ~0x3f */
    and     $a0, $a0, $t0          /* start = addr & ~63 */

    /* end = start + len */
    add.d   $a1, $a0, $a1

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
