/****************************************************************************
 * boards/arm/t113/t113-evb/scripts/nonsecure_trampoline.S
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

/* Trampoline to switch CPU from secure to non-secure state.
 * Loaded at 0x41000000, executed via JLink before NuttX boot.
 *
 * 1. Write CNTFRQ = 24 MHz (only writable in secure state)
 * 2. Allow non-secure access to CP10/CP11 (FPU) via NSACR
 * 3. Switch to Monitor mode, set SCR.NS=1, return to non-secure SVC
 * 4. Jump to NuttX at 0x40000000
 */

    .arm
    .section .text
    .global _start

_start:
    /* CNTFRQ = 24000000 (0x016E3600) */
    movw    r0, #0x3600
    movt    r0, #0x016E
    mcr     p15, 0, r0, c14, c0, 0
    isb

    /* NSACR: allow non-secure access to CP10, CP11 (FPU) */
    mrc     p15, 0, r0, c1, c1, 2
    orr     r0, r0, #0xC00
    mcr     p15, 0, r0, c1, c1, 2
    isb

    /* Switch to Monitor mode (0x16 = MON, IRQ+FIQ disabled) */
    cps     #0x16

    /* Set SCR: NS=1, FW=1, AW=1 = 0x31 */
    mrc     p15, 0, r0, c1, c1, 0
    orr     r0, r0, #0x31
    mcr     p15, 0, r0, c1, c1, 0
    isb

    /* Set up non-secure SVC return: SPSR = SVC mode, IRQ+FIQ disabled */
    movw    r0, #0x01D3
    msr     spsr_cxsf, r0

    /* Return to non-secure SVC at 0x40000000 */
    ldr     lr, =0x40000000
    movs    pc, lr
