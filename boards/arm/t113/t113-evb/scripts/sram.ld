/****************************************************************************
 * boards/arm/t113/t113-evb/scripts/sram.ld
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

#include <nuttx/config.h>

/* T113-S3 SRAM-only linker script for boot0 NuttX.
 *
 * SRAM layout: 160KB total
 *   0x20000 - 0x27FFF  SRAM_A1      (32KB)
 *   0x28000 - 0x37FFF  DSP0 IRAM    (64KB) - remapped to CPU by default
 *   0x38000 - 0x3FFFF  DSP0 DRAM0   (32KB) - remapped to CPU by default
 *   0x40000 - 0x47FFF  DSP0 DRAM1   (32KB) - remapped to CPU by default
 */

MEMORY
{
    sram (WRX) : ORIGIN = 0x00020000, LENGTH = 160K
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_stext)

SECTIONS
{
    .text :
    {
        /* boot0 eGON header must be at offset 0 — BROM checks magic here.
         * KEEP prevents --gc-sections from discarding boot0 code.
         */
        KEEP(*t113_boot0*(.text*))
        KEEP(*t113_load*(.text*))

        _stext = ABSOLUTE(.);
        *(.vectors)
        *(.text .text.*)
        *(.fixup)
        *(.gnu.warning)
        *(.rodata .rodata.*)
        *(.gnu.linkonce.t.*)
        *(.glue_7)
        *(.glue_7t)
        *(.got)
        *(.gcc_except_table)
        *(.gnu.linkonce.r.*)
        *(.ARM.extab*)
        *(.gnu.linkonce.armextab.*)
        PROVIDE_HIDDEN (__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE_HIDDEN (__exidx_end = .);
        _etext = ABSOLUTE(.);

        /* boot0 needs a pointer to NuttX __start.
         * For boot0-nuttx: __start is NuttX entry in same binary.
         * For boot0-noos: __start undefined, provide 0x40000000 (DDR).
         *   (boot0-noos doesn't use _t113_start — it jumps via t113_load.c)
         */
        PROVIDE(__start = 0x40000000);

        . = ALIGN(4);
        _t113_start = .;
        LONG(__start)

        PROVIDE(__spl_size = _etext - ORIGIN(sram));
    } > sram

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > sram

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram

    .noinit :
    {
        _snoinit = ABSOLUTE(.);
        *(.noinit*)
        _enoinit = ABSOLUTE(.);
    } > sram

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = ABSOLUTE(.);
    } > sram

    /* Runtime stacks grow above _ebss.  Catch overflow at link time. */

    ASSERT(_ebss + CONFIG_IDLETHREAD_STACKSIZE +
           CONFIG_ARCH_INTERRUPTSTACK <= 0x48000,
           "SRAM overflow: stacks collide with end of usable SRAM")
    _sram_free = 0x48000 - _ebss - CONFIG_IDLETHREAD_STACKSIZE -
                 CONFIG_ARCH_INTERRUPTSTACK;
    _sram_used = _ebss - ORIGIN(sram);

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .stab.excl 0 : { *(.stab.excl) }
    .stab.exclstr 0 : { *(.stab.exclstr) }
    .stab.index 0 : { *(.stab.index) }
    .stab.indexstr 0 : { *(.stab.indexstr) }
    .comment 0 : { *(.comment) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}

/* boot0-noos: provide default DDR addresses for __image_start/__image_end.
 * sys_copyself() uses these to know where to load NuttX AP in DDR.
 * boot0-nuttx: __start is NuttX entry within the same SRAM binary.
 * These are only used when not provided by another linker script. */
PROVIDE(__start = 0x40000000);
PROVIDE(__image_start = 0x40000000);
PROVIDE(__image_end = 0x40100000);
