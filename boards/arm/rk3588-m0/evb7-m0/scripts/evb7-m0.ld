/****************************************************************************
 * boards/arm/rk3588-m0/evb7-m0/scripts/evb7-m0.ld
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

/* The RK3588 PMU Cortex-M0 executes from its 0x00000000 window, which the
 * hardware remaps onto the DDR carveout named by u-boot's CODE_START. On this
 * board that is mcu_reserved (mcu@7a00000 in rk3588-amp.dtsi): physical
 * 0x07a00000, 2MB, no-map so Linux never touches it.
 *
 * Per RK3588 TRM Table 9-3 the window itself is 512MB, so the limit here is the
 * carveout, not the window. ARMv6-M has no VTOR: the vector table must sit at
 * address 0, which is why the image is linked at 0.
 *
 * There is no FLASH/SRAM split. The window is backed by writable DDR - proven
 * on target by the bare-metal M0 demo, which stored a heartbeat into it and had
 * Linux read it back via /dev/mem - so .text, .data, .bss, the idle stack and
 * the heap all live in one region and .data needs no relocation. Hence
 * _eronly == _sdata and the start-up code skips the copy loop.
 */

MEMORY
{
  ddr (rwx) : ORIGIN = 0x00000000, LENGTH = 2M
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

SECTIONS
{
    .text : {
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
        _etext = ABSOLUTE(.);
    } > ddr

    .init_section : {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > ddr

    .ARM.extab : {
        *(.ARM.extab*)
    } > ddr

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx : {
        *(.ARM.exidx*)
    } > ddr
    __exidx_end = ABSOLUTE(.);

    /* Same region for code and data, so .data is already in place: keep
     * _eronly == _sdata and let rk3588m0_start.c skip the relocation.
     */

    _eronly = ABSOLUTE(.);

    .data : {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > ddr

    .bss : {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > ddr

    /* Stabs debugging sections. */
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
