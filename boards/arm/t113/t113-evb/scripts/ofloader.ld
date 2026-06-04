/****************************************************************************
 * boards/arm/t113/t113-evb/scripts/ofloader.ld
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * OFLoader linker script for T113-S3 DDR.
 *
 * Section order required by SEGGER Open Flash Loader:
 *   PrgCode  — SEGGER API entry points (must be first, JLink locates them)
 *   .text    — NuttX kernel + drivers
 *   PrgData  — SEGGER API function declarations (locate_code("PrgData"))
 *   .data    — initialized data
 *   DevDscr  — FlashDevice struct + DevStack (JLink stack + transfer buffer)
 *   .bss     — MUST be last (NuttX uses _ebss as heap start)
 *
 * DevDscr/DevStack must come before .bss to prevent JLink's work area
 * from being overwritten by NuttX heap allocations.
 *
 ****************************************************************************/

MEMORY
{
    sdram (W!RX) : ORIGIN = 0x40000000, LENGTH = 128M
}

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_stext)

SECTIONS
{
    PrgCode : ALIGN(4) {
        KEEP(*(PrgCode PrgCode.*));
        . = ALIGN(4);
    } > sdram

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
        *(.ARM.extab*)
        *(.gnu.linkonce.armextab.*)
        PROVIDE_HIDDEN (__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE_HIDDEN (__exidx_end = .);
        _etext = ABSOLUTE(.);
    } > sdram

    .init_section : ALIGN(4) {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > sdram

    PrgData : ALIGN(4) {
        KEEP(*(PrgData PrgData.*));
        . = ALIGN(4);
    } > sdram

    .data : ALIGN(4) {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sdram

    .noinit : {
        _snoinit = ABSOLUTE(.);
        *(.noinit*)
        _enoinit = ABSOLUTE(.);
    } > sdram

    /* DevDscr: FlashDevice struct (JLink reads this to get flash params).
     * DevStack: JLink's call stack + data transfer buffer — sized by
     *   CONFIG_SYSTEM_OFLOADER_STACKSIZE + CONFIG_SYSTEM_OFLOADER_BUFFERSIZE.
     * Both MUST appear before .bss so they are never allocated as heap. */

    DevDscr : ALIGN(4) {
        KEEP(*(DevDscr DevDscr.*));
        KEEP(*(DevStack DevStack.*));
        . = ALIGN(4);
    } > sdram

    /* .bss MUST be last — NuttX sets heap start = _ebss */

    .bss : ALIGN(4) {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = ABSOLUTE(.);
    } > sdram

    .stab 0 : { *(.stab) }
    .stabstr 0 : { *(.stabstr) }
    .debug_abbrev 0 : { *(.debug_abbrev) }
    .debug_info 0 : { *(.debug_info) }
    .debug_line 0 : { *(.debug_line) }
    .debug_pubnames 0 : { *(.debug_pubnames) }
    .debug_aranges 0 : { *(.debug_aranges) }
}
