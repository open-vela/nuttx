/****************************************************************************
 * boards/arm/stm32h7/stm32h750b-dk/scripts/qspi_flash.ld
 *
 * Link the Cortex-M7 image for execution from the STM32H750B-DK external
 * dual-QSPI memory window at 0x90000000. A small internal-Flash stub must
 * initialize QSPI memory-mapped mode and jump to this image.
 *
 ****************************************************************************/

#include <nuttx/config.h>

MEMORY
{
  itcm  (rwx) : ORIGIN = 0x00000000, LENGTH =   64K
  flash (rx)  : ORIGIN = 0x90000000, LENGTH =  128M
  dtcm1 (rwx) : ORIGIN = 0x20000000, LENGTH =   64K
  dtcm2 (rwx) : ORIGIN = 0x20010000, LENGTH =   64K
  sram  (rwx) : ORIGIN = 0x24000000, LENGTH =  512K
  sram1 (rwx) : ORIGIN = 0x30000000, LENGTH =  128K
  sram2 (rwx) : ORIGIN = 0x30020000, LENGTH =  128K
  sram3 (rwx) : ORIGIN = 0x30040000, LENGTH =   32K
  sram4 (rwx) : ORIGIN = 0x38000000, LENGTH =   64K
  bbram (rwx) : ORIGIN = 0x38800000, LENGTH =    4K
  sdram (rwx) : ORIGIN = 0xD0000000, LENGTH =    8M
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)
SECTIONS
{
    .text :
    {
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
    } > flash

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab*)
    } > flash

    __exidx_start = ABSOLUTE(.);
    .ARM.exidx :
    {
        *(.ARM.exidx*)
    } > flash
    __exidx_end = ABSOLUTE(.);

    _eronly = ABSOLUTE(.);

    .data :
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sram AT > flash

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = ABSOLUTE(.);
    } > sram

    .sram4_reserve (NOLOAD) :
    {
        *(.sram4)
        . = ALIGN(4);
        _sram4_heap_start = ABSOLUTE(.);
    } > sram4

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
