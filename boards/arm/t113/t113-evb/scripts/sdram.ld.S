/****************************************************************************
 * boards/arm/t113/t113-evb/scripts/sdram.ld.S
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

/* T113-S3 DDR linker script.
 *
 * When CONFIG_T113_BOOT0=y:
 *   The binary starts with the boot0 BROM header at CONFIG_RAM_START.
 *   BROM loads spl_size bytes to SRAM 0x30000, runs clock/DDR init,
 *   copies the full bin to CONFIG_RAM_START, then jumps to _t113_start
 *   which holds the address of NuttX __start.
 *
 * When CONFIG_T113_BOOT0=n (xfel debug):
 *   Standard layout: vectors at CONFIG_RAM_START, __start entry.
 */

MEMORY
{
    /* T113-S3 on-chip SRAM: 160KB (SRAM_A1 32K + DSP IRAM 64K + DSP DRAM0 32K + DSP DRAM1 32K)
     * Available at reset (DSP SRAM remapped to CPU by default).
     * Reserved for future use - no sections placed here yet.
     */
    sram  (W!RX) : ORIGIN = 0x00020000, LENGTH = 160K

    /* External DDR: 0x40000000, size from defconfig CONFIG_RAM_SIZE */
    sdram (W!RX) : ORIGIN = CONFIG_RAM_START, LENGTH = CONFIG_RAM_SIZE
}

#ifdef CONFIG_T113_RPTUN_SLAVE
/* The slave image is ELF-loaded by the master (Linux remoteproc or
 * core0-NuttX) into its own cacheable DDR region (CONFIG_RAM_START).  The
 * shared vring/buffer carveout now lives in the high-DDR SHM aggregation zone
 * (CONFIG_T113_RPTUN_CORE1_CARVEOUT_BASE = 0x47E00000), separate from and
 * above the image.  The image must end below the DSP firmware window at
 * 0x47900000 so it does not run into the aggregated SHM/DSP region.
 */
ASSERT(CONFIG_RAM_START + CONFIG_RAM_SIZE <= 0x47900000,
       "core1 image overruns into DSP window at 0x47900000")
#endif

OUTPUT_FORMAT("elf32-littlearm", "elf32-littlearm", "elf32-littlearm")
OUTPUT_ARCH(arm)
ENTRY(_stext)

SECTIONS
{
    .text :
    {
        __image_start = .;

        __spl_start = .;
#ifdef CONFIG_T113_BOOT0
        KEEP(*t113_boot0*(.text*))
        KEEP(*t113_load*(.text*))
#endif
        __spl_end = .;
        PROVIDE(__spl_size = __spl_end - __spl_start);

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

#ifdef CONFIG_T113_BOOT0
    .t113_start :
    {
        _t113_start = ABSOLUTE(.);
        LONG(__start)
    } > sdram
#endif

#ifdef CONFIG_T113_RPTUN_SLAVE
    /* OpenAMP resource table.  The master (Linux remoteproc or core0-NuttX)
     * locates this section by name in the loaded ELF and parses the vring
     * geometry; the section is at a deterministic VMA because it follows
     * .text near the image base.  The shared vring/buffer window itself
     * lives below RAM_START in the carveout, not in this section.
     */

    .resource_table : ALIGN(8)
    {
        __resource_table_start = ABSOLUTE(.);
        KEEP(*(.resource_table))
        __resource_table_end = ABSOLUTE(.);
    } > sdram
#endif

    .init_section :
    {
        _sinit = ABSOLUTE(.);
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
    } > sdram

#ifdef CONFIG_PERCPU_SECTION
    . = ALIGN(64);
    _ldata_percpu = LOADADDR(.percpu_data);
    .percpu_data :
    {
        _sdata_percpu = ABSOLUTE(.);
        *(.data..percpu)
        . = ALIGN(8);
        _edata_percpu = ABSOLUTE(.);
    } > sdram AT > sdram

    .percpu_bss (NOLOAD) :
    {
        . = ALIGN(8);
        _sbss_percpu = ABSOLUTE(.);
        *(.bss..percpu)
        . = ALIGN(8);
        _ebss_percpu = ABSOLUTE(.);
    } > sdram

    /* _percpu_offset is multiplied by cpu index to fan-out every
     * DEFINE_PER_CPU_BMP variable to its per-core mirror.  It MUST be
     * a multiple of the strictest load/store alignment (8 bytes on
     * ARMv7-A for ldrd/strd); otherwise secondaries see every percpu
     * variable at an unaligned address and the first stw data-aborts.
     */

    _percpu_size   = _ebss_percpu - _sdata_percpu;
    _percpu_offset = _percpu_size;

    .percpu_reserved (NOLOAD) :
    {
        . = . + _percpu_size * (CONFIG_NCPUS - 1);
    } > sdram
#endif

    .data :
    {
        _sdata = ABSOLUTE(.);
        _eronly = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        CONSTRUCTORS
        . = ALIGN(4);
        _edata = ABSOLUTE(.);
    } > sdram
    __image_end = _edata;

    .noinit :
    {
        _snoinit = ABSOLUTE(.);
        *(.noinit*)
        _enoinit = ABSOLUTE(.);
    } > sdram

    .bss :
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(.gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = ABSOLUTE(.);
    } > sdram

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
