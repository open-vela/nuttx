/* SPDX-License-Identifier: Apache-2.0
 *
 * NuttX linker script for T113-S3 HiFi4 DSP.
 * Layout inherits from firmware/hifi4_hello/hifi4.ld.
 */

MEMORY
{
    IRAM_STUB (rx)  : ORIGIN = 0x00400000, LENGTH = 0x00001000
    IRAM_VECT (rx)  : ORIGIN = 0x00401000, LENGTH = 0x00000800
    /* IRAM_HAND collects the upstream Xtensa low-level asm bodies
     * (xtensa_user_handler.S / xtensa_int_handlers.S / xtensa_context.S /
     * xtensa_panic.S) that all use HANDLER_SECTION = .iram_hand.text.
     * Co-locating in IRAM keeps all vector body short-jumps within +/-128 KB
     * of the vectors at IRAM_VECT.  IRAM is 128 KB total (0x400000..0x420000);
     * IRAM_STUB and IRAM_VECT consume the first 6 KB.  Remaining 122 KB
     * goes here.  Plenty of slack for handler bodies + literal pool.
     */
    IRAM_HAND (rx)  : ORIGIN = 0x00401800, LENGTH = 0x0001E800
    /* First 128 KB of DDR window is reserved for AP<->DSP shmem
     * (probe sentinels at 0x..04000, fault dump base, AR mirror, trace
     * ring at 0x..04400).  Probe / dump / trace-ring constants live at
     * 0x37904xxx in the cacheable view (= 0x17904xxx non-cacheable view
     * = phys 0x47904xxx).  Firmware text/rodata/data/bss must start
     * after this reserved region or trace_init() writes to the
     * non-cacheable alias will silently corrupt firmware code bytes
     * via shared physical RAM.
     */
    /* Firmware image region.  Shortened to 0x3E0000 (was 0x4E0000) so the
     * top 1 MB of the 5 MB window (0x37D00000..0x37E00000) is left free for
     * the rptun carveout (vrings + rpmsg buffer pool).  The carveout is
     * shared DDR the framework builds an mm_heap in; if the firmware bss /
     * stack could reach into it the heap and the firmware would collide.
     */
    DDR       (rwx) : ORIGIN = 0x37920000, LENGTH = 0x003E0000
    /* AP<->DSP rptun resource table at the base of the shared window
     * (DSP-view 0x37900000 = AP-view 0x47900000).  The AP master reads
     * .resource_table out of this ELF by section name and both cores then
     * access the live table here.  Kept clear of the trace ring at
     * 0x37904400.  The carveout (vrings + buffer pool) lives separately at
     * 0x37D00000, past the firmware image.
     */
    RSCTABLE  (rw)  : ORIGIN = 0x37900000, LENGTH = 0x00001000
}

ENTRY(_iram_start)

SECTIONS
{
    .ResetVector.text 0x00400000 : { KEEP(*(.ResetVector.text)) } > IRAM_STUB
    /* IRAM stub: same backward-only l32r constraint as DDR .text -- put
     * the literal pool ahead of the consumers.
     */
    .iram_stub.text :              { *(.iram_stub.literal*) *(.iram_stub.text*) } > IRAM_STUB

    .window_vectors.text      0x00401000 : { KEEP(*(.window_vectors.text)) KEEP(*(.WindowVectors.text)) } > IRAM_VECT
    .Level2InterruptVector.text 0x0040117C : { KEEP(*(.Level2InterruptVector.text)) } > IRAM_VECT
    .Level3InterruptVector.text 0x0040119C : { KEEP(*(.Level3InterruptVector.text)) } > IRAM_VECT
    .DebugExceptionVector.text  0x004011BC : { KEEP(*(.DebugExceptionVector.text)) } > IRAM_VECT
    .NMIExceptionVector.text    0x004011DC : { KEEP(*(.NMIExceptionVector.text)) } > IRAM_VECT
    .KernelExceptionVector.text 0x004011FC : { KEEP(*(.KernelExceptionVector.text)) } > IRAM_VECT
    .UserExceptionVector.text   0x0040121C : { KEEP(*(.UserExceptionVector.text)) } > IRAM_VECT
    .DoubleExceptionVector.text 0x0040123C : { KEEP(*(.DoubleExceptionVector.text)) } > IRAM_VECT

    /* Upstream Xtensa handler bodies (xtensa_user_handler.S,
     * xtensa_int_handlers.S, xtensa_context.S, xtensa_panic.S) emit into
     * HANDLER_SECTION (= .iram_hand.text per chip_macros.h).  Co-locate in
     * IRAM so vector short-jumps reach.  Literal pool comes first because
     * `l32r` is a backward-only PC-relative load.
     */
    .iram_hand.text : ALIGN(4)
    {
        *(.iram_hand.literal*)
        *(.iram_hand.text*)
    } > IRAM_HAND

    .resource_table 0x37900000 : ALIGN(8)
    {
        KEEP(*(.resource_table))
    } > RSCTABLE

    .text : ALIGN(4)
    {
        _stext = ABSOLUTE(.);
        /* Literals must precede their consumers because Xtensa l32r is a
         * backward-only PC-relative load (offset is in [-262144, -4], i.e.
         * 256 KB reach).  In-tree objects are built with
         * -mtext-section-literals (see arch/xtensa cmake/Toolchain.cmake), so
         * their literals are folded into each function's own .text.<fn> and
         * are always in range no matter how large .text grows.  The PREBUILT
         * toolchain libs (libgcc/libstdc++/libsupc++) cannot be recompiled and
         * still emit a separate generic `.literal` section that lands in the
         * front pool below.  To keep every prebuilt l32r within 256 KB of that
         * pool, pull the prebuilt-library .text up here, right after the pool,
         * BEFORE the bulk (interleaved) in-tree .text.*.  Otherwise a few
         * libgcc soft-float objects (_fixdfdi, _fixunsdfsi) get pulled in last
         * and land ~328 KB past their front-pool literal -> l32r out of range.
         */
        *(.literal .literal.*)
        *libgcc.a:*(.text .text.*)
        *libsupc++.a:*(.text .text.*)
        *libstdc++.a:*(.text .text.*)
        *(.text .text.*)
        _etext = ABSOLUTE(.);
    } > DDR

    .rodata : ALIGN(4)
    {
        *(.rodata .rodata.*)
        *(.gnu.linkonce.r.*)
    } > DDR

    .init_array : ALIGN(4)
    {
        _sinit = ABSOLUTE(.);
        /* The xtensa-hifi4-elf GCC 10.3 toolchain libstdc++ emits its global
         * constructors (ios_base::Init, the std::locale classic/_S_classic
         * _Impl, etc.) into legacy .ctors, NOT .init_array.  Collecting only
         * .init_array left _sinit==_einit (empty table), so lib_cxx_initialize
         * ran nothing -> std::locale::_S_classic stayed unconstructed and the
         * first std::ifstream's ios_base::_M_init() NULL-derefed the classic
         * locale _Impl refcount.  KEEP both .init_array and .ctors, mirroring
         * the ARM t113-evb script, so every global ctor is collected and run.
         */
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array EXCLUDE_FILE(*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o) .ctors))
        _einit = ABSOLUTE(.);
        _eronly = ABSOLUTE(.);   /* End of read-only region (.text + .rodata + .init_array) */
    } > DDR

    .data : ALIGN(4)
    {
        _sdata = ABSOLUTE(.);
        *(.data .data.*)
        *(.gnu.linkonce.d.*)
        _edata = ABSOLUTE(.);
    } > DDR

    /* .noinit holds data that must survive both load and the C-runtime
     * BSS-zero, most importantly g_idlestack: the early boot path (__start
     * -> xtensa_start -> the BSS-zero loop -> nx_start) runs ON the idle
     * stack, so zeroing it would wipe the live stack frame.  It must sit
     * OUTSIDE the _sbss.._ebss span the BSS-zero loop clears, AND below the
     * heap base (up_allocate_heap starts at _ebss) so malloc never lands on
     * the tls_info_s the kernel writes at the base of the idle stack
     * (overlap there corrupted info->tl_task -> broke setenv/getenv).
     * Placing it just before .bss satisfies both, matching the esp32/esp32s3
     * port layout.  Without an explicit output section ld puts orphan
     * .noinit above _ebss, straight under the heap -- the original bug.
     */

    .noinit (NOLOAD) : ALIGN(8)
    {
        *(.noinit .noinit.*)
        . = ALIGN(8);
    } > DDR

    .bss (NOLOAD) : ALIGN(8)
    {
        _sbss = ABSOLUTE(.);
        *(.bss .bss.*)
        *(COMMON)
        . = ALIGN(8);
        _ebss = ABSOLUTE(.);
    } > DDR

    _stack_top = ORIGIN(DDR) + LENGTH(DDR);

    /* The firmware image (text/data/bss + stack) must stay below the rptun
     * carveout at 0x37D00000 (vrings + rpmsg buffer pool, see
     * t113_rsctable.c).  The DDR region and the carveout share no link-time
     * relationship -- separate MEMORY regions -- so without this guard a
     * growing .bss/stack would silently corrupt the rpmsg heap with no link
     * error.  Fail the link instead.
     */

    .rptun_carveout_guard (NOLOAD) :
    {
        ASSERT(. <= 0x37D00000, "DSP firmware image overflows into the rptun carveout at 0x37D00000");
    } > DDR
}
