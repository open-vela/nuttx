=======================================
T113 BROM Boot0 Loading Mechanism
=======================================

.. note::

   Based on BROM disassembly analysis of the T113-S3/R528 48KB mask ROM.
   All addresses reference the BROM dump at ``debug/t113_brom_disasm.txt``.

Boot Header Formats: eGON vs TOC
=================================

Allwinner BROM recognizes two boot header formats. T113 BROM supports
both, but our NuttX boot0 uses eGON exclusively.

eGON
----

The original Allwinner boot format, first seen in A10-era BSP code.

- **Single blob** — header followed directly by executable code
- **Magic**: ``eGON.BT0`` (8 bytes) at **header offset +4**
  (offset +0 is an ARM branch instruction that skips the header)
- ``eGON.BT1`` exists for second-stage boot but is rarely used
- No security features — code is loaded and executed without verification
  beyond a simple checksum
- Used by: xfel, sunxi-tools, U-Boot SPL, our NuttX boot0

TOC ("Table Of Contents")
--------------------------

A container format introduced for ARM TrustZone secure boot on newer
Allwinner SoCs.

- **Container with N sub-images** — can package ATF BL31, SCP firmware
  (e.g. `Crust <https://github.com/crust-firmware/crust>`_), U-Boot SPL,
  and certificates into a single file
- **Magic**: ``TOC0.GLH`` (8 bytes) at **header offset +0**
  (no branch instruction prefix — the TOC is not directly executable)
- Each sub-image has an independent ``load_addr`` field
- Supports certificate chain verification (verified boot)
- Numeric magic: ``0x89119800`` (TOC0), ``0x89119801`` (TOC1 for BL3x)

.. list-table:: eGON vs TOC comparison
   :header-rows: 1
   :widths: 25 35 35

   * - Feature
     - eGON
     - TOC0
   * - Magic location
     - offset +4 (8 bytes)
     - offset +0 (8 bytes)
   * - Magic value
     - ``eGON.BT0``
     - ``TOC0.GLH``
   * - Structure
     - Single blob, starts with ARM branch
     - Container with N sub-images
   * - spl_size location
     - offset +0x10
     - offset +0x1C
   * - Secure boot
     - No
     - Yes (TrustZone + certificate chain)
   * - Sub-image load addresses
     - N/A (single load to fixed SRAM addr)
     - Per-item ``load_addr`` field
   * - Typical users
     - xfel, sunxi-tools, NuttX boot0
     - ARM TF-A, Crust SCP, Allwinner secure boot

How BROM distinguishes the two formats
---------------------------------------

The T113 BROM has complete support for both formats. Two ROM data
entries store the magic strings:

- ``0x5B2C``: ``65 47 4F 4E 2E 42 54 30`` = ``eGON.BT0``
- ``0x5B38``: ``54 4F 43 30 2E 47 4C 48`` = ``TOC0.GLH``

The detection function ``sub_059b0`` (address ``0x59B0``) has two paths
depending on a hardware security fuse (``sub_05524``):

**Non-secure mode** (fuse not set, typical for development):
Simple 8-byte comparison of ``addr+4`` against ``eGON.BT0``.
Returns 0 (match) or 1 (no match). TOC is not checked.

**Secure mode** (fuse set):
Calls ``sub_058f4`` (address ``0x58F4``) which tries both formats:

1. First attempt (``r1=0``): compare ``addr+4`` vs ``eGON.BT0``
2. If no match, automatic retry (``r1=1``): compare ``addr+0`` vs ``TOC0.GLH``

Return values:

- ``0`` → eGON match (non-secure path)
- ``2`` → eGON match (secure path)
- ``3`` → TOC match, also writes flag to ``0x47D04`` to record TOC detection
- ``1`` → neither format matched

Callers use the return value to select the spl_size field:

- ``0`` or ``2`` (eGON) → ``spl_size = *(uint32_t*)(addr + 0x10)``
- ``3`` (TOC) → ``spl_size = *(uint32_t*)(addr + 0x1C)``

**Checksum**: Both formats use the **same algorithm** (``sub_05a24``
for eGON, ``sub_05b4c`` for TOC). The checksum field is at offset
``+0x0C`` in both cases. During verification, offset ``+0x0C`` is
temporarily replaced with stamp ``0x5F0A6C39``, then all uint32 words
are summed and compared against the original checksum value.

The +4 byte offset difference in magic location is the key structural
distinction: eGON puts an ARM branch instruction at offset 0 (so the
CPU can jump over the header), while TOC puts its magic directly at
offset 0 (the TOC is a data container, not directly executable code).

Where TOC is defined
--------------------

TOC has no standalone public specification. Its definition is spread
across several codebases:

- **U-Boot** ``include/sunxi_image.h``: C structs ``toc0_main_info``
  and ``toc0_item_info`` with full field definitions
- **U-Boot** ``tools/sunxi_toc0.c``: TOC0 image packing and signing
- **ARM Trusted Firmware** Allwinner platform: uses TOC0 to load
  BL31 and SCP firmware
- **Allwinner BSP/SDK**: internal ``toc0``/``toc1`` toolchain (not
  publicly documented)

Secure BROM Mode (Security Fuse Enabled)
------------------------------------------

When the Secure Enable bit in SID eFuse is burned (``SID[0x03006210]``
bit 8, checked by ``sub_05524`` at BROM address ``0x5524``), BROM enters
Secure BROM Mode. The User Manual (Section 3.4.2) states:

  *"Secure BROM loads only certified firmware"*
  *"Secure BROM ensures that the Secure Boot is in a trusted environment"*

Secure BROM Mode features (User Manual §3.4.2.2):

- X.509 certificate support — verifies firmware integrity before execution
- SHA-256 hash computation via hardware Crypto Engine (CE)
- RSA-2048 signature verification via hardware CE
- OTP/eFuse root of trust (ROTPK hash stored in 2Kbit eFuse)

.. note::

   The User Manual uses "HASH code" for the eFuse-stored key hash. "ROTPK"
   (Root of Trust Public Key) is the ARM TF-A / industry-standard term for
   the same concept. Both refer to the RSA public key hash burned in eFuse
   that anchors the secure boot chain.

Secure boot startup flow (User Manual §3.4.2.2, Figure 3-7)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Secure BROM Mode boot flow differs from Normal Mode only at the
final stage — after loading boot0, it runs "Security Boot Software"
(signature verification) before jumping to boot0.

.. code-block:: text

    Power-on / WDT reset
        │
        ▼
    BROM entry (0x00000000)
        │
        ├── CPU ID != 0? ──YES──→ NON_CPU0 Boot Process
        │                          (read Soft Entry Address Register,
        │                           CPU0=0x070005C4, CPU1=0x070005C8)
        │
        ├── Read Hotplug Flag (0x070005C0)
        │   └── == 0xFA50392F? ──YES──→ Hotplug Process (jump Soft Entry)
        │
        ├── Check Fast Boot register (0x07090120, RTC domain)
        │   └── != 0? ──YES──→ Fast Boot: select medium by bit[31:28]
        │
        ├── Read Secure Enable bit (SID 0x03006210)
        │   ├── == 0 → Normal BROM Mode
        │   └── == 1 → Secure BROM Mode   ← this path
        │
        ├── Read FEL pin (bit[8] of 0x03000024)
        │   └── LOW? ──YES──→ Mandatory Upgrade → FEL (Secure: jump 0x64)
        │
        ▼
    Select boot medium (GPIO pin or eFuse, same as Normal Mode)
        │
        ▼
    Try Media Boot: scan NAND/NOR/SD/eMMC for valid boot0
        │
        ├── FAIL → try next medium → all fail → FEL mode
        │
        └── PASS: boot0 loaded to SRAM 0x20000
                │
                ▼
        ┌─── Normal Mode: jump to 0x20000 directly (no verification)
        │
        └─── Secure Mode: Run Security Boot Software
                │
                ▼
            sub_424C: Secure Boot verification entry
                │
                ├── TOC0 header validation (magic + checksum)
                ├── CE hardware init → SHA-256 → RSA-2048 → ROTPK check
                │
                ├── PASS → jump to boot0/SPL at 0x20000
                └── FAIL → try next boot device → ultimately FEL mode

Key differences between Normal and Secure Mode:

.. list-table::
   :header-rows: 1
   :widths: 25 35 35

   * - Aspect
     - Normal BROM Mode
     - Secure BROM Mode
   * - Header format
     - eGON only (checks ``eGON.BT0``)
     - eGON or TOC0 (tries both)
   * - Verification
     - Checksum only
     - Checksum + SHA-256 + RSA-2048 + ROTPK
   * - FEL entry address
     - ``0x20``
     - ``0x64``
   * - Boot0 format
     - Single blob (eGON)
     - TOC0 container (certificates + sub-images)
   * - Trust anchor
     - None
     - ROTPK hash in eFuse (hardware root of trust)

Hardware Crypto Engine (CE) in BROM
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

T113 has **two independent CE controllers** (User Manual §10.1.1):
one for secure world (CE_S), one for non-secure world (CE_NS). BROM
uses both depending on context.

- ``CE_NS``: ``0x03040000``–``0x030407FF`` (2KB)
- ``CE_S``:  ``0x03040800``–``0x03040FFF`` (2KB)
- ``CE_KEY_SRAM``: ``0x03041000``–``0x03041FFF`` (4KB, CE-only access)

CE supports: AES/DES/3DES (symmetric), RSA-512/1024/2048 (asymmetric),
MD5/SHA1/SHA224/SHA256/SHA384/SHA512/HMAC (hash), PRNG/TRNG (random).
BROM specifically uses **SHA-256** for image hashing and **RSA-2048**
for signature verification.

The CE base address is **not a literal constant** in the ROM — it is
dynamically constructed at BROM address ``0x37D4``::

    mov r0, #0x03000000
    orr r0, r0, r4, lsl #16    ; r4=4 → 0x03040000 (CE)
    orr r0, r0, r5, lsl #8     ; r5=0 → CE_NS, r5=8 → CE_S

This is why a binary search for the literal ``0x03040000`` finds nothing.

Secure boot verification chain (from BROM disassembly)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: text

    Flash read → TOC0 header check (magic + checksum)
       │
       ▼
    sub_424C: Secure Boot verification entry
       │
       ├── sub_4340: CE hardware init (clock, reset, interrupt config)
       │     ├── sub_2550: CE register init ([base+0x00]=7 reset,
       │     │   [base+0x08]=0xFFFFFFFF enable interrupts,
       │     │   [base+0x38]=0xFFFFFFFF clear status)
       │     ├── sub_2498: CE reset sequence
       │     └── sub_24D4: CE clock configuration
       │
       ├── sub_3A84: Core verification
       │     ├── sub_3740: SHA-256 hash of loaded image (via CE hardware)
       │     ├── sub_37A8: SHA + HMAC verification (via CE hardware)
       │     │   Dynamically selects CE_NS (0x03040000) or CE_S (0x03040800)
       │     └── RSA-2048 signature verification (via CE hardware)
       │
       └── ROTPK comparison: hash of RSA public key in TOC0 vs
           hash burned in eFuse — the hardware root of trust

    Pass → jump to boot0/SPL
    Fail → try next boot device → ultimately FEL mode

Why TOC0 replacement cannot bypass secure boot
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

1. BROM computes SHA-256 of the loaded image using CE hardware
2. BROM verifies the RSA-2048 signature in TOC0 against the image hash
3. BROM verifies the RSA public key itself against the **ROTPK hash**
   burned in eFuse (OTP, physically irreversible)
4. An attacker replacing the TOC0 package (including its embedded
   public key and signature) cannot match the eFuse ROTPK hash
5. eFuse is one-time programmable — the trust anchor cannot be forged

**Security fuse is irreversible**: eFuse technology physically blows
metal fuse elements. Once burned, there is no hardware or software
method to restore the original state.

CE task submission function ``sub_02A48`` (address ``0x2A48``,
~2KB, called from 22 sites in BROM):

- Constructs CE task descriptors in SRAM
- Configures algorithm type, key material, IV, data pointers
- Submits descriptor to CE hardware via MMIO registers
- Polls CE status registers (``[base+0x38]``, ``[base+0x88]``) for
  completion with multi-stage timeout
- Returns hash/verification result to caller

Why we use eGON
----------------

- ``xfel spinand splwrite`` only supports eGON (validates ``buf[4]=="eGON.BT0"``)
- Our development boards have **security fuse NOT burned** — BROM
  runs in Normal Mode, no signature verification required
- eGON is simpler: ``mksunxi.py`` patches checksum in ~20 lines of Python
- All community NuttX and U-Boot SPL tooling assumes eGON
- For production with secure boot, switch to TOC0 format with
  proper certificate chain and burn ROTPK into eFuse

Overview
========

After power-on or WDT reset, the BROM (Boot ROM) at address ``0x00000000``
executes and attempts to load a second-stage bootloader (boot0/SPL) from
external storage into SRAM. The loading mechanism has three phases:

1. **Scan** — Try multiple candidate locations on the storage medium
2. **Verify** — Check eGON magic and checksum at each location
3. **Execute** — Jump to the loaded boot0 code at SRAM ``0x20000``

SRAM Target Address
===================

BROM loads boot0 to a **fixed address: ``0x00020000``** (SRAM start).
This is hardcoded in the BROM (``sub_0166c`` at ``0x16D8: mov r1, #0x20000``),
not read from the eGON header. The address is the same for SPI NAND,
SPI NOR, and SD/eMMC boot paths.

T113 SRAM layout::

    0x20000 ┬── SRAM_A1      32KB  ┐
    0x28000 ├── DSP0 IRAM    64KB  │
    0x38000 ├── DSP0 DRAM0   32KB  ├── 160KB total
    0x40000 ├── DSP0 DRAM1   32KB  │
    0x48000 ┘                      ┘

All 160KB is available to boot0 (DSP SRAM is remapped to CPU by default).

eGON Header Format
==================

boot0 must start with an eGON header (checked by ``sub_059b0``):

.. list-table::
   :header-rows: 1
   :widths: 10 20 30

   * - Offset
     - Field
     - Description
   * - 0x00
     - Branch instruction
     - ARM branch to skip header (e.g. ``0xEA00000E``)
   * - 0x04
     - Magic (8 bytes)
     - ``eGON.BT0`` (ASCII). BROM compares all 8 bytes.
   * - 0x0C
     - Checksum
     - Sum of all uint32 words with this field replaced by ``0x5F0A6C39``
   * - 0x10
     - spl_size (eGON)
     - Total boot0 binary size in bytes (eGON format)
   * - 0x1C
     - spl_size (TOC)
     - Total boot0 binary size in bytes (TOC format, alternative)
   * - 0x20
     - boot_media (written by BROM)
     - BROM writes storage type here after loading: 0=SD, 3=NOR, 4=NAND
   * - 0x28
     - boot_media (alternative offset)
     - Same purpose, different header format

BROM detects the header format automatically. If offset 0x04 matches
``eGON.BT0``, it reads spl_size from offset 0x10. Otherwise it tries
the TOC format and reads from offset 0x1C.

spl_size Constraints
====================

.. list-table::
   :header-rows: 1
   :widths: 20 30

   * - Constraint
     - Value
   * - **Alignment (SPI NAND)**
     - **1 KB (0x400)**. BROM checks with ``bfc r0, #10, #22`` at
       ``0x1778``. If spl_size is not 1KB-aligned, BROM reports error
       0xF2 and skips this copy.
   * - **Alignment (SPI NOR)**
     - 512 bytes (0x200). Checked with ``bfc r0, #9, #23`` at ``0x1AD8``.
   * - **Maximum size**
     - **No upper limit check in BROM.** The physical limit is the SRAM
       size: 160 KB (``0x28000`` bytes). BROM will happily load a 160KB
       boot0 — it just overwrites all of SRAM.
   * - **Practical maximum**
     - ~155 KB. The remaining ~5 KB is needed for BROM's own stack
       (SVC SP = ``0x44FFC``, grows downward) and BSS variables
       (``0x47D00``-``0x48000``), but these overlap with late SRAM and
       BROM is done using them by the time boot0 finishes loading.

.. note::

   Our ``mksunxi.py`` aligns spl_size to 16 KB for historical reasons.
   This is conservative but safe. The BROM only requires 1 KB alignment
   for SPI NAND.

SPI NAND Loading: Scan Locations
================================

BROM scans multiple locations on the SPI NAND to find a valid boot0.
The scan is driven by a parameter table at BROM address ``0xA0F4``
(first-attempt table, 2 entries) and ``0xA114`` (retry table, 7 entries).

Each table entry is 16 bytes (4 × uint32)::

    struct scan_entry {
        uint32_t block;           /* NAND block number */
        uint32_t page_size;       /* Expected page size in bytes */
        uint32_t plane;           /* Die/plane select (0 or 1) */
        uint32_t pages_per_block; /* Pages per erase block */
    };

First-attempt table (2 entries)
-------------------------------

Used when BROM first tries SPI NAND (``boot_type == 0``).

.. list-table::
   :header-rows: 1
   :widths: 5 10 10 10 10 25

   * - #
     - block
     - page_size
     - plane
     - ppb
     - NAND byte offset
   * - 0
     - 0
     - 2048
     - 0
     - 64
     - **0x000000** (block 0, page 0 = byte 0)
   * - 1
     - 1
     - 2048
     - 0
     - 64
     - **0x020000** (block 1, page 0 = byte 128KB)

BROM reads page address = ``block × pages_per_block``.

- Entry 0: page address = 0 × 64 = page 0 → **NAND offset 0**
- Entry 1: page address = 1 × 64 = page 64 → **NAND offset 128 KB** (64 × 2048)

So in the first attempt, BROM looks at **block 0 (offset 0)** and
**block 1 (offset 128KB)** only.

Retry table (7 entries)
-----------------------

If the first attempt fails (both locations invalid), BROM retries
with different page_size/ppb combinations (``boot_type != 0``):

.. list-table::
   :header-rows: 1
   :widths: 5 10 10 10 10 30

   * - #
     - block
     - page_size
     - plane
     - ppb
     - Rationale
   * - 0
     - 0
     - 2048
     - 0
     - 64
     - Block 0, 2K page, 64 ppb (128KB block)
   * - 1
     - 0
     - 2048
     - 1
     - 64
     - Block 0, plane 1
   * - 2
     - 1
     - 2048
     - 0
     - 64
     - Block 1, 2K page, 64 ppb
   * - 3
     - 1
     - 2048
     - 1
     - 64
     - Block 1, plane 1
   * - 4
     - 0
     - 2048
     - 0
     - 128
     - Block 0, 2K page, 128 ppb (256KB block)
   * - 5
     - 0
     - 2048
     - 1
     - 128
     - Block 0, plane 1, 256KB block
   * - 6
     - 1
     - 2048
     - 0
     - 128
     - Block 1, 256KB block

The retry table covers two block sizes (128KB and 256KB) and two
planes. All entries assume 2048-byte pages.

Scan algorithm
--------------

For each candidate location (``sub_0166c``):

1. Read **1 page** from the candidate NAND address to SRAM ``0x20000``
2. Check eGON magic at ``0x20004`` (8 bytes == ``eGON.BT0``)
3. If magic matches, read ``spl_size`` from header
4. Check ``spl_size`` alignment (1KB for NAND)
5. Calculate pages needed: ``ceil(spl_size / page_size)``
6. Read all ``pages_needed`` pages to SRAM ``0x20000``
   (overwrites the initial 1-page read — full boot0 now in SRAM)
7. Verify checksum (``sub_05a24``):
   - Replace offset 0x0C with stamp ``0x5F0A6C39``
   - Sum all uint32 words
   - Compare with original checksum value
8. If checksum passes: write ``boot_media = 4`` (NAND) to header offset
   0x28, close SPI controller, **return success**
9. If any step fails: increment iteration counter, try next location

If all locations exhausted → return failure → BROM enters FEL mode.

xfel splwrite NAND Layout
==========================

The ``xfel spinand splwrite`` command writes multiple SPL copies plus
the full image to NAND. The layout ensures BROM can find a valid copy
even if some NAND blocks are bad.

Command syntax::

    xfel spinand splwrite <page_size> <image_offset> <binary>

Example: ``xfel spinand splwrite 2048 1048576 nuttx.bin``

Layout calculation
------------------

Given:

- ``splsz`` = spl_size from eGON header (the SPL portion size)
- ``page_size`` = NAND physical page size (2048)
- ``block_size`` = page_size × pages_per_block (e.g. 2048 × 64 = 128KB)
- ``image_offset`` = full image destination (1MB = 1048576)

xfel computes::

    tsplsz = align_up(splsz + block_size, block_size)
    copies = image_offset / tsplsz

Each SPL copy occupies ``tsplsz`` bytes, block-aligned.

**SPL data reformatting**: Within each copy, the SPL binary is split
into ``page_size``-byte chunks. Each chunk is written to one NAND page
(remaining bytes in the page are ``0xFF``). BROM reads pages
sequentially and concatenates the first ``page_size`` bytes from each.

Concrete example (SPL=32KB)
---------------------------

::

    splsz     = 32768 (32 KB)
    page_size = 2048
    block_size = 2048 × 64 = 131072 (128 KB)
    tsplsz = align_up(32768 + 131072, 131072) = 262144 (256 KB, 2 blocks)
    copies = 1048576 / 262144 = 4

NAND layout::

    Offset       Content
    ─────────────────────────
    0x000000     SPL copy 0 (256KB, blocks 0-1)
    0x040000     SPL copy 1 (256KB, blocks 2-3)
    0x080000     SPL copy 2 (256KB, blocks 4-5)
    0x0C0000     SPL copy 3 (256KB, blocks 6-7)
    0x100000     Full image (nuttx.bin complete, from 1MB offset)

BROM's first-attempt scan checks block 0 (offset 0x000000) and
block 1 (offset 0x020000). Block 0 has SPL copy 0's eGON header.
Block 1 falls within SPL copy 0's second block (no eGON header there),
so it fails and BROM falls back to block 0.

Concrete example (SPL=97KB, NuttX-as-boot0)
--------------------------------------------

::

    splsz     = 99328 (97 KB, 1KB-aligned)
    page_size = 2048
    block_size = 131072 (128 KB)
    tsplsz = align_up(99328 + 131072, 131072) = 262144 (256 KB)
    copies = 1048576 / 262144 = 4

Same layout as above — 4 copies at 256KB intervals. The larger SPL
still fits within the 2-block (256KB) allocation.

Verification Flow
=================

Complete BROM SPI NAND boot sequence::

    BROM reset_handler
        │
        ▼
    init_cpu_modes (0x160)
      - SVC mode, IRQ/FIQ disabled
      - Disable WDT
      - Set SVC SP = 0x44FFC
        │
        ▼
    sub_012a0 → sub_0166c (SPI NAND boot)
        │
        ├── iteration 0: read block 0 page 0
        │   ├── eGON magic check → pass?
        │   │   ├── YES: read full spl_size → checksum → pass?
        │   │   │   ├── YES: write boot_media=4 → return SUCCESS
        │   │   │   └── NO:  try iteration 1
        │   │   └── NO: try iteration 1
        │   │
        ├── iteration 1: read block 1 page 0
        │   └── (same check flow)
        │
        └── all failed → return FAIL → enter FEL mode

    On SUCCESS:
        sub_04934: close SPI controller
        sub_013b0: write boot info to FBOOT_INFO_REG0 (0x07090120)
        Jump to SRAM 0x20000 (boot0 entry)

Key Takeaways
=============

1. **Load address is fixed at 0x20000** — cannot be changed
2. **No spl_size upper limit** — SRAM 160KB is the physical constraint
3. **SPI NAND requires 1KB alignment** for spl_size
4. **First-attempt scans only 2 locations**: block 0 and block 1
   (offsets 0x00000 and 0x20000 for 128KB-block NAND)
5. **Retry scans up to 7 locations** with different block-size assumptions
6. **xfel splwrite** creates multiple SPL copies at block-aligned intervals
7. **Checksum uses stamp value 0x5F0A6C39** replacing offset 0x0C during
   calculation (``mksunxi.py`` implements this)
8. **Secure BROM Mode uses CE hardware** (SHA-256 + RSA-2048) for
   image verification — CE address dynamically constructed, not a literal
9. **ROTPK in eFuse** is the hardware root of trust — cannot be forged
10. **FEL entry addresses differ by boot mode**: Normal BROM FEL = ``0x20``,
    Secure BROM FEL = ``0x64`` (User Manual §3.4.2.5)
11. **Fast Boot register** ``0x07090120`` (RTC domain) — BROM checks this
    on startup; non-zero selects boot medium directly (User Manual §3.4.2.7)

User Manual Cross-Reference
============================

This document was verified against the T113-S3 User Manual v1.1.
Key section references:

.. list-table::
   :header-rows: 1
   :widths: 30 20 40

   * - Topic
     - Manual Section
     - Notes
   * - Boot media selection
     - §3.4.2.1
     - GPIO pin or eFuse select, SID register ``0x03006210``
   * - Normal vs Secure BROM Mode
     - §3.4.2.2
     - Secure Enable bit in SID selects mode
   * - Secure boot features
     - §3.4.2.2
     - X.509, SHA-256, RSA-2048, OTP/eFuse
   * - FEL process
     - §3.4.2.5–§3.4.2.6
     - FEL addresses: Normal=0x20, Secure=0x64
   * - Fast Boot
     - §3.4.2.7
     - Register 0x07090120, bit[31:28] selects medium
   * - CE hardware
     - §10.1
     - Two controllers (secure/non-secure), full algorithm list
   * - SID / eFuse
     - §10.2
     - 2Kbit, one-time programmable, key loading to CE
   * - CE memory map
     - §Memory Map
     - CE_NS=0x03040000, CE_S=0x03040800, KEY_SRAM=0x03041000
