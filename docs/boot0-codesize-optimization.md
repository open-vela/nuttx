# T113 boot0 code size

## Overview

T113-S3 (Allwinner, dual Cortex-A7) boot0 is an SPL (Secondary Program
Loader) that runs entirely from SRAM.  BROM loads it from SPI NAND
block 0 into SRAM at `0x20000`.  Its job is:

1. Initialize clocks and train DDR.
2. Read the next-stage firmware (AP image) from SPI NAND.
3. Copy it to DDR and jump.

SRAM is 160 KB total.  The eGON header at the start of the image is
validated and sized by the BROM, so every boot0 variant must be under
the SRAM budget and must carry a valid eGON header.  See
`tools/mksunxi.py` for the header layout and checksum algorithm.

This tree currently carries three distinct boot0 variants.  They are
independent and serve different use cases:

| Variant                | Build system        | Runtime       | Intended use                      |
|------------------------|---------------------|---------------|-----------------------------------|
| `boot0-noos/`          | standalone Makefile | bare metal    | minimal, production-ready SPL     |
| `configs/boot0-nuttx/` | NuttX CMake+Ninja   | full NuttX    | NuttX-based SPL with MTD/miniboot |
| `configs/boot0/`       | NuttX CMake+Ninja   | minimal NuttX | experimental ultra-minimal SPL    |

Source files shared by all NuttX variants:

- `arch/arm/src/t113/t113_boot0.S` -- eGON header, vector setup,
  CPU mode / cache / stack init, optional SMP secondary park, jump to
  `_start` or to `sys_copyself`.
- `arch/arm/src/t113/t113_load.c` -- bare-metal C loader: clock init,
  DDR training, SPI NAND read, jump to DDR.  Compiled with
  `-O3 -fno-lto -fno-stack-protector`; runs from SRAM `0x30000`
  before DDR is initialized; must not reference global variables
  or libc.

## boot0-noos path (standalone, bare metal)

Directory: `boards/arm/t113/t113-evb/boot0-noos/`

Contents:

```
boot0-noos/
  Makefile          standalone GNU make build
  boot0.bin         pre-built artifact (checked in)
  build/            object / ELF output of last build
    boot0.elf
    boot0_asm.o
    boot0_load.o
```

The Makefile does not use NuttX CMake/Kconfig.  It invokes
`arm-none-eabi-gcc` directly on the two source files and links with
`boards/arm/t113/t113-evb/scripts/sram.ld`:

```
SRCS_S = arch/arm/src/t113/t113_boot0.S
SRCS_C = arch/arm/src/t113/t113_load.c
CFLAGS = -mcpu=cortex-a7 -marm -O3 -fno-lto -fno-stack-protector \
         -ffunction-sections -fdata-sections -ffreestanding -nostdlib \
         -DCONFIG_T113_BOOT0_NOOS
LDFLAGS = -T scripts/sram.ld -nostdlib \
          --defsym CONFIG_IDLETHREAD_STACKSIZE=0 \
          --defsym CONFIG_ARCH_INTERRUPTSTACK=0
```

After linking, `boot0.bin` is produced by `objcopy -O binary` and then
`tools/mksunxi.py --nand` patches the eGON `spl_size` and BROM
checksum (NAND requires 1 KB alignment).

Usage (from the `boot0-noos/` directory):

```
make                 # build boot0.bin
make flash           # requires device in FEL mode
make clean
make CROSS=xxx-      # override compiler prefix
```

The flash step runs:

```
xfel spinand splwrite 2048 1048576 boot0.bin
```

That writes boot0 to SPI NAND offset `0x100000` (block 8) with a 2048
byte page size.  See the Makefile for the current command.

Current on-disk artifact size (from this tree, pre-built):

```
boot0-noos/boot0.bin       9216 bytes
boot0-noos/build/boot0.elf 76412 bytes (with debug symbols; stripped
                                         down to 9 KB in boot0.bin)
```

The corresponding stub `configs/boot0-noos/defconfig` exists only so
NuttX CMake can recognise the configuration name; the actual build is
driven by the standalone Makefile and does not go through Kconfig.

## boot0-nuttx path (full NuttX)

Directory: `boards/arm/t113/t113-evb/configs/boot0-nuttx/`

This is a full NuttX configuration that produces a boot0 image using
the NuttX kernel, SPI NAND MTD drivers, and the miniboot loader
framework.  It is built via the standard Vela flow:

```
./build.sh t113-evb/boot0-nuttx -j
```

Key options currently set in `configs/boot0-nuttx/defconfig`:

| Option                              | Value            | Effect                                  |
|-------------------------------------|------------------|-----------------------------------------|
| `CONFIG_T113_BOOT0`                 | y                | eGON header, SRAM linker script         |
| `CONFIG_BOOT_MINIBOOT`              | y                | use miniboot loader framework           |
| `CONFIG_BOOT_RUNFROMISRAM`          | y                | run from internal SRAM                  |
| `CONFIG_MINIBOOT_SLOT_PATH`         | `/dev/mtd1`      | AP image partition                      |
| `CONFIG_MINIBOOT_HEADER_SIZE`       | 0x0              | raw AP image, no wrapper header         |
| `CONFIG_T113_DMA`                   | y                | DMA enabled for SPI                     |
| `CONFIG_T113_SPI0`                  | y                | SPI0 driver (connects to SPI NAND)      |
| `CONFIG_MTD` / `MTD_PARTITION`      | y / y            | MTD + partition support                 |
| `CONFIG_MTD_BYTE_WRITE`             | y                | byte-level writes (firmware update)     |
| `CONFIG_MTD_MX35` / `MX35_QSPI`     | y / y            | Macronix MX35 SPI NAND driver (QSPI)    |
| `CONFIG_MTD_GD5F` / `GD5F_QSPI`     | y / y            | Giga GD5F SPI NAND driver (QSPI)        |
| `CONFIG_BCH`                        | y                | BCH library                             |
| `CONFIG_DEBUG_CUSTOMOPT`            | y                | custom optimization level               |
| `CONFIG_DEBUG_OPTLEVEL`             | `-Os`            | size-oriented optimization              |
| `CONFIG_DEFAULT_SMALL`              | y                | small-footprint defaults                |
| `CONFIG_ARCH_INTERRUPTSTACK`        | 1024             | shared IRQ stack                        |
| `CONFIG_IDLETHREAD_STACKSIZE`       | 4096             | idle task stack                         |
| `CONFIG_DEFAULT_TASK_STACKSIZE`     | 4096             | default task stack                      |
| `CONFIG_PTHREAD_STACK_DEFAULT`      | 1024             | default pthread stack                   |
| `CONFIG_SCHED_TICKLESS`             | y                | tickless scheduler                      |
| `CONFIG_SYSTEM_NSH`                 | y                | ship NSH (useful for debugging boot0)   |
| `CONFIG_NSH_PROMPT_STRING`          | `"boot0>"`       | distinguish from full-NuttX shell       |

`boot0-nuttx` is the "batteries-included" path: it keeps MTD, the
miniboot framework, and a stripped NSH so engineers can inspect SPI
NAND from the boot0 shell during bring-up.  The size is correspondingly
larger than the bare-metal `boot0-noos` variant.

## boot0 path (experimental ultra-minimal NuttX)

Directory: `boards/arm/t113/t113-evb/configs/boot0/`

A third variant that aims at the smallest viable NuttX-based boot0.
It is preserved in-tree for experimentation but has known
issues (see "Caveats" below).

Notable settings:

| Option                              | Value  | Intent                                          |
|-------------------------------------|--------|-------------------------------------------------|
| `CONFIG_T113_BOOT0`                 | y      | eGON header, SRAM linker script                 |
| `CONFIG_INIT_NONE`                  | y      | no init task (board hook handles the jump)     |
| `CONFIG_LTO_FULL`                   | y      | full LTO                                       |
| `CONFIG_DEBUG_OPTLEVEL`             | `-Os`  | size-oriented optimization                     |
| `CONFIG_ARM_DCACHE_DISABLE`         | y      | skip D-cache init path                         |
| `CONFIG_ARCH_MINIMAL_VECTORTABLE`   | y      | shrink vector table                            |
| `CONFIG_ARCH_NUSER_INTERRUPTS`      | 1      | only the timer interrupt slot                  |
| `CONFIG_ARCH_INTERRUPTSTACK`        | 512    | tighter IRQ stack                              |
| `CONFIG_MTD` / `MTD_MX35`           | y      | MTD + MX35 driver only                         |
| `CONFIG_SYSLOG_NONE`                | y      | no logging                                     |
| `CONFIG_NAME_MAX`                   | 8      | shorten inode name table                       |
| `CONFIG_DEV_CONSOLE` (unset)        | n      | no console device                              |
| `CONFIG_ARCH_FPU` (unset)           | n      | no VFP/NEON context save                       |

### Caveats

Two options in this defconfig refer to symbols whose Kconfig
declarations are not currently in the tree:

- `CONFIG_MTD_READONLY` is referenced inside
  `drivers/mtd/gd5f.c` (gates a write-support block) but has no
  `config MTD_READONLY` entry in `drivers/mtd/Kconfig`.  The line in
  the defconfig therefore has no effect on the rest of the MTD stack.
  In particular, it does not strip write paths from `ftl.c` or
  `mx35.c`.
- `CONFIG_INIT_NONE` is accepted at the Kconfig level, but
  `sched/init/nx_bringup.c` still contains an `#error No
  initialization mechanism selected (CONFIG_INIT_NONE)` (around line
  70) outside of any guard.  A `#ifndef CONFIG_INIT_NONE` wraps the
  `task_spawn` call site (around line 296), but the `#error` must be
  removed before this variant can actually compile to completion with
  `INIT_NONE=y`.

Treat `configs/boot0/` as a research artifact: it documents the
intended direction but will need Kconfig / source changes before it
builds.  `boot0-noos` (bare metal) and `boot0-nuttx` (full NuttX) are
the two paths that build today.

## eGON header and BROM flow

Every BROM-loadable image on the T113-S3 begins with an eGON header
(first 64 bytes of the .bin file).  The layout is:

```
offset 0x00: 4-byte ARM branch instruction (past the header)
offset 0x04: 8-byte magic "eGON.BT0"
offset 0x0C: 4-byte checksum (patched by mksunxi.py)
offset 0x10: 4-byte spl_size (patched by mksunxi.py)
offset 0x14: 4-byte header size (0x30)
offset 0x18: 4-byte header version
offset 0x1C: 4-byte return value
offset 0x20: 4-byte run address hint
```

Checksum algorithm: replace offset `0x0C` with the seed `0x5F0A6C39`,
sum all 32-bit words in `[0, spl_size)`, store the result at
offset `0x0C`.

Alignment requirements (from T113 BROM disassembly):

- SPI NAND: 1 KB (`bfc r0, #10, #22` at BROM 0x1778)
- SPI NOR:  512 B (`bfc r0, #9, #23`  at BROM 0x1AD8)

`tools/mksunxi.py --nand <file>` or `--nor <file>` aligns `spl_size`
to the media requirement and recomputes the checksum.  All three
boot0 variants are post-processed through it.

## Flashing

From FEL mode (after `reboot 99` or JLink poke):

```
xfel spinand splwrite 2048 1048576 <path-to-boot0.bin>
```

`2048` is the SPI NAND page size, `1048576` is the write offset
(`0x100000`, block 8).  For `boot0-noos` this is also wired into
`make flash`.

For AP image flashing and full filesystem provisioning, see the
top-level boards/t113-evb flash / deploy documentation.

## Measuring size

For the standalone `boot0-noos` build:

```
cd boards/arm/t113/t113-evb/boot0-noos
make
arm-none-eabi-size build/boot0.elf
ls -l boot0.bin
```

For either NuttX-based variant, after a build:

```
arm-none-eabi-size nuttx
ls -l nuttx.bin
```

Per-symbol breakdown:

```
arm-none-eabi-nm --size-sort --radix=d nuttx | tail -40
```

`tools/bloaty` or `tools/size.py` can also be used for section-level
attribution.  Do not quote size numbers in design docs that were not
captured from an actual build of the current tree; they go stale
fast.
