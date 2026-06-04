#!/usr/bin/env python3
"""mksunxi.py - Patch Allwinner eGON boot0 header for BROM boot.

Validates eGON magic, sets spl_size to the full file size (aligned to
the target media requirement), and computes the BROM checksum.

eGON header layout:
  offset 0x00: ARM branch instruction (4 bytes)
  offset 0x04: magic "eGON.BT0" (8 bytes)
  offset 0x0C: checksum (4 bytes)
  offset 0x10: spl_size (4 bytes)

Checksum algorithm (same for eGON and TOC):
  Replace offset 0x0C with stamp 0x5F0A6C39, sum all uint32 words
  in [0, spl_size), store result at offset 0x0C.

Alignment requirements (from T113 BROM disassembly):
  SPI NAND: 1 KB (0x400)  — bfc r0, #10, #22 at BROM 0x1778
  SPI NOR:  512 B (0x200) — bfc r0, #9, #23  at BROM 0x1AD8

Usage:
  python3 mksunxi.py [--nand|--nor] <boot0.bin>
"""

import argparse
import struct
import sys

STAMP_VALUE = 0x5F0A6C39
CHECKSUM_OFFSET = 0x0C
SPL_SIZE_OFFSET = 0x10
EGON_MAGIC = b"eGON.BT0"
SRAM_MAX = 160 * 1024  # T113 SRAM: 160KB (0x20000-0x47FFF)

ALIGN_NAND = 1024  # 1 KB
ALIGN_NOR = 512  # 512 B


def main():
    parser = argparse.ArgumentParser(
        description="Patch Allwinner eGON boot0 header checksum"
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--nand",
        action="store_true",
        default=True,
        help="SPI NAND: align to 1KB (default)",
    )
    group.add_argument("--nor", action="store_true", help="SPI NOR: align to 512B")
    parser.add_argument("binary", help="boot0.bin file to patch")
    args = parser.parse_args()

    align = ALIGN_NOR if args.nor else ALIGN_NAND

    with open(args.binary, "rb") as f:
        data = bytearray(f.read())

    raw_size = len(data)

    # Validate eGON magic
    magic = data[4:12]
    if magic != EGON_MAGIC:
        print(
            f"ERROR: eGON magic not found at offset 0x04 "
            f"(got {magic!r}, expected {EGON_MAGIC!r})"
        )
        sys.exit(1)

    # spl_size = entire file, aligned up
    spl_size = (raw_size + align - 1) & ~(align - 1)

    # SRAM upper limit check
    if spl_size > SRAM_MAX:
        print(
            f"ERROR: spl_size {spl_size} ({spl_size // 1024}KB) exceeds "
            f"SRAM limit {SRAM_MAX} ({SRAM_MAX // 1024}KB)"
        )
        sys.exit(1)

    # Write aligned spl_size to header
    struct.pack_into("<I", data, SPL_SIZE_OFFSET, spl_size)

    # Pad to aligned size with 0xFF (NAND erased state)
    if len(data) < spl_size:
        data.extend(b"\xff" * (spl_size - len(data)))

    # Compute checksum: replace checksum field with stamp, sum all words
    struct.pack_into("<I", data, CHECKSUM_OFFSET, STAMP_VALUE)

    checksum = 0
    for i in range(0, spl_size, 4):
        checksum += struct.unpack_from("<I", data, i)[0]
        checksum &= 0xFFFFFFFF

    struct.pack_into("<I", data, CHECKSUM_OFFSET, checksum)

    # Write back
    with open(args.binary, "wb") as f:
        f.write(data)

    media = "NOR" if args.nor else "NAND"
    padding = spl_size - raw_size
    print(
        f"Patched: media={media}, raw={raw_size}, "
        f"aligned={spl_size} (0x{spl_size:x}, {spl_size // 1024}KB), "
        f"padding={padding}, checksum=0x{checksum:08x}"
    )


if __name__ == "__main__":
    main()
