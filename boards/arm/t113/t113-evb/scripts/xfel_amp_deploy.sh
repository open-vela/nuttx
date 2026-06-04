#!/bin/sh
# Deploy the T113 core0 image over USB FEL and boot.
#
# Usage:
#   xfel_amp_deploy.sh <core0_image.bin>
#
# Workflow:
#   1. xfel ddr t113-s3   - run the BROM DDR3 init for T113-S3.
#   2. xfel write          - drop image_a at 0x40000000 (core0 CONFIG_RAM_START).
#   3. xfel exec 0x40000000 - jump CPU0 to core0 entry.  core0's
#                            board_late_initialize() finishes its bringup and
#                            registers /dev/rptun/core1.
#
# core1 rides in the fwromfs image as /fw/core1.elf and is loaded + started
# by the rptun loader with
#   rptun start /dev/rptun/core1
# (the loader resolves the ELF entry and releases CPU1 there).
#
# core0 console: UART0 on PF2 (TX) / PF4 (RX) — see board.h T113_UART0_TX_1.
# core1 console: UART2 on PE2 (TX) / PE3 (RX) — see board.h T113_UART2_TX_2.
#
# DDR profile: t113-s3.  Confirmed from prior usage in
# /home/neo/projects/t113/deploy_and_test.sh.

set -eu

if [ $# -ne 1 ]; then
    echo "Usage: $0 <core0_image.bin>" >&2
    exit 1
fi

CORE0_BIN=$1

if [ ! -f "$CORE0_BIN" ]; then
    echo "ERROR: core0 bin not found: $CORE0_BIN" >&2
    exit 1
fi

echo "=== T113 core0 deploy ==="
echo "  core0 : $CORE0_BIN ($(stat -c%s "$CORE0_BIN") bytes -> 0x40000000)"

xfel ddr t113-s3
xfel write 0x40000000 "$CORE0_BIN"
xfel exec 0x40000000

echo "=== exec'd core0 at 0x40000000 ==="
echo "core0 should bring up its bringup chain on UART0 and register"
echo "/dev/rptun/core1.  Start CPU1 from the core0 nsh prompt with:"
echo "  rptun start /dev/rptun/core1"
echo "core1 nsh prompt 'core1' will then appear on UART2."
