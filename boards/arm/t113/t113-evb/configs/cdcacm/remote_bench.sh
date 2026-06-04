#!/usr/bin/env bash
# Runs on opt7050. FEL-recovery the T113 board, load nuttx.bin via xfel,
# then drive bench.py over /dev/ttyACM0 (CH343 NSH UART).
set -uo pipefail

cd ~/remote-deploy/t113-cdcacm-bench

mkdir -p results
rm -f results/*

echo "=== pre-deploy /dev/ttyACM* ==="
ls -1 /dev/ttyACM* 2>/dev/null | tee results/pre_deploy_tty.list || true
echo

# Kernel log capture (no sudo): journalctl -k. Fallback note in dmesg.log if it
# can't read.
DMESG_PID=""
if command -v journalctl >/dev/null 2>&1; then
    # write raw kernel stream; grep at read-time. bash $! captures journalctl
    # itself (no subshell wrapper), so kill in cleanup actually terminates it.
    journalctl -k -f --since "now" -o short -n 0 > results/dmesg.log 2>&1 &
    DMESG_PID=$!
    echo "kernel log: journalctl -k pid=${DMESG_PID}"
else
    echo "journalctl unavailable; skipping dmesg capture" > results/dmesg.log
fi

# 4. FEL recovery via JTAG (single line, EXACT, do not modify)
echo "=== JLinkExe FEL recovery ==="
printf 'connect\nhalt\nw4 0x07090100, 0x5AA50001\nw4 0x020500B8, 0x16AA0000\nw4 0x020500A8, 0x16AA0001\nexit\n' | timeout 15 JLinkExe -device Cortex-A7 -if JTAG -speed 20000 -nogui 1
JLINK_RC=$?
echo "JLinkExe rc=${JLINK_RC}"

sleep 1

echo "=== xfel handshake ==="
XFEL_VER="$(timeout 10 xfel version 2>&1 || true)"
echo "${XFEL_VER}" | tee results/xfel_version.log
if ! grep -q 'AWUSBFEX' <<<"${XFEL_VER}"; then
    echo "ERROR: xfel did not see AWUSBFEX device after FEL recovery" >&2
    [[ -n "${DMESG_PID}" ]] && kill "${DMESG_PID}" 2>/dev/null || true
    exit 1
fi

echo "=== xfel ddr t113-s3 ==="
timeout 15 xfel ddr t113-s3 | tee -a results/xfel_version.log

echo "=== xfel write nuttx.bin ==="
timeout 15 xfel write 0x40000000 ~/remote-deploy/t113-cdcacm-bench/nuttx.bin

echo "=== xfel exec ==="
timeout 10 xfel exec 0x40000000

sleep 2

echo "=== bench.py ==="
timeout 60 python3 ~/remote-deploy/t113-cdcacm-bench/bench.py /dev/ttyACM0
BENCH_RC=$?
echo "bench rc=${BENCH_RC}"

if [[ -n "${DMESG_PID}" ]]; then
    kill "${DMESG_PID}" 2>/dev/null || true
    wait "${DMESG_PID}" 2>/dev/null || true
fi

# capture post-deploy listing too
ls -1 /dev/ttyACM* 2>/dev/null > results/post_deploy_tty.list || true

exit ${BENCH_RC}
