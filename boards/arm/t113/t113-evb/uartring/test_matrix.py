#!/usr/bin/env python3
# boards/arm/t113/t113-evb/uartring/test_matrix.py
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The ASF licenses this file to you under the Apache License, Version
# 2.0 (the "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied.  See the License for the specific language governing
# permissions and limitations under the License.
#
"""5-port UART DMA stress matrix runner (host-side).

Drives a single deployed alluart firmware over /dev/ttyACM0 (NSH @115200).
For each (baud, size) cell in MATRIX, issues `uartring -p -b BAUD -s SIZE`
on /dev/ttyS1..ttyS5 and parses PASS/FAIL + per-port throughput.

Cells run sequentially (same physical UARTs cannot be at two bauds at
once).  Within each cell all 5 ports run in parallel via uartring's
pthread mode (-p flag).

Usage:
    python3 test_matrix.py             # FEL recovery + xfel deploy + run
    python3 test_matrix.py --no-deploy # skip deploy, run on whatever's loaded

Requires: pyserial, JLink + xfel on PATH, t113-evb wired with TX-RX
loopback shorts on UART1..UART5.
"""

import re
import subprocess
import sys
import time

import serial

CONSOLE_DEV = "/dev/ttyACM0"
CONSOLE_BAUD = 115200

# Test matrix.  T113 APB1=100MHz so DL=100M/(16*baud).
# DL=1 -> 6250000, DL=2 -> 3125000, DL=4 -> 1562500, DL=54 -> 115200.
MATRIX = [
    (115200, 262144),
    (1562500, 1048576),
    (3125000, 1048576),
    (6250000, 1048576),
]

PORTS = "/dev/ttyS1 /dev/ttyS2 /dev/ttyS3 /dev/ttyS4 /dev/ttyS5"


def drain(s, secs):
    end = time.time() + secs
    out = b""
    while time.time() < end:
        n = s.in_waiting
        if n:
            out += s.read(n)
            end = time.time() + 0.2
        else:
            time.sleep(0.02)
    return out


def wait_prompt(s, secs=5.0):
    """Wait for 'nsh>' prompt, returning whatever was read."""
    end = time.time() + secs
    buf = b""
    while time.time() < end:
        n = s.in_waiting
        if n:
            buf += s.read(n)
            if b"nsh>" in buf[-128:]:
                drain(s, 0.2)
                return buf
        else:
            time.sleep(0.05)
    return buf


def kill_running(s, max_retries=3):
    """Send Ctrl-C and force a fresh prompt.  Returns True on success.

    On true device hang the prompt may never come back; the caller
    should treat False as a fatal-for-this-matrix condition (re-deploy
    or abort) rather than continue with the next cell.
    """
    for _ in range(max_retries):
        s.write(b"\x03")
        time.sleep(0.2)
        drain(s, 0.3)
        s.write(b"\r")
        if b"nsh>" in wait_prompt(s, 2.0)[-128:]:
            drain(s, 0.2)
            return True
    return False


def run_cell(s, baud, size):
    expected = max(2.0, size * 10.0 / baud)
    timeout = max(15.0, expected * 5 + 5.0)

    cmd = f"uartring -p -b {baud} -s {size} {PORTS}\r\n".encode()
    for i in range(0, len(cmd), 16):
        s.write(cmd[i : i + 16])
        time.sleep(0.005)

    # Anchor on the full uartring summary shape, which only the
    # tool itself prints — the command echo or a prior history echo
    # cannot match because they lack the "<m>/<n> passed" suffix.
    # Running the regex on every iteration (no banner gating) avoids
    # the off-by-one trap where all output arrives in a single burst
    # ending with PASS, then the device falls silent — gated banner
    # detection would set its flag on the same read but `continue`
    # past the regex check, and the next iteration would never come.
    deadline = time.time() + timeout
    buf = b""
    verdict = None
    while time.time() < deadline:
        n = s.in_waiting
        if n:
            buf += s.read(n)
            text = buf.decode("ascii", errors="replace")
            m = re.search(r"^(PASS|FAIL):\s+\d+/\d+\s+passed\s*$", text, re.M)
            if m:
                verdict = m.group(1)
                drain(s, 0.3)
                break
        else:
            time.sleep(0.02)

    text = buf.decode("ascii", errors="replace")
    rows = re.findall(
        r"(/dev/ttyS\d)\s*:\s*(OK|FAIL)\s+(\d+)/(\d+)\s+([\d.]+)s\s+(\d+)\s+B/s\s+([\d.]+)%",
        text,
    )
    return verdict, rows, text, timeout


def fel_recover_and_deploy():
    print("[deploy] FEL recovery + xfel")
    for _ in range(3):
        subprocess.run(
            'printf "connect\\nhalt\\nw4 0x07090100, 0x5AA50001\\n'
            'w4 0x020500B8, 0x16AA0000\\nw4 0x020500A8, 0x16AA0001\\nexit\\n" | '
            "timeout 12 JLinkExe -device Cortex-A7 -if JTAG -speed 20000 "
            "-nogui 1 -autoconnect 1 >/dev/null 2>&1",
            shell=True,
        )
        time.sleep(2)
        if subprocess.run("xfel version >/dev/null 2>&1", shell=True).returncode == 0:
            break
    subprocess.run("xfel ddr t113-s3 >/dev/null 2>&1", shell=True, check=True)
    subprocess.run(
        "xfel write 0x40000000 build_alluart/nuttx.bin >/dev/null 2>&1",
        shell=True,
        check=True,
    )
    subprocess.run("xfel exec 0x40000000 >/dev/null 2>&1", shell=True, check=True)
    print("[deploy] waiting for NSH...")
    time.sleep(5)


def main():
    if "--no-deploy" not in sys.argv:
        fel_recover_and_deploy()

    # --nand-load=mtdN launches `nandtest /dev/mtdN &` on the device
    # before the matrix runs and kills it after, so the SPI DMA path
    # is busy in parallel with the UART matrix.  In the full config
    # SPI0 already permanently owns 2 of the 8 ARM-reachable DMA
    # channels; this option additionally exercises those channels
    # with continuous traffic to expose any timing coupling between
    # SPI bursts and PIO UART servicing.
    nand_dev = None
    for arg in sys.argv[1:]:
        if arg.startswith("--nand-load="):
            nand_dev = "/dev/" + arg.split("=", 1)[1]

    s = serial.Serial(CONSOLE_DEV, CONSOLE_BAUD, timeout=0.2)
    time.sleep(1)
    s.write(b"\r\r")
    drain(s, 1.0)
    wait_prompt(s, 3.0)

    if nand_dev is not None:
        print(f"[nand-load] launching: nandtest {nand_dev} &")
        s.write(f"nandtest {nand_dev} &\r\n".encode())
        # Give nandtest a head start so it's actively pumping when
        # the matrix opens its first port.
        drain(s, 1.5)

    results = []
    for baud, size in MATRIX:
        print(f"\n=== baud={baud:>7d} size={size//1024:>5d}KB ===")
        if not kill_running(s):
            print(
                "  ABORT: NSH unresponsive after 3 Ctrl-C retries; "
                "device likely hung.  Re-deploy and retry."
            )
            results.append((baud, size, "HUNG", []))
            break
        # uartring sets each port's baud unconditionally via tcsetattr
        # at start of every invocation, so no extra reset between cells.
        # When --nand-load is active kill_running's Ctrl-C also stops
        # nandtest, so re-launch it before each cell so SPI traffic
        # overlaps the entire UART transfer.
        if nand_dev is not None:
            s.write(f"nandtest {nand_dev} &\r\n".encode())
            drain(s, 1.0)
        t0 = time.monotonic()
        verdict, rows, text, tmo = run_cell(s, baud, size)
        dur = time.monotonic() - t0
        if verdict is None:
            print(f"  TIMEOUT after {dur:.1f}s (budget {tmo:.0f}s)")
            print("  tail:", text[-400:].replace("\r", " ").replace("\n", " | "))
            results.append((baud, size, "TIMEOUT", []))
        else:
            for r in rows:
                print(
                    f"  {r[0]} {r[1]:>4}  {r[2]}/{r[3]}  {r[4]}s  {r[5]} B/s  {r[6]}%"
                )
            print(f"  {verdict}  ({dur:.1f}s wall)")
            results.append((baud, size, verdict, rows))

    if nand_dev is not None:
        # Stop the background nandtest cleanly so the prompt comes back
        # for any post-matrix interactive use.
        s.write(b"\x03")
        drain(s, 0.5)

    s.close()

    print("\n========== MATRIX SUMMARY ==========")
    print(f"{'baud':>8}  {'size':>6}  {'verdict':>7}  {'min%':>6}  {'max%':>6}  notes")
    for baud, size, verdict, rows in results:
        if rows:
            pcts = [float(r[6]) for r in rows]
            errs = sum(int(r[3]) - int(r[2]) for r in rows)
            print(
                f"{baud:>8}  {size//1024:>5}KB  {verdict:>7}  "
                f"{min(pcts):>5.1f}  {max(pcts):>5.1f}  errors={errs} 5/5"
            )
        else:
            print(
                f"{baud:>8}  {size//1024:>5}KB  {verdict:>7}  ----   ----   no rows parsed"
            )

    fail = sum(1 for _, _, v, _ in results if v != "PASS")
    sys.exit(1 if fail else 0)


if __name__ == "__main__":
    main()
