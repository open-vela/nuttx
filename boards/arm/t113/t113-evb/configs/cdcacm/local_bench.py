#!/usr/bin/env python3
"""
T113 cdcacm local bench — robust drainer + idle-window detection.

Pre-conditions (board must be in this state, run boot_capture_local.py + sercon
or the wrapper script first):
  - /dev/ttyUSB0 (FTDI) up, NSH at prompt
  - /dev/ttyACM0 (USB CDC) up, board cdcacm registered

Critical fixes (verified on cdcopt baseline -> 13.5 MiB/s sustained):
  1. stty raw + disable flow control / echo / ixon-ixoff / crtscts on ttyACM0
  2. Independent drainer thread on ttyACM0 — never block on NSH channel
  3. Idle-window done detection — NuttX `dd` does not print "X bytes copied"
     in this build; rely on data-channel idle (>idle_window with no new bytes)
"""

import json
import os
import re
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

NSH_PORT = os.environ.get("NSH_PORT", "/dev/ttyUSB0")
DATA_PORT = os.environ.get("DATA_PORT", "/dev/ttyACM0")

TS = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
RESULTS = Path(os.path.expanduser(f"~/projects/t113/cdcopt/cdcbench/{TS}-bench"))
RESULTS.mkdir(parents=True, exist_ok=True)

# bs * count = total bytes per case. count chosen so each case takes >= 1.5s
# at expected ~13.5 MiB/s, amortizing dd startup overhead.
CASES = [
    ("B1_20M", 4096, 5120),  # 20 MiB
    ("B2_40M", 4096, 10240),  # 40 MiB
    ("B3_80M", 4096, 20480),  # 80 MiB
]
MAX_WAIT = 30.0
IDLE_WINDOW = 0.4


def main():
    print(f"NSH={NSH_PORT}  DATA={DATA_PORT}  results={RESULTS}", flush=True)

    nsh = serial.Serial(NSH_PORT, 115200, timeout=0.3)
    nsh.reset_input_buffer()
    nsh.write(b"\r")
    time.sleep(0.3)
    nsh.read(500)

    # essential: flatten ttyACM0 line discipline so cdc-acm acts as raw pipe
    subprocess.run(
        [
            "stty",
            "-F",
            DATA_PORT,
            "raw",
            "-echo",
            "-ixon",
            "-ixoff",
            "-crtscts",
            "-clocal",
            "115200",
        ],
        check=False,
    )

    data = serial.Serial(
        DATA_PORT,
        115200,
        timeout=0.05,
        rtscts=False,
        xonxoff=False,
        dsrdtr=False,
    )

    stop = threading.Event()
    state = {"bytes": 0, "last_byte_time": None}

    def drain():
        while not stop.is_set():
            try:
                d = data.read(65536)
                if d:
                    state["bytes"] += len(d)
                    state["last_byte_time"] = time.monotonic()
            except Exception:
                break

    td = threading.Thread(target=drain, daemon=True)
    td.start()
    time.sleep(0.2)

    def run_case(bs, count):
        state["bytes"] = 0
        state["last_byte_time"] = None
        nsh.reset_input_buffer()
        cmd = f"dd if=/dev/zero of=/dev/ttyACM0 bs={bs} count={count}\r".encode()

        t0 = time.monotonic()
        nsh.write(cmd)
        deadline = t0 + MAX_WAIT
        nsh_buf = b""
        expected = bs * count

        effective = None
        while time.monotonic() < deadline:
            n = nsh.read(4096)
            if n:
                nsh_buf += n
            lbt = state["last_byte_time"]
            if lbt is not None:
                idle = time.monotonic() - lbt
                if idle > IDLE_WINDOW and state["bytes"] >= expected - 100:
                    effective = lbt - t0
                    break
            time.sleep(0.01)

        wall = time.monotonic() - t0
        if effective is None:
            effective = wall

        m = re.search(rb"(\d+)\s+bytes copied[^\n]*?([\d\.]+)\s*sec", nsh_buf)
        dev_mib = None
        if m:
            b_ = int(m.group(1))
            s_ = float(m.group(2))
            if s_ > 0:
                dev_mib = b_ / s_ / 1048576

        return {
            "bytes": state["bytes"],
            "wall": wall,
            "effective": effective,
            "host_mib": state["bytes"] / effective / 1048576 if effective > 0 else 0,
            "dev_mib": dev_mib,
            "nsh_tail": nsh_buf[-300:].decode("utf-8", "replace"),
        }

    summary = {"nsh_port": NSH_PORT, "data_port": DATA_PORT, "cases": []}
    print(
        f"\n{'name':>8s} {'size':>9s} {'host_bytes':>11s} {'eff':>7s} {'host MiB/s':>11s}"
    )
    for name, bs, count in CASES:
        r = run_case(bs, count)
        summary["cases"].append({"name": name, "bs": bs, "count": count, **r})
        (RESULTS / f"{name}.tail").write_text(r["nsh_tail"])
        print(
            f"{name:>8s} {bs*count:>9d} "
            f"{r['bytes']:>11d} {r['effective']:6.2f}s "
            f"{r['host_mib']:>10.2f}"
        )

    (RESULTS / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"\nsummary: {RESULTS / 'summary.json'}")

    stop.set()
    time.sleep(0.2)
    nsh.close()
    data.close()


if __name__ == "__main__":
    main()
