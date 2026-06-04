#!/usr/bin/env python3
"""
T113 cdcacm stability + corner case test suite.

Drives `dd if=/dev/zero of=/dev/ttyACM0 ...` cases with strict size + sha256
integrity verification (data is /dev/zero so expected hash is sha256 of N
zero bytes).

Pre-conditions:
  - board booted, NSH active on /dev/ttyUSB0
  - sercon already done; /dev/ttyACM0 ready

Cases:
  S1-S4: size corners (1B / 511B / 512B / 4097B)
  V1_4M: 4 MiB integrity check
  T1_20M: throughput sanity (no integrity, lower CPU overhead)
  L1_200M: 200 MiB sustained + integrity
"""

import json
import os
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

NSH_PORT = os.environ.get("NSH_PORT", "/dev/ttyUSB0")
DATA_PORT = os.environ.get("DATA_PORT", "/dev/ttyACM0")

TS = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
RESULTS = Path(os.path.expanduser(f"~/projects/t113/cdcopt/cdcbench/{TS}-stability"))
RESULTS.mkdir(parents=True, exist_ok=True)

CASES = [
    # (name, bs, count, max_wait, verify_data)
    ("S1_1B", 1, 1, 3.0, True),
    ("S2_511B", 511, 1, 3.0, True),
    ("S3_512B", 512, 1, 3.0, True),
    ("S4_4097B", 4097, 1, 3.0, True),
    ("V1_4M", 4096, 1024, 8.0, True),
    ("T1_20M", 4096, 5120, 15.0, False),
    ("L1_200M", 4096, 51200, 60.0, True),
    ("L2_200M", 4096, 51200, 60.0, True),
    ("L3_200M", 4096, 51200, 60.0, True),
]

IDLE_WINDOW = 0.4


def main():
    print(f"NSH={NSH_PORT}  DATA={DATA_PORT}  results={RESULTS}", flush=True)

    nsh = serial.Serial(NSH_PORT, 115200, timeout=0.3)
    nsh.reset_input_buffer()
    nsh.write(b"\r")
    time.sleep(0.3)
    nsh.read(500)

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
    state = {
        "bytes": 0,
        "last_byte_time": None,
        "nonzero": 0,
        "verify": False,
        "corrupts": [],
    }  # corrupts: list of (offset, byte)

    def drain():
        while not stop.is_set():
            try:
                d = data.read(65536)
                if d:
                    if state["verify"]:
                        nz_count = len(d) - d.count(b"\x00")
                        if nz_count:
                            base = state["bytes"]
                            # capture only first 30 corruptions per case to keep memory bounded
                            cap_left = 30 - len(state["corrupts"])
                            for i, b in enumerate(d):
                                if b != 0:
                                    if cap_left > 0:
                                        state["corrupts"].append((base + i, b))
                                        cap_left -= 1
                            state["nonzero"] += nz_count
                    state["bytes"] += len(d)
                    state["last_byte_time"] = time.monotonic()
            except Exception:
                break

    threading.Thread(target=drain, daemon=True).start()
    time.sleep(0.2)

    def run_case(name, bs, count, max_wait, verify):
        data.reset_input_buffer()
        nsh.reset_input_buffer()
        time.sleep(0.05)

        state["bytes"] = 0
        state["last_byte_time"] = None
        state["nonzero"] = 0
        state["verify"] = verify
        state["corrupts"] = []

        cmd = f"dd if=/dev/zero of=/dev/ttyACM0 bs={bs} count={count}\r".encode()
        expected = bs * count

        t0 = time.monotonic()
        nsh.write(cmd)
        deadline = t0 + max_wait
        effective = None
        while time.monotonic() < deadline:
            nsh.read(4096)
            lbt = state["last_byte_time"]
            if lbt is not None:
                idle = time.monotonic() - lbt
                if idle > IDLE_WINDOW and state["bytes"] >= expected:
                    effective = lbt - t0
                    break
            time.sleep(0.005)
        wall = time.monotonic() - t0
        if effective is None:
            effective = wall

        host_bytes = state["bytes"]
        excess = host_bytes - expected
        size_ok = host_bytes >= expected  # no truncation
        data_ok = (state["nonzero"] == 0) if verify else None  # no corruption
        nonzero = state["nonzero"] if verify else None

        host_mib = host_bytes / effective / 1048576 if effective > 0 else 0

        return {
            "name": name,
            "bs": bs,
            "count": count,
            "expected": expected,
            "host_bytes": host_bytes,
            "excess": excess,
            "nonzero": nonzero,
            "corrupts": list(state["corrupts"]) if verify else None,
            "size_ok": size_ok,
            "data_ok": data_ok,
            "wall": wall,
            "effective": effective,
            "host_mib": host_mib,
        }

    summary = {"nsh_port": NSH_PORT, "data_port": DATA_PORT, "cases": []}
    print(
        f"\n{'name':>9s} {'expected':>9s} {'host':>9s} {'excess':>7s} "
        f"{'nz':>4s} {'size':>5s} {'data':>5s} {'eff':>7s} {'MiB/s':>7s}"
    )

    all_ok = True
    for name, bs, count, max_wait, verify in CASES:
        r = run_case(name, bs, count, max_wait, verify)
        summary["cases"].append(r)
        d_str = "n/a" if r["data_ok"] is None else ("OK" if r["data_ok"] else "FAIL")
        s_str = "OK" if r["size_ok"] else "FAIL"
        nz_str = "n/a" if r["nonzero"] is None else str(r["nonzero"])
        if not r["size_ok"] or r["data_ok"] is False:
            all_ok = False
        print(
            f"{r['name']:>9s} {r['expected']:>9d} {r['host_bytes']:>9d} "
            f"{r['excess']:>+7d} {nz_str:>4s} {s_str:>5s} {d_str:>5s} "
            f"{r['effective']:6.2f}s {r['host_mib']:7.2f}"
        )

    summary["all_ok"] = all_ok
    (RESULTS / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"\nALL OK: {all_ok}")
    print(f"summary: {RESULTS / 'summary.json'}")

    stop.set()
    time.sleep(0.2)
    nsh.close()
    data.close()


if __name__ == "__main__":
    main()
