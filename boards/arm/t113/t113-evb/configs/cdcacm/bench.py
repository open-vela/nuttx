#!/usr/bin/env python3
"""
T113 cdcacm Phase 1 throughput baseline.

Drives NuttX over the CH343 UART NSH console (argv[1], typically /dev/ttyACM0
on opt7050), triggers `sercon` to enumerate USB CDC ACM, then runs three
`dd if=/dev/zero of=/dev/ttyACM0 bs=N count=M` cases while reading the bulk
data on host side.

Outputs per-case .log + summary.json under ~/remote-deploy/t113-cdcacm-bench/results/.
"""

import glob
import json
import pathlib
import re
import sys
import time

import serial

RESULTS = pathlib.Path("~/remote-deploy/t113-cdcacm-bench/results").expanduser()
RESULTS.mkdir(parents=True, exist_ok=True)

CASES = [
    # (name, bs, count) -- each case = ~20 MiB
    ("B1", 4096, 5120),
    ("B2", 8192, 2560),
    ("B3", 512, 40960),
]

CASE_TIMEOUT = 10.0  # per-case wallclock cap (seconds)
PROMPT = b"nsh> "


def wait_prompt(ser: serial.Serial, timeout: float = 5.0) -> bytes:
    """Read until 'nsh> ' shows up. On stall, nudge with CR."""
    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
            if PROMPT in buf:
                return buf
        else:
            ser.write(b"\r")
            time.sleep(0.1)
    raise RuntimeError(f"NSH prompt timeout after {timeout}s, tail={buf[-200:]!r}")


def run_case(
    nsh: serial.Serial, data: serial.Serial, name: str, bs: int, count: int
) -> dict:
    nsh.reset_input_buffer()
    data.reset_input_buffer()

    cmd = f"dd if=/dev/zero of=/dev/ttyACM0 bs={bs} count={count}\r".encode()
    t0 = time.monotonic()
    nsh.write(cmd)

    bytes_total = 0
    nsh_buf = b""
    deadline = t0 + CASE_TIMEOUT
    dd_done = False
    while time.time() < deadline:
        chunk = data.read(65536)
        if chunk:
            bytes_total += len(chunk)

        nchunk = nsh.read(4096)
        if nchunk:
            nsh_buf += nchunk
            if PROMPT in nsh_buf and (b"copied" in nsh_buf or b"bytes" in nsh_buf):
                dd_done = True
                # drain trailing data
                tail_deadline = time.time() + 0.3
                while time.time() < tail_deadline:
                    c2 = data.read(65536)
                    if c2:
                        bytes_total += len(c2)
                    else:
                        time.sleep(0.02)
                break

        if not chunk and not nchunk:
            time.sleep(0.005)
    t1 = time.monotonic()

    m = re.search(rb"(\d+)\s+bytes copied[^\n]*?([\d\.]+)\s*sec", nsh_buf)
    dev_bytes = int(m.group(1)) if m else None
    dev_sec = float(m.group(2)) if m else None
    dev_mib = (dev_bytes / dev_sec / 1048576) if (dev_bytes and dev_sec) else None

    host_sec = t1 - t0
    host_mib = bytes_total / host_sec / 1048576 if host_sec > 0 else 0.0

    (RESULTS / f"{name}.log").write_bytes(nsh_buf)

    return {
        "name": name,
        "bs": bs,
        "count": count,
        "dd_done": dd_done,
        "host_bytes": bytes_total,
        "host_sec": host_sec,
        "host_mib_s": host_mib,
        "dev_bytes": dev_bytes,
        "dev_sec": dev_sec,
        "dev_mib_s": dev_mib,
    }


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: bench.py <nsh_serial_port>", file=sys.stderr)
        return 2

    nsh_port = sys.argv[1]
    print(f"NSH port: {nsh_port}", flush=True)

    nsh = serial.Serial(nsh_port, 115200, timeout=0.1)
    nsh.reset_input_buffer()

    try:
        wait_prompt(nsh, timeout=5.0)

        pre_tty = set(glob.glob("/dev/ttyACM*"))
        print(f"pre /dev/ttyACM* = {sorted(pre_tty)}", flush=True)

        nsh.write(b"sercon\r")
        wait_prompt(nsh, timeout=10.0)

        # poll for new ttyACMx
        deadline = time.time() + 5.0
        data_port = None
        while time.time() < deadline:
            cur = set(glob.glob("/dev/ttyACM*"))
            new = cur - pre_tty
            if new:
                data_port = sorted(new)[0]
                break
            time.sleep(0.2)
        if data_port is None:
            cur = sorted(glob.glob("/dev/ttyACM*"))
            raise RuntimeError(f"USB CDC port not enumerated, current={cur}")
        print(f"data port: {data_port}", flush=True)

        # let udev settle, wait for permissions
        time.sleep(0.5)
        data = serial.Serial(data_port, 115200, timeout=0.1)
        data.reset_input_buffer()

        try:
            summary = {"data_port": data_port, "cases": []}
            for name, bs, count in CASES:
                row = run_case(nsh, data, name, bs, count)
                summary["cases"].append(row)
                dev_str = (
                    f"{row['dev_mib_s']:.2f}" if row["dev_mib_s"] is not None else "?"
                )
                print(
                    f"{name}: host {row['host_mib_s']:.2f} MiB/s "
                    f"({row['host_bytes']} B / {row['host_sec']:.2f}s); "
                    f"dev {dev_str} MiB/s "
                    f"(dd_done={row['dd_done']})",
                    flush=True,
                )

            (RESULTS / "summary.json").write_text(json.dumps(summary, indent=2))
            print(json.dumps(summary, indent=2), flush=True)
        finally:
            data.close()
    finally:
        nsh.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
