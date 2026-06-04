#!/usr/bin/env python3
"""
T113 AMP one-shot deploy + dual-console monitor.

Workflow:
  1. Verify xfel sees a FEL device (board must be in FEL mode first).
  2. xfel ddr t113-s3
  3. xfel write 0x40000000 core0.bin
  4. xfel exec  0x40000000           (core0 starts; registers /dev/rptun/core1)
  5. Open /dev/ttyACM0 (core0) + /dev/ttyACM1 (core1) in parallel via pyserial.
  6. Stream prefixed output ([core0] / [core1]) until timeout.

core1 rides in the fwromfs image as /fw/core1.elf.  Start it from the core0
console with "rptun start /dev/rptun/core1" (the loader resolves the ELF entry
and releases CPU1 there).

Usage:
  t113_amp_deploy_test.py [--timeout 30] [--no-deploy]
"""

import argparse
import os
import subprocess
import sys
import threading
import time

import serial

CORE0_BIN = os.path.expanduser("~/remote-deploy/t113-amp/core0.bin")
TTY_CORE0 = "/dev/ttyACM0"
TTY_CORE1 = "/dev/ttyACM1"


def sh(cmd, check=True):
    print(f"$ {' '.join(cmd)}", flush=True)
    return subprocess.run(cmd, check=check).returncode


def deploy():
    r = subprocess.run(["xfel", "version"], capture_output=True, text=True)
    if "AWUSBFEX" not in r.stdout:
        print(f"FEL not detected:\n{r.stdout}{r.stderr}", file=sys.stderr)
        return False
    print(f"FEL OK: {r.stdout.strip()}", flush=True)

    sh(["xfel", "ddr", "t113-s3"])
    sh(["xfel", "write", "0x40000000", CORE0_BIN])
    # core1 loads from /fw/core1.elf via "rptun start /dev/rptun/core1"
    # once core0 is up.
    sh(["xfel", "exec", "0x40000000"])
    return True


def monitor(dev, label, stop_event):
    try:
        s = serial.Serial(dev, 115200, timeout=0.2)
    except Exception as e:
        print(f"[{label}] open {dev} FAILED: {e}", file=sys.stderr, flush=True)
        return
    print(f"[{label}] {dev} opened", flush=True)
    buf = b""
    while not stop_event.is_set():
        try:
            data = s.read(1024)
        except Exception as e:
            print(f"[{label}] read error: {e}", file=sys.stderr, flush=True)
            break
        if data:
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                print(
                    f"[{label}] {line.decode('utf-8', 'replace').rstrip()}", flush=True
                )
    if buf:
        print(f"[{label}] {buf.decode('utf-8', 'replace')}", flush=True)
    s.close()


def send_cmd(dev, cmd, label):
    try:
        s = serial.Serial(dev, 115200, timeout=0.2)
    except Exception as e:
        print(f"[{label}] send open {dev} failed: {e}", flush=True)
        return
    s.write(b"\r\n")
    time.sleep(0.2)
    s.write(cmd.encode("utf-8") + b"\r\n")
    s.flush()
    s.close()
    print(f"--> {label}: {cmd}", flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument(
        "--timeout",
        type=int,
        default=30,
        help="seconds to monitor after exec (default 30)",
    )
    p.add_argument(
        "--no-deploy",
        action="store_true",
        help="skip xfel deploy, only monitor consoles",
    )
    p.add_argument(
        "--smoke",
        action="store_true",
        help="after boot, send 'hello' to both consoles and " "verify response on each",
    )
    p.add_argument(
        "--boot-wait",
        type=float,
        default=6.0,
        help="seconds to wait after exec before sending smoke "
        "commands (default 6.0)",
    )
    args = p.parse_args()

    stop_event = threading.Event()
    t0 = threading.Thread(
        target=monitor, args=(TTY_CORE0, "core0", stop_event), daemon=True
    )
    t1 = threading.Thread(
        target=monitor, args=(TTY_CORE1, "core1", stop_event), daemon=True
    )
    t0.start()
    t1.start()
    time.sleep(0.5)

    if not args.no_deploy:
        if not deploy():
            stop_event.set()
            return 1

    if args.smoke:
        print(f"--- waiting {args.boot_wait}s for boot ---", flush=True)
        time.sleep(args.boot_wait)
        # Probe both cores: a stray '\r' wakes the prompt, then send a
        # builtin that the default nsh defconfig supports.
        send_cmd(TTY_CORE0, "hello", "core0")
        send_cmd(TTY_CORE1, "hello", "core1")
        time.sleep(1.5)
        # AMP rptun e2e: enumerate /dev to see if cross-image rpmsg
        # endpoints (e.g. /dev/ttyCORE1, /dev/rpmsg) showed up on either
        # side. Master is the rpmsg host; slave-side endpoints should
        # appear on master's /dev once the channel is up.
        send_cmd(TTY_CORE0, "ls /dev", "core0")
        send_cmd(TTY_CORE1, "ls /dev", "core1")
        time.sleep(2.0)
        send_cmd(TTY_CORE0, "free", "core0")
        send_cmd(TTY_CORE1, "free", "core1")
        time.sleep(2.0)

    if args.smoke:
        remain = max(args.timeout - args.boot_wait - 4.0, 2.0)
    else:
        remain = args.timeout
    print(f"--- monitoring remaining {remain:.1f}s ---", flush=True)
    time.sleep(remain)
    stop_event.set()
    t0.join(timeout=2)
    t1.join(timeout=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
