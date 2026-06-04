#!/usr/bin/env python3
"""Local pyserial pre-capture + FEL/xfel deploy + boot log dump.

Single-port NSH console (default /dev/ttyACM0 = CH343 USB-UART; override with TTY env var).
"""

import os
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import serial

TTY = os.environ.get("TTY", "/dev/ttyACM0")
NUTTX = Path(os.path.expanduser("~/projects/t113/cdcopt/build_cdcacm/nuttx.bin"))
TS = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
LOGDIR = Path(os.path.expanduser(f"~/projects/t113/cdcopt/cdcbench/{TS}-bootcap"))
LOGDIR.mkdir(parents=True, exist_ok=True)

stop = threading.Event()
t0 = time.monotonic()


def reader():
    raw = open(LOGDIR / "ttyUSB0.bin", "wb", buffering=0)
    txt = open(LOGDIR / "ttyUSB0.log", "w", buffering=1)
    try:
        s = serial.Serial(TTY, 115200, timeout=0.05, rtscts=False, xonxoff=False)
        s.reset_input_buffer()
        txt.write(f"[{time.monotonic()-t0:7.3f}] OPEN {TTY}\n")
        while not stop.is_set():
            d = s.read(4096)
            if d:
                ts = time.monotonic() - t0
                raw.write(d)
                try:
                    decoded = (
                        d.decode("utf-8", "replace")
                        .replace("\r\n", "\n")
                        .replace("\r", "\n")
                    )
                    txt.write(f"[{ts:7.3f}] {decoded}")
                except Exception:
                    txt.write(f"[{ts:7.3f}] <{len(d)}B raw>\n")
        s.close()
        txt.write(f"[{time.monotonic()-t0:7.3f}] CLOSE\n")
    except Exception as e:
        txt.write(f"ERR: {e}\n")
    finally:
        raw.close()
        txt.close()


t = threading.Thread(target=reader, daemon=False)
t.start()
time.sleep(0.5)

print(f"[{time.monotonic()-t0:.2f}] === FEL recovery ===", flush=True)
fel = (
    "printf 'connect\\nhalt\\n"
    "w4 0x07090100, 0x5AA50001\\n"
    "w4 0x020500B8, 0x16AA0000\\n"
    "w4 0x020500A8, 0x16AA0001\\n"
    "exit\\n' | timeout 15 JLinkExe -device Cortex-A7 -if JTAG "
    "-speed 20000 -nogui 1"
)
with open(LOGDIR / "jlink.log", "w") as f:
    subprocess.run(["bash", "-c", fel], check=False, stdout=f, stderr=subprocess.STDOUT)

time.sleep(1.5)

print(f"[{time.monotonic()-t0:.2f}] === xfel ===", flush=True)
with open(LOGDIR / "xfel.log", "w") as xlog:
    for cmd in [
        ["xfel", "version"],
        ["xfel", "ddr", "t113-s3"],
        ["xfel", "write", "0x40000000", str(NUTTX)],
        ["xfel", "exec", "0x40000000"],
    ]:
        xlog.write(f"\n=== {' '.join(cmd)} ===\n")
        xlog.flush()
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        xlog.write(f"rc={r.returncode}\nSTDOUT:\n{r.stdout}\nSTDERR:\n{r.stderr}\n")
        xlog.flush()

print(
    f"[{time.monotonic()-t0:.2f}] === xfel exec returned, waiting 8s for boot log ===",
    flush=True,
)
time.sleep(8)

stop.set()
t.join(timeout=2)

print(f"\n[{time.monotonic()-t0:.2f}] === DUMP ===", flush=True)
log = LOGDIR / "ttyUSB0.log"
binf = LOGDIR / "ttyUSB0.bin"
print(f"\n--- ttyUSB0 ({binf.stat().st_size} raw bytes) ---")
text = log.read_text(errors="replace")
if len(text) <= 6000:
    print(text)
else:
    print(text[:2000])
    print("  ... [TRUNCATED] ...")
    print(text[-3500:])

print(f"\nlogs: {LOGDIR}")
