#!/usr/bin/env python3
"""
T113 USB MSC corner-case + integrity stress test.

Pre-conditions:
  - board running t113-evb/msc image
  - host has /dev/sde = NuttX Mass Storage (4 MiB ram0)
  - run as user with R/W to /dev/sde (dialout/disk via sudo helper)
  - /dev/ttyACM0 = NSH console

Test matrix:
  S1   read 1  byte at offset 0
  S2   read 511 bytes at offset 1
  S3   read 512 bytes at offset 0 (block aligned)
  S4   read 4097 bytes at offset 0 (cross block + tail)
  V1   write 1MB random + read back + sha256 compare (integrity)
  V2   write whole device (4MB) + read back + sha256 compare
  V3   read at unaligned LBA boundary (offset 257 bytes, len 1MB)
  T1   sequential read throughput (4MB device, 4 passes = 16MB)
  T2   sequential write throughput (1 MB pattern x 4 = 4MB)
"""

import hashlib
import json
import os
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

DEV = os.environ.get("MSC_DEV", "/dev/sde")
SUDO_PW = os.environ.get("SUDO_PW", "XXLXXL")

TS = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
RESULTS = Path(
    os.path.expanduser(f"~/projects/t113/cdcopt/cdcbench/{TS}-msc-stability")
)
RESULTS.mkdir(parents=True, exist_ok=True)

print(f"DEV={DEV}  results={RESULTS}", flush=True)


def sudo_dd(args):
    cmd = ["sudo", "-S"] + ["dd"] + args
    return subprocess.run(
        cmd, input=SUDO_PW.encode() + b"\n", capture_output=True, timeout=60
    )


def open_read_raw(path, offset, length):
    """Read length bytes starting at offset from raw block dev."""
    with open(path, "rb") as f:
        f.seek(offset)
        return f.read(length)


def write_block(path, data, bs=4096):
    """Write data to path with fdatasync."""
    cmd = ["dd", f"of={path}", f"bs={bs}", "conv=fdatasync", "status=none"]
    r = subprocess.run(cmd, input=data, capture_output=True, timeout=120)
    subprocess.run(["sync"], check=False)
    return r


def read_block(path, bs, count, skip=0):
    """Aligned block read with O_DIRECT."""
    cmd = [
        "dd",
        f"if={path}",
        f"bs={bs}",
        f"count={count}",
        f"skip={skip}",
        "iflag=fullblock,direct",
        "status=none",
    ]
    r = subprocess.run(cmd, capture_output=True, timeout=120)
    return r.stdout


CASES = []


def run_size_corner(name, length, offset=0):
    t0 = time.monotonic()
    data = open_read_raw(DEV, offset, length)
    t1 = time.monotonic()
    ok = len(data) == length
    return {
        "name": name,
        "expected": length,
        "got": len(data),
        "size_ok": ok,
        "data_ok": None,
        "time_s": t1 - t0,
    }


def run_v1():
    """Write 1MB random + read back + sha256 compare."""
    pattern = os.urandom(1024 * 1024)
    sha_in = hashlib.sha256(pattern).hexdigest()
    write_block(DEV, pattern, 4096)
    rb = read_block(DEV, 4096, 256)  # 1MB
    sha_out = hashlib.sha256(rb[: 1024 * 1024]).hexdigest()
    return {
        "name": "V1_1M_integrity",
        "expected": 1024 * 1024,
        "got": len(rb),
        "size_ok": len(rb) >= 1024 * 1024,
        "data_ok": sha_in == sha_out,
        "sha_in": sha_in,
        "sha_out": sha_out,
    }


def run_v2():
    """Write whole 4MB device + read back + sha256 compare."""
    pattern = os.urandom(4 * 1024 * 1024)
    sha_in = hashlib.sha256(pattern).hexdigest()
    write_block(DEV, pattern, 4096)
    rb = read_block(DEV, 4096, 1024)  # 4MB
    sha_out = hashlib.sha256(rb[: 4 * 1024 * 1024]).hexdigest()
    return {
        "name": "V2_4M_integrity",
        "expected": 4 * 1024 * 1024,
        "got": len(rb),
        "size_ok": len(rb) >= 4 * 1024 * 1024,
        "data_ok": sha_in == sha_out,
        "sha_in": sha_in,
        "sha_out": sha_out,
    }


def run_v3():
    """Unaligned read crossing block boundary (offset 257, len 1MB)."""
    pattern = os.urandom(2 * 1024 * 1024)
    write_block(DEV, pattern, 4096)
    # Read 1MB at offset 257 via raw fd (arbitrary offset)
    rb = open_read_raw(DEV, 257, 1024 * 1024)
    sha_out = hashlib.sha256(rb).hexdigest()
    sha_expected = hashlib.sha256(pattern[257 : 257 + 1024 * 1024]).hexdigest()
    return {
        "name": "V3_unaligned_read",
        "expected": 1024 * 1024,
        "got": len(rb),
        "size_ok": len(rb) == 1024 * 1024,
        "data_ok": sha_out == sha_expected,
        "sha_expected": sha_expected,
        "sha_got": sha_out,
    }


def _drop_cache(fd):
    """Drop page cache for the block device so re-read goes to USB,
    not Linux page cache.  Without this, read throughput numbers are
    bogus (memory bandwidth, not USB)."""
    try:
        os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED)
    except Exception:
        pass


def run_throughput_read():
    """Read benchmark via direct fd (no subprocess overhead) with
    explicit cache drop between write-prep and read measurement."""
    pattern = os.urandom(4 * 1024 * 1024)
    sha_in = hashlib.sha256(pattern).hexdigest()
    fd = os.open(DEV, os.O_RDWR)
    try:
        # prime device with known pattern
        os.lseek(fd, 0, 0)
        os.write(fd, pattern)
        os.fsync(fd)
        _drop_cache(fd)

        total = 0
        sha_last = None
        t0 = time.monotonic()
        for _ in range(4):
            os.lseek(fd, 0, 0)
            _drop_cache(fd)  # force re-fetch from USB each pass
            rb = os.read(fd, 4 * 1024 * 1024)
            total += len(rb)
            sha_last = hashlib.sha256(rb).hexdigest()
        t1 = time.monotonic()
    finally:
        os.close(fd)
    return {
        "name": "T1_read_4x4M",
        "expected": 16 * 1024 * 1024,
        "got": total,
        "size_ok": total >= 16 * 1024 * 1024,
        "data_ok": sha_last == sha_in,
        "host_mib_s": total / (t1 - t0) / 1048576 if (t1 - t0) > 0 else 0,
        "time_s": t1 - t0,
    }


def run_throughput_write():
    """Write benchmark via direct fd (no subprocess fork overhead),
    fsync per pass to actually push to USB."""
    fd = os.open(DEV, os.O_RDWR)
    try:
        sha_in = sha_out = None
        total_written = 0
        t0 = time.monotonic()
        for i in range(4):
            pattern = os.urandom(1024 * 1024)
            os.lseek(fd, 0, 0)
            os.write(fd, pattern)
            os.fsync(fd)
            total_written += 1024 * 1024
            # verify last write
            if i == 3:
                sha_in = hashlib.sha256(pattern).hexdigest()
                _drop_cache(fd)
                os.lseek(fd, 0, 0)
                rb = os.read(fd, 1024 * 1024)
                sha_out = hashlib.sha256(rb).hexdigest()
        t1 = time.monotonic()
    finally:
        os.close(fd)
    return {
        "name": "T2_write_4x1M",
        "expected": 4 * 1024 * 1024,
        "got": total_written,
        "size_ok": True,
        "data_ok": sha_in == sha_out,
        "host_mib_s": total_written / (t1 - t0) / 1048576 if (t1 - t0) > 0 else 0,
        "time_s": t1 - t0,
    }


CASES_FNS = [
    ("S1_1B", lambda: run_size_corner("S1_1B", 1)),
    ("S2_511B", lambda: run_size_corner("S2_511B", 511, 1)),
    ("S3_512B", lambda: run_size_corner("S3_512B", 512)),
    ("S4_4097B", lambda: run_size_corner("S4_4097B", 4097)),
    ("V1_1M_integrity", run_v1),
    ("V2_4M_integrity", run_v2),
    ("V3_unaligned_read", run_v3),
    ("T1_read_4x4M", run_throughput_read),
    ("T2_write_4x1M", run_throughput_write),
]


def main():
    summary = {"dev": DEV, "cases": []}
    print(
        f"\n{'name':>20s} {'expected':>10s} {'got':>10s} "
        f"{'size':>5s} {'data':>5s} {'time':>7s}  extra"
    )
    all_ok = True
    for name, fn in CASES_FNS:
        try:
            r = fn()
        except Exception as e:
            r = {
                "name": name,
                "expected": "?",
                "got": "?",
                "size_ok": False,
                "data_ok": False,
                "exc": str(e),
            }
        summary["cases"].append(r)
        size = "OK" if r.get("size_ok") else "FAIL"
        data = "n/a" if r.get("data_ok") is None else ("OK" if r["data_ok"] else "FAIL")
        if not r.get("size_ok") or r.get("data_ok") is False:
            all_ok = False
        extra = ""
        if "host_mib_s" in r:
            extra = f"  {r['host_mib_s']:.2f} MiB/s"
        elapsed = r.get("time_s", 0)
        print(
            f"{r['name']:>20s} {str(r['expected']):>10s} {str(r['got']):>10s} "
            f"{size:>5s} {data:>5s} {elapsed:6.2f}s{extra}"
        )

    summary["all_ok"] = all_ok
    (RESULTS / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"\nALL OK: {all_ok}")
    print(f"summary: {RESULTS / 'summary.json'}")


if __name__ == "__main__":
    main()
