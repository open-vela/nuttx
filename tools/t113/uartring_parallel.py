#!/usr/bin/env python3
"""Rigorous 5-UART parallel test for t113-evb alluart config.

Host-side driver for the ``uartring -p`` firmware command shipped with
``boards/arm/t113/t113-evb/configs/alluart``.  The firmware runs the 5
per-port workers as sibling pthreads inside a single process, emitting
shared-clock MARK_START / MARK_END lines.  Previous attempts that
launched 5 NSH commands with ``&`` suffered ~1.5 s per-port launch skew
and therefore never overlapped in steady state (see commit
3f57a152443 for the firmware-side fix that introduced ``-p`` mode).

This script:
  * sends one ``uartring -p ...`` line over the NSH ACM console
  * parses MARK_START / MARK_END / OK / FAIL lines
  * reports per-port start_ns, end_ns, elapsed, bytes, B/s, errors
  * computes the envelope ``min(end_ns) - max(start_ns)`` = the window
    during which all 5 ports are simultaneously active
  * aggregates throughput only inside that envelope
  * emits grep-friendly ``VERDICT case=... {PASS|FAIL} ...`` lines and
    a trailing ``VERDICT AGGREGATE ...`` line for CI consumption

Usage:
  python3 uartring_parallel.py [--dev /dev/ttyACMx] [--smoke]

Device selection priority (highest to lowest):
  1. --dev CLI argument
  2. UARTRING_DEV environment variable
  3. /dev/ttyACM0 (default)

Requires:
  * pyserial
  * Board flashed with alluart firmware and reachable at the serial
    device selected above

Exit status:
  0  all executed cases PASS
  1  at least one case FAILED
  2  runtime error (missing pyserial, serial open failure, I/O error,
     board unresponsive, unhandled exception)
"""

import argparse
import os
import re
import sys
import time

DEFAULT_DEV = "/dev/ttyACM0"
PORTS = ["/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3", "/dev/ttyS4", "/dev/ttyS5"]

T113_UART_CLK = 100_000_000


def t113_actual_baud(requested_baud: int) -> tuple[int, int]:
    dl = T113_UART_CLK // (requested_baud << 4)
    if dl == 0:
        dl = 1
    actual = T113_UART_CLK // (dl << 4)
    return actual, dl


RE_MARK_START = re.compile(r"MARK_START\s+port=(\S+)\s+ns=(\d+)")
RE_MARK_END = re.compile(
    r"MARK_END\s+port=(\S+)\s+ns=(\d+)\s+rcvd=(\d+)\s+errors=(\d+)"
)
RE_OK = re.compile(r"OK\s+(\d+)/(\d+)\s+([\d.]+)s\s+(\d+)\s*B/s\s+([\d.]+)%")
# Firmware emits the per-run verdict as a line-anchored "FAIL: <port> ..."
# summary.  Anchor to start-of-line + colon so benign syslog text such as
# "UART base=0x... DMA alloc FAIL (rx=0 tx=0) -> PIO fallback" does not
# get counted as a test failure.
RE_FAIL = re.compile(r"^FAIL:\s", re.MULTILINE)

PROFILES_FULL = [
    (65536, 115200, 15),
    (65536, 3000000, 10),
    (65536, 4000000, 10),
    (1048576, 4000000, 20),
    (65537, 4000000, 10),
    (1048575, 4000000, 20),
]
PROFILES_SMOKE = [
    (65536, 115200, 15),
]


def build_cmd(baud, size):
    """Single-command parallel mode (firmware -p flag).
    All 5 ports run as sibling pthreads within one uartring process:
    no NSH launch skew, no getopt race, shared CLOCK_MONOTONIC base.
    """
    ports = " ".join(PORTS)
    return f"uartring -p -b {baud} -s {size} {ports}"


def drain_to_prompt(s, timeout=5.0):
    """Synchronize with the NSH prompt before issuing a new command."""
    buf = b""
    deadline = time.time() + timeout
    synced = False

    s.write(b"\r\n")
    s.flush()
    while time.time() < deadline:
        c = s.read(4096)
        if c:
            buf += c
            if b"nsh>" in buf:
                synced = True
                break
        else:
            time.sleep(0.02)

    if synced:
        time.sleep(0.05)
        s.reset_input_buffer()

    return synced


def case_label(baud):
    """Grep-stable short label used in VERDICT lines."""
    if baud % 1000000 == 0:
        return f"65k@{baud // 1000000}M"
    if baud >= 1000000:
        return f"65k@{baud / 1000000:g}M"
    return f"65k@{baud}"


def run_one(s, baud, size, timeout):
    cmd = build_cmd(baud, size)
    s.write((cmd + "\r\n").encode())
    s.flush()

    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        c = s.read(8192)
        if c:
            buf += c
        # Wait for the final uartring PASS/FAIL summary line so NSH
        # has printed its new prompt before we return.  Otherwise the
        # next run_one's command races uartring's tail output and
        # gets swallowed.
        if b"PASS: 5/5 passed" in buf or b"FAIL: " in buf:
            end = time.time() + 2.0
            while time.time() < end:
                tail = s.read(16384)
                if tail:
                    buf += tail
                    if b"nsh>" in buf:
                        break
                else:
                    time.sleep(0.02)
            break

    text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", buf.decode(errors="replace"))

    per_port = {
        p: {"start_ns": None, "end_ns": None, "rcvd": 0, "errors": 0} for p in PORTS
    }
    for p, ns in RE_MARK_START.findall(text):
        per_port.setdefault(
            p, {"start_ns": None, "end_ns": None, "rcvd": 0, "errors": 0}
        )
        per_port[p]["start_ns"] = int(ns)
    for p, ns, rcvd, err in RE_MARK_END.findall(text):
        per_port.setdefault(
            p, {"start_ns": None, "end_ns": None, "rcvd": 0, "errors": 0}
        )
        per_port[p]["end_ns"] = int(ns)
        per_port[p]["rcvd"] = int(rcvd)
        per_port[p]["errors"] = int(err)

    return (
        per_port,
        len(RE_OK.findall(text)),
        len(RE_FAIL.findall(text)),
        len(re.findall(r"MISMATCH @", text)),
        len(re.findall(r"ERROR: stall", text)),
        text,
    )


def summarize(label, per_port, baud, actual_baud, n_ok, n_fail, n_mm, n_stall):
    print(f"\n## {label}")
    valid = [
        (p, d) for p, d in per_port.items() if d.get("start_ns") and d.get("end_ns")
    ]
    tag = case_label(baud)
    if len(valid) < 5:
        print(f"  INCOMPLETE: only {len(valid)}/5 ports reported MARKs")
        print(f"  OK={n_ok} FAIL={n_fail} MM={n_mm} stall={n_stall}")
        print(
            f"VERDICT case={tag} FAIL ok={n_ok} fail={n_fail} "
            f"mm={n_mm} stall={n_stall}"
        )
        return False

    starts = [d["start_ns"] for _, d in valid]
    ends = [d["end_ns"] for _, d in valid]
    launch_span_us = (max(starts) - min(starts)) / 1000.0
    envelope_ns = min(ends) - max(starts)
    parallel_ok = envelope_ns > 0

    print(f"  launch_span = {launch_span_us:>8.1f} us  " f"(max_start - min_start)")
    print(
        f"  envelope    = {envelope_ns/1e9:>8.4f} s  "
        f"(min_end - max_start)  "
        f'{"[PARALLEL]" if parallel_ok else "[NON-OVERLAP]"}'
    )

    base = min(starts)
    print(
        f'  {"port":<14} {"start_ms":>9} {"end_ms":>9} '
        f'{"elapsed":>9} {"rcvd":>9} {"B/s":>10}'
    )
    sum_rate = 0.0
    for p, d in sorted(valid):
        e_s = (d["end_ns"] - d["start_ns"]) / 1e9
        rate = d["rcvd"] / e_s if e_s > 0 else 0
        sum_rate += rate
        print(
            f'  {p:<14} {(d["start_ns"]-base)/1e6:>9.2f} '
            f'{(d["end_ns"]-base)/1e6:>9.2f} '
            f'{e_s:>8.3f}s {d["rcvd"]:>9} {rate:>10.0f}'
        )

    if parallel_ok:
        env_s = envelope_ns / 1e9
        env_agg = sum_rate * env_s
        env_tp = env_agg / env_s
        per_port_ceiling = actual_baud // 10
        five_port_ceiling = 5 * per_port_ceiling
        pct = env_tp / five_port_ceiling * 100
        print(f"  sum_of_port_rates  = {sum_rate:>10.0f} B/s")
        print(
            f"  env_aggregate_B    = {env_agg:>10.0f} bytes "
            f"(5 ports delivered concurrently inside envelope)"
        )
        print(f"  env_aggregate_B/s  = {env_tp:>10.0f} B/s")
        print(
            f"  5-port ceiling     = {five_port_ceiling:>10.0f} B/s "
            f"(actual_baud={actual_baud})"
        )
        print(f"  pct_of_ceiling     = {pct:>5.1f}%")

    print(f"  OK={n_ok} FAIL={n_fail} MM={n_mm} stall={n_stall}")
    ok = parallel_ok and n_fail == 0 and n_mm == 0 and n_stall == 0
    verdict_word = "PASS" if ok else "FAIL"
    if parallel_ok:
        print(
            f"VERDICT case={tag} {verdict_word} req={baud} "
            f"actual={actual_baud} ceiling={five_port_ceiling} "
            f"agg={env_tp:.0f} pct={pct:.1f}% "
            f"ok={n_ok} fail={n_fail} mm={n_mm} stall={n_stall}"
        )
    else:
        print(
            f"VERDICT case={tag} {verdict_word} req={baud} "
            f"actual={actual_baud} ok={n_ok} fail={n_fail} "
            f"mm={n_mm} stall={n_stall}"
        )
    return ok


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Rigorous 5-UART parallel test driver for "
        't113-evb alluart firmware.  Sends "uartring -p" '
        "over the NSH console and reports parallel "
        "throughput + grep-friendly VERDICT lines."
    )
    parser.add_argument(
        "--dev",
        default=os.environ.get("UARTRING_DEV", DEFAULT_DEV),
        help="NSH console serial device "
        "(default: $UARTRING_DEV or %s)" % DEFAULT_DEV,
    )
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="Smoke mode: only run the 65536 bytes @ 115200 baud case",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    try:
        import serial  # noqa: WPS433 - delayed for exit-code-2 semantics
    except ImportError as e:
        print(f"ERROR: pyserial not available: {e}", file=sys.stderr)
        return 2

    profiles = PROFILES_SMOKE if args.smoke else PROFILES_FULL

    try:
        s = serial.Serial(args.dev, 115200, timeout=0.3)
    except (OSError, serial.SerialException) as e:
        print(f"ERROR: cannot open {args.dev}: {e}", file=sys.stderr)
        return 2

    try:
        total_cases = len(profiles)
        passed = 0
        failed = 0

        for size, baud, tmo in profiles:
            if not drain_to_prompt(s, timeout=8.0):
                s.close()
                s = serial.Serial(args.dev, 115200, timeout=0.3)
                if not drain_to_prompt(s, timeout=8.0):
                    raise TimeoutError("NSH prompt not found before case")

            actual_baud, dl = t113_actual_baud(baud)
            per_port_ceiling = actual_baud // 10
            five_port_ceiling = 5 * per_port_ceiling
            print(
                f"[baud] requested={baud} actual={actual_baud}  "
                f"(DL={dl}, theoretical per-port payload={per_port_ceiling} "
                f"bytes/s @ 8N1)"
            )
            print(f"[baud] 5-port theoretical ceiling={five_port_ceiling} " f"bytes/s")
            per_port, n_ok, n_fail, n_mm, n_stall, _ = run_one(s, baud, size, tmo)
            ok = summarize(
                f"size={size} baud={baud}",
                per_port,
                baud,
                actual_baud,
                n_ok,
                n_fail,
                n_mm,
                n_stall,
            )
            if ok:
                passed += 1
            else:
                failed += 1
            if not drain_to_prompt(s, timeout=8.0):
                raise TimeoutError("NSH prompt not found after case")
            time.sleep(1.0)
    except (OSError, serial.SerialException, TimeoutError) as e:
        print(f"ERROR: serial I/O failed: {e}", file=sys.stderr)
        return 2
    finally:
        try:
            s.close()
        except Exception:
            pass

    print(
        f"\nVERDICT AGGREGATE cases={total_cases} " f"passed={passed} failed={failed}"
    )
    # Legacy human-readable summary kept for backward compat.
    print(
        "VERDICT: ALL TRULY PARALLEL"
        if failed == 0
        else "VERDICT: OVERLAP/ERRORS PRESENT"
    )
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nERROR: interrupted", file=sys.stderr)
        sys.exit(2)
    except Exception as e:  # noqa: BLE001 - top-level safety net
        print(f"ERROR: unhandled exception: {e}", file=sys.stderr)
        sys.exit(2)
