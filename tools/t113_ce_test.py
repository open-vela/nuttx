#!/usr/bin/env python3
"""T113 CE Driver Automated Test Script.

Build, deploy via xfel, and verify CE test results over serial.

Usage:
    python3 tools/t113_ce_test.py                # full build + deploy + test
    python3 tools/t113_ce_test.py --skip-build   # deploy + test only
    python3 tools/t113_ce_test.py --serial-only   # serial test only (already deployed)
    python3 tools/t113_ce_test.py --port /dev/ttyUSB1  # custom serial port
"""

import argparse
import os
import re
import subprocess
import sys
import time

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

NUTTX_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD_DIR = os.path.join(NUTTX_DIR, "build_ce")
BOARD_CFG = "t113-evb/ce"
BINARY = os.path.join(BUILD_DIR, "nuttx.bin")
DEFAULT_PORT = "/dev/ttyUSB0"
BAUD = 115200
SCRIPTS_DIR = os.path.join(NUTTX_DIR, "boards/arm/t113/t113-evb/scripts")
FEL_SCRIPT = os.path.join(SCRIPTS_DIR, "t113_fel_recovery.JLinkScript")

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def log(tag, msg, color=None):
    colors = {"green": "\033[32m", "red": "\033[31m", "yellow": "\033[33m"}
    reset = "\033[0m"
    prefix = colors.get(color, "") if sys.stdout.isatty() else ""
    suffix = reset if prefix else ""
    print(f"{prefix}[{tag}]{suffix} {msg}", flush=True)


def run(cmd, **kwargs):
    """Run a shell command, return (returncode, stdout)."""
    kwargs.setdefault("capture_output", True)
    kwargs.setdefault("text", True)
    kwargs.setdefault("cwd", NUTTX_DIR)
    r = subprocess.run(cmd, shell=True, **kwargs)
    return r.returncode, (r.stdout or "") + (r.stderr or "")


def run_check(tag, cmd):
    """Run and abort on failure."""
    rc, out = run(cmd)
    if rc != 0:
        log(tag, f"FAILED: {cmd}", "red")
        for line in out.strip().split("\n")[-20:]:
            print(f"  {line}")
        sys.exit(1)
    return out


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------


def do_build():
    log("BUILD", f"rm -rf {BUILD_DIR}")
    run(f"rm -rf {BUILD_DIR}")

    log("BUILD", f"cmake -B {BUILD_DIR} -DBOARD_CONFIG={BOARD_CFG} -GNinja")
    run_check("BUILD", f"cmake -B {BUILD_DIR} -DBOARD_CONFIG={BOARD_CFG} -GNinja")

    log("BUILD", "ninja ...")
    run_check("BUILD", f"ninja -C {BUILD_DIR}")

    if not os.path.exists(BINARY):
        log("BUILD", f"nuttx.bin not found at {BINARY}", "red")
        sys.exit(1)

    size = os.path.getsize(BINARY)
    log("BUILD", f"OK — {BINARY} ({size} bytes)", "green")


# ---------------------------------------------------------------------------
# Deploy
# ---------------------------------------------------------------------------


def enter_fel(max_attempts=10):
    """Enter FEL mode via JLink FEL recovery script."""
    for i in range(1, max_attempts + 1):
        log("DEPLOY", f"FEL attempt {i}/{max_attempts} ...")
        run(
            f"printf 'exit\\n' | timeout 10 JLinkExe "
            f"-device Cortex-A7 -if JTAG -speed 20000 "
            f"-nogui 1 -autoconnect 1 "
            f"-JLinkScriptFile {FEL_SCRIPT} 2>&1"
        )
        time.sleep(0.5)
        rc, out = run("xfel version 2>&1")
        if "AWUSBFEX" in out:
            log("DEPLOY", "FEL OK", "green")
            return True
    log("DEPLOY", "Failed to enter FEL", "red")
    return False


def do_deploy():
    if not enter_fel():
        sys.exit(1)

    log("DEPLOY", "DDR init ...")
    run_check("DEPLOY", "xfel ddr t113-s3")
    time.sleep(0.5)

    log("DEPLOY", f"Writing {BINARY} to 0x40000000 ...")
    run_check("DEPLOY", f"xfel write 0x40000000 {BINARY}")

    log("DEPLOY", "Executing ...")
    run_check("DEPLOY", "xfel exec 0x40000000")

    log("DEPLOY", "OK — firmware running", "green")


# ---------------------------------------------------------------------------
# Serial test
# ---------------------------------------------------------------------------


def do_serial_test(port):
    try:
        import serial
    except ImportError:
        log("SERIAL", "pyserial not installed: pip install pyserial", "red")
        sys.exit(1)

    log("SERIAL", f"Opening {port} @ {BAUD} ...")

    # Wait a moment for board to boot
    time.sleep(2)

    ser = serial.Serial(port, BAUD, timeout=3)
    ser.reset_input_buffer()

    # Send a few enters to get nsh prompt
    for _ in range(3):
        ser.write(b"\r")
        time.sleep(0.3)

    # Drain boot output
    boot_data = ser.read(4096).decode("utf-8", errors="replace")

    # Check for ALGTEST results in boot log
    if "Failed" in boot_data and "test #" in boot_data:
        log("SERIAL", "WARNING: ALGTEST failures in boot log!", "yellow")
        for line in boot_data.split("\n"):
            if "Failed" in line:
                print(f"  {line.strip()}")

    # Wait for nsh prompt
    if "nsh>" not in boot_data:
        ser.write(b"\r")
        time.sleep(1)
        boot_data += ser.read(2048).decode("utf-8", errors="replace")

    if "nsh>" not in boot_data:
        log("SERIAL", "No nsh> prompt detected", "red")
        print(f"  Got: {boot_data[-200:]}")
        ser.close()
        sys.exit(1)

    log("SERIAL", "nsh> prompt OK", "green")

    # Run cetest
    log("SERIAL", "Running cetest ...")
    ser.reset_input_buffer()
    ser.write(b"cetest\r")

    # Collect output with timeout
    output = ""
    deadline = time.time() + 15  # 15 second timeout
    while time.time() < deadline:
        chunk = ser.read(1024).decode("utf-8", errors="replace")
        output += chunk
        if "RESULT:" in output:
            # Read a bit more to get trailing newline
            time.sleep(0.2)
            output += ser.read(512).decode("utf-8", errors="replace")
            break

    ser.close()

    if not output.strip():
        log("SERIAL", "No output from cetest", "red")
        sys.exit(1)

    # Parse results
    passes = re.findall(r"^PASS: (.+)$", output, re.MULTILINE)
    fails = re.findall(r"^FAIL: (.+)$", output, re.MULTILINE)
    result_match = re.search(r"RESULT: (\d+)/(\d+) passed", output)

    # Print test output
    for line in output.strip().split("\n"):
        line = line.strip()
        if not line:
            continue
        if line.startswith("PASS:"):
            log("TEST", line, "green")
        elif line.startswith("FAIL:"):
            log("TEST", line, "red")
        elif "RESULT:" in line:
            color = "green" if not fails else "red"
            log("TEST", line, color)
        elif line.startswith("  "):
            print(f"       {line}")

    if result_match:
        p, t = int(result_match.group(1)), int(result_match.group(2))
        log("RESULT", f"{p}/{t} passed", "green" if p == t else "red")
        return len(fails) == 0
    else:
        log(
            "RESULT",
            f"PASS={len(passes)} FAIL={len(fails)} (no summary line)",
            "green" if not fails else "red",
        )
        return len(fails) == 0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description="T113 CE automated test")
    parser.add_argument("--skip-build", action="store_true", help="Skip build step")
    parser.add_argument(
        "--serial-only",
        action="store_true",
        help="Skip build and deploy, serial test only",
    )
    parser.add_argument(
        "--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})"
    )
    args = parser.parse_args()

    os.chdir(NUTTX_DIR)

    if not args.skip_build and not args.serial_only:
        do_build()

    if not args.serial_only:
        do_deploy()

    ok = do_serial_test(args.port)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
