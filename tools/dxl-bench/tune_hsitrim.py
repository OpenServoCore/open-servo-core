#!/usr/bin/env python3
"""Sweep HSITRIM, flash, run the straggle test at each candidate, pick the
highest value that stays out of contention. Quick-and-dirty bench-only.

Workflow per candidate:
  1. Patch `set_hsitrim(N)` in firmware/ch32/src/board/bringup.rs
  2. cargo build --release (board crate, riscv32ec_zmmul target)
  3. wlink flash the resulting elf
  4. Bring chip + injector back to TEST_BAUD
  5. Run the straggle bulk-read test N times, classify

Classification per trial:
  - CRC error / corrupted INJ slot         → HSI too fast (contention)
  - Frame ok + extra injector IDLE stamp   → HSI too slow (visible gap)
  - Frame ok + only req-end + frame-end    → ideal (coalesce passes)

Verdict per candidate (over `--trials` trials):
  - any contention                  → "fast"
  - all clean + 0 extra-IDLE trials → "ideal"
  - all clean + some extra-IDLE     → "slow"

Optimum = highest HSITRIM with verdict "ideal". If none, highest "slow".

Usage:
  uv run python tools/dxl-bench/tune_hsitrim.py
  uv run python tools/dxl-bench/tune_hsitrim.py --candidates 18 19 20 21
  uv run python tools/dxl-bench/tune_hsitrim.py --apply   # flash optimum
"""

import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

import serial
import serial.tools.list_ports

sys.path.insert(0, str(Path(__file__).parent))

from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    build_ping,
    build_write,
    parse_fast_response,
    read_status_frame,
)
from echo_port_handler import EchoDrainingPortHandler

REPO = Path(__file__).resolve().parents[2]
BRINGUP_RS = REPO / "firmware/ch32/src/board/bringup.rs"
BOARD_DIR = REPO / "firmware/boards/osc-dev-v006"
ELF = BOARD_DIR / "target/riscv32ec_zmmul-unknown-none-elf/release/osc-dev-v006-app-rev-b"
HSITRIM_RE = re.compile(r"w\.set_hsitrim\((\d+)\)")

INJ_ID = 50
DUT_LEN = 4
TEST_BAUD = 1_000_000
BAUD_INDEX = {9600: 0, 57600: 1, 115200: 2, 1_000_000: 3, 2_000_000: 4, 3_000_000: 5}
BAUD_RATE_IDX_ADDR = 13

WCH_LINKE_VID = 0x1A86
WCH_LINKE_PID = 0x8010
INJECTOR_VID = 0xC0DE
INJECTOR_PID = 0xCAFE
USB_UART_VIDS = frozenset([0x0403, 0x067B, 0x10C4, 0x1A86])


def autodetect_dxl_port() -> str:
    cands = [p for p in serial.tools.list_ports.comports() if p.vid in USB_UART_VIDS]
    pref = [p for p in cands if not (p.vid == WCH_LINKE_VID and p.pid == WCH_LINKE_PID)] or cands
    if len(pref) != 1:
        raise SystemExit(f"need exactly one DXL UART, found {len(pref)}: {[p.device for p in pref]}")
    return pref[0].device


def autodetect_injector() -> str:
    m = [p.device for p in serial.tools.list_ports.comports()
         if p.vid == INJECTOR_VID and p.pid == INJECTOR_PID]
    if len(m) != 1:
        raise SystemExit(f"need exactly one injector, found {len(m)}")
    return m[0]


def get_hsitrim() -> int:
    m = HSITRIM_RE.search(BRINGUP_RS.read_text())
    if not m:
        raise SystemExit("set_hsitrim() not found in bringup.rs")
    return int(m.group(1))


def patch_hsitrim(value: int) -> None:
    text = BRINGUP_RS.read_text()
    BRINGUP_RS.write_text(HSITRIM_RE.sub(f"w.set_hsitrim({value})", text))


def build_and_flash() -> bool:
    print("    build", end="", flush=True)
    r = subprocess.run(
        ["cargo", "build", "--release", "--bin", "osc-dev-v006-app-rev-b"],
        cwd=BOARD_DIR, capture_output=True,
    )
    if r.returncode != 0:
        print(" FAILED")
        print(r.stderr.decode())
        return False
    print(" ok", end="", flush=True)
    print("    flash", end="", flush=True)
    for _ in range(3):
        r = subprocess.run(["wlink", "flash", str(ELF)], capture_output=True)
        if r.returncode == 0:
            print(" ok", flush=True)
            time.sleep(1.5)
            return True
        time.sleep(0.5)
    print(" FAILED")
    print(r.stderr.decode())
    return False


def ping_at(port_path: str, dxl_id: int, baud: int, attempts: int = 3) -> bool:
    s = serial.Serial(port_path, baud, timeout=0.2)
    try:
        time.sleep(0.15)
        for _ in range(attempts):
            s.reset_input_buffer()
            req = build_ping(dxl_id)
            s.write(req)
            s.flush()
            s.read(len(req))
            if read_status_frame(s):
                return True
            time.sleep(0.05)
        return False
    finally:
        s.close()


def ensure_chip_baud(port_path: str, dxl_id: int, target: int) -> None:
    if ping_at(port_path, dxl_id, target):
        return
    for cur in sorted(BAUD_INDEX, reverse=True):
        if cur == target or not ping_at(port_path, dxl_id, cur):
            continue
        s = serial.Serial(port_path, cur, timeout=0.2)
        try:
            time.sleep(0.15)
            s.reset_input_buffer()
            req = build_write(dxl_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[target]]))
            s.write(req)
            s.flush()
            s.read(len(req))
            read_status_frame(s)
        finally:
            s.close()
        time.sleep(0.05)
        return
    raise SystemExit(f"chip not reachable at any known baud")


def ensure_injector_baud(injector_path: str, target: int) -> None:
    inj = serial.Serial(injector_path, 115200, timeout=0.3)
    try:
        inj.write(b"TICK?\n")
        inj.readline()
        inj.write(f"BAUD {target}\n".encode())
        if inj.readline().strip() != b"OK":
            raise SystemExit("injector BAUD command failed")
    finally:
        inj.close()


class Injector:
    def __init__(self, port_path: str):
        self.ser = serial.Serial(port_path, 115200, timeout=0.3)

    def close(self): self.ser.close()

    def cmd(self, line: str) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(line.encode() + b"\n")
        self.ser.flush()
        return self.ser.readline().decode(errors="replace").rstrip()

    def drain_stamps(self) -> list[tuple[int, int]]:
        out = []
        while True:
            r = self.cmd("DRAIN")
            if r == "EMPTY":
                return out
            if not r.startswith("STAMP "):
                return out
            _, t, h = r.split()
            out.append((int(t), int(h)))


def run_trial(port, injector: Injector, dut_id: int, inj_len: int) -> tuple[str, int]:
    """Returns (status, n_stamps). status ∈ {clean, crc_error, timeout, ...}"""
    inj_data = b"\xaa" * inj_len
    pl = 1 + (2 + inj_len) + (2 + DUT_LEN) + 2

    while port.ser.in_waiting:
        port.ser.read(port.ser.in_waiting)
    injector.drain_stamps()

    inj_bytes = build_fast_first_bytes(
        packet_length=pl, err=0, slot_id=INJ_ID, data=inj_data,
    )
    if injector.cmd(f"ARM bytes={inj_bytes.hex()} after_idle={250 * 18}") != "OK":
        return "arm_failed", 0

    port.writePort(build_fast_bulk_read([(INJ_ID, 0, inj_len), (dut_id, 0, DUT_LEN)]))
    frame = read_status_frame(port.ser, timeout_s=0.5)
    if frame is None:
        injector.drain_stamps()
        return "timeout", 0
    try:
        slots = parse_fast_response(frame, slot_lengths=[inj_len, DUT_LEN])
    except ValueError:
        injector.drain_stamps()
        return "crc_error", 0
    if len(slots) != 2 or slots[0].id != INJ_ID or slots[0].data != inj_data:
        injector.drain_stamps()
        return "inj_corrupt", 0
    if slots[1].id != dut_id or slots[1].error != 0 or len(slots[1].data) != DUT_LEN:
        injector.drain_stamps()
        return "dut_bad", 0
    return "clean", len(injector.drain_stamps())


def classify(port, injector: Injector, dut_id: int, inj_len: int,
             trials: int) -> tuple[str, dict]:
    counts = {"clean": 0, "extra_idle": 0, "crc": 0, "other": 0}
    for _ in range(trials):
        status, n_stamps = run_trial(port, injector, dut_id, inj_len)
        if status == "clean":
            if n_stamps > 2:
                counts["extra_idle"] += 1
            else:
                counts["clean"] += 1
        elif status in ("crc_error", "inj_corrupt"):
            counts["crc"] += 1
        else:
            counts["other"] += 1
    if counts["crc"] > 0:
        v = "fast"
    elif counts["clean"] == trials:
        v = "ideal"
    elif counts["extra_idle"] >= 1 and counts["other"] == 0:
        v = "slow"
    else:
        v = "noisy"
    return v, counts


def sweep_inj_len(port, injector: Injector, dut_id: int, base: int,
                  trials: int) -> dict[int, tuple[str, dict]]:
    """At chosen HSITRIM, push INJ_LEN to find headroom margin."""
    results = {}
    for length in [base, base + 32, base + 64, base + 96, 240]:
        if length > 240 or length in results:
            continue
        v, c = classify(port, injector, dut_id, length, trials)
        results[length] = (v, c)
        print(f"    INJ_LEN={length:3d}: {v:7s}  {c}")
        if v == "fast":
            break
    return results


def measure_candidate(port_path: str, dut_id: int, injector_path: str,
                      trim: int, inj_len: int, trials: int) -> tuple[str, dict] | None:
    patch_hsitrim(trim)
    if not build_and_flash():
        return None
    time.sleep(1.0)
    try:
        ensure_chip_baud(port_path, dut_id, TEST_BAUD)
        ensure_injector_baud(injector_path, TEST_BAUD)
    except SystemExit as e:
        print(f"    baud setup failed: {e}")
        return None
    port = EchoDrainingPortHandler(port_path)
    if not port.openPort() or not port.setBaudRate(TEST_BAUD):
        print("    openPort failed")
        return None
    injector = Injector(injector_path)
    try:
        return classify(port, injector, dut_id, inj_len, trials)
    finally:
        port.closePort()
        injector.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--port", help="DXL bus serial path")
    ap.add_argument("--injector-port", help="V203 injector CDC path")
    ap.add_argument("--id", type=int, default=1, help="DUT DXL id")
    ap.add_argument("--candidates", type=int, nargs="+",
                    default=list(range(16, 24)),
                    help="HSITRIM values to sweep (default: 16..23)")
    ap.add_argument("--inj-len", type=int, default=128,
                    help="straggle INJ payload length per trial")
    ap.add_argument("--trials", type=int, default=5,
                    help="trials per candidate")
    ap.add_argument("--apply", action="store_true",
                    help="patch + flash the chosen optimum at the end")
    ap.add_argument("--fine-sweep", action="store_true",
                    help="after picking optimum, sweep INJ_LEN to map margin")
    args = ap.parse_args()

    port_path = args.port or autodetect_dxl_port()
    injector_path = args.injector_port or autodetect_injector()
    original = get_hsitrim()
    print(f"DXL port:    {port_path}")
    print(f"INJ port:    {injector_path}")
    print(f"DUT id:      {args.id}")
    print(f"baud:        {TEST_BAUD}")
    print(f"INJ_LEN:     {args.inj_len}")
    print(f"trials:      {args.trials}")
    print(f"candidates:  {args.candidates}")
    print(f"original HSITRIM: {original}")

    results: dict[int, tuple[str, dict]] = {}
    try:
        for trim in args.candidates:
            print(f"\n=== HSITRIM={trim} ===")
            r = measure_candidate(port_path, args.id, injector_path,
                                  trim, args.inj_len, args.trials)
            if r is None:
                continue
            results[trim] = r
            v, c = r
            print(f"    → {v}  {c}")
    except KeyboardInterrupt:
        print("\ninterrupted — restoring original HSITRIM and reflashing")
        patch_hsitrim(original)
        build_and_flash()
        return

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    for trim in sorted(results):
        v, c = results[trim]
        print(f"  HSITRIM={trim:2d}: {v:7s}  {c}")

    ideal = [t for t, (v, _) in results.items() if v == "ideal"]
    slow = [t for t, (v, _) in results.items() if v == "slow"]
    if ideal:
        optimum = max(ideal)
        rationale = "ideal — clean coalesce across all trials"
    elif slow:
        optimum = max(slow)
        rationale = "slow but no contention"
    else:
        optimum = original
        rationale = "no acceptable candidate found; restoring original"
    print(f"\nchosen: HSITRIM={optimum} ({rationale})")

    if args.fine_sweep and optimum != original:
        print(f"\n=== fine sweep at HSITRIM={optimum} ===")
        patch_hsitrim(optimum)
        if build_and_flash():
            time.sleep(1.0)
            ensure_chip_baud(port_path, args.id, TEST_BAUD)
            ensure_injector_baud(injector_path, TEST_BAUD)
            port = EchoDrainingPortHandler(port_path)
            port.openPort()
            port.setBaudRate(TEST_BAUD)
            injector = Injector(injector_path)
            try:
                sweep_inj_len(port, injector, args.id, args.inj_len, args.trials)
            finally:
                port.closePort()
                injector.close()

    if args.apply:
        if get_hsitrim() != optimum:
            print(f"\napplying HSITRIM={optimum}")
            patch_hsitrim(optimum)
            build_and_flash()
        else:
            print(f"\nHSITRIM={optimum} already flashed.")
    else:
        # Leave repo + chip matching the original state so the user can review
        # results without an unintended diff in bringup.rs or a mismatched chip.
        if get_hsitrim() != original:
            print(f"\nrestoring HSITRIM={original} (rerun with --apply to flash {optimum})")
            patch_hsitrim(original)
            build_and_flash()


if __name__ == "__main__":
    main()
