#!/usr/bin/env python3
"""Sweep HSITRIM, flash, run the straggle test at each candidate, pick the
highest value that stays out of contention. Quick-and-dirty bench-only.

Workflow per candidate:
  1. Patch `set_hsitrim(N)` in firmware/ch32/src/board/bringup.rs
  2. cargo build --release (board crate, riscv32ec_zmmul target)
  3. wlink flash the resulting elf
  4. Bring chip + pirate back to TEST_BAUD
  5. Run the straggle FastBulkRead trial N times, classify

Classification per trial (pirate is master + INJ injector):
  - CRC error / corrupted INJ slot         → HSI too fast (contention)
  - Frame ok + extra IDLE stamp            → HSI too slow (visible gap)
  - Frame ok + single burst stamp          → ideal (coalesce passes)

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

import serial.tools.list_ports

sys.path.insert(0, str(Path(__file__).parent))

from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    build_ping,
    build_write,
    parse_fast_response,
    parse_status,
)
from pirate import Pirate

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

PIRATE_VID = 0xC0DE
PIRATE_PID = 0xCAFE


def autodetect_pirate() -> str:
    m = [p.device for p in serial.tools.list_ports.comports()
         if p.vid == PIRATE_VID and p.pid == PIRATE_PID]
    if len(m) != 1:
        raise SystemExit(f"need exactly one pirate, found {len(m)}")
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


def ping_at(pirate: Pirate, dxl_id: int, baud: int, attempts: int = 3) -> bool:
    pirate.set_baud(baud)
    time.sleep(0.05)
    for _ in range(attempts):
        pirate.drain_stamps()
        if pirate.xfer(build_ping(dxl_id), reply_us=200_000):
            return True
        time.sleep(0.05)
    return False


def ensure_chip_baud(pirate: Pirate, dxl_id: int, target: int) -> None:
    if ping_at(pirate, dxl_id, target):
        return
    for cur in sorted(BAUD_INDEX, reverse=True):
        if cur == target or not ping_at(pirate, dxl_id, cur):
            continue
        reply = pirate.xfer(
            build_write(dxl_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[target]])),
            reply_us=200_000,
        )
        if reply:
            st = parse_status(reply)
            if st.error != 0:
                raise SystemExit(f"baud write rejected: 0x{st.error:02X}")
        time.sleep(0.1)
        pirate.set_baud(target)
        return
    raise SystemExit("chip not reachable at any known baud")


def run_trial(pirate: Pirate, dut_id: int, inj_len: int) -> tuple[str, int]:
    """Returns (status, n_stamps). status ∈ {clean, crc_error, timeout, ...}"""
    inj_data = b"\xaa" * inj_len
    pl = 1 + (2 + inj_len) + (2 + DUT_LEN) + 2
    inj_bytes = build_fast_first_bytes(
        packet_length=pl, err=0, slot_id=INJ_ID, data=inj_data,
    )
    request = build_fast_bulk_read([(INJ_ID, 0, inj_len), (dut_id, 0, DUT_LEN)])

    pirate.drain_stamps()
    b0 = pirate.bytes_count()
    pirate.arm(inj_bytes, after_idle_ticks=250 * 18)
    pirate.master(request)
    time.sleep(0.05)
    b1 = pirate.bytes_count()

    total = b1 - b0
    if total > 256:
        pirate.drain_stamps()
        return "overflow", 0
    all_rx = pirate.rx_range(b0, total)
    frame = all_rx[len(request):]
    if len(frame) < 11:
        pirate.drain_stamps()
        return "timeout", 0
    try:
        slots = parse_fast_response(frame, slot_lengths=[inj_len, DUT_LEN])
    except ValueError:
        pirate.drain_stamps()
        return "crc_error", 0
    if len(slots) != 2 or slots[0].id != INJ_ID or slots[0].data != inj_data:
        pirate.drain_stamps()
        return "inj_corrupt", 0
    if slots[1].id != dut_id or slots[1].error != 0 or len(slots[1].data) != DUT_LEN:
        pirate.drain_stamps()
        return "dut_bad", 0
    return "clean", len(pirate.drain_stamps())


def classify(pirate: Pirate, dut_id: int, inj_len: int,
             trials: int) -> tuple[str, dict]:
    # n_stamps == 1 → coalesced (single burst). > 1 → extra IDLE between INJ+DUT.
    counts = {"clean": 0, "extra_idle": 0, "crc": 0, "other": 0}
    for _ in range(trials):
        status, n_stamps = run_trial(pirate, dut_id, inj_len)
        if status == "clean":
            if n_stamps > 1:
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


def sweep_inj_len(pirate: Pirate, dut_id: int, base: int,
                  trials: int) -> dict[int, tuple[str, dict]]:
    results = {}
    for length in [base, base + 32, base + 64, base + 96, 240]:
        if length > 240 or length in results:
            continue
        v, c = classify(pirate, dut_id, length, trials)
        results[length] = (v, c)
        print(f"    INJ_LEN={length:3d}: {v:7s}  {c}")
        if v == "fast":
            break
    return results


def measure_candidate(pirate_path: str, dut_id: int, trim: int, inj_len: int,
                      trials: int) -> tuple[str, dict] | None:
    patch_hsitrim(trim)
    if not build_and_flash():
        return None
    time.sleep(1.0)
    pirate = Pirate(pirate_path)
    try:
        ensure_chip_baud(pirate, dut_id, TEST_BAUD)
    except SystemExit as e:
        print(f"    baud setup failed: {e}")
        pirate.close()
        return None
    try:
        return classify(pirate, dut_id, inj_len, trials)
    finally:
        pirate.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--pirate-port", help="dxl-pirate CDC path")
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

    pirate_path = args.pirate_port or autodetect_pirate()
    original = get_hsitrim()
    print(f"pirate port: {pirate_path}")
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
            r = measure_candidate(pirate_path, args.id, trim, args.inj_len, args.trials)
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
            pirate = Pirate(pirate_path)
            try:
                ensure_chip_baud(pirate, args.id, TEST_BAUD)
                sweep_inj_len(pirate, args.id, args.inj_len, args.trials)
            finally:
                pirate.close()

    if args.apply:
        if get_hsitrim() != optimum:
            print(f"\napplying HSITRIM={optimum}")
            patch_hsitrim(optimum)
            build_and_flash()
        else:
            print(f"\nHSITRIM={optimum} already flashed.")
    else:
        if get_hsitrim() != original:
            print(f"\nrestoring HSITRIM={original} (rerun with --apply to flash {optimum})")
            patch_hsitrim(original)
            build_and_flash()


if __name__ == "__main__":
    main()
