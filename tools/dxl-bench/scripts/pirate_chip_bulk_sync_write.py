"""Bench: exercise Sync Write and Bulk Write against telemetry-link counters.

Targets `parity_error` and `framing_error` in the `TelemetryDxlLink` block.
Both are `#[ct_field(access = rw)]` with a documented bench-write carve-out
(see `firmware/lib/core/src/regions/telemetry.rs`) — the next wire fault
overwrites whatever sentinel we leave behind, so a stray value is benign.
Pre-test values are snapshotted and restored on exit regardless.

Tests:
  1. Sync Write: foreign-id slot at offset 0, our slot at offset 1, both
     targeting `parity_error`. Verifies the dispatcher walks past the
     foreign slot and applies our payload.
  2. Bulk Write: foreign-id slot first, then our slot targeting
     `framing_error` at a different address than the foreign one. Verifies
     per-slot address/length parsing.

Run:  python scripts/pirate_chip_bulk_sync_write.py [--id ID] [--baud BPS]
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dxl_packet import (
    BROADCAST_ID,
    build_bulk_write,
    build_ping,
    build_read,
    build_sync_write,
    build_write,
    parse_status,
)
from pirate import Pirate, PirateError
from pirate_chip_common import (
    BAUD_INDEX,
    FOREIGN_ID,
    autodetect_pirate,
    set_chip_baud,
    step_hsi,
)


def discover_chip_id(pirate: Pirate, baud: int) -> int:
    """Sweep known bauds for a broadcast Ping reply; return the chip's ID.
    Chip ID is UID-derived (see firmware/ch32/src/board/bringup.rs), so
    callers can't assume a fixed default."""
    candidates = [baud] + [b for b in sorted(BAUD_INDEX, reverse=True) if b != baud]
    for b in candidates:
        pirate.set_baud(b)
        time.sleep(0.05)
        pirate.drain_stamps()
        reply = pirate.xfer(build_ping(BROADCAST_ID), reply_us=200_000)
        if reply:
            return parse_status(reply).id
    raise PirateError("no chip responded to broadcast Ping at any known baud")

# Mirror `dxl_link.LINK_BASE + 4*offset` for the two fields we touch.
# These come from `firmware/lib/core/src/regions/telemetry.rs::TelemetryDxlLink`.
PARITY_ERROR_ADDR = 0x0254
FRAMING_ERROR_ADDR = 0x0258


def read_u32(pirate: Pirate, dxl_id: int, addr: int) -> int:
    frame = pirate.xfer(build_read(dxl_id, addr, 4), reply_us=200_000)
    if not frame:
        raise PirateError(f"no Status frame on read @ 0x{addr:04X}")
    st = parse_status(frame)
    if st.error != 0:
        raise PirateError(f"read @ 0x{addr:04X} err 0x{st.error:02X}")
    return struct.unpack("<I", st.params)[0]


def write_u32(pirate: Pirate, dxl_id: int, addr: int, value: int) -> None:
    payload = struct.pack("<I", value & 0xFFFFFFFF)
    frame = pirate.xfer(build_write(dxl_id, addr, payload), reply_us=200_000)
    if not frame:
        raise PirateError(f"no Status frame on write @ 0x{addr:04X}")
    st = parse_status(frame)
    if st.error != 0:
        raise PirateError(f"write @ 0x{addr:04X} err 0x{st.error:02X}")


def sync_write_test(pirate: Pirate, dxl_id: int) -> bool:
    sentinel = 0xDEADBEEF
    packet = build_sync_write(
        PARITY_ERROR_ADDR, 4,
        [
            (FOREIGN_ID, b"\x00\x00\x00\x00"),
            (dxl_id, struct.pack("<I", sentinel)),
        ],
    )
    pirate.master(packet)
    # No Status reply on broadcast; wait one bus-quiet window for the chip to
    # finish dispatch before reading back.
    pirate.wait_quiet()
    got = read_u32(pirate, dxl_id, PARITY_ERROR_ADDR)
    ok = got == sentinel
    tag = "PASS" if ok else "FAIL"
    print(f"  {tag} sync_write   parity_error = 0x{got:08X} "
          f"(expected 0x{sentinel:08X})", flush=True)
    return ok


def bulk_write_test(pirate: Pirate, dxl_id: int) -> bool:
    sentinel = 0xCAFEBABE
    packet = build_bulk_write([
        (FOREIGN_ID, PARITY_ERROR_ADDR, b"\x00\x00\x00\x00"),
        (dxl_id, FRAMING_ERROR_ADDR, struct.pack("<I", sentinel)),
    ])
    pirate.master(packet)
    pirate.wait_quiet()
    got = read_u32(pirate, dxl_id, FRAMING_ERROR_ADDR)
    ok = got == sentinel
    tag = "PASS" if ok else "FAIL"
    print(f"  {tag} bulk_write   framing_error = 0x{got:08X} "
          f"(expected 0x{sentinel:08X})", flush=True)
    return ok


def run(pirate: Pirate, dxl_id: int) -> int:
    parity_pre = read_u32(pirate, dxl_id, PARITY_ERROR_ADDR)
    framing_pre = read_u32(pirate, dxl_id, FRAMING_ERROR_ADDR)
    print(f"  snapshot     parity_error = {parity_pre}  "
          f"framing_error = {framing_pre}", flush=True)

    failures = 0
    try:
        if not sync_write_test(pirate, dxl_id):
            failures += 1
        if not bulk_write_test(pirate, dxl_id):
            failures += 1
    finally:
        # Restore the snapshotted values so the next bench / matrix run starts
        # from the same baseline. Plain Write is unicast → uses Status reply
        # confirmation, so we know each restore landed.
        write_u32(pirate, dxl_id, PARITY_ERROR_ADDR, parity_pre)
        write_u32(pirate, dxl_id, FRAMING_ERROR_ADDR, framing_pre)
        parity_post = read_u32(pirate, dxl_id, PARITY_ERROR_ADDR)
        framing_post = read_u32(pirate, dxl_id, FRAMING_ERROR_ADDR)
        restored = parity_post == parity_pre and framing_post == framing_pre
        tag = "OK" if restored else "MISMATCH"
        print(f"  restore {tag}  parity_error = {parity_post}  "
              f"framing_error = {framing_post}", flush=True)
        if not restored:
            failures += 1

    if failures:
        print(f"\nFAIL: {failures} check(s) failed", flush=True)
    else:
        print("\nPASS: all checks succeeded", flush=True)
    return failures


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--id", type=int, default=None,
                    help="chip DXL ID. If omitted, broadcast-Ping the bus "
                         "and use the responding chip's ID (UID-derived).")
    ap.add_argument("--baud", type=int, default=1_000_000,
                    help="DXL baud rate (default 1_000_000)")
    ap.add_argument("--tune-baud", type=int, default=1_000_000,
                    help="Baud at which to run HSI cal before the bench "
                         "(default 1M — where fresh chips boot). Chip is "
                         "switched to --baud after cal completes.")
    ap.add_argument("--skip-tune", action="store_true",
                    help="Skip the HSI auto-tune. Use when the chip already "
                         "has a calibrated trim/fine state from a prior run.")
    args = ap.parse_args()

    if args.baud not in BAUD_INDEX:
        sys.exit(f"unsupported --baud {args.baud}; supported: {sorted(BAUD_INDEX)}")
    if args.tune_baud not in BAUD_INDEX:
        sys.exit(f"unsupported --tune-baud {args.tune_baud}; "
                 f"supported: {sorted(BAUD_INDEX)}")

    port = autodetect_pirate()
    pirate = Pirate(port)
    try:
        dxl_id = args.id if args.id is not None else discover_chip_id(pirate, args.baud)
        print(f"pirate: {port}   chip id: {dxl_id}")
        if not args.skip_tune:
            # Mirror pirate_chip_stress's pre-flight: cal at --tune-baud where
            # fresh chips boot, then switch to --baud for the actual bench.
            # Trim/fine state survives the baud switch (RAM-backed CT).
            print(f"\n[tune — HSI cal @ {args.tune_baud}]")
            try:
                set_chip_baud(pirate, dxl_id, args.tune_baud)
                pirate.wait_quiet()
                trim, q88 = step_hsi(pirate, dxl_id, args.tune_baud)
                print(f"  → clock_trim={trim:+d}  clock_fine_trim_us={q88:+d} "
                      f"({q88/256:+.3f}µs)")
            except PirateError as e:
                sys.exit(f"  HSI cal failed: {e}")
        set_chip_baud(pirate, dxl_id, args.baud)
        time.sleep(0.05)
        pirate.drain_stamps()
        print(f"\n[run @ {args.baud}]")
        sys.exit(run(pirate, dxl_id))
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
