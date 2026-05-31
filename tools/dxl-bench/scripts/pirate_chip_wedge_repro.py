"""Scope-friendly reproducer for the V006 TX_EN-stuck-HIGH wedge.

The chip wedges (~30% Vcc bus contention from chip's PC2 staying asserted)
most reliably in the Fast Bulk Read last-slave pass at high baud with a
≥32B reply payload. This tool fires that exact pattern, either as a single
shot for scope capture or continuously to pressure-test for wedging.

Single-shot pattern (same as test_timing_fast_coalesce per shot):
  - pirate master:  FAST_BULK_READ [(INJ_ID, 0, INJ_LEN), (chip_id, 0, DUT_LEN)]
  - pirate ARMs the INJ predecessor reply so chip's snoop walk sees real bytes
  - chip replies as last-slave with chain CRC patched over the INJ snoop

Modes:
  python scripts/pirate_chip_wedge_repro.py
      Single shot at 3M baud, DUT_LEN=32. Sleeps 0.5s first so you can
      arm the scope.

  python scripts/pirate_chip_wedge_repro.py --continuous
      Keep firing back-to-back. Stops on --wedge-threshold consecutive
      NOREPLY/other shots (true bus-stuck signature: chip stops replying
      entirely) or --max-shots (default 10000). CRC failures are counted
      but do NOT halt — those are corruption, not wedge. On stop, halt
      the chip with dump_v006_dxl_state.py while the bus is wedged.

Both modes print TelemetryDxlLink counters before + after.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from dxl_link import clear_counters, print_counters, read_counters
from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    parse_fast_response,
)
from pirate import Pirate, PirateError
from pirate_chip_tune import BAUD_INDEX, autodetect_pirate, set_chip_baud

INJ_ID = 50
# This script measures bus/chain-CRC health, not protocol-level correctness.
# Any well-formed Fast Bulk Read reply (right framing, slot1 id+length match,
# valid CRC) counts as `clean` — including replies whose error byte is non-zero
# (e.g. AccessError 0x07 when the requested range crosses a control-table
# gap; that's the firmware doing its job). Wedge signal = chip silent
# (NOREPLY) or bus flood (other = >256 RX bytes/shot).
WEDGE_BUCKETS = ("noreply", "other")
RESULT_BUCKETS = ("clean", "extra_idle", "crc", "other", "noreply")


def run_shot(
    pirate: Pirate, dut_id: int, inj_len: int, dut_len: int,
) -> tuple[str, bytes, bytes]:
    """Returns (result, request_bytes, reply_bytes).

    reply_bytes is everything pirate observed AFTER the master's own request
    bytes. For 'noreply' cases where total < len(request), reply_bytes is
    empty. For 'other' (total > 256) it's a truncated view of the flood.
    """
    inj_data = b"\xAA" * inj_len
    packet_length = 1 + (2 + inj_len) + (2 + dut_len) + 2
    inj_bytes = build_fast_first_bytes(
        packet_length=packet_length, err=0, slot_id=INJ_ID, data=inj_data,
    )
    request = build_fast_bulk_read([(INJ_ID, 0, inj_len), (dut_id, 0, dut_len)])

    pirate.drain_stamps()
    b0 = pirate.bytes_count()
    try:
        pirate.arm(inj_bytes, after_idle_ticks=250 * 18)
        pirate.master(request)
    except PirateError:
        # ARM/MASTER themselves don't depend on the chip; a failure here is
        # a pirate-side issue, not a wedge. Re-raise so the loop stops.
        raise
    time.sleep(0.02)
    b1 = pirate.bytes_count()
    total = b1 - b0

    capture = min(total, 256) if total > 0 else 0
    all_rx = pirate.rx_range(b0, capture) if capture > 0 else b""
    reply = all_rx[len(request):] if len(all_rx) >= len(request) else b""

    expected_min_reply_bytes = 11 + inj_len + 2 + dut_len  # Status header + slots
    if total < len(request) + expected_min_reply_bytes // 2:
        pirate.drain_stamps()
        return "noreply", request, reply
    if total > 256:
        pirate.drain_stamps()
        return "other", request, reply
    if len(reply) < 11:
        pirate.drain_stamps()
        return "noreply", request, reply
    try:
        slots = parse_fast_response(reply, slot_lengths=[inj_len, dut_len])
    except ValueError:
        pirate.drain_stamps()
        return "crc", request, reply
    if len(slots) != 2 or slots[0].id != INJ_ID or slots[0].data != inj_data:
        pirate.drain_stamps()
        return "crc", request, reply
    if slots[1].id != dut_id or len(slots[1].data) != dut_len:
        pirate.drain_stamps()
        return "other", request, reply
    # Frame is well-formed. error byte (zero or not) is a protocol-layer
    # detail; the bus + chain-CRC path served a complete reply, so it's clean.
    final = "clean" if len(pirate.drain_stamps()) == 1 else "extra_idle"
    return final, request, reply


def diff_counters(before: dict, after: dict) -> dict:
    return {k: after[k] - before[k] for k in before if after[k] != before[k]}


def try_read_counters(pirate: Pirate, dxl_id: int) -> dict | None:
    """Return counters or None if the chip refuses to reply (wedge symptom)."""
    try:
        return read_counters(pirate, dxl_id)
    except (AssertionError, PirateError, TimeoutError):
        return None


def print_wedge_banner(shot_label: str) -> None:
    print(f"\n*** WEDGE DETECTED {shot_label} ***")
    print("   Chip is not responding to plain Read (counter read failed).")
    print("   Bus likely at ~30% Vcc. Halt + dump regs now:")
    print("     python scripts/dump_v006_dxl_state.py --keep-halted")


def print_capture(label: str, request: bytes, reply: bytes) -> None:
    print(f"\n  [{label}] request ({len(request)}B):  {request.hex(' ')}")
    print(f"  [{label}] reply   ({len(reply)}B):  {reply.hex(' ')}")


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--baud", type=int, default=3_000_000)
    ap.add_argument("--inj-len", type=int, default=4,
                    help="Predecessor INJ slot data length (default 4).")
    ap.add_argument("--dut-len", type=int, default=32,
                    help="Chip reply data length (default 32 — wedge-prone).")
    ap.add_argument("--continuous", action="store_true",
                    help="Fire repeatedly until wedge or --max-shots.")
    ap.add_argument("--max-shots", type=int, default=10_000)
    ap.add_argument("--inter-shot-ms", type=float, default=0.0,
                    help="Sleep between shots in continuous mode (0 = back-to-back).")
    ap.add_argument("--scope-delay-s", type=float, default=0.5,
                    help="Pre-shot delay in single-shot mode so you can arm the scope.")
    ap.add_argument("--wedge-threshold", type=int, default=5,
                    help="Consecutive NOREPLY/other shots before declaring wedge (default 5).")
    args = ap.parse_args()

    if args.baud not in BAUD_INDEX:
        sys.exit(f"unsupported --baud {args.baud}; supported: {sorted(BAUD_INDEX)}")

    port = args.port or autodetect_pirate()
    pirate = Pirate(port)
    try:
        set_chip_baud(pirate, args.id, args.baud)
        pirate.wait_quiet()
        print(f"pirate: {port}   chip id: {args.id}")
        print(f"baud: {args.baud}   INJ_LEN: {args.inj_len}   DUT_LEN: {args.dut_len}")

        before = read_counters(pirate, args.id)
        clear_counters(pirate, args.id)

        if not args.continuous:
            print(f"\n[single shot — arming scope in {args.scope_delay_s:.1f}s]",
                  flush=True)
            time.sleep(args.scope_delay_s)
            result, req, reply = run_shot(
                pirate, args.id, args.inj_len, args.dut_len)
            print(f"result: {result}")
            print_capture(result, req, reply)
            after = try_read_counters(pirate, args.id)
            if after is None:
                print_wedge_banner("on single shot")
                return
            delta = diff_counters({k: 0 for k in after}, after)
            if delta:
                print("counters bumped this shot:")
                for k, v in delta.items():
                    print(f"  {k:25s} += {v}")
            else:
                print("counters bumped this shot: none")
            return

        print(f"\n[continuous — up to {args.max_shots} shots; "
              f"stop on {args.wedge_threshold} consecutive NOREPLY/other "
              f"(CRC alone does not stop)]",
              flush=True)
        tally = {k: 0 for k in RESULT_BUCKETS}
        consecutive_wedge = 0
        wedge_shot = None
        first_capture: dict[str, tuple[int, bytes, bytes]] = {}
        for i in range(1, args.max_shots + 1):
            try:
                r, req, reply = run_shot(
                    pirate, args.id, args.inj_len, args.dut_len)
            except PirateError as exc:
                print(f"\nshot {i}: pirate side failed: {exc}")
                break
            tally[r] += 1
            if r not in first_capture:
                first_capture[r] = (i, req, reply)
            consecutive_wedge = consecutive_wedge + 1 if r in WEDGE_BUCKETS else 0
            if consecutive_wedge >= args.wedge_threshold or i % 100 == 0:
                summary = "  ".join(f"{k}={tally[k]}" for k in RESULT_BUCKETS)
                print(f"  shot {i:>5d}: {r:<10s}  {summary}  "
                      f"(consec_wedge={consecutive_wedge})", flush=True)
            if consecutive_wedge >= args.wedge_threshold:
                wedge_shot = i
                break
            if args.inter_shot_ms > 0:
                time.sleep(args.inter_shot_ms / 1000.0)

        for bucket in ("clean", "extra_idle", "crc", "other", "noreply"):
            if bucket in first_capture:
                shot_i, req, reply = first_capture[bucket]
                print_capture(f"first {bucket} (shot {shot_i})", req, reply)

        after = try_read_counters(pirate, args.id)

        if wedge_shot is not None:
            print_wedge_banner(f"at shot {wedge_shot} "
                               f"({args.wedge_threshold} consecutive NOREPLY/other)")
        elif after is None:
            print_wedge_banner("after run completed")
        else:
            print(f"\n[completed {args.max_shots} shots without wedge]")
        summary = "  ".join(f"{k}={tally[k]}" for k in RESULT_BUCKETS)
        print(f"final: {summary}")
        if after is None:
            print("counter deltas: unavailable — chip not responding.")
        else:
            delta = diff_counters({k: 0 for k in after}, after)
            print("counter deltas over this run:")
            if delta:
                for k, v in delta.items():
                    print(f"  {k:25s} += {v}")
            else:
                print("  none")
        _ = before  # baseline kept for future before/after comparison; not printed now.
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
