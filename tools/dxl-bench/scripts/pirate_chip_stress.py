"""Scope-friendly reproducer for the V006 TX_EN-stuck-HIGH wedge.

The chip wedges (~30% Vcc bus contention from chip's PC2 staying asserted)
when its dxl_fast TC ISR fails to re-arm and drop TX_EN. The most reliable
trigger observed is the verify matrix in `pirate_chip_tune.py` walking
from chip-as-First at 2M (precursor cell: 1B payload silently drops some
shots) into the cells that follow.

Modes:
  --matrix
      Walks (baud × dut_len × position) cells in the same order as
      `pirate_chip_tune.step_verify`, with a counter-Read health probe
      after each cell + per-cell fault-counter delta printout. Stops on
      first health-probe failure. Best one-command repro of the
      verify-matrix wedge.
      Assumes chip is already in wedge-prone state — typically because
      `pirate_chip_tune.py` was run first (tuning TX_FAST at 1M leaves
      the chip mistimed for chain coalesce at 2M+).
      Narrowing knobs:
        --matrix-repeat N       loop the matrix N times (non-det wedge).
        --matrix-health-every N probe counters every N shots inside a
                                cell so the wedge shot is bracketed to N
                                instead of the full 128.

  --continuous
      Fires one position repeatedly. Stops on --wedge-threshold consecutive
      NOREPLY/other shots or --max-shots (default 10000). CRC failures
      counted but don't halt — those are corruption, not wedge.

  (default)
      Single shot at --baud / --dut-len / --position. Sleeps --scope-delay-s
      first so you can arm a scope.

Position semantics (used by all modes):
  only   plain Read; chip is the sole responder.
  first  Fast Bulk Read [(chip), (FOREIGN_ID)] — chip fires slot 1, bus
         IDLEs after (FOREIGN_ID never replies). Validates chip's First-slot
         emission without a chain CRC.
  last   Fast Bulk Read [(INJ_ID), (chip)] — pirate ARMs INJ predecessor
         reply so chip's snoop walk + chain CRC patch fire as in production.

On wedge, halt + dump regs while the bus is still wedged:
  python scripts/dump_v006_dxl_state.py --keep-halted
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
    build_read,
    parse_fast_response,
    parse_status,
)
from pirate import Pirate, PirateError
from pirate_chip_tune import (
    BAUD_INDEX,
    FOREIGN_ID,
    autodetect_pirate,
    set_chip_baud,
)

INJ_ID = 50
POSITIONS = ("only", "first", "last")
# This script measures bus/chain-CRC health, not protocol-level correctness.
# Any well-formed reply (right framing, expected id+length) counts as `clean`
# — including Status replies whose error byte is non-zero (e.g. AccessError
# 0x07 when the requested range crosses a control-table gap; that's the
# firmware doing its job). Wedge signal = chip silent (NOREPLY) or bus
# flood (other = >256 RX bytes/shot).
WEDGE_BUCKETS = ("noreply", "other")
RESULT_BUCKETS = ("clean", "extra_idle", "crc", "other", "noreply")


def run_shot(
    pirate: Pirate, dut_id: int, position: str, inj_len: int, dut_len: int,
) -> tuple[str, bytes, bytes]:
    """Returns (result, request_bytes, reply_bytes). Dispatches on position."""
    if position == "only":
        return _shot_only(pirate, dut_id, dut_len)
    if position == "first":
        return _shot_first(pirate, dut_id, dut_len)
    if position == "last":
        return _shot_last(pirate, dut_id, inj_len, dut_len)
    raise ValueError(f"unknown position {position!r}")


def _shot_only(
    pirate: Pirate, dut_id: int, dut_len: int,
) -> tuple[str, bytes, bytes]:
    request = build_read(dut_id, 0, dut_len)
    pirate.drain_stamps()
    reply = pirate.xfer(request, reply_us=10_000)
    if reply is None:
        return "noreply", request, b""
    if len(reply) < 11:
        return "noreply", request, reply
    try:
        st = parse_status(reply)
    except ValueError:
        return "crc", request, reply
    if st.id != dut_id:
        return "other", request, reply
    return "clean", request, reply


def _shot_first(
    pirate: Pirate, dut_id: int, dut_len: int,
) -> tuple[str, bytes, bytes]:
    # Fast Bulk Read with chip as slot 1, FOREIGN_ID as silent slot 2.
    # Chip emits its First-slot bytes (HEADER + chain-length + STATUS + err
    # + slot1_id + slot1_data) then waits for slot 2 that never fires; the
    # chain CRC patch deadline eventually misses. We don't validate CRC
    # here — for a wedge probe, presence of the expected First-slot byte
    # count is sufficient.
    request = build_fast_bulk_read(
        [(dut_id, 0, dut_len), (FOREIGN_ID, 0, 1)]
    )
    pirate.drain_stamps()
    b0 = pirate.bytes_count()
    pirate.master(request)
    time.sleep(0.02)
    b1 = pirate.bytes_count()
    total = b1 - b0

    capture = min(total, 256) if total > 0 else 0
    all_rx = pirate.rx_range(b0, capture) if capture > 0 else b""
    reply = all_rx[len(request):] if len(all_rx) >= len(request) else b""

    first_slot_bytes = 10 + dut_len  # HEADER+BCAST+len2+STATUS+err+slot_id+data
    if total > 256:
        pirate.drain_stamps()
        return "other", request, reply
    if total < len(request) + first_slot_bytes // 2:
        pirate.drain_stamps()
        return "noreply", request, reply
    if len(reply) < first_slot_bytes:
        pirate.drain_stamps()
        return "noreply", request, reply
    # reply[8] is the error byte (any value OK — protocol-level detail);
    # reply[9] is the First slot's id, which must be the chip's id.
    if reply[:4] != bytes([0xFF, 0xFF, 0xFD, 0x00]) or reply[4] != 0xFE \
            or reply[7] != 0x55 or reply[9] != dut_id:
        pirate.drain_stamps()
        return "crc", request, reply
    pirate.drain_stamps()
    return "clean", request, reply


def _shot_last(
    pirate: Pirate, dut_id: int, inj_len: int, dut_len: int,
) -> tuple[str, bytes, bytes]:
    inj_data = b"\xAA" * inj_len
    packet_length = 1 + (2 + inj_len) + (2 + dut_len) + 2
    inj_bytes = build_fast_first_bytes(
        packet_length=packet_length, err=0, slot_id=INJ_ID, data=inj_data,
    )
    request = build_fast_bulk_read([(INJ_ID, 0, inj_len), (dut_id, 0, dut_len)])

    pirate.drain_stamps()
    b0 = pirate.bytes_count()
    pirate.arm(inj_bytes, after_idle_ticks=250 * 18)
    pirate.master(request)
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


def _counter_delta(prev: dict, curr: dict) -> dict:
    return {k: curr[k] - prev.get(k, 0) for k in curr if curr[k] != prev.get(k, 0)}


def _fmt_counter_delta(delta: dict) -> str:
    if not delta:
        return ""
    return " ".join(f"{k}+={v}" for k, v in delta.items())


def run_matrix(
    pirate: Pirate, dxl_id: int,
    bauds: list[int], dut_lens: list[int], positions: list[str],
    n_per_cell: int, inj_len: int,
    health_every: int = 0,
) -> bool:
    """Walk the verify-style sweep. Returns True if a wedge was caught
    (caller should stop any outer repeat loop). Per-cell counter deltas
    are printed alongside the tally — a nonzero `crc_patch_deadline_miss`
    or `previous_slot_timeout` in the cell that precedes the wedge is
    the strongest narrowing clue. With `health_every > 0`, counters are
    probed every N shots inside a cell so the wedge shot can be bracketed
    to a window of N rather than the full 128-shot cell."""
    print(f"\n[matrix — bauds={bauds} dut_lens={dut_lens} "
          f"positions={positions}  n={n_per_cell}/cell"
          f"  health_every={health_every or 'cell-end'}]",
          flush=True)
    print(f"  {'cell':<22} {'   '.join(f'{b:>10}' for b in RESULT_BUCKETS)}  health")
    baseline = try_read_counters(pirate, dxl_id) or {f: 0 for f in (
        "illegal_transition", "unexpected_byte_count", "previous_slot_timeout",
        "slot_timing_miss", "crc_patch_deadline_miss", "dma_overrun",
        "parity_error", "framing_error", "noise_error",
    )}
    last_counters = baseline
    for baud in bauds:
        try:
            set_chip_baud(pirate, dxl_id, baud)
            pirate.wait_quiet()
        except PirateError as e:
            print(f"  {baud//1_000_000}M baud:   set_chip_baud failed: {e}")
            if try_read_counters(pirate, dxl_id) is None:
                print_wedge_banner(f"on set_chip_baud({baud})")
                return True
            continue
        for dut_len in dut_lens:
            for pos in positions:
                cell = f"{baud//1_000_000}M {pos:<5} {dut_len:>3}B"
                tally = {k: 0 for k in RESULT_BUCKETS}
                first_fail: tuple[int, str, bytes, bytes] | None = None
                aborted = None
                wedge_shot: int | None = None
                for shot in range(1, n_per_cell + 1):
                    try:
                        r, req, reply = run_shot(
                            pirate, dxl_id, pos, inj_len, dut_len,
                        )
                    except PirateError as exc:
                        aborted = f"pirate side fail shot {shot}: {exc}"
                        break
                    tally[r] += 1
                    if r not in ("clean", "extra_idle") and first_fail is None:
                        first_fail = (shot, r, req, reply)
                    if health_every > 0 and shot % health_every == 0:
                        probe = try_read_counters(pirate, dxl_id)
                        if probe is None:
                            wedge_shot = shot
                            break
                        last_counters = probe
                summary = "  ".join(f"{tally[k]:>10d}" for k in RESULT_BUCKETS)
                if aborted:
                    print(f"  {cell:<22} {summary}  ABORT ({aborted})")
                    return True
                if wedge_shot is not None:
                    print(f"  {cell:<22} {summary}  WEDGE@shot{wedge_shot}",
                          flush=True)
                    if first_fail is not None:
                        shot_i, r, req, reply = first_fail
                        print_capture(f"first non-clean in {cell} (shot {shot_i})",
                                      req, reply)
                    print_wedge_banner(
                        f"in cell {cell} between shot "
                        f"{max(1, wedge_shot - health_every + 1)} and {wedge_shot}"
                    )
                    return True
                health = try_read_counters(pirate, dxl_id)
                if health is None:
                    tag = "WEDGE@cell-end"
                else:
                    delta = _counter_delta(last_counters, health)
                    last_counters = health
                    tag = "ok" if not delta else f"ok ({_fmt_counter_delta(delta)})"
                print(f"  {cell:<22} {summary}  {tag}", flush=True)
                if first_fail is not None:
                    shot_i, r, req, reply = first_fail
                    print_capture(f"first non-clean in {cell} (shot {shot_i})",
                                  req, reply)
                if health is None:
                    print_wedge_banner(f"after cell {cell}")
                    return True
    return False


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--baud", type=int, default=2_000_000,
                    help="Baud for single-shot/continuous (default 2M).")
    ap.add_argument("--position", choices=POSITIONS, default="first",
                    help="Chip slot role for single-shot/continuous "
                         "(default 'first' — the precursor wedge cell).")
    ap.add_argument("--inj-len", type=int, default=4,
                    help="Predecessor INJ slot data length, position=last (default 4).")
    ap.add_argument("--dut-len", type=int, default=1,
                    help="Chip reply data length (default 1 — 2M-First-1B wedge cell).")
    ap.add_argument("--continuous", action="store_true",
                    help="Fire repeatedly until wedge or --max-shots.")
    ap.add_argument("--max-shots", type=int, default=10_000)
    ap.add_argument("--inter-shot-ms", type=float, default=0.0,
                    help="Sleep between shots in continuous mode (0 = back-to-back).")
    ap.add_argument("--scope-delay-s", type=float, default=0.5,
                    help="Pre-shot delay in single-shot mode so you can arm the scope.")
    ap.add_argument("--wedge-threshold", type=int, default=5,
                    help="Consecutive NOREPLY/other shots before declaring wedge (default 5).")
    ap.add_argument("--matrix", action="store_true",
                    help="Walk (baud × dut_len × position) cells (verify-style). "
                         "Stops on first health-probe failure. Assumes chip is "
                         "already in wedge-prone state (run pirate_chip_tune.py "
                         "first to tune TX_FAST at 1M).")
    ap.add_argument("--matrix-bauds", type=str, default="1000000,2000000,3000000",
                    help="Comma-separated baud list for --matrix.")
    ap.add_argument("--matrix-dut-lens", type=str, default="1,4,32",
                    help="Comma-separated payload list for --matrix.")
    ap.add_argument("--matrix-positions", type=str, default="only,first,last",
                    help="Comma-separated position list for --matrix.")
    ap.add_argument("--matrix-n", type=int, default=128,
                    help="Shots per --matrix cell (default 128, matches verify).")
    ap.add_argument("--matrix-repeat", type=int, default=1,
                    help="Run --matrix this many times in a loop, halting "
                         "on first wedge. Default 1. Use >1 to surface "
                         "non-deterministic wedges in a single command.")
    ap.add_argument("--matrix-health-every", type=int, default=0,
                    help="In --matrix mode, probe chip counters every N shots "
                         "within a cell. 0 (default) = end-of-cell only. "
                         "Small values (e.g. 16) bracket the wedge shot tighter "
                         "but slow the run.")
    args = ap.parse_args()

    if args.baud not in BAUD_INDEX:
        sys.exit(f"unsupported --baud {args.baud}; supported: {sorted(BAUD_INDEX)}")

    port = args.port or autodetect_pirate()
    pirate = Pirate(port)
    try:
        if args.matrix:
            bauds = [int(x) for x in args.matrix_bauds.split(",")]
            for b in bauds:
                if b not in BAUD_INDEX:
                    sys.exit(f"unsupported matrix baud {b}; supported: {sorted(BAUD_INDEX)}")
            dut_lens = [int(x) for x in args.matrix_dut_lens.split(",")]
            positions = [p.strip() for p in args.matrix_positions.split(",")]
            for p in positions:
                if p not in POSITIONS:
                    sys.exit(f"unsupported position {p!r}; supported: {POSITIONS}")
            print(f"pirate: {port}   chip id: {args.id}")
            for iteration in range(1, args.matrix_repeat + 1):
                if args.matrix_repeat > 1:
                    print(f"\n=== matrix iteration {iteration}/{args.matrix_repeat} ===",
                          flush=True)
                wedged = run_matrix(
                    pirate, args.id, bauds, dut_lens, positions,
                    args.matrix_n, args.inj_len,
                    health_every=args.matrix_health_every,
                )
                if wedged:
                    sys.exit(1)
            return

        set_chip_baud(pirate, args.id, args.baud)
        pirate.wait_quiet()
        print(f"pirate: {port}   chip id: {args.id}")
        print(f"baud: {args.baud}   position: {args.position}   "
              f"INJ_LEN: {args.inj_len}   DUT_LEN: {args.dut_len}")

        before = read_counters(pirate, args.id)
        clear_counters(pirate, args.id)

        if not args.continuous:
            print(f"\n[single shot — arming scope in {args.scope_delay_s:.1f}s]",
                  flush=True)
            time.sleep(args.scope_delay_s)
            result, req, reply = run_shot(
                pirate, args.id, args.position, args.inj_len, args.dut_len)
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
                    pirate, args.id, args.position, args.inj_len, args.dut_len)
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
