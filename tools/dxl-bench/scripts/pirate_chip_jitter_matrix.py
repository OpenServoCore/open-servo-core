"""Chip-side DUT reply jitter — Plain and Fast paths × baud × payload.

What this measures and why:
  Per-cell σ of (T_first - T_req), the pirate-stamped chip reply fire-time
  relative to the master request_end. With both TX_PLAIN_LATENCY and
  TX_FAST_LATENCY pinned to 0, the chip fires as early as its dispatcher
  permits, so σ is the chip's intrinsic fire jitter on each code path —
  not a tuning margin's residual.

  Plain vs Fast σ at matching (baud, size) tells us whether the Fast
  dispatcher (snoop ring + chain phase + SysTick CMP arm) adds jitter
  over Plain. Cross-size constancy tells us whether reply size leaks
  into fire time (would indicate snoop/CRC compute leaking back).

  T_last - T_first σ is reported separately as the pirate's stamping
  noise floor (reply wire-time is constant per cell, so any variance is
  pirate-side, not chip-side).

Pipeline:
  1. set_chip_baud(tune_baud)
  2. step_hsi — converge clock_trim + clock_fine_trim_us
  3. Zero TX_PLAIN_LATENCY_US and TX_FAST_LATENCY_US
  4. Print all 4 tuning values
  5. Sweep (baud × size) for both paths and print σ matrices

Run:  python scripts/pirate_chip_jitter_matrix.py  [--n 256]
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cal import Calibrator
from dxl_packet import (
    build_fast_bulk_read,
    build_read,
    parse_fast_response,
    parse_status,
)
from pirate import Pirate, Round
from pirate_chip_tune import (
    DXL_TX_FAST_LATENCY_US_ADDR,
    DXL_TX_PLAIN_LATENCY_US_ADDR,
    autodetect_pirate,
    set_chip_baud,
    step_hsi,
    write_ct_u16,
)

BAUDS = [1_000_000, 2_000_000, 3_000_000]
SIZES = [1, 4, 16, 64, 128]


def measure(pirate: Pirate, dxl_id: int, baud: int, size: int, n: int,
            path: str) -> tuple[float, float, int, int]:
    """Returns (σ(first-req), σ(last-first), n_valid, n_attempted) in pirate ticks."""
    if path == "plain":
        packet = build_read(dxl_id, 0, size)
        def parse_ok(reply: bytes) -> bool:
            try:
                parse_status(reply)
                return True
            except ValueError:
                return False
    elif path == "fast":
        packet = build_fast_bulk_read([(dxl_id, 0, size)])
        def parse_ok(reply: bytes) -> bool:
            try:
                parse_fast_response(reply, [size])
                return True
            except ValueError:
                return False
    else:
        raise ValueError(f"unknown path: {path}")

    byte_us = 10.0e6 / baud
    reply_us = max(2_000, int(1_500 + size * byte_us + 500))

    fmr: list[int] = []
    lmf: list[int] = []
    for _ in range(n):
        pirate.drain_stamps()
        reply = pirate.xfer(packet, reply_us=reply_us)
        if reply is None or not parse_ok(reply):
            continue
        rounds = [s for s in pirate.drain_stamps() if isinstance(s, Round)]
        if len(rounds) != 1:
            continue
        r = rounds[0]
        fmr.append((r.first - r.req) & 0xFFFFFFFF)
        lmf.append((r.last - r.first) & 0xFFFFFFFF)

    sigma_fmr = statistics.stdev(fmr) if len(fmr) > 1 else 0.0
    sigma_lmf = statistics.stdev(lmf) if len(lmf) > 1 else 0.0
    return sigma_fmr, sigma_lmf, len(fmr), n


def print_matrix(title: str,
                 sigmas: dict[tuple[int, int], tuple[float, float, int, int]],
                 ticks_per_us: int) -> None:
    print(f"\n{title}")
    cell_w = 16
    hdr = "  baud  |" + "".join(f"size={s:<4}".rjust(cell_w) for s in SIZES)
    print(hdr)
    print("  " + "─" * (len(hdr) - 2))
    for baud in BAUDS:
        label = f"{baud // 1_000_000}M"
        row = f"  {label:<6}|"
        for size in SIZES:
            s, _, n_v, _ = sigmas[(baud, size)]
            if n_v == 0:
                row += "NO DATA".rjust(cell_w)
            else:
                s_us = s / ticks_per_us
                row += f"{s:4.2f}t / {s_us:5.3f}µs".rjust(cell_w)
        drops = sum(sigmas[(baud, s)][3] - sigmas[(baud, s)][2] for s in SIZES)
        if drops:
            row += f"  drops={drops}"
        print(row)


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--tune-baud", type=int, default=1_000_000)
    ap.add_argument("--n", type=int, default=256)
    ap.add_argument("--skip-hsi", action="store_true",
                    help="Don't re-tune HSI; just read current trim/fine.")
    args = ap.parse_args()

    port = args.port or autodetect_pirate()
    print(f"pirate: {port}   chip id: {args.id}   tune baud: {args.tune_baud}")

    pirate = Pirate(port)
    try:
        set_chip_baud(pirate, args.id, args.tune_baud)

        if args.skip_hsi:
            cal = Calibrator(pirate, dxl_id=args.id, baud=args.tune_baud)
            trim = cal.read_clock_trim()
            fine = cal.read_clock_fine_trim_us()
        else:
            print(f"\n[1+2] HSI coarse + fine @ {args.tune_baud}")
            trim, fine = step_hsi(pirate, args.id, args.tune_baud)

        write_ct_u16(pirate, args.id, DXL_TX_PLAIN_LATENCY_US_ADDR, 0)
        write_ct_u16(pirate, args.id, DXL_TX_FAST_LATENCY_US_ADDR, 0)
        time.sleep(0.01)

        print(f"\nTuning state (latencies pinned to 0 for jitter floor):")
        print(f"   clock_trim              = {trim:+d}")
        print(f"   clock_fine_trim_us      = {fine:+d}q88 ({fine/256:+.3f}µs)")
        print(f"   dxl_tx_plain_latency_us = 0q88 (0.000µs)")
        print(f"   dxl_tx_fast_latency_us  = 0q88 (0.000µs)")

        ticks_per_us = pirate.hz_per_us()
        plain_sigmas: dict[tuple[int, int], tuple[float, float, int, int]] = {}
        fast_sigmas: dict[tuple[int, int], tuple[float, float, int, int]] = {}

        for baud in BAUDS:
            print(f"\n  → baud {baud // 1_000_000}M  (n={args.n}/cell × {len(SIZES)} sizes × 2 paths)")
            set_chip_baud(pirate, args.id, baud)
            for size in SIZES:
                plain_sigmas[(baud, size)] = measure(
                    pirate, args.id, baud, size, args.n, "plain")
                fast_sigmas[(baud, size)] = measure(
                    pirate, args.id, baud, size, args.n, "fast")

        print()
        print(f"1 pirate tick = {1e6 / ticks_per_us / 1e6 * 1e9:.1f} ns ({ticks_per_us} MHz SysTick)")

        print_matrix("Plain (TX_PLAIN_LATENCY=0) — σ of (T_first - T_req):",
                     plain_sigmas, ticks_per_us)
        print_matrix("Fast (TX_FAST_LATENCY=0, single-slot) — σ of (T_first - T_req):",
                     fast_sigmas, ticks_per_us)

        all_lmf: list[float] = []
        for sigmas in (plain_sigmas, fast_sigmas):
            for _, lmf_s, n_v, _ in sigmas.values():
                if n_v > 1:
                    all_lmf.append(lmf_s)
        if all_lmf:
            print(f"\nPirate floor (σ of T_last - T_first across all cells):")
            print(f"  mean={statistics.mean(all_lmf):.2f}t  "
                  f"max={max(all_lmf):.2f}t  "
                  f"(pure pirate stamping noise; should be ≪ chip σ)")
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
