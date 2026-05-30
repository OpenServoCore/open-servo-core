"""Round-trip stamp jitter for the actual #52 metric.

#52 uses the pirate to measure `T_first - T_request_end` for repeated chip
replies under fixed conditions, sweeping the firmware's per-path TX_LATENCY
const to find the zero-crossing. For that sweep to be meaningful, the
measurement noise floor (σ of the round-trip stamp pair) must be well under
1 SysTick (~56 ns), so what we see is chip drift, not pirate noise.

This script PINGs the chip N times and reports σ on:
  - Round.first - Round.req  (the absolute reply latency — picks up RDT + chip
    TX_LATENCY drift + pirate stamp noise)
  - Round.last - Round.first (reply length stability — should be exactly the
    nominal Status frame wire-time; σ here = pirate's stamp-pair noise alone,
    no chip-side contribution since the chip's reply byte count is constant)

The second metric is the cleanest pirate-stamping σ we can get without
chip variance contaminating it.

Run:  python scripts/pirate_chip_reply_jitter.py  [--n 200]
Requires the bench's normal --baud / --id chip to be plugged in and calibrated.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import serial.tools.list_ports

from dxl_packet import build_ping
from pirate import Pirate, Round

PIRATE_VID = 0xC0DE
PIRATE_PID = 0xCAFE


def autodetect_pirate() -> str:
    matches = [
        p.device for p in serial.tools.list_ports.comports()
        if p.vid == PIRATE_VID and p.pid == PIRATE_PID
    ]
    if len(matches) != 1:
        sys.exit(f"expected exactly 1 pirate, found {matches}")
    return matches[0]


def summarize(label: str, vals: list[int], baud: int) -> None:
    if not vals:
        print(f"  {label}:  NO DATA")
        return
    n = len(vals)
    med = statistics.median(vals)
    mn, mx = min(vals), max(vals)
    sigma = statistics.stdev(vals) if n > 1 else 0.0
    one_bit_ticks = 18_000_000 / baud
    sigma_us = sigma * 1e6 / 18_000_000
    print(
        f"  {label:34s}  n={n:<4} "
        f"med={med:>7}t  span=[{mn:>7},{mx:>7}]  "
        f"σ={sigma:7.2f}t = {sigma_us:5.3f} µs  "
        f"(σ/bit = {sigma / one_bit_ticks:5.2f})"
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--n", type=int, default=200)
    args = ap.parse_args()

    port = args.port or autodetect_pirate()
    print(f"pirate: {port}  baud: {args.baud}  id: {args.id}\n")

    pirate = Pirate(port)
    try:
        pirate.set_baud(args.baud)
        time.sleep(0.05)
        pirate.drain_stamps()

        ping = build_ping(args.id)
        firsts: list[int] = []
        lengths: list[int] = []
        firsts_minus_req: list[int] = []

        for _ in range(args.n):
            pirate.drain_stamps()
            reply = pirate.xfer(ping, reply_us=10_000)
            if reply is None:
                continue
            stamps = pirate.drain_stamps()
            rounds = [s for s in stamps if isinstance(s, Round)]
            if len(rounds) != 1:
                continue
            r = rounds[0]
            # Wrap-safe u32 deltas (idle is always after fire/first).
            fmr = (r.first - r.req) & 0xFFFFFFFF
            lmf = (r.last - r.first) & 0xFFFFFFFF
            firsts.append(r.first & 0xFFFFFFFF)
            firsts_minus_req.append(fmr)
            lengths.append(lmf)

        print(f"PING round-trip jitter (n attempted={args.n}):")
        summarize("T_first - T_req (reply latency)", firsts_minus_req, args.baud)
        summarize("T_last - T_first (reply len)  ", lengths, args.baud)
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
