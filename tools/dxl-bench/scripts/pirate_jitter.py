"""Pirate-only fire→stamp jitter, swept across baud × payload size.

Pure-pirate self-loop: pirate's USART3 listener hears its own USART1 HDSEL
TX, so no DUT is needed. For each (baud, size) cell we schedule N FIREs at a
known future SysTick tick and read back the Plain IDLE stamp. With the
listener's one-char-time backdate, the residual

    idle_stamp.tick - fire_at_tick

is deterministic-but-for-noise. σ across N trials = pirate's full
fire→stamp noise floor.

Memory note `pirate-fire-jitter-floor` claims σ ≈ 1 TX bit-time (uniform
distribution from USART start-bit BRR sync) at every baud, payload-independent.
This script reproduces that across a (baud × size) matrix and reports the
per-cell σ + the matrix average.

We reject trials where macOS USB stalled long enough to push the FIRE command
past its 3 ms TIM4-OPM schedule (immediate-fire fallback) — those residuals
are host latency, not wire jitter. See the `missed` reject counter.

Run: `python scripts/pirate_jitter.py [--n 200] [--port /dev/...]`
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import serial.tools.list_ports

from pirate import Pirate, Plain

PIRATE_VID = 0xC0DE
PIRATE_PID = 0xCAFE

BAUDS = [115_200, 1_000_000, 2_000_000, 3_000_000]
# 128 is the largest meaningful payload (full Fast slot). Pushing to 256
# matches the pirate's RX DMA ring size — NDTR auto-reloads on full wrap and
# `bytes_count` reads 0 delta, which fails the per-trial byte-count check.
SIZES = [1, 4, 16, 64, 128]


def autodetect_pirate() -> str:
    matches = [
        p.device for p in serial.tools.list_ports.comports()
        if p.vid == PIRATE_VID and p.pid == PIRATE_PID
    ]
    if len(matches) != 1:
        sys.exit(f"expected exactly 1 pirate, found {matches}")
    return matches[0]


def measure(pirate: Pirate, baud: int, size: int, n: int) -> tuple[list[int], dict[str, int]]:
    """Returns (residuals, reject_counts). Residuals are in SysTicks (18 MHz).

    Reject categories:
      - `missed`: USB stall pushed the FIRE command past `fire_at`, so
        `schedule_or_fire_now` took the immediate-fire branch (residual is
        host latency, not wire jitter). Detected via LAST? after FIRE — the
        firmware stamps `fire_at` on the scheduled path and `now` on the
        immediate path, so a mismatch is unambiguous.
      - `bus`: stamp/byte-count check failed — extra/missing bytes from a DUT
        echo or framing chatter. With DUT unplugged this should be 0.
    """
    pirate.set_baud(baud)
    time.sleep(0.05)
    pirate.drain_stamps()

    # TIM4 OPM ARR is u16 (65 535 SysTicks ≈ 3.64 ms max). 54 000 ≈ 3 ms
    # leaves headroom against USB/CDC + Python timing variance; longer than
    # this and `schedule_or_fire_now` saturates delta=0 → immediate-fire.
    # The LAST? cross-check below catches when that happens regardless.
    schedule_ahead = 54_000

    # Per-trial wait: schedule headroom + TX duration + 1 char_time for IDLE
    # to assert + slop. Floor at 10 ms for USB drain.
    byte_us = 10.0e6 / baud
    wait_s = max(0.010, (2_000 + size * byte_us + 500) / 1e6)

    payload = bytes([0xAA] * size)
    residuals: list[int] = []
    rejects = {"missed": 0, "bus": 0}

    for _ in range(n):
        pirate.drain_stamps()
        bytes_before = pirate.bytes_count()
        now = pirate.tick()
        fire_at = (now + schedule_ahead) & 0xFFFFFFFFFFFFFFFF
        pirate.fire(payload, at_tick=fire_at)
        # LAST? returns the tick stored inside schedule_or_fire_now. Scheduled
        # path stores `fire_at`; immediate-fire stores `now-on-device`. Any
        # mismatch = USB stall ate our schedule headroom.
        fired_at = pirate.last_fired()
        if fired_at != (fire_at & 0xFFFFFFFF):
            rejects["missed"] += 1
            time.sleep(wait_s)
            pirate.drain_stamps()
            continue
        time.sleep(wait_s)
        stamps = [s for s in pirate.drain_stamps() if isinstance(s, Plain)]
        bytes_after = pirate.bytes_count()
        if len(stamps) != 1 or (bytes_after - bytes_before) & 0xFFFFFFFF != size:
            rejects["bus"] += 1
            continue
        residual = (stamps[0].tick - (fire_at & 0xFFFFFFFF)) & 0xFFFFFFFF
        if residual > 1 << 31:
            residual -= 1 << 32
        residuals.append(residual)

    return residuals, rejects


def sigma_ticks(vals: list[int]) -> float:
    return statistics.stdev(vals) if len(vals) > 1 else 0.0


def peak_dev(vals: list[int]) -> int:
    """Max |v - median|. Diagnostic only — if σ and peak disagree drastically,
    something is leaking outliers past the reject filters."""
    if not vals:
        return 0
    med = statistics.median(vals)
    return max(abs(v - med) for v in vals)


def fmt_cell(s: float, mx: int) -> str:
    if s == 0:
        return "    —    "
    return f"σ={s:5.2f}t  pk={mx:>5}t"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--n", type=int, default=200)
    args = ap.parse_args()

    port = args.port or autodetect_pirate()
    print(f"pirate: {port}")
    print(f"sweep: bauds={BAUDS}  sizes={SIZES}  n={args.n} per cell")
    print(f"reporting σ in SysTicks (1t = 55.6 ns @ 18 MHz) + µs")
    print()

    pirate = Pirate(port)
    sigmas: dict[tuple[int, int], float] = {}
    rejects: dict[tuple[int, int], dict[str, int]] = {}
    cell_w = 18
    try:
        hdr = "baud".rjust(8) + " |" + "".join(f"size={s:<4}".rjust(cell_w) for s in SIZES)
        print(hdr)
        print("-" * len(hdr))

        for baud in BAUDS:
            row = f"{baud:>8} |"
            row_sigmas: list[float] = []
            for size in SIZES:
                vals, rej = measure(pirate, baud, size, args.n)
                s = sigma_ticks(vals)
                mx = peak_dev(vals)
                sigmas[(baud, size)] = s
                rejects[(baud, size)] = rej
                row_sigmas.append(s)
                row += " " + fmt_cell(s, mx).rjust(cell_w - 1)
            avg = statistics.mean(row_sigmas)
            row += f"   avg σ={avg:5.2f}t"
            print(row)

        print("-" * len(hdr))
        col_avgs = "  avg σ |"
        for size in SIZES:
            col_vals = [sigmas[(b, size)] for b in BAUDS]
            col_avg = statistics.mean(col_vals)
            col_avgs += " " + f"{col_avg:5.2f}t".rjust(cell_w - 1)
        overall = statistics.mean(sigmas.values())
        col_avgs += f"   all={overall:5.2f}t"
        print(col_avgs)

        # Two-bucket reject matrix. `missed` = USB stall ate the 3 ms schedule
        # headroom (immediate-fire fallback); these are host-side and expected
        # to be small but non-zero on macOS. `bus` = framing/echo from a DUT
        # that wasn't unplugged; should be 0.
        total_missed = sum(r["missed"] for r in rejects.values())
        total_bus = sum(r["bus"] for r in rejects.values())
        total = args.n * len(BAUDS) * len(SIZES)
        if total_missed or total_bus:
            print()
            print(f"rejects: missed-schedule={total_missed}/{total} (USB stall)"
                  f"  bus-chatter={total_bus}/{total} (unplug DUT)")
            rrow = "baud".rjust(8) + " |" + "".join(f"  s={s:<3}m/b".rjust(12) for s in SIZES)
            print(rrow)
            for baud in BAUDS:
                line = f"{baud:>8} |"
                for size in SIZES:
                    r = rejects[(baud, size)]
                    line += " " + f"{r['missed']:>3}/{r['bus']:<3}".rjust(11)
                print(line)
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
