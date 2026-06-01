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


def measure(pirate: Pirate, baud: int, size: int, n: int
            ) -> tuple[list[int], list[int], dict[str, int]]:
    """Returns (stamp_residuals, fire_pipelines, reject_counts). All SysTicks (18 MHz).

    Two parallel measurements per trial:
      stamp_residual = LAST_RXNE_TICK - fire_at − N × byte_time
                     = fire_pipeline + stamp_offset (combined latency, end of frame)
      fire_pipeline  = FIRE_T_FIRST - fire_at - 1 × byte_time
                     = fire_pipeline + rxne_offset (combined latency, start of frame)

    For a self-consistent RXNE behavior, stamp_residual − fire_pipeline_proxy
    ≈ 0 (sanity check: every byte stamps with the same offset). The two
    medians should also be equal — they measure the same combined bias from
    two different bytes of the same frame.

    Reject categories:
      - `missed`: USB stall pushed the FIRE command past `fire_at`, so
        `schedule_or_fire_now` took the immediate-fire branch.
      - `bus`: stamp/byte-count check failed.
      - `nofire`: FIRE_T_FIRST stayed 0 — RXNE didn't fire (likely bus
        disconnected or first-byte RXNE pile-up).
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
    # Float byte_time — at non-integer-ratio bauds (e.g. 115200 ≡ 1562.5
    # SysTicks/byte) rounding to int would accumulate ~½-tick error per
    # byte, showing up as a size-dependent residual bias. Subtract as
    # float, round the final residual.
    byte_time_ticks_f = byte_us * pirate.hz_per_us()

    payload = bytes([0xAA] * size)
    stamp_residuals: list[int] = []
    fire_pipelines: list[int] = []
    rejects = {"missed": 0, "bus": 0, "nofire": 0}

    for _ in range(n):
        pirate.drain_stamps()
        bytes_before = pirate.bytes_count()
        now = pirate.tick()
        fire_at = (now + schedule_ahead) & 0xFFFFFFFFFFFFFFFF
        pirate.fire(payload, at_tick=fire_at)
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
        fire_first = pirate.last_fire_t_first()
        if fire_first == 0:
            rejects["nofire"] += 1
            continue
        fire_at_lo = fire_at & 0xFFFFFFFF
        stamp_r = (stamps[0].tick - fire_at_lo) & 0xFFFFFFFF
        if stamp_r > 1 << 31:
            stamp_r -= 1 << 32
        stamp_residuals.append(round(stamp_r - size * byte_time_ticks_f))
        fire_p = (fire_first - fire_at_lo) & 0xFFFFFFFF
        if fire_p > 1 << 31:
            fire_p -= 1 << 32
        fire_pipelines.append(round(fire_p - byte_time_ticks_f))

    return stamp_residuals, fire_pipelines, rejects


def sigma_ticks(vals: list[int]) -> float:
    return statistics.stdev(vals) if len(vals) > 1 else 0.0


def median_or_zero(vals: list[int]) -> float:
    return float(statistics.median(vals)) if vals else 0.0


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--n", type=int, default=200)
    args = ap.parse_args()

    port = args.port or autodetect_pirate()
    pirate = Pirate(port)
    hz_per_us = pirate.hz_per_us()
    ns_per_tick = 1000.0 / hz_per_us
    systick_mhz = hz_per_us
    systick_hz = systick_mhz * 1_000_000

    print(f"pirate: {port}")
    print(f"sweep: bauds={BAUDS}  sizes={SIZES}  n={args.n} per cell")
    print()
    print(f"  pirate SysTick clock: {systick_hz:>12,} Hz  ({systick_mhz} MHz)")
    print(f"  tick unit (t):        {1.0/systick_mhz:.4f} µs  ({ns_per_tick:.2f} ns)")
    print(f"  derivation:           V203 HCLK 144 MHz ÷ 8 (SYSTICK.STCLK = HCLK_DIV8)")
    print()
    print("  fire_pipeline  = median(FIRE_T_FIRST - fire_at - 1·byte_time)")
    print("                   wall-clock latency from commanded fire_at → wire start bit")
    print("  stamp_residual = median(LAST_RXNE_TICK - fire_at - N·byte_time)")
    print("                   same latency observed from wire-end of last byte")
    print("  delta          = stamp_residual − fire_pipeline (≈ 0 → RXNE consistent across bytes)")
    print()

    medians_stamp: dict[tuple[int, int], float] = {}
    medians_fire: dict[tuple[int, int], float] = {}
    sigmas_fire: dict[tuple[int, int], float] = {}
    rejects: dict[tuple[int, int], dict[str, int]] = {}

    def fmt_t_us(v: float) -> str:
        return f"{v:>+5.1f}t / {v/systick_mhz:>+6.3f}µs"

    def fmt_t_us_unsigned(v: float) -> str:
        return f"{v:>5.2f}t / {v/systick_mhz:>5.3f}µs"

    try:
        hdr = (f"{'baud':>8} {'size':>5}   "
               f"{'fire_p (med)':>15}   {'stamp_r (med)':>15}   "
               f"{'delta':>15}   {'σ_fire':>14}   {'σ_stamp':>14}   {'n':>4}")
        print(hdr)
        print("-" * len(hdr))
        for baud in BAUDS:
            for size in SIZES:
                stamp_r, fire_p, rej = measure(pirate, baud, size, args.n)
                m_s = median_or_zero(stamp_r)
                m_f = median_or_zero(fire_p)
                s_s = sigma_ticks(stamp_r)
                s_f = sigma_ticks(fire_p)
                medians_stamp[(baud, size)] = m_s
                medians_fire[(baud, size)] = m_f
                sigmas_fire[(baud, size)] = s_f
                rejects[(baud, size)] = rej
                delta = m_s - m_f
                print(f"{baud:>8} {size:>5}   "
                      f"{fmt_t_us(m_f):>15}   {fmt_t_us(m_s):>15}   "
                      f"{fmt_t_us(delta):>15}   {fmt_t_us_unsigned(s_f):>14}   "
                      f"{fmt_t_us_unsigned(s_s):>14}   {len(fire_p):>4}")
            print()

        # Per-baud summary: should be flat across `size`. The cleaner the
        # silicon, the tighter the across-size spread.
        print("-" * len(hdr))
        print("Per-baud fire_pipeline median (averaged across sizes):")
        for baud in BAUDS:
            cells = [medians_fire[(baud, s)] for s in SIZES if medians_fire[(baud, s)] != 0]
            if not cells:
                continue
            avg = statistics.mean(cells)
            spread = max(cells) - min(cells)
            byte_us = 10.0e6 / baud
            comp_us = avg / systick_mhz
            spread_us = spread / systick_mhz
            print(f"  {baud:>8}: avg={avg:+7.1f}t ({comp_us:+.3f}µs)  "
                  f"spread={spread:>5.1f}t ({spread_us:.3f}µs) over sizes  "
                  f"({byte_us:.2f}µs/byte)")

        total = args.n * len(BAUDS) * len(SIZES)
        total_missed = sum(r["missed"] for r in rejects.values())
        total_bus = sum(r["bus"] for r in rejects.values())
        total_nofire = sum(r["nofire"] for r in rejects.values())
        if total_missed or total_bus or total_nofire:
            print()
            print(f"rejects: missed-schedule={total_missed}/{total} (USB stall)"
                  f"  bus={total_bus}/{total} (unplug DUT)"
                  f"  nofire={total_nofire}/{total} (RXNE missed)")
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
