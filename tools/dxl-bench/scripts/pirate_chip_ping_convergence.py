#!/usr/bin/env python3
"""Bench the chip's autonomous HSI-trim convergence under plain Ping load.

The chip owns its clock trim: it watches inter-byte timing on every non-Status
packet and nudges HSITRIM toward zero drift on its own (no master CAL — see
`firmware/lib/drivers/src/dxl/uart/clock.rs`). A boot batch lands the bulk of
the factory drift in the first reply; steady batches (~3.3 pings each) squeeze
the residual.

This script reboots the chip to its factory-default HSI (the only way to a
pristine "cold" point without touching the deprecated `clock_trim` field), then
drives Pings and periodically probes the residual drift via the pirate's
stable-clock `Round` stamp (`drift.probe_drift_ppm`). It prints the convergence
curve and reports how many Pings the trim took to settle inside one step.

Run:  python scripts/pirate_chip_ping_convergence.py [--id ID] [--baud BPS]
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from drift import DRIFT_STEP_PPM, probe_drift_ppm
from dxl_packet import build_ping, build_reboot
from pirate import Pirate, PirateError
from pirate_chip_common import (
    BAUD_INDEX,
    autodetect_pirate,
    discover_chip_id,
    set_chip_baud,
)

# Baud a fresh / just-rebooted chip boots at (mirrors BaudRate default B1000000
# in firmware/lib/core/src/regions/config.rs).
BOOT_BAUD = 1_000_000


def _reboot(pirate: Pirate, dxl_id: int) -> None:
    """Reboot the chip and wait for it to come back at BOOT_BAUD. The reboot
    is ACKed before the chip resets, so a missing reply just means the ACK
    raced the reset — not fatal."""
    pirate.xfer(build_reboot(dxl_id), reply_us=200_000)
    time.sleep(0.5)
    pirate.set_baud(BOOT_BAUD)
    pirate.drain_stamps()


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=None,
                    help="chip DXL ID. If omitted, broadcast-Ping the bus "
                         "and use the responding chip's ID (UID-derived).")
    ap.add_argument("--baud", type=int, default=BOOT_BAUD,
                    help="DXL baud to drive convergence at (default 1M, the "
                         "boot baud). The cold point is always probed at the "
                         "boot baud; drift is baud-independent.")
    ap.add_argument("--max-pings", type=int, default=2000,
                    help="Give up if the trim hasn't settled by this many Pings.")
    ap.add_argument("--probe-every", type=int, default=20,
                    help="Pings driven between drift probes (default 20 ≈ one "
                         "steady batch).")
    ap.add_argument("--probe-samples", type=int, default=8,
                    help="Long-Read probes averaged per drift measurement.")
    ap.add_argument("--converge-streak", type=int, default=3,
                    help="Consecutive in-bound probes to declare convergence.")
    ap.add_argument("--forever", action="store_true",
                    help="Probe + print indefinitely: no convergence early-exit, "
                         "--max-pings ignored. Ctrl-C to stop. Pair with "
                         "--probe-every 1 to watch per-ping drift evolve.")
    args = ap.parse_args()

    if args.baud not in BAUD_INDEX:
        sys.exit(f"unsupported --baud {args.baud}; supported: {sorted(BAUD_INDEX)}")

    bound = 1.5 * DRIFT_STEP_PPM
    port = args.port or autodetect_pirate()
    pirate = Pirate(port)
    try:
        if args.id is None:
            dxl_id = discover_chip_id(pirate, args.baud)
        else:
            dxl_id = args.id
            set_chip_baud(pirate, dxl_id, args.baud)
        print(f"pirate: {port}   chip id: {dxl_id}   bound: ±{bound:.0f} ppm")

        print("\n[reboot → factory-default HSI]")
        _reboot(pirate, dxl_id)
        # First post-reboot transaction: the reply ships at the un-corrected
        # HSI (the boot correction applies only after this reply's TX), so this
        # single probe captures the cold drift. Non-fatal: a flaky first
        # post-reboot reply must not rob us of the convergence table below.
        try:
            cold = probe_drift_ppm(pirate, dxl_id, BOOT_BAUD, samples=1)
            print(f"  cold drift   = {cold:+.0f} ppm")
        except PirateError as e:
            cold = None
            print(f"  cold drift   = n/a ({e})")

        # The first post-reboot instruction trips the boot-phase trim: a
        # one-shot full-envelope correction, distinct from the steady ±4-step
        # nudges that follow. Drive one ping and re-probe (at the boot baud,
        # before any switch) to isolate what that single boot correction left.
        pirate.xfer(build_ping(dxl_id), reply_us=200_000)
        try:
            after_first = probe_drift_ppm(pirate, dxl_id, BOOT_BAUD, samples=1)
            print(f"  after 1 ping = {after_first:+.0f} ppm")
        except PirateError as e:
            print(f"  after 1 ping = n/a ({e})")

        if args.baud != BOOT_BAUD:
            set_chip_baud(pirate, dxl_id, args.baud)

        print(f"\n[converge @ {args.baud}]")
        print(f"  {'pings':>6}  {'drift_ppm':>10}")
        pings = 0
        streak = 0
        converged_at = None
        try:
            while args.forever or pings < args.max_pings:
                for _ in range(args.probe_every):
                    pirate.xfer(build_ping(dxl_id), reply_us=200_000)
                pings += args.probe_every
                try:
                    drift = probe_drift_ppm(pirate, dxl_id, args.baud,
                                            samples=args.probe_samples)
                except PirateError as e:
                    print(f"  {pings:>6}  {'n/a':>10}  ({e})", flush=True)
                    streak = 0
                    continue
                flag = "ok" if abs(drift) <= bound else ""
                print(f"  {pings:>6}  {drift:>+10.0f}  {flag}", flush=True)
                if args.forever:
                    continue
                if abs(drift) <= bound:
                    streak += 1
                    if streak >= args.converge_streak:
                        converged_at = pings
                        break
                else:
                    streak = 0
        except KeyboardInterrupt:
            print("\n(stopped)")

        cold_str = f"{cold:+.0f} ppm" if cold is not None else "n/a"
        print()
        if args.forever:
            print(f"ran {pings} pings (cold {cold_str}); no verdict in --forever mode")
        elif converged_at is not None:
            print(f"converged within ±{bound:.0f} ppm after ~{converged_at} pings "
                  f"(cold {cold_str})")
        else:
            sys.exit(f"did NOT converge within ±{bound:.0f} ppm in "
                     f"{args.max_pings} pings (cold {cold_str})")
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
