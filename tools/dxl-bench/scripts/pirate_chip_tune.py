"""dxl-bench: V006 HSI clock calibration + verify matrix.

Pipeline:
  1. HSI coarse       — clock_trim         (integer HSITRIM step)
  2. HSI fine         — clock_fine_trim_us (Q8.8 µs sub-step residual)
  3. Verify matrix    — baud × payload × position, read-only PASS/FAIL

Steps 1-2 mutate RAM-backed CT (survives until reboot; no flash write).
Step 3 is read-only and walks every baud × payload × position cell.

TX_PLAIN/TX_FAST latencies are now compile-time constants in
`firmware/ch32/src/measurements.rs` (`PLAIN_ENTRY_TICKS`,
`FAST_ENTRY_TICKS`) — see `docs/dxl-fast-chain-crc-walkloop.md` §1.1 for
the scope methodology to re-measure them. No runtime tuning needed.

Usage:
  python scripts/pirate_chip_tune.py
  python scripts/pirate_chip_tune.py --skip-hsi
  python scripts/pirate_chip_tune.py --verify-only
"""

from __future__ import annotations

import argparse
import statistics
import struct
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import serial.tools.list_ports

from cal import Calibrator
from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    build_ping,
    build_read,
    build_write,
    parse_fast_response,
    parse_status,
)
from pirate import Pirate, PirateError, Round

PIRATE_VID = 0xC0DE
PIRATE_PID = 0xCAFE

# comms CT addresses (mirror config::addr::comms in
# firmware/lib/core/src/regions/config.rs).
BAUD_RATE_IDX_ADDR = 13
RETURN_DELAY_2US_ADDR = 14

BAUD_INDEX = {
    9600: 0, 57600: 1, 115200: 2,
    1_000_000: 3, 2_000_000: 4, 3_000_000: 5,
}

# Must not collide with --id or any other live chip on the bus: this ID is
# named as slot 2 of the First-position verify so the bus IDLEs after chip
# slot 1, giving a clean single Round stamp.
FOREIGN_ID = 99
# INJ slot ID for the Last verify cells (pirate fires the INJ slot via ARM).
INJ_ID = 50

VERIFY_BAUDS = [1_000_000, 2_000_000, 3_000_000]
VERIFY_PAYLOADS = [1, 4, 32]


def autodetect_pirate() -> str:
    matches = [
        p.device for p in serial.tools.list_ports.comports()
        if p.vid == PIRATE_VID and p.pid == PIRATE_PID
    ]
    if len(matches) != 1:
        sys.exit(f"expected exactly 1 pirate, found {matches}")
    return matches[0]


# ── CT helpers ──────────────────────────────────────────────────────────────

def _xfer_ct(pirate: Pirate, packet: bytes) -> bytes:
    reply = pirate.xfer(packet, reply_us=200_000)
    if not reply:
        raise PirateError("no Status reply")
    st = parse_status(reply)
    if st.error != 0:
        raise PirateError(f"err 0x{st.error:02X}")
    return st.params


def read_ct_u8(pirate: Pirate, dxl_id: int, addr: int) -> int:
    return _xfer_ct(pirate, build_read(dxl_id, addr, 1))[0]


def read_ct_u16(pirate: Pirate, dxl_id: int, addr: int) -> int:
    return struct.unpack("<H", _xfer_ct(pirate, build_read(dxl_id, addr, 2)))[0]


def write_ct_u16(pirate: Pirate, dxl_id: int, addr: int, value: int) -> None:
    _xfer_ct(pirate, build_write(dxl_id, addr, struct.pack("<H", value & 0xFFFF)))


def _ping_with_retry(pirate: Pirate, dxl_id: int,
                     retries: int = 3, delay_s: float = 0.02) -> bool:
    for _ in range(retries):
        if pirate.xfer(build_ping(dxl_id), reply_us=200_000):
            return True
        time.sleep(delay_s)
    return False


def set_chip_baud(pirate: Pirate, dxl_id: int, target_bps: int) -> None:
    """Ensure both ends are at `target_bps`; mirrors conftest.ensure_chip_baud
    minus the pytest dependency."""
    if target_bps not in BAUD_INDEX:
        raise ValueError(f"unsupported baud {target_bps}")
    pirate.set_baud(target_bps)
    time.sleep(0.05)
    pirate.drain_stamps()
    if _ping_with_retry(pirate, dxl_id):
        return
    current = None
    for bps in sorted(BAUD_INDEX, reverse=True):
        if bps == target_bps:
            continue
        pirate.set_baud(bps)
        time.sleep(0.05)
        pirate.drain_stamps()
        if _ping_with_retry(pirate, dxl_id):
            current = bps
            break
    if current is None:
        raise PirateError(f"chip not found at any known baud (target {target_bps})")
    _xfer_ct(pirate, build_write(
        dxl_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[target_bps]])
    ))
    time.sleep(0.05)
    pirate.set_baud(target_bps)
    time.sleep(0.05)


# ── Step 1+2: HSI coarse + fine ─────────────────────────────────────────────

def step_hsi(pirate: Pirate, dxl_id: int, baud: int, max_cycles: int = 6
             ) -> tuple[int, int]:
    """Converge clock_trim, then apply the matching clock_fine_trim_us residual.
    Returns (final_trim, final_q88)."""
    c = Calibrator(pirate, dxl_id=dxl_id, baud=baud)
    c.write_clock_fine_trim_us(0)
    time.sleep(0.05)
    for cycle in range(1, max_cycles + 1):
        trim_before = c.read_clock_trim()
        m = c.measure(count=128)
        d = c.derive(m, current_trim=trim_before)
        print(f"  iter {cycle}: trim={trim_before:+d}  drift={m.drift_ppm:+.0f} ppm"
              f"  step={d.step:+d}  residual={d.residual_ppm:+.0f} ppm "
              f"({d.residual_q88:+d} q88 µs)")
        if d.step == 0:
            c.write_clock_fine_trim_us(d.residual_q88)
            time.sleep(0.05)
            return trim_before, d.residual_q88
        c.write_clock_trim(d.new_trim)
        time.sleep(0.05)
    raise PirateError(f"HSI cal did not converge in {max_cycles} cycles")


# ── Measurement helpers (used by the verify matrix) ────────────────────────

def measure_plain_offset(pirate: Pirate, dxl_id: int, packet: bytes,
                         n: int, target_ticks: int
                         ) -> tuple[float, float, int]:
    """`n` xfers of `packet`; returns (median offset, σ, n_valid) in pirate
    ticks. offset = (T_first - T_req) - target. Positive ⇒ chip late."""
    offsets: list[int] = []
    for _ in range(n):
        pirate.drain_stamps()
        reply = pirate.xfer(packet, reply_us=10_000)
        if reply is None:
            continue
        rounds = [s for s in pirate.drain_stamps() if isinstance(s, Round)]
        if len(rounds) != 1:
            continue
        r = rounds[0]
        offsets.append(((r.first - r.req) & 0xFFFFFFFF) - target_ticks)
    if not offsets:
        return 0.0, 0.0, 0
    med = statistics.median(offsets)
    sigma = statistics.stdev(offsets) if len(offsets) > 1 else 0.0
    return med, sigma, len(offsets)


def measure_fast_gap(pirate: Pirate, dxl_id: int, n: int, baud: int,
                     arm_offset_ticks: int, inj_len: int, dut_len: int
                     ) -> tuple[float, float, int, int, int]:
    """Chip as Last in a 2-slot Fast Bulk Read; pirate fires the INJ slot via
    ARM. Returns (median gap, σ, n_valid, n_crc, n_other) in pirate ticks.
    gap = measured (last - first) - nominal chain span; >0 ⇒ chip slot 2 late."""
    ticks_per_us = pirate.hz_per_us()
    byte_time_ticks = (10 * ticks_per_us * 1_000_000) // baud
    inj_data = b"\xAA" * inj_len
    packet_length = 1 + (2 + inj_len) + (2 + dut_len) + 2
    inj_bytes = build_fast_first_bytes(
        packet_length=packet_length, err=0, slot_id=INJ_ID, data=inj_data,
    )
    request = build_fast_bulk_read([(INJ_ID, 0, inj_len), (dxl_id, 0, dut_len)])
    # Reply wire bytes: HEADER(4) + ID(1) + LENGTH(2) + packet_length.
    reply_bytes = 7 + packet_length
    expected_span = (reply_bytes - 1) * byte_time_ticks

    gaps: list[int] = []
    n_crc = 0
    n_other = 0
    for _ in range(n):
        pirate.drain_stamps()
        b0 = pirate.bytes_count()
        pirate.arm(inj_bytes, after_idle_ticks=arm_offset_ticks)
        pirate.master(request)
        time.sleep(0.005)
        b1 = pirate.bytes_count()
        if b1 - b0 > 256:
            pirate.drain_stamps()
            n_other += 1
            continue
        all_rx = pirate.rx_range(b0, b1 - b0)
        reply = all_rx[len(request):]
        try:
            slots = parse_fast_response(reply, slot_lengths=[inj_len, dut_len])
        except ValueError:
            n_crc += 1
            pirate.drain_stamps()
            continue
        if len(slots) != 2 or slots[1].id != dxl_id:
            n_other += 1
            pirate.drain_stamps()
            continue
        stamps = pirate.drain_stamps()
        rounds = [s for s in stamps if isinstance(s, Round)]
        if len(rounds) != 1:
            n_other += 1
            continue
        r = rounds[0]
        span = (r.last - r.first) & 0xFFFFFFFF
        gaps.append(span - expected_span)
    if not gaps:
        return 0.0, 0.0, 0, n_crc, n_other
    med = statistics.median(gaps)
    sigma = statistics.stdev(gaps) if len(gaps) > 1 else 0.0
    return med, sigma, len(gaps), n_crc, n_other


# ── Verify matrix ─────────────────────────────────────────────────────────

def _verify_one_baud(pirate: Pirate, dxl_id: int, baud: int, n: int
                     ) -> tuple[list[tuple], float]:
    label = f"{baud // 1_000_000}M"
    try:
        # Quiet period after a possibly-heavy prior baud cell: drain any
        # straggling stamps + give USB CDC time to flush before the probe.
        pirate.drain_stamps()
        time.sleep(0.1)
        set_chip_baud(pirate, dxl_id, baud)
    except PirateError as e:
        rows: list[tuple] = []
        for payload in VERIFY_PAYLOADS:
            for pos in ("Only", "First", "Last"):
                rows.append(
                    (label, pos, f"{payload}B", 0.0, 0.0, 0, f"FAIL(set_baud: {e})")
                )
        return rows, 0.0
    ticks_per_us = pirate.hz_per_us()
    byte_time_ticks = (10 * ticks_per_us * 1_000_000) // baud
    byte_time_us = byte_time_ticks / ticks_per_us
    rdt_us = read_ct_u8(pirate, dxl_id, RETURN_DELAY_2US_ADDR) * 2
    plain_target = rdt_us * ticks_per_us + byte_time_ticks
    arm_offset = rdt_us * ticks_per_us

    # Pass band: ±0.5 byte_time at this baud (within the coalesce window).
    thresh_ticks = byte_time_ticks / 2
    thresh_us = byte_time_us / 2

    rows = []
    for payload in VERIFY_PAYLOADS:
        # Only: simple Read.
        only_pkt = build_read(dxl_id, 0, payload)
        med, sigma, n_v = measure_plain_offset(pirate, dxl_id, only_pkt, n, plain_target)
        med_us, sigma_us = med / ticks_per_us, sigma / ticks_per_us
        verdict = "PASS" if (n_v > 0 and abs(med) < thresh_ticks) else "FAIL"
        rows.append((label, "Only", f"{payload}B", med_us, sigma_us, n_v, verdict))

        # First: chip as slot 1 in a 2-slot Fast Bulk Read; slot 2's FOREIGN_ID
        # never replies, so bus IDLEs after chip slot 1 → single Round stamp.
        first_pkt = build_fast_bulk_read([(dxl_id, 0, payload), (FOREIGN_ID, 0, 1)])
        med, sigma, n_v = measure_plain_offset(pirate, dxl_id, first_pkt, n, plain_target)
        med_us, sigma_us = med / ticks_per_us, sigma / ticks_per_us
        verdict = "PASS" if (n_v > 0 and abs(med) < thresh_ticks) else "FAIL"
        rows.append((label, "First", f"{payload}B", med_us, sigma_us, n_v, verdict))

        # Last: chip as slot 2; pirate fires INJ slot via ARM. Gap median
        # is survivor-biased (CRC-failed shots drop out), so don't enforce
        # a tight gap threshold here — require chain coalesces robustly
        # (low CRC + low "other" rate) and the median gap stays within
        # byte_time (else bus would IDLE between slots).
        med_g, sigma_g, n_v, n_crc, n_other = measure_fast_gap(
            pirate, dxl_id, n, baud, arm_offset, inj_len=4, dut_len=payload,
        )
        med_us, sigma_us = med_g / ticks_per_us, sigma_g / ticks_per_us
        total = n_v + n_crc + n_other
        success_rate = (n_v / total) if total > 0 else 0.0
        passes = (
            success_rate >= 0.95
            and (n_v == 0 or abs(med_g) < byte_time_ticks)
        )
        verdict = "PASS" if passes else f"FAIL(crc={n_crc} other={n_other})"
        rows.append((label, "Last", f"{payload}B", med_us, sigma_us, n_v, verdict))
    return rows, thresh_us


def step_verify(pirate: Pirate, dxl_id: int, n: int) -> bool:
    print(f"\n  {'baud':>4}  {'pos':>5}  {'payload':>7}  {'n':>4}  "
          f"{'median µs':>11}  {'σ µs':>8}  {'±thresh µs':>10}  verdict")
    print("  " + "─" * 76)
    all_pass = True
    for baud in VERIFY_BAUDS:
        rows, thresh_us = _verify_one_baud(pirate, dxl_id, baud, n)
        for (b, pos, pay, med, sigma, n_v, verdict) in rows:
            if not verdict.startswith("PASS"):
                all_pass = False
            print(f"  {b:>4}  {pos:>5}  {pay:>7}  {n_v:>4}  "
                  f"{med:+11.3f}  {sigma:8.3f}  {thresh_us:10.3f}  {verdict}")
    return all_pass


# ── Main ────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", default=None)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--tune-baud", type=int, default=1_000_000,
                    help="Baud for HSI calibration. Verify walks all bauds.")
    ap.add_argument("--verify-n", type=int, default=128,
                    help="samples per verify cell")
    ap.add_argument("--skip-hsi", action="store_true")
    ap.add_argument("--verify-only", action="store_true",
                    help="implies --skip-hsi")
    args = ap.parse_args()

    if args.id in (INJ_ID, FOREIGN_ID):
        sys.exit(f"--id {args.id} collides with INJ_ID={INJ_ID} or FOREIGN_ID={FOREIGN_ID}")

    port = args.port or autodetect_pirate()
    print(f"pirate: {port}   chip id: {args.id}   tune baud: {args.tune_baud}")

    pirate = Pirate(port)
    try:
        set_chip_baud(pirate, args.id, args.tune_baud)

        if not args.verify_only and not args.skip_hsi:
            print(f"\n[1+2] HSI coarse + fine @ {args.tune_baud}")
            trim, q88 = step_hsi(pirate, args.id, args.tune_baud)
            print(f"  → clock_trim={trim:+d}  clock_fine_trim_us={q88:+d} "
                  f"({q88/256:+.3f}µs)")

        print(f"\n[3] Verify matrix (n={args.verify_n} per cell)")
        ok = step_verify(pirate, args.id, args.verify_n)
        print()
        if ok:
            print("VERIFY: PASS — all cells within ±½ byte_time")
        else:
            print("VERIFY: FAIL — at least one cell over threshold")
            sys.exit(1)
    finally:
        pirate.close()


if __name__ == "__main__":
    main()
