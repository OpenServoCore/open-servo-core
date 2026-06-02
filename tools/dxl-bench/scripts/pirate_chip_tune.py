"""dxl-bench: end-to-end V006 clock + TX-latency tuner & verifier.

Pipeline (each step depends on the previous):
  1. HSI coarse       — clock_trim              (integer HSITRIM step)
  2. HSI fine         — clock_fine_trim_us      (Q8.8 µs sub-step residual)
  3. Plain TX latency — dxl_tx_plain_latency_us (Ping continuous offset)
  4. Fast TX latency  — dxl_tx_fast_latency_us  (chip-as-Last 2-slot gap)
  5. Verify matrix    — baud × payload × position, read-only PASS/FAIL

Why this order:
  Plain/Fast convert Q8.8 µs → chip ticks at the nominal 48 MHz constant,
  so HSI drift is silently absorbed into the converged latency. Re-cal'ing
  HSI later would invalidate them. Fast also depends on TX_PLAIN math
  (chain slot start uses TX_PLAIN), and on HSI fine trim (chain slot is
  byte-time wide; per-slot drift accumulates).

Steps 1-4 mutate RAM-backed CT (survives until reboot; no flash write).
Step 5 is read-only and walks every baud × payload × position cell.

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
DXL_TX_PLAIN_LATENCY_US_ADDR = 22
DXL_TX_FAST_LATENCY_US_ADDR = 24

BAUD_INDEX = {
    9600: 0, 57600: 1, 115200: 2,
    1_000_000: 3, 2_000_000: 4, 3_000_000: 5,
}

# Must not collide with --id or any other live chip on the bus: this ID is
# named as slot 2 of the First-position verify so the bus IDLEs after chip
# slot 1, giving a clean single Round stamp.
FOREIGN_ID = 99
# INJ slot ID for Last-position fast tuning + Last verify cells.
INJ_ID = 50

VERIFY_BAUDS = [1_000_000, 2_000_000, 3_000_000]
VERIFY_PAYLOADS = [1, 4, 32]

# Multi-corner Fast tune sweep. Worst-case baud (tightest post-fire slack)
# × DUT length spread matching verify Last cells. INJ length fixed at 4 to
# match `_verify_one_baud`'s Last path. The tune writes the min safe_upper
# across corners, so verify cells stress-test the joint boundary instead of
# the per-corner one.
DEFAULT_FAST_TUNE_CORNERS = [
    # (baud, inj_len, dut_len)
    (3_000_000, 4, 1),
    (3_000_000, 4, 4),
    (3_000_000, 4, 32),
]


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


# ── Step 3: plain TX latency ────────────────────────────────────────────────

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


def step_plain_tune(pirate: Pirate, dxl_id: int, baud: int,
                    n: int, converge_q88: int, max_iter: int) -> int:
    ticks_per_us = pirate.hz_per_us()
    rdt_us = read_ct_u8(pirate, dxl_id, RETURN_DELAY_2US_ADDR) * 2
    # 1 DXL byte = 10 bit-times. T_first stamps wire-END of first reply byte,
    # so a perfectly-tuned chip lands at T_req + RDT + byte_time.
    byte_time_ticks = (10 * ticks_per_us * 1_000_000) // baud
    target = rdt_us * ticks_per_us + byte_time_ticks
    ping = build_ping(dxl_id)
    cur_q88 = read_ct_u16(pirate, dxl_id, DXL_TX_PLAIN_LATENCY_US_ADDR)
    print(f"  RDT={rdt_us}µs  byte_time={byte_time_ticks}t  target={target}t  "
          f"start TX_PLAIN={cur_q88}q88 ({cur_q88/256:.3f}µs)")
    for it in range(1, max_iter + 1):
        offset, sigma, n_v = measure_plain_offset(pirate, dxl_id, ping, n, target)
        if n_v == 0:
            raise PirateError(f"plain iter {it}: no valid samples")
        offset_q88 = round((offset / ticks_per_us) * 256)
        print(f"  iter {it}: n={n_v}/{n}  offset={offset:+7.2f}t "
              f"({offset/ticks_per_us:+.3f}µs = {offset_q88:+5d}q88)  "
              f"σ={sigma:5.2f}t  TX_PLAIN={cur_q88}q88")
        if abs(offset_q88) <= converge_q88:
            return cur_q88
        cur_q88 = max(0, min(0xFFFF, cur_q88 + offset_q88))
        write_ct_u16(pirate, dxl_id, DXL_TX_PLAIN_LATENCY_US_ADDR, cur_q88)
        time.sleep(0.01)
    raise PirateError(
        f"plain tune did not converge in {max_iter} iter (last TX_PLAIN={cur_q88}q88)"
    )


# ── Step 4: fast TX latency ─────────────────────────────────────────────────

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


def step_fast_tune(pirate: Pirate, dxl_id: int, baud: int,
                   n: int, max_iter: int,
                   max_crc_rate: float = 0.01,
                   tolerance_q88: int = 16,
                   start_q88: int = 768,
                   probe_step_q88: int = 64,
                   max_probe_step_q88: int = 96,
                   inj_len: int = 4, dut_len: int = 4) -> int:
    """Bracket search at a single corner. Returns the highest TX_FAST_LATENCY
    (Q8.8 µs) confirmed SAFE at this (baud, inj_len, dut_len) corner. The
    caller applies the safety margin and writes the CT field.

    Why bracket-on-CRC instead of damped-Newton-on-gap: when TX_FAST is high
    enough that chip slot 2 fires into the snoop race, ~all such shots fail
    CRC and get filtered. The surviving Rounds' median gap is pinned to the
    safe edge of the jitter distribution — gap median barely moves as TX_FAST
    crosses the cliff (observed: 0.28 µs of gap shift for 0.75 µs of TX_FAST
    bump). CRC rate IS the honest signal; gap is just a diagnostic readout.

    Phase A — probe upward from `start_q88` (step doubles each SAFE iter,
    capped at `max_probe_step_q88`) until crc_rate exceeds `max_crc_rate`.
    Phase B — bisect the (safe, unsafe) bracket until the bracket width
    ≤ `tolerance_q88`. Tight `max_probe_step_q88` keeps the post-A bracket
    narrow so the final settle isn't dominated by Phase A overshoot."""
    ticks_per_us = pirate.hz_per_us()
    rdt_us = read_ct_u8(pirate, dxl_id, RETURN_DELAY_2US_ADDR) * 2
    arm_offset = rdt_us * ticks_per_us

    def _measure(cur):
        write_ct_u16(pirate, dxl_id, DXL_TX_FAST_LATENCY_US_ADDR, cur)
        time.sleep(0.01)
        gap, sigma, n_v, n_crc, n_other = measure_fast_gap(
            pirate, dxl_id, n, baud, arm_offset, inj_len, dut_len,
        )
        total = n_v + n_crc
        crc_rate = (n_crc / total) if total > 0 else 1.0
        gap_us = (gap / ticks_per_us) if n_v > 0 else float("nan")
        sigma_us = sigma / ticks_per_us if n_v > 1 else 0.0
        return crc_rate <= max_crc_rate, crc_rate, gap_us, sigma_us, n_v, n_crc

    print(f"  inj_len={inj_len} dut_len={dut_len}  arm_offset={arm_offset}t "
          f"(=RDT {rdt_us}µs)  max_crc_rate={max_crc_rate:.1%}  "
          f"start={start_q88}q88 ({start_q88/256:.3f}µs)")

    lo: int | None = None
    hi: int | None = None
    cur = start_q88
    step = probe_step_q88
    iters = 0

    print(f"  Phase A — probe upward")
    while iters < max_iter:
        iters += 1
        safe, crc_rate, gap_us, sigma_us, n_v, n_crc = _measure(cur)
        verdict = "SAFE  " if safe else "UNSAFE"
        print(f"    iter {iters}: TX_FAST={cur:5d}q88 ({cur/256:.3f}µs)  "
              f"crc={n_crc:>3d}/{n_v + n_crc:>3d} ({crc_rate:5.1%})  "
              f"gap={gap_us:+.3f}µs  σ={sigma_us:.3f}µs  {verdict}")
        if safe:
            lo = cur
            cur = min(0xFFFF, cur + step)
            step = min(max_probe_step_q88, step * 2)
            if lo == 0xFFFF:
                break
        else:
            hi = cur
            break

    if lo is None:
        # First probe was already unsafe — back off until safe.
        cur = max(0, start_q88 - probe_step_q88)
        print(f"  start unsafe — back off below {start_q88}")
        while cur > 0 and iters < max_iter:
            iters += 1
            safe, crc_rate, gap_us, sigma_us, n_v, n_crc = _measure(cur)
            verdict = "SAFE  " if safe else "UNSAFE"
            print(f"    iter {iters}: TX_FAST={cur:5d}q88 ({cur/256:.3f}µs)  "
                  f"crc={n_crc:>3d}/{n_v + n_crc:>3d} ({crc_rate:5.1%})  "
                  f"gap={gap_us:+.3f}µs  σ={sigma_us:.3f}µs  {verdict}")
            if safe:
                lo = cur
                hi = start_q88
                break
            cur = max(0, cur - probe_step_q88)
        if lo is None:
            raise PirateError(
                f"no safe TX_FAST found between 0 and {start_q88}q88 — "
                f"chip-side framing fault?"
            )

    if hi is None:
        print(f"  no cliff below 0xFFFF — safe_upper={lo}q88")
        return lo

    print(f"  Phase B — bisect [{lo}, {hi}]")
    while hi - lo > tolerance_q88 and iters < max_iter:
        iters += 1
        mid = (lo + hi) // 2
        safe, crc_rate, gap_us, sigma_us, n_v, n_crc = _measure(mid)
        verdict = "SAFE  " if safe else "UNSAFE"
        print(f"    iter {iters}: TX_FAST={mid:5d}q88 ({mid/256:.3f}µs)  "
              f"crc={n_crc:>3d}/{n_v + n_crc:>3d} ({crc_rate:5.1%})  "
              f"gap={gap_us:+.3f}µs  σ={sigma_us:.3f}µs  {verdict}  "
              f"[lo={lo}, hi={hi}]")
        if safe:
            lo = mid
        else:
            hi = mid

    print(f"  bracket lo={lo}q88 hi={hi}q88 → safe_upper={lo}q88 "
          f"({lo/256:.3f}µs)")
    return lo


def step_fast_tune_multi(pirate: Pirate, dxl_id: int,
                         corners: list[tuple[int, int, int]],
                         n: int, max_iter: int,
                         max_crc_rate: float = 0.01,
                         tolerance_q88: int = 16,
                         safety_margin_q88: int = 32,
                         start_q88: int = 768,
                         probe_step_q88: int = 64,
                         max_probe_step_q88: int = 96,
                         ) -> int:
    """Tune TX_FAST_LATENCY across multiple (baud, inj_len, dut_len) corners
    and write min(safe_upper) − safety_margin. Pins the field to the worst-
    case slack budget so verify cells stress the joint boundary."""
    results: list[tuple[int, int, int, int]] = []
    for baud, inj_len, dut_len in corners:
        print(f"\n  ── corner: {baud//1_000_000}M  inj={inj_len}  dut={dut_len}")
        set_chip_baud(pirate, dxl_id, baud)
        time.sleep(0.05)
        safe_upper = step_fast_tune(
            pirate, dxl_id, baud, n, max_iter,
            max_crc_rate=max_crc_rate,
            tolerance_q88=tolerance_q88,
            start_q88=start_q88,
            probe_step_q88=probe_step_q88,
            max_probe_step_q88=max_probe_step_q88,
            inj_len=inj_len, dut_len=dut_len,
        )
        results.append((safe_upper, baud, inj_len, dut_len))
        print(f"  corner safe_upper={safe_upper}q88 ({safe_upper/256:.3f}µs)")

    min_safe, worst_b, worst_il, worst_dl = min(results, key=lambda r: r[0])
    final = max(0, min_safe - safety_margin_q88)
    write_ct_u16(pirate, dxl_id, DXL_TX_FAST_LATENCY_US_ADDR, final)
    print(f"\n  per-corner safe_upper:")
    for safe, b, il, dl in results:
        marker = " ← min" if safe == min_safe else ""
        print(f"    {b//1_000_000}M  inj={il:<2d}  dut={dl:<2d}  "
              f"safe_upper={safe}q88 ({safe/256:.3f}µs){marker}")
    print(f"  → final TX_FAST = {final}q88 ({final/256:.3f}µs, "
          f"margin {safety_margin_q88}q88, worst corner "
          f"{worst_b//1_000_000}M inj={worst_il} dut={worst_dl})")
    return final


# ── Step 5: verify matrix ───────────────────────────────────────────────────

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
                    help="Baud for the tuning steps. Verify walks all bauds.")
    ap.add_argument("--n", type=int, default=256,
                    help="samples per tune iteration")
    ap.add_argument("--converge-q88", type=int, default=20,
                    help="tune stop threshold (Q8.8 µs)")
    ap.add_argument("--max-iter", type=int, default=10)
    ap.add_argument("--verify-n", type=int, default=128,
                    help="samples per verify cell")
    ap.add_argument("--fast-max-crc-rate", type=float, default=0.01,
                    help="Bracket search treats CRC rate ≤ this as SAFE.")
    ap.add_argument("--fast-tolerance-q88", type=int, default=16,
                    help="Bisect stops once the (safe, unsafe) bracket is "
                         "narrower than this (Q8.8 µs).")
    ap.add_argument("--fast-safety-margin-q88", type=int, default=32,
                    help="Final TX_FAST = safe_boundary − margin, for jitter "
                         "headroom (default 32q88 = 0.125µs).")
    ap.add_argument("--fast-start-q88", type=int, default=768,
                    help="Probe starts here. Default 768 (=3µs, firmware "
                         "default) so a prior crashed run leaving CT in the "
                         "cliff zone doesn't poison iter 1.")
    ap.add_argument("--skip-hsi", action="store_true")
    ap.add_argument("--skip-plain", action="store_true")
    ap.add_argument("--skip-fast", action="store_true")
    ap.add_argument("--verify-only", action="store_true",
                    help="implies --skip-hsi --skip-plain --skip-fast")
    args = ap.parse_args()

    if args.id in (INJ_ID, FOREIGN_ID):
        sys.exit(f"--id {args.id} collides with INJ_ID={INJ_ID} or FOREIGN_ID={FOREIGN_ID}")

    port = args.port or autodetect_pirate()
    print(f"pirate: {port}   chip id: {args.id}   tune baud: {args.tune_baud}")

    pirate = Pirate(port)
    try:
        set_chip_baud(pirate, args.id, args.tune_baud)

        if not args.verify_only:
            if not args.skip_hsi:
                print(f"\n[1+2] HSI coarse + fine @ {args.tune_baud}")
                trim, q88 = step_hsi(pirate, args.id, args.tune_baud)
                print(f"  → clock_trim={trim:+d}  clock_fine_trim_us={q88:+d} "
                      f"({q88/256:+.3f}µs)")
            if not args.skip_plain:
                print(f"\n[3] Plain TX latency @ {args.tune_baud}")
                v = step_plain_tune(pirate, args.id, args.tune_baud,
                                    args.n, args.converge_q88, args.max_iter)
                print(f"  → dxl_tx_plain_latency_us={v}q88 ({v/256:.3f}µs = "
                      f"{(v * 48) // 256} chip ticks @48MHz)")
            if not args.skip_fast:
                print(f"\n[4] Fast TX latency — multi-corner sweep")
                v = step_fast_tune_multi(
                    pirate, args.id, DEFAULT_FAST_TUNE_CORNERS,
                    args.n, args.max_iter,
                    max_crc_rate=args.fast_max_crc_rate,
                    tolerance_q88=args.fast_tolerance_q88,
                    safety_margin_q88=args.fast_safety_margin_q88,
                    start_q88=args.fast_start_q88,
                )
                print(f"  → dxl_tx_fast_latency_us={v}q88 ({v/256:.3f}µs = "
                      f"{(v * 48) // 256} chip ticks @48MHz)")
                set_chip_baud(pirate, args.id, args.tune_baud)

        print(f"\n[5] Verify matrix (n={args.verify_n} per cell)")
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
