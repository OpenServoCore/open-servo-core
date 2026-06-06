"""End-to-end validation of chip-owned HSI calibration.

The chip's drift filter runs autonomously: every non-Status packet on the bus
(`Ping`, `Read`, `Write`, `CAL`, …) feeds the snoop loop, which batches
samples, drives an EMA in ppm-space, and queues HSITRIM + Q8.8 µs fine
residual updates at ±½-step hysteresis. Master no longer applies trim.

Two tests:
  - Explicit-CAL convergence: drive `Calibrate` packets; assert the chip's
    own `applied_trim_delta` reaches a steady-zero deadband, with final drift
    bounded by the ±½-step hysteresis plus EMA slack.
  - Dynamic recovery: drive plain `Ping` packets after a manual `clock_trim`
    perturbation; assert the chip recovers (drift falls back inside ±step)
    without any explicit CAL or master-side math.

Per docs/dxl-hsi-calibration.md §7. Does not assert any particular HSITRIM
value — the per-chip optimum depends on factory variation.
"""

import time

import pytest

from cal import Calibrator
from dxl_packet import build_ping

MAX_CAL_SHOTS = 60
CONVERGED_NOOP_STREAK = 3
PERTURBATION_STEPS = 2
RECOVERY_PINGS = 2000


@pytest.fixture
def calibrator(pirate, osc_id, baud):
    return Calibrator(pirate, dxl_id=osc_id, baud=baud)


def _drive_chip_cal_until_steady(c: Calibrator) -> list[tuple[float, int, int]]:
    """Burst CAL packets until the chip's `applied_trim_delta` is zero for
    `CONVERGED_NOOP_STREAK` consecutive shots — that's the chip-side filter's
    own "deadband locked" signal. Returns the (drift_ppm, trim_delta,
    fine_us_q88) history for diagnostics."""
    history: list[tuple[float, int, int]] = []
    consecutive_noop = 0
    for _ in range(MAX_CAL_SHOTS):
        m = c.measure(count=128)
        history.append((m.drift_ppm, m.applied_trim_delta, m.applied_fine_trim_us))
        if m.applied_trim_delta == 0:
            consecutive_noop += 1
            if consecutive_noop >= CONVERGED_NOOP_STREAK:
                return history
        else:
            consecutive_noop = 0
    pytest.fail(
        f"chip cal did not converge in {MAX_CAL_SHOTS} shots; tail={history[-6:]}"
    )


def test_explicit_cal_converges_via_chip(calibrator):
    c = calibrator
    initial_trim = c.read_clock_trim()
    initial_fine = c.read_clock_fine_trim_us()
    try:
        history = _drive_chip_cal_until_steady(c)
        print(
            f"\n  baseline trim={initial_trim:+d}  ppm_per_step={c.ppm_per_step}"
            f"  converged in {len(history)} shots",
            flush=True,
        )
        for i, (d, t, f) in enumerate(history):
            print(f"  shot {i:2d}: drift={d:+7.0f} ppm  applied_trim={t:+d}  "
                  f"applied_fine_q88={f:+d}", flush=True)

        # Final drift bounded by ½-step hysteresis + ~1-step EMA slack
        # (the EMA may sit anywhere inside the deadband at lock).
        m = c.measure(count=128)
        bound = 1.5 * c.ppm_per_step
        print(f"  final shot: drift={m.drift_ppm:+.0f} ppm  bound=±{bound:.0f}",
              flush=True)
        assert abs(m.drift_ppm) <= bound, (
            f"converged but |drift| {m.drift_ppm:+.0f} > {bound:.0f} ppm"
        )
    finally:
        c.write_clock_trim(initial_trim)
        time.sleep(0.05)
        c.write_clock_fine_trim_us(initial_fine)
        time.sleep(0.05)


def test_dynamic_cal_recovers_from_trim_perturbation(calibrator, pirate, osc_id):
    """Plain-Ping recovery: master does no CAL after the perturbation, just
    streams Pings. Chip-side filter must drive HSITRIM back to where drift is
    small. Verifies the deployment-mode path (no explicit cal request)."""
    c = calibrator
    initial_trim = c.read_clock_trim()
    initial_fine = c.read_clock_fine_trim_us()
    try:
        _drive_chip_cal_until_steady(c)
        m_pre = c.measure(count=128)
        print(f"\n  pre-perturb drift = {m_pre.drift_ppm:+.0f} ppm "
              f"(trim={initial_trim:+d})", flush=True)

        # Manual perturbation. The Write itself is one non-Status packet, so
        # the chip's snoop sees it too — but the snoop only sees ONE sample
        # before the HSI shifts, so the actual drift response lands on the
        # subsequent Pings.
        c.write_clock_trim(initial_trim + PERTURBATION_STEPS)
        time.sleep(0.05)
        m_perturbed = c.measure(count=128)
        print(f"  post-perturb drift = {m_perturbed.drift_ppm:+.0f} ppm "
              f"(target |shift| ≥ {c.ppm_per_step} ppm)", flush=True)
        shift_ppm = m_perturbed.drift_ppm - m_pre.drift_ppm
        assert abs(shift_ppm) >= c.ppm_per_step, (
            f"perturbation didn't shift drift as expected: "
            f"pre={m_pre.drift_ppm:+.0f}, post={m_perturbed.drift_ppm:+.0f}, "
            f"shift={shift_ppm:+.0f} ppm"
        )

        # Drive recovery via plain Pings. Each non-Status packet feeds the
        # chip's snoop; with BATCH_K=8 + α=1/32 the EMA needs hundreds of
        # samples to cross the apply threshold for each remaining step. Budget
        # is generous to absorb noise + variable per-shot timing.
        for _ in range(RECOVERY_PINGS):
            pirate.xfer(build_ping(osc_id), reply_us=200_000)

        # Verify drift recovered. The CT mirror's `clock_trim` reflects only
        # the last master write (+PERTURBATION_STEPS); chip-side internal
        # applies bypass the CT hook. So we check via drift_ppm, not via
        # `read_clock_trim()`.
        m_recovered = c.measure(count=128)
        bound = 1.5 * c.ppm_per_step  # ±step margin: deadband + EMA slack
        print(f"  recovered drift = {m_recovered.drift_ppm:+.0f} ppm "
              f"(bound ±{bound:.0f})", flush=True)
        assert abs(m_recovered.drift_ppm) <= bound, (
            f"chip didn't recover from +{PERTURBATION_STEPS}-step perturb in "
            f"{RECOVERY_PINGS} pings: pre={m_pre.drift_ppm:+.0f}, "
            f"post-perturb={m_perturbed.drift_ppm:+.0f}, "
            f"recovered={m_recovered.drift_ppm:+.0f}, bound=±{bound:.0f}"
        )
    finally:
        c.write_clock_trim(initial_trim)
        time.sleep(0.05)
        c.write_clock_fine_trim_us(initial_fine)
        time.sleep(0.05)
