"""End-to-end validation of master-side HSI calibration.

Exercises the chip's CAL handler + master-side §7 math + apply-after-TC path
together. Converges from the test-start `clock_trim` to a steady state where
step=0 holds, then asserts:
  - one more cycle still yields step=0 (idempotency),
  - the residual_ppm sits in [0, ppm_per_step) (biased-rounding invariant).

Per docs/dxl-hsi-calibration.md §7. Does not assert any particular HSITRIM
value — the per-chip optimum depends on factory variation.
"""

import time

import pytest

from cal import Calibrator

MAX_CYCLES = 6


@pytest.fixture
def calibrator(pirate, osc_id, baud):
    return Calibrator(pirate, dxl_id=osc_id, baud=baud)


def _converge(c: Calibrator) -> tuple[int, list[tuple[int, float, int]]]:
    """Run cycles until two consecutive step==0 results. Returns
    (final_trim, history) where rows are (trim_before, drift_ppm, step)."""
    history: list[tuple[int, float, int]] = []
    consecutive_noop = 0
    for _ in range(MAX_CYCLES):
        trim_before = c.read_clock_trim()
        m = c.measure(count=128)
        d = c.derive(m, current_trim=trim_before)
        history.append((trim_before, m.drift_ppm, d.step))
        c.apply(d)
        if d.step == 0:
            consecutive_noop += 1
            if consecutive_noop >= 2:
                return c.read_clock_trim(), history
        else:
            consecutive_noop = 0
    pytest.fail(f"cal did not converge within {MAX_CYCLES} cycles; history: {history}")


def test_master_cal_converges_idempotently(calibrator):
    c = calibrator
    initial_trim = c.read_clock_trim()
    initial_fine = c.read_clock_fine_trim_us()

    try:
        final_trim, history = _converge(c)

        print(f"\n  baseline trim={initial_trim}, ppm_per_step={c.ppm_per_step}")
        for i, (t, d, s) in enumerate(history):
            print(f"  cycle {i}: trim={t:+d}  drift={d:+6.0f} ppm  step={s:+d}")
        print(f"  converged at clock_trim={final_trim:+d}")

        m = c.measure(count=128)
        d = c.derive(m, current_trim=final_trim)
        print(f"  idempotency shot: drift={m.drift_ppm:+.0f} ppm  step={d.step:+d}  "
              f"resid_ppm={d.residual_ppm:+.0f}  resid_q88={d.residual_q88:+d}")
        assert d.step == 0, (
            f"non-zero step ({d.step:+d}) on idempotency check after converging "
            f"to clock_trim={final_trim:+d} (drift={m.drift_ppm:+.0f} ppm)"
        )
        assert 0 <= d.residual_ppm < c.ppm_per_step, (
            f"residual_ppm {d.residual_ppm:+.0f} outside [0, {c.ppm_per_step}) — "
            "biased rounding broken?"
        )
    finally:
        c.write_clock_trim(initial_trim)
        time.sleep(0.05)
        c.write_clock_fine_trim_us(initial_fine)
        time.sleep(0.05)
