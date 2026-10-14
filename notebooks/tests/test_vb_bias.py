"""The rev-2A VB bias node under drive: the bias plan's sec 2.2 rows, the
single-terminal error they cause, and the node's low pass."""

import numpy as np
import pytest

from oscnb.boards import BOARDS

B = BOARDS["dev-v006-2A"]
LSB = B.v_adc_per_count
K = 1 / B.div_ratio
VB0 = B.vb_nom_counts
C_V = B.bias_top * B.bias_bot / (B.bias_top + B.bias_bot) / (B.vmotor_div_top + B.vmotor_div_bot)


def taps_at(vr, d):
    """The settled node with terminal A at the rail and B low, and the crest taps
    it gives (the sum of terminals is D x Vr)."""
    vb = (VB0 + C_V * d * vr / LSB) / (1 + 2 * C_V)
    return vb, K * vr / LSB + (1 - K) * vb, (1 - K) * vb


@pytest.mark.parametrize("vr, row", [
    (4.5, [-12.4, -7.9, -5.7, -1.2, 10.0, 21.2, 26.5, 32.4]),
    (8.4, [-12.4, -4.0, 0.1, 8.5, 29.4, 50.2, 60.3, 71.1]),
])
def test_vb_shift_matches_the_430_100_rows(vr, row):
    for d, want in zip([0, 0.10, 0.15, 0.25, 0.50, 0.75, 0.87, 1.00], row):
        vb, a, b = taps_at(vr, d)
        got = B.vb_steady(VB0, d, a, b)
        assert got == pytest.approx(vb, abs=1e-9)
        assert (got - VB0) * LSB * 1e3 == pytest.approx(want, abs=0.06)


@pytest.mark.parametrize("vr, err_mv", [(4.5, 129.4), (8.4, 284.5)])
def test_one_terminal_against_the_boot_bias_reads_four_times_the_shift(vr, err_mv):
    _, a, b = taps_at(vr, 1.0)
    live = B.vb_steady(VB0, 1.0, a, b)
    assert B.term_v(a, live) == pytest.approx(vr, abs=1e-9)
    assert (B.term_v(a, VB0) - vr) * 1e3 == pytest.approx(err_mv, abs=0.3)


def test_vb_follows_through_a_0_8_ms_low_pass():
    _, a, b = taps_at(8.4, 1.0)
    target = B.vb_steady(VB0, 1.0, a, b)
    live = B.vb_live(VB0, 1.0, np.full(17, a), np.full(17, b))
    assert (live[0] - VB0) / (target - VB0) == pytest.approx(0.060, abs=0.001)
    assert (live[15] - VB0) / (target - VB0) == pytest.approx(1 - np.exp(-1), abs=0.01)
