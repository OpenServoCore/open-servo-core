"""The I2t replay against the kernel's own arithmetic and closed forms."""

import numpy as np
import pytest

from oscnb import i2t


def test_tau_shift_rounds_to_the_nearest_octave_and_caps():
    assert i2t.TAU_SHIFT_MAX == 14
    assert i2t.HEAT_GUARD == 5
    assert i2t.HEAT_MAX == (2 ** 31 - 1) >> 5
    assert i2t.tau_shift(0) == 0
    assert i2t.tau_shift(1) == 6                       # 62 periods, nearer 64 than 32
    assert i2t.tau_shift(4) == 8                       # the firmware's boot default, 250 periods
    assert i2t.tau_shift(66) == 12
    assert i2t.tau_shift(115) == 13                    # 7187 periods, nearer 8192 than 4096
    assert i2t.tau_shift(262) == 14
    assert i2t.tau_shift(65535) == i2t.TAU_SHIFT_MAX   # the u16 thermal_tau_s ceiling


def test_heat_is_the_kernels_shifted_ewma():
    h = i2t.heat([100, 100, 100, 0, 0], 1)
    # x_qg = 10000 << 5 = 320000; h += (x - h) >> 1
    assert h.tolist() == [5000, 7500, 8750, 4375, 2187]      # 70000 >> 5 = 2187 (floor)


def test_heat_at_the_longest_leak_still_moves():
    # the kernel's heat_at_the_longest_leak_still_moves: a hold at 100 counts heats to within
    # 512 counts^2 of its square, and rest leaks back to within the same band of 0
    n = 20 << i2t.TAU_SHIFT_MAX
    h = i2t.heat(np.r_[np.full(n, 100), np.zeros(n, dtype=int)], i2t.TAU_SHIFT_MAX)
    assert 10_000 - 512 <= h[n - 1] <= 10_000
    assert h[-1] <= 512


def test_heat_caps_the_square():
    h = i2t.heat([32767], 0)                          # the largest i16 current the kernel can hold
    assert h[0] == i2t.HEAT_MAX


def test_rise_is_heat_times_k_over_q16():
    assert i2t.rise_cc([65536], 1).tolist() == [1]
    assert i2t.rise_cc([76729], i2t.k_q16_of(0.025)).tolist() == [(76729 * 1638) >> 16]


def test_rise_saturates_at_i16_max():
    assert i2t.rise_cc([i2t.HEAT_MAX], 65535).tolist() == [i2t.RISE_MAX]
    assert i2t.rise_cc([65536], 32767).tolist() == [32767]
    assert i2t.rise_cc([65536], 32766).tolist() == [32766]


def test_ewma_settles_on_its_input_at_one_tau_to_63_percent():
    y = i2t.ewma(np.ones(10000), 1.0, 0.001)
    assert y[999] == pytest.approx(1 - np.exp(-1), abs=0.002)


def test_grid_holds_the_last_poll():
    tg, ig = i2t.grid([0.0, 1.0, 2.5], [1, 2, 3], 0.0, 3.0, dt=0.5)
    assert ig.tolist() == [1, 1, 2, 2, 2, 3]


def test_fit_k_recovers_a_proportional_truth():
    h = np.array([1e4, 2e4, 3e4])
    k, sd = i2t.fit_k(h, 0.02 * h)
    assert k == pytest.approx(0.02)
    assert sd == pytest.approx(0.0)


def test_k_physical_is_rth_r_i2_in_centi_c():
    # 30 C/W x 4.6 ohm x (0.0009 A/count)^2 = 0.1118 mC/count^2 -> 0.0112 cC/count^2
    assert i2t.k_physical(30, 4.6, 0.0009) == pytest.approx(100 * 30 * 4.6 * 0.0009 ** 2)
