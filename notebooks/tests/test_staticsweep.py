"""The static-load sweep loader against the run log's own cross-checks and
against signals with a known answer."""

import numpy as np
import pytest

from oscnb import datasets, staticsweep as ss

DS = datasets.load("mg90-a__dev-v006-2A__dps__bare-motor")
CY = ss.cycles(DS)


def test_the_index_matches_the_captures():
    assert len(CY) == 100
    assert CY.captured.sum() == 96
    # the four failed sweeps left no capture, every other cycle did
    assert sorted(CY[~CY.captured].index) == sorted(CY[CY.rc != 0].index) == [8, 22, 28, 69]
    assert DS.board.key == "dev-v006-2A"


@pytest.mark.parametrize("k, roles", [(1, (6000, 6000, 28000)), (2, (6000, 6000, 10000)),
                                      (7, (6000, 6000, 10000)), (51, (0, 6000, 10000)), (63, (0, 6000, 0)),
                                      (31, (6000, 6000, 0))])
def test_segments_follow_the_schedule(k, roles):
    f = ss.frame(DS, k)
    s = ss.segments(f, CY.loc[k, "steps"])
    assert (len(s["stall"]), len(s["spin"]), len(s["coast"])) == roles
    assert len(f) == sum(roles)
    # the coast holds duty 0 once its drop lands; the stall is the 6% pre-pulse
    if roles[2]:
        assert (s["coast"].duty_q15.iloc[200:] == 0).all()
    if roles[0]:
        assert abs(s["stall"].duty_q15.iloc[-100:]).mean() == pytest.approx(0.06 * 32767, rel=0.01)


def test_the_runner_gates_are_the_spin_current():
    # the runner's nudge and J4 guard: spin current over the trough, second half of the spin
    for k in (1, 24, 51, 75):
        r = CY.loc[k]
        want = r.nudge_counts if r.nudge_counts == r.nudge_counts and r.nudge_counts is not None else r.spin_counts
        sp = ss.segments(ss.frame(DS, k), r.steps)["spin"]
        x = (sp.current_raw - sp.current_trough).to_numpy(float)
        assert x[len(x) // 2:].mean() == pytest.approx(want, abs=0.05), k


def test_count_cycles_and_line_freq_on_a_sweeping_line():
    t = np.arange(4000) / ss.FS
    f0, f1 = 900.0, 780.0                                    # the line sweeps 13% inside the window
    phase = 2 * np.pi * (f0 * t + (f1 - f0) * t ** 2 / (2 * t[-1]))
    x = 0.25 * np.sin(phase) + 0.01 * np.random.default_rng(1).normal(size=t.size)
    fpk, prom = ss.line_freq(x)
    assert 760 < fpk < 920 and prom > 10
    f, a, b, n = ss.count_cycles(x, fpk)
    # the whole cycles between a and b, against the true phase advance over the same span
    true = (phase[b] - phase[a]) / (2 * np.pi) / ((b - a) / ss.FS)
    assert f == pytest.approx(true, rel=2e-3)
    assert n > 100
