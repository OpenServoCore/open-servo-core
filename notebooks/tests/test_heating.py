"""The heating-curve loaders against the dataset's own cross-checks and against
curves with a known answer."""

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, datasets, heating as hc

DS = datasets.load("mg90-a__dev-v006-2A__2s__heating")
BOARD = boards.BOARDS["dev-v006-2A"]


def test_every_block_in_the_index_is_a_folder_with_its_runs():
    for b in hc.blocks(DS).itertuples():
        p = DS.path / b.capture
        runs = [d for d in (p / "ident").iterdir() if d.is_dir()]
        assert len(runs) == b.runs, b.capture
        assert len(pd.read_csv(p / "bursts.csv")) == b.runs, b.capture
        for r in runs:
            assert (r / "report.log.gz").exists()
            assert len(list(r.glob("burst-*.csv.gz"))) >= 8


def test_the_aggregate_r_through_the_board_is_the_scripts():
    h = hc.hold(DS, "hold/capture-1", BOARD)
    m = h.r_agg_ohm.notna() & h.r_agg.notna()
    assert m.sum() > 2000
    assert np.allclose(h.r_agg[m], h.r_agg_ohm[m], rtol=2e-4)


def test_the_report_reads_what_the_driver_logged():
    b = hc.bursts(DS, "dither/capture-1")
    for x in b.head(3).itertuples():
        r = hc.report(DS.path / "dither/capture-1" / "ident" / str(x.dir))
        assert r["wave_r_ohm"] == pytest.approx(x.wave_r_ohm, abs=1e-3)
        assert r["v_over_i_ohm"] == pytest.approx(x.v_over_i_ohm, abs=1e-3)
        assert r["verdict"] == x.verdict


def test_the_hold_bursts_get_unix_and_folders_in_order():
    b = hc.bursts(DS, "hold/capture-1")
    assert (np.diff(b.unix) > 0).all()
    assert [int(d) for d in b.dir] == sorted(int(d) for d in b.dir)
    # the folder's start sits within a few seconds of the row's own clock
    assert np.abs(b.unix - b.dir.astype(int)).max() < 5


def test_heat_windows_cover_the_heater_and_skip_the_bursts():
    polls = hc.heater(DS, "dither/capture-1")
    win, on_s = hc.heat_windows(polls, BOARD)
    wall = polls.unix.iloc[-1] - polls.unix.iloc[0]
    covered = sum(b - a for a, b, _ in win)
    assert 0.8 * wall < on_s < 0.9 * wall           # the burst runs are about 15% of the block
    assert abs(covered - on_s) < 0.05 * wall
    i2 = np.array([q for _, _, q in win])
    assert 0.2 ** 2 < np.median(i2) < 0.26 ** 2     # the 280-count limiter's 0.23 A rms


def test_step_recovers_a_synthetic_curve():
    t = np.arange(0, 1200, 5.0)
    rng = np.random.default_rng(3)
    y = 26.5 + 7.5 * (1 - np.exp(-t / 123.0)) + rng.normal(0, 0.05, t.size)
    f = hc.step(t, y, "heat")
    assert f["tau"] == pytest.approx(123.0, rel=0.03)
    assert f["dt"] == pytest.approx(7.5, rel=0.02)
    y = 27.0 + 11.0 * np.exp(-t / 130.0) + rng.normal(0, 0.05, t.size)
    f = hc.step(t, y, "cool")
    assert f["tau"] == pytest.approx(130.0, rel=0.03)
    assert f["ta"] == pytest.approx(27.0, abs=0.05)
