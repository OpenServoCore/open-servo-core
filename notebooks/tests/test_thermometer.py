"""The shipped-thermometer loaders against what the bench scripts printed from
the same files, and the ident thermal fit port against the tool's own report
and a curve with a known answer."""

import re

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, datasets, thermometer as tm

DS = datasets.load("mg90-a__dev-v006-2A__dps__thermometer")
BOARD = boards.BOARDS["dev-v006-2A"]
W = BOARD.v_term_per_count * BOARD.a_per_count
IX = tm.index(DS).set_index("capture")


def score_rows(capture):
    """The per-gap table score_r3.py / score_r4.py printed into score.out."""
    lines = (DS.path / capture / "score.out").read_text().splitlines()
    k = next(i for i, ln in enumerate(lines) if ln.split()[:2] == ["gap", "gap_s"])
    hdr = lines[k].split()
    rows = [dict(zip(hdr, map(float, ln.split()))) for ln in lines[k + 1:] if re.match(r"^\s+\d", ln)]
    return pd.DataFrame(rows)


def test_every_capture_in_the_index_is_a_folder():
    for c in IX.index:
        p = DS.path / c
        assert p.is_dir(), c
        assert any(p.glob("*.csv.gz")), c
    assert set(IX.loc[IX.unix0.notna()].index) == {"thermal/capture-1", "thermal/capture-2"}


@pytest.mark.parametrize("capture,offset", [("oab/capture-1", -2.75), ("oab/capture-2", -2.03)])
def test_the_rest_offset_is_the_scorers(capture, offset):
    m = tm.meter(DS, capture)
    assert m.attrs["dropped"] == 0
    r = tm.rest_offset(tm.th_log(DS, capture, "rest"), m)
    assert r["offset"] == pytest.approx(offset, abs=0.005)
    assert r["n"] == 360


@pytest.mark.parametrize("capture,which", [("oab/capture-1", "last"), ("oab/capture-2", "first")])
def test_the_gap_table_reproduces_the_bench_score(capture, which):
    # run 3's scorer took each hold's last base, run 4's the first; both rules
    # are the same function
    m = tm.meter(DS, capture)
    off = tm.rest_offset(tm.th_log(DS, capture, "rest"), m)["offset"]
    g = tm.gaps(tm.th_log(DS, capture, "oab"), m, off, which)
    s = score_rows(capture)
    assert list(g.gap) == list(s.gap.astype(int))
    assert np.allclose(g.gap_s, s.gap_s)
    assert np.allclose(g.carried, s.carried, atol=0.005)
    assert np.allclose(g.t_star, s.T_star, atol=0.005)
    assert np.allclose(g.carry_m_t, s.carry_m_Tstar, atol=0.006)
    if "truth" in s:
        assert np.allclose(g.ratio, s.truth, atol=0.006)


def test_bases_and_exits_find_run_4_hold0():
    log = tm.th_log(DS, "oab/capture-2", "oab")
    b = tm.bases(log)
    h0 = b[b.hold == "hold0"]
    assert list(h0.r_hat) == [3209, 3304, 4137]
    assert h0.t_s.iloc[0] == pytest.approx(2.72, abs=0.01)
    e = tm.exits(log)
    assert list(e.t_s.round(2)) == [11.16, 24.27]
    assert list(e.moved) == ["none", "duty"]


def test_ident_run_puts_the_three_files_on_one_clock():
    th, sn, win = tm.ident_run(DS, "thermal/capture-1", IX.loc["thermal/capture-1", "unix0"])
    assert len(th) > 20000 and len(sn) > 20000 and len(win) > 20000
    hold = sn[(sn.mode_active == 0) & (sn.duty_applied_q15 < 0) & (sn.pos < 400)]
    # the fit's rows lie inside the hold's polls
    assert hold.t_unix.min() - 1 < th.t_unix.min() and th.t_unix.max() < hold.t_unix.max() + 1
    assert (np.diff(win.t_unix) >= 0).all()


def test_the_free_fit_reproduces_the_tools_report():
    th, _, _ = tm.ident_run(DS, "thermal/capture-1", IX.loc["thermal/capture-1", "unix0"])
    f = tm.fit_free(th, W)
    rep = (DS.path / "thermal/capture-1" / "report.out").read_text()
    alpha = int(re.search(r"th_alpha_q24\s+fitted\s+(\d+)", rep)[1])
    g = int(re.search(r"th_g_q016\s+fitted\s+(\d+)", rep)[1])
    assert (f["alpha_q24"], f["g_q016"]) == (alpha, g)
    assert (f["chunks"], f["rejected"]) == (230, 2)


def test_the_tau_fixed_fit_recovers_a_one_node_curve():
    # x rises on tau toward g P with nothing else in it: both forms are exact
    tau, g = 128.0, 0.0229
    t = np.arange(0, 480, 1 / tm.SLOW_HZ)
    p = 74000.0 * (1 + 0.02 * np.sin(t / 7))       # a power that wanders a little, as a hold's does
    x = np.zeros_like(t)
    for k in range(1, len(t)):
        x[k] = x[k - 1] + (t[k] - t[k - 1]) / tau * (g * p[k] - x[k - 1])
    rows = pd.DataFrame(dict(t_s=t, x_cc=x, p=p, flags=tm.TRACK))
    f = tm.fit_g(rows, tau, W)
    assert f["g"] == pytest.approx(g, rel=0.01)
    ff = tm.fit_free(rows, W)
    assert ff["tau"] == pytest.approx(tau, rel=0.02)
    assert ff["g"] == pytest.approx(g, rel=0.01)


def test_calib_fields_reads_the_wrote_block_and_register_reads():
    f = tm.calib_fields((DS.path / "anchor-m1.out").read_text())
    assert (f["th_alpha_q24"], f["th_g_q016"], f["r0_q12"], f["ntc_k2_q24"]) == (2097, 1500, 3932, 5779)
    assert tm.calib_fields((DS.path / "r0-write.out").read_text())["r0_q12"] == 4078


def test_can_at_is_a_windowed_median():
    m = pd.DataFrame(dict(t_unix=[0.0, 1.0, 2.0, 3.0, 10.0], can_c=[20.0, 21.0, 22.0, 30.0, 40.0]))
    assert tm.can_at(m, 1.5, w=1.5) == pytest.approx(21.5)
    assert np.isnan(tm.can_at(m, 6.0, w=1.5))
    assert list(tm.can_at(m, [1.0, 10.0], w=1.5)) == [21.0, 40.0]
