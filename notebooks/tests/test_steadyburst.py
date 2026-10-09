"""The steady-burst loader against bursts built with a known answer, and
against the checked-in current-sense datasets where the answer is a count."""

import gzip
from pathlib import Path

import numpy as np
import pytest

from oscnb import datasets, steadyburst as sb

HDR = ["k", "code", "pre_q15", "step_q15", "step_index", "start_cnt", "pwm_arr", "start_dir",
       "restore_dir", "vbus_raw", "bias", "chans", "frame_len", "vmotor_bias", "pos", "seated",
       "drive_polarity"]


def synth(tmp_path, w=120, level=300.0, zero=1160.0, start_cnt=139, start_dir=1, n=960,
          edge_tau=0.0, edge_amp=0.0, slope=0.0, name="burst-0.csv.gz", fold_wrong=False):
    """A chans-0 burst of a window of w ticks a side: zero outside, level + slope
    x tau inside, plus edge_amp x exp(-tau / edge_tau) after the ON compare.
    `fold_wrong` stamps the burst with the other counter direction, the
    misfold the firmware race produced."""
    meta = dict(start_cnt=start_cnt, start_dir=start_dir)
    k = np.arange(n)
    t = sb.tau_on(meta, k)
    tau = t + w
    on = (t >= -w) & (t <= w)
    code = np.full(n, zero)
    x = level + slope * tau
    if edge_tau:
        x = x + edge_amp * np.exp(-tau / edge_tau)
    code[on] += x[on]
    q = round(w / sb.HALF * 32768)
    stamp_dir = 1 - start_dir if fold_wrong else start_dir
    first = [0, f"{code[0]:.0f}", q, q, 485, start_cnt, 1200, stamp_dir, stamp_dir, 1047, 1160, 0, 1, 764, 2015, 0, 1]
    p = tmp_path / name
    with gzip.open(p, "wt") as fh:
        fh.write(",".join(HDR) + "\n")
        fh.write(",".join(str(v) for v in first) + "\n")
        for i in range(1, n):
            fh.write(f"{i},{code[i]:.6f}\n")
    return p


def test_tau_on_spans_the_period():
    t = sb.tau_on(dict(start_cnt=139, start_dir=1), np.arange(960))
    assert t.min() >= -sb.PERIOD // 2 and t.max() < sb.PERIOD // 2
    # 52 ticks a sample against a 2400-tick period: 600 distinct phases, 4 ticks apart
    assert len(np.unique(t[:600])) == 600
    assert set(np.diff(np.unique(t))) == {4}


def test_flat_window_reads_its_level(tmp_path):
    b = sb.burst(synth(tmp_path, w=120, level=300.0))
    assert b["w"] == pytest.approx(120, abs=0.02)
    assert b["zero"] == pytest.approx(1160.0)
    assert np.allclose(b["crest"], 300.0)
    a, s, sd = b["fit"]
    assert a == pytest.approx(300.0) and s == pytest.approx(0.0, abs=1e-9)
    assert max(abs(r) for _, r in b["resid"]) < 1e-6
    # the crest samples sit at w + 31 ticks after the ON compare
    assert all(abs(t - (120 + sb.CREST)) <= 6 for t, x in b["on"] if abs(t - 151) <= 6)


def test_edge_shows_in_the_residual_and_late_line(tmp_path):
    b = sb.burst(synth(tmp_path, w=150, level=200.0, slope=0.05, edge_tau=15.0, edge_amp=60.0))
    a, s, _ = b["fit"]
    assert s == pytest.approx(0.05, abs=0.01)
    r58 = np.mean([r for t, r in b["resid"] if abs(t - 58) <= 2])
    assert r58 == pytest.approx(60.0 * np.exp(-58 / 15.0), abs=1.0)
    assert sb.resid_at(b, 58, tol=2) == pytest.approx(r58 / (a + s * 58))


def test_phase_check_flags_a_misfolded_stamp(tmp_path):
    good = [sb.burst(synth(tmp_path, start_cnt=100 + 37 * i, name=f"burst-{i}.csv.gz")) for i in range(6)]
    bad = sb.burst(synth(tmp_path, start_cnt=480, fold_wrong=True, name="burst-9.csv.gz"))
    hold = sb.phase_drops(good + [bad])
    assert [b["phase_bad"] for b in hold] == [False] * 6 + [True]
    assert all(abs(b["delta"]) < 6 for b in good)
    assert abs(bad["delta"]) > 100


def test_score_on_a_flat_load_reads_no_error(tmp_path):
    bs = []
    for D in (6, 8, 10, 15):
        for i in range(3):
            b = sb.burst(synth(tmp_path, w=D * 12, level=10.0 * D, start_cnt=100 + 41 * i, name=f"b{D}-{i}.csv.gz"))
            b["D"], b["stop"] = D, "low"
            bs.append(b)
    r = sb.score(bs)
    assert r["floors"] == {2: 40, 1: 40}
    assert all(abs(r["E"](w)) < 1e-6 for w in sb.WS)


def test_hold_dirs_parse_both_namings(tmp_path):
    for n in ("4.5_8_high", "8.4_15_low", "4.5_3.5", "zero_1"):
        (tmp_path / n).mkdir()
    got = [(r, p, s) for r, p, s, _ in sb.hold_dirs(tmp_path)]
    assert got == [(4.5, 3.5, None), (4.5, 8.0, "high"), (8.4, 15.0, "low")]


# ---- the checked-in data: counts the analysis leans on

MOT = "mg90-a__dev-v006-2A__dps__current-sense"
RES = "res-15r3__dev-v006-2A__dps__current-sense"


def _drops(key, exp, caps):
    ds = datasets.load(key)
    out = {}
    for c in caps:
        n = bad = 0
        for *_, d in sb.hold_dirs(ds.path / exp / c):
            for b in sb.hold_bursts(d):
                n += 1
                bad += b["phase_bad"]
        out[c] = (bad, n)
    return out


def test_resistor_phase_check_drops_two_of_880():
    d = _drops(RES, "ladder", ["capture-1", "capture-2", "capture-3", "capture-4"])
    assert sum(b for b, _ in d.values()) == 2
    assert sum(n for _, n in d.values()) == 880


def test_final_network_repeats_drop_counts():
    d = _drops(MOT, "floor", ["capture-8", "capture-9", "capture-10"])
    assert d == {"capture-8": (2, 140), "capture-9": (2, 140), "capture-10": (0, 140)}
