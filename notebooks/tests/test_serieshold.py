"""The series-meter hold loader against a hold with a known answer, and
against one checked-in R1 hold."""

import numpy as np
import pandas as pd
import pytest

from oscnb import boards, datasets, meterlog as ml, serieshold as sh


def hold_frame():
    t = 1000.0 + 0.025 * np.arange(400)
    phase = ["pre"] + ["seek650"] * 39 + ["hold650"] * 300 + ["rest650"] * 60
    on = np.array([p == "hold650" for p in phase])
    return pd.DataFrame({
        "t_unix": t, "phase": phase,
        "current": np.where(on, 1180 + 223.6, 1180),        # 200 mA over the trough at 1118 counts/A
        "current_trough": np.where(on, 1180 + 0.0, 1180),
        "i_hat": np.where(on, -224, 0),
        "duty_applied_q15": np.where(on, -6554, 0),
        "vdiff_mean": np.where(on, -1000, 0),
        "vbus_raw": 1000, "limit_flags": 8, "pos": 240,
    })


def test_holds_and_rest_split_by_phase():
    df = hold_frame()
    h = sh.holds(df)
    assert list(h) == ["hold650"] and len(h["hold650"]) == 300
    assert len(sh.rest_after(df, "hold650")) == 60


def test_score_against_a_known_meter():
    df = hold_frame()
    rows = sh.holds(df)["hold650"]
    tm = np.arange(1000.0, 1010.0, 0.5)
    vm = np.full(len(tm), -0.190)                             # the meter reads 190 mA, negative lead
    s = sh.score(rows, (tm, vm), skip_s=3.0, counts_per_a=1118.0, v_per_count=0.004)
    assert s["D"] == pytest.approx(0.2, abs=1e-4)
    assert s["vterm"] == pytest.approx(4.0)
    assert s["crest_mA"] == pytest.approx(200.0, abs=0.01)
    assert s["meter_mA"] == pytest.approx(190.0)
    assert s["meter_sign"] == -1
    assert s["gap_mA"] == pytest.approx(10.0, abs=0.01)
    # only the meter rows inside [first row + 3 s, last row] count
    t0 = rows["t_unix"].iloc[0]
    assert s["n_meter"] == int(((tm >= t0 + 3) & (tm <= rows["t_unix"].iloc[-1])).sum())


def test_one_r1_hold_from_the_dataset():
    ds = datasets.load("mg90-a__dev-v006-2A__dps__current-sense")
    d = ds.path / "gap" / "capture-1"
    df = sh.read(d / "r1_8.4_20_low.csv.gz")
    s = sh.score(sh.holds(df)["hold650"], ml.read(d / "dmm.csv", func="DC A"),
                 v_per_count=boards.BOARDS["dev-v006-2A"].v_term_per_count)
    assert s["D"] == pytest.approx(0.2, abs=1e-3)
    assert s["n_meter"] == 24 and s["meter_sign"] == 1
    assert s["gap_mA"] == pytest.approx(32.4, abs=0.1)
