"""The meter-log loader against the meter's own packets and against series
with a known answer."""

import numpy as np
import pandas as pd
import pytest

from oscnb import datasets, meterlog as ml

DS = datasets.load("mg90-a__dev-v006-2A__dps__bare-motor")
LOGS = ["char/dmm.csv", "char/dmm2.csv", "char/dmm3.csv", "two-current/dmm.csv", "hand-map/dmm.csv", "hand-map/dmm2.csv"]


@pytest.mark.parametrize("log", LOGS)
def test_every_packet_decodes_to_the_logged_value(log):
    d = ml.frame(DS.path / log)
    d = d[d.raw.notna()]
    assert len(d) > 200
    for r in d.itertuples():
        func, val = ml.decode(r.raw)
        assert func == r.func
        assert val == pytest.approx(r.value, rel=1e-9, abs=1e-9), r.raw


def test_decode_known_packets():
    assert ml.decode("22f10400e001") == ("ohm", pytest.approx(4.80))     # scale 4 = ohm, 2 decimals
    assert ml.decode("2af10400a125")[1] == pytest.approx(96330.0)        # scale 5 = kohm
    name, v = ml.decode("22f104008080")                                  # bit 15 = negative
    assert v == pytest.approx(-1.28)


def test_read_sorts_and_keeps_one_function():
    t, v = ml.read(DS.path / "char/dmm.csv", DS.path / "char/dmm3.csv")
    assert (np.diff(t) >= 0).all()
    assert len(t) == len(ml.frame(DS.path / "char/dmm.csv")) + len(ml.frame(DS.path / "char/dmm3.csv"))


def test_settle_reads_its_windows():
    t = np.arange(0, 70, 0.5)
    v = 4.6 + 0.2 * np.exp(-t / 20.0)
    v[10] = 0.0                                                          # an open-lead glitch, left out
    s = ml.settle(t + 1000.0, v, 1000.0)
    assert s["first_s"] == pytest.approx(np.median(v[(t >= 0.5) & (t < 1.5)]))
    assert s["rest"] == pytest.approx(np.median(v[(t >= 50) & (t < 60)]))
    assert s["slope_per_min"] < 0


def test_stills_finds_the_flat_stretches():
    t = np.arange(0, 100, 0.5)
    v = np.where(t < 40, 4.40, 4.90) + 0.001 * np.sin(t)
    s = ml.stills(t, v, min_s=20.0, tol=0.03)
    assert len(s) == 2
    assert s.v_start.tolist() == pytest.approx([4.40, 4.90], abs=0.002)
    assert s.v30.notna().all()
