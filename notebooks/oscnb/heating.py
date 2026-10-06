"""The heating-curve bench: heater blocks with their cool-downs and a locked-rotor
hold, a thermocouple on the can, `osc ident burst` runs for the winding R.

A dataset of this kind holds one folder per block, named in `blocks.csv`
(capture, source, label, heater, limit_counts, heat_phase, cool_phase,
r_ref_ohm, t_ref_c, r_ref_from, runs, ended):

  temp.csv            the thermocouple every 5 s with the heater's 5 s current
                      statistics; `can_c` is the can, `ntc_raw` is the board's
                      dead NTC channel
  heater.csv.gz       every heater poll: pos, duty or goal offset, i_hat, fault
  bursts.csv          one row per `osc ident burst` run, as the driver logged it
  hold.csv.gz         the locked-rotor hold's polls with the ident aggregates
  ident/<unix>/       the run's burst-*.csv.gz captures and its report.log.gz

  blocks(ds)                           the index
  can_log(ds, capture)                 the thermocouple rows of one capture, glitches marked
  heater(ds, capture)                  the heater polls, with a unix column
  bursts(ds, capture)                  the runs, with a unix column where the log had none
  hold(ds, capture)                    the hold polls, R from the aggregates through the board
  report(path)                         the waveform and line results in a run's report
  heat_windows(polls, board, bin_s)    (start, end, I^2) windows for thermal.TwoNode.run
  step(t, y, kind)                     one exponential through a heating or cooling curve

THE HEAT

The heater's power is read off its own current polls. Over each `bin_s` of
wall time the mean of i_hat^2 (in A^2 through the board's amps per count) is
one window; the gaps where no poll landed (the burst runs) carry no heat. The
model multiplies I^2 by R(T_w) itself, so the windows are current, not power.

THE AGGREGATE R

The firmware's identification aggregates fold the ON-window shunt current, the
terminal difference and the applied duty. Their ratio

  R_agg = (duty_mean x vdiff_mean / 32767) x V_per_count / (|i_mean| x A_per_count)

is the `ident resistance` route. It takes the commanded window for the drive
pulse, so its level is not the burst's; its ratio over a hold is what reads.
"""

import gzip
import re
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import curve_fit

Q15 = 32767.0

WAVE = re.compile(r"waveform\s+R ([\d.]+) \+/- ([\d.]+) ohm \(winding\), L ([\d.]+) mH, V0 (-?[\d.]+) V; (\d+) of (\d+) captures")
LINE = re.compile(r"slope ([\d.]+) ohm with the bridge, V/I ([\d.]+) ohm at the ([\d.]+) A")
VERDICT = re.compile(r"verdict\s+(\S+)")


def blocks(ds):
    return pd.read_csv(ds.path / "blocks.csv")


def _unix0(ds, capture):
    """The script's unix time at t_s = 0, from the rows that carry both."""
    p = ds.path / capture
    for name in ("temp.csv", "bursts.csv"):
        f = p / name
        if f.exists():
            t = pd.read_csv(f)
            if "unix" in t and len(t):
                return float((t.unix - t.t_s).median())
    # the hold logs no unix: the first run folder under ident/ is the first burst's start
    b = pd.read_csv(p / "bursts.csv")
    runs = sorted(int(d.name) for d in (p / "ident").iterdir() if d.is_dir())
    return runs[0] - float(b.t_s.iloc[0])


def can_log(ds, capture, glitch_c=3.0):
    """temp.csv with `can_c` numeric and a `glitch` flag on readings more than
    `glitch_c` off the median of their 9 neighbours (the BLE link drops a digit
    now and then)."""
    t = pd.read_csv(ds.path / capture / "temp.csv")
    t["can_c"] = pd.to_numeric(t.can_c, errors="coerce")
    med = t.can_c.rolling(9, center=True, min_periods=1).median()
    t["glitch"] = t.can_c.isna() | ((t.can_c - med).abs() > glitch_c)
    return t


def heater(ds, capture):
    with gzip.open(ds.path / capture / "heater.csv.gz", "rt") as fh:
        h = pd.read_csv(fh)
    if len(h):
        h["unix"] = h.t_s + _unix0(ds, capture)
    return h


def bursts(ds, capture):
    b = pd.read_csv(ds.path / capture / "bursts.csv")
    b["can_c"] = pd.to_numeric(b.can_c, errors="coerce")
    if "unix" not in b:
        b["unix"] = b.t_s + _unix0(ds, capture)
    if "dir" not in b:                          # the hold names no folder: run order is folder order
        runs = sorted(int(d.name) for d in (ds.path / capture / "ident").iterdir() if d.is_dir())
        b["dir"] = [str(r) for r in runs[:len(b)]]
    return b


def hold(ds, capture, board):
    """hold.csv.gz with `r_agg` recomputed from the aggregates through the
    board's constants (the script's own column stays as `r_agg_ohm`) and a
    unix column."""
    with gzip.open(ds.path / capture / "hold.csv.gz", "rt") as fh:
        h = pd.read_csv(fh)
    h["can_c"] = pd.to_numeric(h.can_c, errors="coerce")
    h["unix"] = h.t_s + _unix0(ds, capture)
    sgn = np.sign(h.duty_mean)
    v = h.duty_mean * h.vdiff_mean / Q15 * board.v_term_per_count
    i = h.i_mean * sgn * board.a_per_count
    h["i_a"] = i
    h["r_agg"] = np.where(i > 50 * board.a_per_count, v / i.where(i != 0), np.nan)
    return h


def report(path):
    """The waveform fit and the line of a run's report.log.gz: wave_r_ohm,
    wave_r_sd_ohm, l_mh, v0_v, kept, of, slope_ohm, v_over_i_ohm, i_limit_a,
    verdict; NaN where the report has no such line."""
    path = Path(path)
    if path.is_dir():
        path = path / "report.log.gz"
    with gzip.open(path, "rt") as fh:
        txt = fh.read()
    w, ln, vd = WAVE.search(txt), LINE.search(txt), VERDICT.search(txt)
    out = dict(wave_r_ohm=np.nan, wave_r_sd_ohm=np.nan, l_mh=np.nan, v0_v=np.nan, kept=0, of=0,
               slope_ohm=np.nan, v_over_i_ohm=np.nan, i_limit_a=np.nan, verdict="")
    if w:
        out.update(wave_r_ohm=float(w[1]), wave_r_sd_ohm=float(w[2]), l_mh=float(w[3]), v0_v=float(w[4]),
                   kept=int(w[5]), of=int(w[6]))
    if ln:
        out.update(slope_ohm=float(ln[1]), v_over_i_ohm=float(ln[2]), i_limit_a=float(ln[3]))
    if vd:
        out["verdict"] = vd[1]
    return out


def heat_windows(polls, board, bin_s=5.0, gap_s=1.0):
    """(start, end, I^2) windows in unix seconds from the heater polls: the mean
    of i_hat^2 over each bin of wall time, bins the heater did not poll in
    (the burst runs) left out. Also returns the heater's on-time in s, the sum
    of poll gaps shorter than `gap_s`."""
    if len(polls) == 0:
        return [], 0.0
    i2 = (polls.i_hat.to_numpy() * board.a_per_count) ** 2
    t = polls.unix.to_numpy()
    b = np.floor((t - t[0]) / bin_s).astype(int)
    df = pd.DataFrame({"b": b, "i2": i2, "t": t})
    g = df.groupby("b").agg(i2=("i2", "mean"), t0=("t", "min"), t1=("t", "max"), n=("t", "size"))
    g = g[g.n >= 3]
    windows = [(float(r.t0), float(max(r.t1, r.t0 + 0.5)), float(r.i2)) for r in g.itertuples()]
    dt = np.diff(t)
    return windows, float(dt[dt < gap_s].sum())


def _heat(t, ta, dt, tau):
    return ta + dt * (1 - np.exp(-t / tau))


def _cool(t, ta, dt, tau):
    return ta + dt * np.exp(-t / tau)


def step(t, y, kind):
    """One exponential through a curve: `kind` 'heat' is ta + dT (1 - e^(-t/tau)),
    'cool' is ta + dT e^(-t/tau), t from the curve's own start. Returns a dict
    with ta, dt, tau, their standard errors (ta_se, dt_se, tau_se) and the rms
    of the residual."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    f = _heat if kind == "heat" else _cool
    p0 = [y[0], y[-1] - y[0], (t[-1] - t[0]) / 4] if kind == "heat" else [y[-1], y[0] - y[-1], (t[-1] - t[0]) / 4]
    p, cov = curve_fit(f, t, y, p0=p0, maxfev=20000)
    se = np.sqrt(np.diag(cov))
    return dict(ta=p[0], dt=p[1], tau=p[2], ta_se=se[0], dt_se=se[1], tau_se=se[2],
                rms=float(np.std(y - f(t, *p))))
