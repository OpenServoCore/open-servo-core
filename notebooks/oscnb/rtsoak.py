"""Winding R against a known can temperature: one servo in a dryer, a run of
`osc ident burst` at each temperature plateau, a thermocouple on the can.

A dataset of this kind holds one folder per plateau under `plateau/capture-N`,
named in `plateaus.csv` (capture, source, label, dryer_panel, unix_first_run,
runs):

  runs.csv            one row per `osc ident burst` run, as the soak script
                      logged it; `wave_r_ohm` is osc ident's waveform-fit R
  ntc.csv             the can temperature every 5 s; `can_c` is the
                      thermocouple on the meter, `ntc_raw` is empty
  regs.csv            torque-off register snapshots before the runs
  soak.log            the script's console
  ident/<unix>/       the run's burst-*.csv.gz captures and its report.log.gz

Beside them, `cool/capture-1` is a thermocouple log with no runs, `dc.csv` the
meter's ohms across the motor terminals, and `extras.csv` the bursts taken
around the soak (`cal/` and `polarity/`), each with the same burst captures.

  plateaus(ds), extras(ds), dc(ds)   the three index tables
  can_log(ds)                        every thermocouple reading, glitches marked
  runs(ds)                           every plateau run, the can at its start
  captures(folder, board)            one row per burst capture, the
                                     period-average R of each
  period_average(df, board)          the same for one capture

THE PERIOD-AVERAGE ESTIMATOR

The winding obeys V = R i + L di/dt + e. Averaged over whole PWM periods the
switching drops out, so over the late part of a burst, from t0 to t1,

  int V dt = R int i dt + L (i(t1) - i(t0)) + int e dt

The mean terminal voltage of a period is the effective duty times the driven
tap's step, its ON level less its brake level (both late, settled samples).
The effective duty is the window less the drive pulse's shortfall,
(w - 27 ticks) / 2400 (notebook 16 sec 2). The mean current of a period is the
shunt's settled ON samples fitted with a line and read at the window's centre,
where a symmetric ripple crosses its own mean. L is held at 0.81 mH, near what
osc ident fits for this motor. The back-EMF term is dropped: the rotor starts
from rest and the burst is half a millisecond long. What is left is

  R = (V_mean (t1 - t0) - L (i(t1) - i(t0))) / int i dt
"""

import gzip

import numpy as np
import pandas as pd

from . import opasweep as ow

CEN = 1208              # a sample's phase offset to the crest, as opasweep.burst_levels reads it

PULSE_SHORT = 27        # the drive pulse's shortfall in ticks, notebook 16 sec 2
L_H = 0.81e-3           # winding inductance held in the estimator, H
LATE_US = 200.0         # periods centred this long after the step enter the estimate
SETTLE_US = 3.5         # ON samples this soon after the window opens are amplifier edge
FROM_REST = 16          # captures 0..15 of a run step from rest; 16..23 from a hold


def plateaus(ds):
    return pd.read_csv(ds.path / "plateaus.csv")


def extras(ds):
    return pd.read_csv(ds.path / "extras.csv", dtype={"unix": int})


def dc(ds):
    return pd.read_csv(ds.path / "dc.csv")


def can_log(ds, glitch_c=5.0):
    """Every thermocouple reading in time order, from the plateaus and the
    cool-down. A reading more than `glitch_c` off the median of its 9
    neighbours is marked `glitch` (the BLE link drops a digit now and then)."""
    parts = [pd.read_csv(ds.path / "plateau" / c / "ntc.csv").assign(src=c)
             for c in plateaus(ds).capture]
    parts.append(pd.read_csv(ds.path / "cool" / "capture-1" / "ntc.csv").assign(src="cool"))
    t = pd.concat(parts)
    t["can_c"] = pd.to_numeric(t.can_c, errors="coerce")
    t = t.dropna(subset=["can_c"]).sort_values("unix").drop_duplicates("unix").reset_index(drop=True)
    med = t.can_c.rolling(9, center=True, min_periods=1).median()
    t["glitch"] = (t.can_c - med).abs() > glitch_c
    return t


def runs(ds):
    """Every run of every plateau, with the thermocouple interpolated to the
    run's start (`can_tc`). The runs.csv `can_c` column is the board's NTC
    channel, not the can."""
    log = can_log(ds)
    log = log[~log.glitch]
    out = []
    for p in plateaus(ds).itertuples():
        r = pd.read_csv(ds.path / "plateau" / p.capture / "runs.csv", dtype={"dir": str})
        r.insert(0, "plateau", p.label)
        r.insert(1, "capture", p.capture)
        r["can_tc"] = np.interp(r.unix, log.unix, log.can_c)
        out.append(r)
    return pd.concat(out, ignore_index=True)


def run_path(ds, capture, unix_dir):
    return ds.path / "plateau" / capture / "ident" / str(unix_dir)


def period_average(df, board, l_h=L_H, short=PULSE_SHORT, late_us=LATE_US, settle_us=SETTLE_US):
    """The period-average R of one burst capture, from the driven tap only.
    `i_mean` is the mean current over the periods used, `l_share` how much of
    R the L di/dt term carried (about 17% here, so a 10% error in L is 1.7%
    of R)."""
    us = ow.TICKS_PER_US
    period = 2 * board.pwm_half_ticks
    f = ow.burst_frame(df, board, CEN)
    d, w, code, post, k = f.d, f.w, f.code, f.post, f.k
    per = np.cumsum(np.r_[0, np.diff(d) < 0])          # period number: d wraps once a period
    t_us = (k - f.m["step_index"]) * ow.STEP_TICKS / us  # time from the step
    taps = [t for t in (f.ta, f.tb) if t.any()]        # one tap streamed (the driven one) or both
    on_s = (d >= -w / 2 + settle_us * us) & (d <= w / 2 - us)
    brk = np.abs(d) >= w / 2 + 3 * us
    # each tap's step, ON level less brake level; the driven tap has the big one
    step = max(ow._tm(code[t & post & on_s]) - ow._tm(code[t & post & brk]) for t in taps)
    out = dict(duty=round(f.duty * 100 / 32768), sgn=f.sgn, pos=f.m["pos"], frame_len=f.m["frame_len"],
               zero=f.zero, phase_ok=f.phase_ok, step_counts=step, pa=np.nan, n_periods=0)
    rows = []
    for n in np.unique(per[post]):
        s = post & f.sh & on_s & (per == n)
        if s.sum() >= 2:
            c = np.polyfit(d[s], code[s] - f.zero, 1)    # a line through the settled ON samples
            rows.append((t_us[s].mean() - d[s].mean() / us, c[1]))   # read at the window's centre
    pp = np.array(rows)
    lp = pp[pp[:, 0] >= late_us] if len(pp) else pp
    out["n_periods"] = len(lp)
    if len(lp) >= 3 and f.phase_ok:
        tc = lp[:, 0] * 1e-6
        ic = lp[:, 1] * board.a_per_count
        span = tc[-1] - tc[0]
        v_mean = (w - short) / period * board.v_term_per_count * step
        q = np.trapezoid(ic, tc)
        out["pa"] = (v_mean * span - l_h * (ic[-1] - ic[0])) / q
        out["i_mean"] = q / span
        out["l_share"] = l_h * (ic[-1] - ic[0]) / q / out["pa"]
    return out


def captures(folder, board, **kw):
    """One row per burst-*.csv.gz in `folder`, by capture index."""
    rows = []
    for f in sorted(folder.glob("burst-*.csv.gz"), key=lambda p: int(p.name.split("-")[1].split(".")[0])):
        with gzip.open(f, "rt") as fh:
            df = pd.read_csv(fh)
        rows.append(dict(idx=int(f.name.split("-")[1].split(".")[0]), **period_average(df, board, **kw)))
    return pd.DataFrame(rows)


def run_pa(folder, board, **kw):
    """A run's period-average R: the median over its from-rest captures whose
    phase checks out."""
    c = captures(folder, board, **kw)
    c = c[(c.idx < FROM_REST) & c.phase_ok]
    return c.pa.median(), len(c)
