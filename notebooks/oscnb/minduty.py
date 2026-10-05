"""The duty floor: where each crest sample lands in a short drive window.

A drive window of h TIM1 ticks sits either side of the counter crest. The
regular crest scan closes its shunt, tap A and tap B samples 31, 83 and 135
ticks after the crest (opasweep.sample_close_ticks); an injected conversion
triggered by TIM1 CC4 at ARR - 68 closes 37 ticks before it. A sample reads
the drive only when it closes after the window's edge has risen and settled
and before the edge has fallen, so the shortest window a reading survives is
a timing fact of the scan, not of the load.

A dataset of this kind holds one `osc sweep --static-load` ladder per capture
under `ladder/capture-N`, named in `ladders.csv` (capture, source, stage,
opamp, decay), and a direction flip under `flip/capture-1`:

  ladders(ds)                     the ladder table, in run order
  Ladder.all(ds)                  one Ladder per ladder/capture-N
  lad.rungs()                     one row per rung: current, both taps, R and its error
  lad.vdiff_rungs()               the firmware's own vdiff per rung, live or held
  floor_q15(ticks)                the smallest duty whose window clears `ticks`
  settle_gain()                   the shunt settle-gain table dev-v006 ships
  folds(plateau)                  burst streams folded onto the PWM period
  Stream(fold, h0, sgn, "ta")     one folded stream, read at any window and sample time
  predict_err(fold, ...)          the R error a scan layout reads, per window width
"""

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import gzip
import json
import re

import numpy as np
import pandas as pd

from . import opasweep as ow

ARR = 1200
# The injected group's trigger: TIM1 CC4 compares at ARR - 68 on the way up,
# 68 ticks before the crest. The conversion then takes the same trigger delay
# and aperture as a regular slot.
INJ_CC4_BEFORE_CREST = 68
INJ_CLOSE = -INJ_CC4_BEFORE_CREST + (ow.TRIGGER_DELAY_CLK + ow.APERTURE_CLK) * ow.TICKS_PER_ADC_CLK
CLOSE = {"shunt": ow.sample_close_ticks(0), "tap A": ow.sample_close_ticks(1),
         "tap B": ow.sample_close_ticks(2), "injected": INJ_CLOSE}
APERTURE_TICKS = ow.APERTURE_CLK * ow.TICKS_PER_ADC_CLK
REF_DUTIES = (30, 40)

BOARD_SRC = Path(__file__).resolve().parents[2] / "firmware" / "boards" / "osc-dev-v006" / "app" / "src" / "main.rs"


def drive_ticks(q15, arr=ARR):
    """The firmware's window width for a duty: (|d| x ARR + 2^14) >> 15."""
    return (np.abs(np.asarray(q15, dtype=np.int64)) * arr + (1 << 14)) >> 15


def floor_q15(ticks, arr=ARR):
    """The smallest duty whose window is at least `ticks` wide, the number
    the servo publishes as window_floor_q15 / window_v_floor_q15
    (window::floor_duty)."""
    return -(-((ticks << 15) - (1 << 14)) // arr)


def settle_gain(path=BOARD_SRC):
    """(start_ticks, [q15 per 8-tick band]) of `i_settle_gain` as the board
    file ships it. A window past the last band reads at unity."""
    src = Path(path).read_text()
    m = re.search(r"i_settle_gain:\s*SettleGain\s*\{\s*start_ticks:\s*(\d+),\s*q15:\s*&\[([^\]]*)\]", src)
    q = [int(v) for v in m.group(2).replace("\n", " ").split(",") if v.strip()]
    return int(m.group(1)), q


def settle_gain_at(h, table):
    start, q = table
    band = max(int(h) - start, 0) >> 3
    return q[band] / 32768 if band < len(q) else 1.0


def ladders(ds):
    return pd.read_csv(ds.path / "ladders.csv")


@dataclass(frozen=True)
class Ladder:
    ds: object
    capture: str
    source: str
    stage: str
    opamp: str
    decay: str
    experiment: str = "ladder"

    @classmethod
    def all(cls, ds):
        return [cls(ds, r.capture, r.source, r.stage, r.opamp, r.decay) for r in ladders(ds).itertuples()]

    @classmethod
    def flip(cls, ds):
        return cls(ds, "capture-1", "swap3-flip", "swap", "normal", "slow", "flip")

    @property
    def path(self):
        return self.ds.path / self.experiment / self.capture

    @cached_property
    def meta(self):
        return json.loads((self.path / "sweep.meta.json").read_text())

    @cached_property
    def frame(self):
        """The sweep, read through the dataset so its sense block and drive
        rule are checked. `ticks` is each row's applied window width."""
        s = self.ds.read(self.experiment, self.capture, "sweep")
        s["ticks"] = drive_ticks(s.duty_q15)
        return s

    @property
    def zero(self):
        """The current's torque-off zero: the baseline segment's mean."""
        s = self.frame
        return s[s.seg == 0].current_raw.mean()

    def held(self, settle_rows=40):
        """Rows of each rung once its applied duty has reached the commanded
        one, and never before `settle_rows` (2 ms) into the rung."""
        s = self.frame
        s = s[s.seg > 0].copy()
        s["k"] = s.groupby("seg").cumcount()
        return s[(s.duty_q15 == s.cmd_duty_q15) & (s.k >= settle_rows)]

    def rungs(self, tap_b="vmotor_b"):
        """One row per rung. `hi` and `lo` are the taps on the driven-high and
        driven-low terminal (A and B forward, B and A reverse), `vab` the
        drive-signed difference, R = vab / I at the nominal chain, and `err`
        its departure from the mean of the 30% and 40% rungs. `tap_b` picks
        the column read as tap B (the probe image streams the regular scan's
        tap B in `ntc_raw`). `valid` is the firmware's current flag."""
        b = self.ds.board
        rows = []
        for seg, g in self.held().groupby("seg"):
            q = int(g.cmd_duty_q15.iloc[0])
            sgn = 1 if q > 0 else -1
            va, vb = g.vmotor_a.mean(), g[tap_b].mean()
            hi, lo = (va, vb) if sgn > 0 else (vb, va)
            i = g.current_raw.mean() - self.zero
            rows.append(dict(seg=seg, duty=sgn * round(abs(q) * 100 / 32768), ticks=int(drive_ticks(q)), sgn=sgn,
                             n=len(g), valid=bool(g.window_valid.mean() > 0.5), i_net=i, hi=hi, lo=lo,
                             hi_sd=(g.vmotor_a if sgn > 0 else g[tap_b]).std(),
                             vab=hi - lo, r=(hi - lo) * b.v_term_per_count / (i * b.a_per_count)))
        t = pd.DataFrame(rows)
        ref = t[t.duty.abs().isin(REF_DUTIES)].r.mean()
        t["err"] = (t.r / ref - 1) * 100
        t.attrs["ref"] = ref
        return t

    def vdiff_rungs(self):
        """The firmware's own terminal difference per rung. Under its floor
        the field holds the last valid value, so a held rung is one constant;
        a live rung moves. R from a live rung takes the current the decay
        mode drives through: the crest sample under Slow, the trough sample
        under Fast, each over its own torque-off zero."""
        b = self.ds.board
        col = "current_trough" if self.decay == "fast" else "current_raw"
        s = self.frame
        zero = s[s.seg == 0][col].mean()
        rows = []
        for seg, g in self.held().groupby("seg"):
            q = int(g.cmd_duty_q15.iloc[0])
            sgn = 1 if q > 0 else -1
            i = g[col].mean() - zero
            live = g.vdiff.nunique() > 1
            v = sgn * g.vdiff.mean()
            rows.append(dict(duty=sgn * round(abs(q) * 100 / 32768), ticks=int(drive_ticks(q)), live=live,
                             distinct=g.vdiff.nunique(), i_net=i,
                             r=v * b.v_term_per_count / (i * b.a_per_count) if live else np.nan))
        t = pd.DataFrame(rows)
        ref = t[t.duty.abs().isin(REF_DUTIES)].r.mean()
        t["err"] = (t.r / ref - 1) * 100
        return t


def folds(plateau):
    """Every burst stream of one opasweep Plateau folded onto the PWM period,
    keyed (h, sign, stream) with stream 'sh', 'ta' or 'tb': (t, code), t the
    sample's close in ticks from the crest. Captures whose counter read missed
    the real phase (opasweep.burst_levels phase_ok) are left out."""
    b = plateau.board
    got = {}
    for x, f in plateau.burst_files():
        with gzip.open(f, "rt") as fh:
            df = pd.read_csv(fh)
        m = {k: int(df[k].iloc[0]) for k in df.columns[2:]}
        if m["step_q15"] == 0 or not ow.burst_levels(df, b)["phase_ok"]:
            continue
        h = int(drive_ticks(m["step_q15"], b.pwm_half_ticks))
        sgn = 1 if m["step_q15"] > 0 else -1
        for s in ("sh", "ta", "tb"):
            t, y, _ = ow.fold(df, b, s)
            if len(t):
                got.setdefault((h, sgn, s), []).append((t, y))
    return {k: (np.concatenate([a for a, _ in v]), np.concatenate([c for _, c in v])) for k, v in got.items()}


def profile(t, y, lo, hi, w=4):
    """Median of y in w-tick bins of t from lo to hi; empty bins are filled
    by linear interpolation from their neighbours."""
    edges = np.arange(lo, hi + w, w)
    i = np.digitize(t, edges) - 1
    x = edges[:-1] + w / 2
    p = np.array([np.median(y[i == k]) if np.any(i == k) else np.nan for k in range(len(x))])
    ok = np.isfinite(p)
    return x, np.interp(x, x[ok], p[ok])


class Stream:
    """One stream of one burst set (h0 the widest window) as a function of
    any narrower window. The rising edge and everything after it are keyed to
    the ON compare, the falling edge to the OFF compare, so a sample closing
    c ticks from the crest in a window of h reads

        off + (P(h + c) - off) * G(c - h)

    with P the stream against time from the ON compare and G its falling
    edge as a fraction of the level just before it. Valid while h + c stays
    inside the h0 window, h <= h0."""

    def __init__(self, fold, h0, sgn, stream, w=2):
        t, y = fold[(h0, sgn, stream)]
        self.h0 = h0
        self.x, self.p = profile(t + h0, y, -300, 2 * h0 + 400, w)
        self.off = np.median(self.p[(self.x < -100) | (self.x > 2 * h0 + 200)])
        pre = np.median(self.p[(self.x > 2 * h0 - 80) & (self.x < 2 * h0 - 10)])
        self.u = self.x - 2 * h0
        g = (self.p - self.off) / (pre - self.off)
        self.g = np.where(self.u < -40, 1.0, g)

    def read(self, h, c):
        return self.off + (np.interp(h + np.asarray(c, float), self.x, self.p) - self.off) * \
            np.interp(np.asarray(c, float) - h, self.u, self.g)


def predict_err(fold, h0, sgn, c_hi, c_lo, ticks):
    """R error against the mean of the 360- and 480-tick windows (the 30%
    and 40% rungs) that a scan closing the high-side tap at c_hi, the
    low-side tap at c_lo and the shunt at +31 would read, per window width."""
    hi, lo = ("ta", "tb") if sgn > 0 else ("tb", "ta")
    s = {k: Stream(fold, h0, sgn, v) for k, v in (("sh", "sh"), ("hi", hi), ("lo", lo))}
    ticks = np.unique(np.concatenate([np.asarray(ticks), [360, 480]]))
    r = pd.Series([(s["hi"].read(h, c_hi) - s["lo"].read(h, c_lo)) / (s["sh"].read(h, CLOSE["shunt"]) - s["sh"].off)
                   for h in ticks], index=ticks)
    return (r / r.loc[[360, 480]].mean() - 1) * 100


def crossing(x, y, level, rising, lo, hi):
    """The first x in [lo, hi] where y crosses `level` in the given direction,
    linearly interpolated between bins."""
    m = (x >= lo) & (x <= hi)
    xs, ys = x[m], y[m]
    for k in range(1, len(xs)):
        a, c = ys[k - 1], ys[k]
        if (rising and a < level <= c) or (not rising and a > level >= c):
            return xs[k - 1] + (level - a) / (c - a) * (xs[k] - xs[k - 1])
    return np.nan
