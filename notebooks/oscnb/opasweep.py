"""The op-amp sweep: one board in a dryer, the same captures at each board
temperature in each op-amp speed mode.

A dataset of this kind holds one folder per plateau and mode under
`plateau/capture-N`, named in `plateaus.csv` (capture, plateau, hs):

  regs.csv            torque-off snapshots before and after the plateau's runs
  dc.csv, v*/         100% duty DC levels on the bench supply, the meter on Rs1
  sweep.csv.gz        an `osc sweep --static-load` duty ladder, with its meta.json
  runs.csv, ident/    `osc ident burst --static-load` runs, 24 captures each

  plateaus(ds)                     the plateau table, in run order
  Plateau.all(ds)                  one Plateau per plateau/capture-N
  p.dc()                           the DC levels with the meter's current
  p.ladder()                       one row per ladder rung, R from the taps
  p.bursts()                       one row per burst capture, late-window R
  fold(df, board, "tb")            one stream of a burst against its sample time
  burst_frame(df, board)           a burst laid onto its PWM period, for estimators

THE BURST CAPTURES

A burst capture samples one ADC slot pattern free running: every sample is one
conversion later than the last, 52 TIM1 ticks, so a capture walks through the
PWM period and folds back onto it. `start_cnt` and `start_dir` give the counter
at the first sample, from which every sample's phase follows.
"""

from dataclasses import dataclass
from functools import cached_property

import gzip
import json

import numpy as np
import pandas as pd
from scipy.stats import trim_mean

# The board NTC (JP2 on IN) as the firmware reads it: 10K 3950 to GND under a
# 10K pull-up to 3V3 (osc-dev-v006 main.rs, Ntc).
NTC_PULLUP, NTC_R25, NTC_BETA = 10_000.0, 10_000.0, 3950.0

# ADC timing (hardware/boards/osc-dev-v006 README, PWM and ADC timing): 24 MHz
# ADC clock against 48 MHz TIM1 ticks, 13.5-clock aperture, 26 clocks a
# conversion, a 2-clock trigger delay, scan order shunt, tap A, tap B.
TICKS_PER_ADC_CLK = 2
APERTURE_CLK = 13.5
CONVERSION_CLK = 26
TRIGGER_DELAY_CLK = 2
STEP_TICKS = CONVERSION_CLK * TICKS_PER_ADC_CLK          # one burst sample to the next
SCAN = ("shunt", "tap A", "tap B")


def sample_close_ticks(slot):
    """Ticks from the crest trigger to the moment slot `slot` of the TEL scan
    stops tracking its input."""
    return (TRIGGER_DELAY_CLK + APERTURE_CLK + slot * CONVERSION_CLK) * TICKS_PER_ADC_CLK


def ntc_c(raw):
    raw = np.asarray(raw, float)
    r = NTC_PULLUP * raw / (4095 - raw)
    return 1 / (1 / 298.15 + np.log(r / NTC_R25) / NTC_BETA) - 273.15


def plateaus(ds):
    return pd.read_csv(ds.path / "plateaus.csv")


def _tm(a):
    return trim_mean(a, 0.1) if len(a) else np.nan


@dataclass(frozen=True)
class Plateau:
    ds: object
    capture: str
    plateau: str
    hs: str

    @classmethod
    def all(cls, ds):
        return [cls(ds, r.capture, str(r.plateau), r.hs) for r in plateaus(ds).itertuples()]

    @property
    def path(self):
        return self.ds.path / "plateau" / self.capture

    @property
    def board(self):
        return self.ds.board

    def regs(self):
        r = pd.read_csv(self.path / "regs.csv")
        r["board_c"] = ntc_c(r.ntc_raw)
        return r

    def samples(self, folder):
        return pd.read_csv(self.path / folder / "samples.csv")

    def dc(self):
        """One row per DC level. `i_meter` is the meter's volts over the
        nominal shunt; `i_psu` the supply's readback less the board's idle draw;
        `i_net_mean` the chip's current less the torque-off zero, in counts."""
        d = pd.read_csv(self.path / "dc.csv")
        d["v_rs1"] = d.meter_v.abs()
        d["i_meter"] = d.v_rs1 / self.board.shunt_ohm
        d["i_psu"] = d.i_true_a
        d["v_ab"] = (d.vmotor_a - d.vmotor_b) * self.board.v_term_per_count
        d["board_c"] = ntc_c(d.ntc_raw)
        return d

    @cached_property
    def meta(self):
        return json.loads((self.path / "sweep.meta.json").read_text())

    def sweep(self):
        return self.ds.read("plateau", self.capture, "sweep")

    def ladder(self):
        """One row per rung: the current over the TEL baseline's zero, both
        taps, and R = (vA - vB) / I at the nominal counts per amp. Rungs whose
        drive window is under the current floor are kept, read whole, and
        marked `valid` False."""
        s, b = self.sweep(), self.board
        zero = s[s.seg == 0].current_raw.mean()
        rows = []
        for seg, g in s[s.seg > 0].groupby("seg"):
            v = g[g.window_valid == 1]
            valid = len(v) > len(g) // 2
            if not valid:
                v = g
            i = v.current_raw.mean() - zero
            vab = (v.vmotor_a - v.vmotor_b).mean()
            rows.append(dict(duty=round(g.cmd_duty_q15.iloc[0] * 100 / 32768),
                             ticks=round(g.cmd_duty_q15.iloc[0] * b.pwm_half_ticks / 32768),
                             valid=valid, n=len(v), i_net=i, va=v.vmotor_a.mean(), vb=v.vmotor_b.mean(),
                             vab=vab, r=vab * b.v_term_per_count / (i * b.a_per_count)))
        lad = pd.DataFrame(rows)
        lad.attrs["zero"] = zero
        return lad

    def runs(self):
        r = pd.read_csv(self.path / "runs.csv")
        r["board_c"] = ntc_c((r.ntc_raw_before + r.ntc_raw_after) / 2)
        return r

    def burst_files(self):
        runs = self.runs()
        for x in runs.itertuples():
            fs = sorted((self.path / "ident" / str(x.dir)).glob("burst-*.csv.gz"),
                        key=lambda p: int(p.name.split("-")[1].split(".")[0]))
            for f in fs:
                yield x, f

    def bursts(self):
        """One row per burst capture: the late-window current and the R the
        taps give with it, at the nominal counts per amp."""
        rows = []
        for x, f in self.burst_files():
            with gzip.open(f, "rt") as fh:
                df = pd.read_csv(fh)
            rows.append(dict(kind=x.kind, run=x.k, f=f.name, **burst_levels(df, self.board)))
        return pd.DataFrame(rows)


def burst_phase(df, pwm_period):
    """Every sample's phase in the PWM period, 0 at the trough, and its slot
    pattern: (shunt, tap A, tap B) masks."""
    m = {k: int(df[k].iloc[0]) for k in df.columns[2:]}
    k = np.arange(len(df))
    phi0 = m["start_cnt"] if m["start_dir"] == 0 else pwm_period - m["start_cnt"]
    phase = (phi0 + STEP_TICKS * k) % pwm_period
    sh = k % 2 == 0
    if m["frame_len"] == 2:
        ta = (k % 2 == 1) if m["chans"] & 1 else np.zeros(len(k), bool)
        tb = (k % 2 == 1) if m["chans"] & 2 else np.zeros(len(k), bool)
    else:
        ta, tb = k % 4 == 1, k % 4 == 3
    return m, phase, sh, ta, tb


TICKS_PER_US = 48                                        # TIM1 ticks per microsecond


@dataclass(frozen=True)
class BurstFrame:
    """One burst capture laid onto its PWM period: `d` is each sample's offset
    from `cen` in ticks, `w` the drive window in ticks, `sh`/`ta`/`tb` the slot
    masks, `pre`/`post` the samples before the step and well after it, `zero`
    the shunt's pre-step median. `phase_ok` is False when the shunt peaks fall
    outside the window the `start_cnt` predicts: the counter read occasionally
    lands a few hundred ticks off."""
    m: dict
    code: np.ndarray
    k: np.ndarray
    d: np.ndarray
    w: float
    sgn: int
    duty: int
    sh: np.ndarray
    ta: np.ndarray
    tb: np.ndarray
    pre: np.ndarray
    post: np.ndarray
    zero: float
    phase_ok: bool


def burst_frame(df, board, cen=1208, late_after_step=60):
    period = 2 * board.pwm_half_ticks
    us = TICKS_PER_US
    m, phase, sh, ta, tb = burst_phase(df, period)
    code = df.code.to_numpy(float)
    k = np.arange(len(code))
    sgn = 1 if m["step_q15"] > 0 else -1
    duty = abs(m["step_q15"])
    d = ((phase - cen + period // 2) % period) - period // 2
    si = m["step_index"]
    post, pre = k >= si + late_after_step, k < si - 8
    w = duty / 32768 * period
    zero = float(np.median(code[sh & pre]))
    hi = post & sh & (code > zero + 0.5 * (code[post & sh].max() - zero))
    phase_ok = bool(hi.any() and np.mean(np.abs(d[hi]) <= w / 2 + us) >= 0.8)
    return BurstFrame(m, code, k, d, w, sgn, duty, sh, ta, tb, pre, post, zero, phase_ok)


def burst_levels(df, board, cen=1208, late_after_step=60):
    """The late-window levels of one burst capture. The late window runs from
    55% of the drive window (at least 5 us after it opens) to 1 us before it
    closes, and the OFF level is taken 3 us or more outside it."""
    us = TICKS_PER_US
    f = burst_frame(df, board, cen, late_after_step)
    d, w, code, post = f.d, f.w, f.code, f.post
    drv, idl = (f.ta, f.tb) if f.sgn > 0 else (f.tb, f.ta)
    lo = max(-w / 2 + 0.55 * w, -w / 2 + 5 * us) if w > 600 else -w / 2 + 0.5 * w
    late = post & (d >= lo) & (d <= w / 2 - us)
    off = post & (np.abs(d) >= w / 2 + 3 * us)
    i_on = _tm(code[f.sh & late]) - f.zero
    vhi = board.v_term_per_count * (_tm(code[drv & late]) - _tm(code[drv & off]))
    vlo = board.v_term_per_count * (_tm(code[idl & late]) - _tm(code[idl & off])) if idl.any() else np.nan
    i_a = i_on * board.a_per_count
    if not f.phase_ok:
        vhi = vlo = np.nan
    return dict(duty=round(f.duty * 100 / 32768), sgn=f.sgn, zero=f.zero, i_on=i_on, phase_ok=f.phase_ok,
                r_drv=vhi / i_a, r_diff=(vhi - vlo) / i_a if idl.any() else np.nan)


def fold(df, board, mask_name, after_step=60):
    """The post-step samples of one stream ('sh', 'ta', 'tb') against the time
    their sample closed, in ticks from the crest."""
    period = 2 * board.pwm_half_ticks
    m, phase, sh, ta, tb = burst_phase(df, period)
    mask = {"sh": sh, "ta": ta, "tb": tb}[mask_name]
    k = np.arange(len(df))
    post = k >= m["step_index"] + after_step
    # the phase counts to the aperture's start; its sample closes one
    # aperture later
    t = ((phase + APERTURE_CLK * TICKS_PER_ADC_CLK - board.pwm_half_ticks + period // 2) % period) - period // 2
    sel = mask & post
    return t[sel], df.code.to_numpy(float)[sel], m
