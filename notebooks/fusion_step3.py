"""Fusion plan step 3: the pot observer below the terminal floor, scored offline.

Below the terminal floor (160 ticks, 13.3% duty, in every capture scored here)
the servo has no back-EMF: the velocity source switch hands the loop the pot
observer, and the MG90 on 2S runs 600 to 1500 counts/s at 8 to 11% duty, so a
whole operating band lives there. This script scores what the servo can know
about speed in that band, against the ripple tach, the way fusion_step2.py
scores the correction above the floor, so the two read side by side:

  fw observer    the 3-state fixed-gain observer the firmware runs
                 (core estimator/fusion.rs), its integer arithmetic reproduced
                 step for step, with the gains the servo carries (ident gains.rs
                 placement at f_o = 15 Hz) and again at other f_o, on the
                 leave-one-out table and on the raw pot
  model          back-EMF predicted, not measured: (D V_rail - R i - V0) / Ke on
                 the 1 ms boxcar, R the servo's r_q12, V_rail the rail the
                 capture started at, Ke and V0 fitted per direction on the
                 session's 15 to 50% rungs; `hold` carries the last valid
                 current (0 where none), `fc` substitutes the servo's Coulomb
                 current fric_fc where the current window is invalid
  scale/offset   fusion_step2's slow pot-steered correction with the model
                 boxcar in place of the measured one
  switch         what the loop reads today: the measured boxcar after four valid
                 halves above the floor, the fw observer otherwise
  pot 4/20 ms    plain linearized-pot differences, the reference rows

The firmware has no plain differentiator: the observer IS the pot
differentiator, a critically damped alpha-beta-gamma filter (l1 = 1 - p^3,
l2 = 1.5 (1-p)^2 (1+p) f_med, l3 = (1-p)^3 f_med / 2B, p = exp(-2 pi f_o /
f_med)) with the drive current as its model input through B. Below the current
floor that input is a stale hold, which tau_d absorbs, so the observer there
is the pot alone.

Truth is the ripple tach. The breakaway rungs (1 to 18% at 200 ms) and the
grid's 10% rung run the ripple line at 140 to 560 Hz, under notebook 09's 250 Hz
floor, so this script tracks them with a 40 ms ridge window and accepts a rung
only if its cycles per linearized count agree with the session's core rungs
within 3%: a tracker locked to the f/2 family fails that by half.

Bands: 5-7% (breakaway onset, forward only), 8-11% (the MG90's band under the
floor), 12-13% (under the floor, above the current floor on rev 2A), 14-15%
(the handover: both sources valid, so the step the switch makes is the
difference of the two biases), 16-20% (overlap with step 2). Settle is read
from the first tracked sample of each rung in bins of time.

Result, 8-11% duty, rms / bias in percent of band speed (D governed, D free,
2A governed, 2A free; the ripple's own 1 ms scatter is 0.5 to 0.7%):

  pot 20 ms                        9.1 / -0.2   9.7 / +0.1  10.8 / +0.3   9.7 / -0.1
  fw observer, servo gains         9.9 / -1.3  10.4 / -1.0  11.0 / +0.2  10.0 / -0.3
  fw observer, predict gated       8.6 / -0.6   9.4 / -0.3  11.0 / +0.2  10.0 / -0.3
  fw observer f_o 8               13.9 / -6.3  13.8 / -6.4   9.6 / +1.2   8.4 / +0.1
  fw observer f_o 25              13.7 / -0.4  14.8 / +0.0  15.6 / +0.1  14.0 / -0.4
  fw observer on the raw pot      22.6 / -1.3  24.9 / -0.4  23.0 / +0.5  21.1 / +0.1
  model fc (session Ke, V0)       13.4 / +10.4 13.7 / +11.5 15.5 / -4.6  12.7 / +1.4
  model servo (servo Ke, no V0)   26.8 / +25.9 17.7 / +16.5 17.5 / -10.0 23.1 / -20.2
  model fc + offset W40 tau50      4.7 / -1.0   6.1 / -1.0  14.5 / +1.2   9.9 / -0.1

What the numbers say:

- The observer the servo runs is the linearized pot's 20 ms difference with no
  bias: 10% rms at 8-11%, 7 to 9% at 12-13%, 40 ms to settle from the first
  tracked sample on 2A free (2A governed never settles under 1% because its
  table is 1 to 2% off where the breakaway rungs run: the pot 20 ms row
  carries the same bias). On the raw pot it is twice as noisy and carries a
  rung-to-rung bias of 5 to 13% that moves with position. The table is what
  made the band usable; the gains need no new synthesis: f_o 8 buys 1.5 points
  of rms on rev 2A and pays 4 to 12 points of lag bias on an accelerating rung,
  f_o 25 costs 4 points of rms for nothing.
- Below the current floor the firmware pushes a stale held current through
  the model: on board D (floor 160 ticks) the observer's input is whatever was
  last valid, zero here, so the predict decelerates the shaft by fric_fc every
  tick until tau_d has integrated the lie away. That costs -4 to -16% bias on
  the 200 ms breakaway rungs at 6 to 10% and 120 ms to settle at f_o 15, and
  leaves f_o 8 and 5 at -6 to -98%. Gating the predict on the current window (accel 0 when
  the sample is invalid) removes it; rev 2A's 64-tick current floor already
  does, which is why the two 2A sets never show it.
- A predicted back-EMF is no speedometer under the floor. 100 mV of V0 is
  25% of speed at 8%, and the fitted V0 moves from +30 to -94 mV between
  sessions on the same servo; the servo's own set, which has no V0, reads
  -20% on 2A free and +26% on board D. With a measured current the 1 ms model
  also carries the commutation ripple itself, whose period at these speeds is
  3 to 7 ms. The pot-steered offset form corrects the bias (within 1.5% on
  every set) and lands at the observer's rms on rev 2A; its 4.7 to 6% on board
  D is an artifact: with no valid current the model there is a constant, and a
  constant corrected by the pot every 40 ms is a long pot average. The scale
  form diverges on a predicted speed that passes through zero.
- The handover at 14-15%: the measured boxcar reads +2.2 / +2.1 (board D) and
  -0.2 / +1.2 (2A) against the observer's +0.7 / +0.7 / +0.5 / +0.2, so the
  switch steps the loop by at most 1.5% of speed, about 30 counts/s, with the
  two rms within a point of each other (5 to 8%). Nothing to fix there; step 2's
  correction pulls the boxcar side to zero and shrinks the step further.

Cost on the V006 per medium tick (Zmmul, no divide): the observer as it is 5
widening multiplies and 3 states; the predict gate is one branch; the model
boxcar would be 1 multiply per fast tick and 2 per medium tick (the back-EMF
boxcar's shape with D x vbus in place of the terminal sum); the offset form 2
more and two rings of 80 words for a 40 ms window.

Run from notebooks/:  uv run python fusion_step3.py [--sets "D governed" ...]
                      [--cache DIR] [--out DIR]
The cache is fusion_step2's (the core rungs); the slow rungs get their own file.
"""

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import oscnb
from oscnb import governed, ripple, session

import fusion_step2 as s2
from fusion_step2 import BOX, M0, MED, Q15, SETS, TAUS_MS, WINDOWS_MS

F_MED = 2000.0
DT_MED_Q32 = (1 << 32) // 2000          # kernel KernelTiming::dt_med_q32 at 20 kHz / DECIM_MED
V_FLOOR_Q15 = 4356                      # 160 ticks of 1200 (window::drive_ticks), the terminal floor in these captures
PWM_ARR = 1200
BANDS = [4, 7, 11, 13, 15, 20]
BAND_NAMES = ["5-7%", "8-11%", "12-13%", "14-15%", "16-20%"]
F_O_HZ = (5, 8, 15, 25)
SETTLE_BINS_MS = [0, 20, 40, 80, 120, 200]
MIN_TRAVEL = 40                         # linearized counts over a rung's tail, else the shaft did not turn
SLOW_WIN, SLOW_HOP, SLOW_FMIN, SLOW_FLOOR, SLOW_NFFT = 800, 40, 60.0, 80.0, 16384
TO_BEMF, TO_POT = 4, 2                  # omega_switch.rs hysteresis

# The gains the servo carried into these sessions. Not in the capture meta (the
# plant block carries only the stamp), so they are copied from the SAVEd set
# whose stamp the meta names: board D 0x23aa = the set `osc ident synth` built
# from mg90-a__2s through the table (b 0.3081, fc 53.26, memory note), rev 2A
# 0xdc03 = telemetry/mg90-a__dev-v006-2A__dps__r-vs-t/cal/capture-2/snapshot.json.
# l1 and l2 depend on f_o and f_med only, so every set shares 8640 / 3180.
SERVO_GAINS = {
    "dev-v006-D": dict(b_i_q313=2524, l1_q016=8640, l2_q88=3180, l3_q88=81, fric_fc_counts=53, ke_vpc_q=580),
    "dev-v006-2A": dict(b_i_q313=2459, l1_q016=8640, l2_q88=3180, l3_q88=83, fric_fc_counts=90, ke_vpc_q=371),
}


# ---------------------------------------------------------------- gains

def synth(b, f_o, f_med=F_MED):
    """ident/src/gains.rs synthesize + encode for the fusion gains."""
    p = np.exp(-2 * np.pi * f_o / f_med)
    q = 1 - p
    return dict(b_i_q313=int(round(b * 8192)), l1_q016=int(round((1 - p ** 3) * 65536)),
                l2_q88=int(round(1.5 * q * q * (1 + p) * f_med * 256)),
                l3_q88=int(round(q ** 3 * f_med / (2 * b) * 256)))


# ---------------------------------------------------------------- the firmware observer, bit for bit

E_LIM, THETA_LIM, OMEGA_LIM, TAU_LIM, ACCEL_LIM = 1 << 23, 1 << 29, 32767 << 16, 4095 << 16, 8192


def fw_observer(i_counts, pos_q4, g, dt_q32=DT_MED_Q32, i_valid=None):
    """fusion.rs FusionObs::step over one segment, seeded at its first sample
    (the kernel seeds on the torque edge and the segment opens at rest after a
    seek). Python ints are unbounded, so every saturating op is the clamp that
    follows it in the firmware; the products stay inside i32 by the module's
    own bounds, so no truncation is lost. Returns omega in csQ16.

    `i_valid` is the candidate the firmware does not have: where the current
    window is invalid the predict is skipped (accel 0, as if the drive exactly
    balanced friction and the disturbance), instead of pushing the held
    current through the model."""
    b_i, l1, l2, l3, fc = g["b_i_q313"], g["l1_q016"], g["l2_q88"], g["l3_q88"], g["fric_fc_counts"]
    T = len(pos_q4)
    out = np.empty(T, dtype=np.int64)
    theta = int(pos_q4[0]) << 12
    omega = 0
    tau = 0
    one = 1 << 16
    ii, pp = i_counts.tolist(), pos_q4.tolist()
    vv = [True] * T if i_valid is None else i_valid.tolist()
    for t in range(T):
        fric = fc if omega > one else (-fc if omega < -one else 0)
        accel = ii[t] - fric - (tau >> 16) if vv[t] else 0
        accel = -ACCEL_LIM if accel < -ACCEL_LIM else (ACCEL_LIM if accel > ACCEL_LIM else accel)
        omega += (b_i * accel) << 3
        omega = -OMEGA_LIM if omega < -OMEGA_LIM else (OMEGA_LIM if omega > OMEGA_LIM else omega)
        theta += (omega * dt_q32) >> 32
        theta = -THETA_LIM if theta < -THETA_LIM else (THETA_LIM if theta > THETA_LIM else theta)
        e = (pp[t] << 12) - theta
        e = -E_LIM if e < -E_LIM else (E_LIM if e > E_LIM else e)
        theta += (l1 * e) >> 16
        theta = -THETA_LIM if theta < -THETA_LIM else (THETA_LIM if theta > THETA_LIM else theta)
        omega += (l2 * e) >> 8
        omega = -OMEGA_LIM if omega < -OMEGA_LIM else (OMEGA_LIM if omega > OMEGA_LIM else omega)
        if b_i == 0:
            tau = 0
        else:
            tau -= (l3 * e) >> 8
            tau = -TAU_LIM if tau < -TAU_LIM else (TAU_LIM if tau > TAU_LIM else tau)
        out[t] = omega
    return out


def omega_switch(bemf_valid, bemf, pot):
    """omega_switch.rs on the medium grid: the loop's omega_hat in c/s."""
    src, run, held = 0, 0, 0.0
    out = np.empty(len(pot))
    for t in range(len(pot)):
        if src == 0:
            if bemf_valid[t]:
                held = bemf[t]
                run += 1
                if run >= TO_BEMF:
                    src, run = 1, 0
            else:
                run = 0
        else:
            if bemf_valid[t]:
                held = bemf[t]
                run = 0
            else:
                run += 1
                if run >= TO_POT:
                    src, run = 0, 0
        out[t] = held if src == 1 else pot[t]
    return out


# ---------------------------------------------------------------- the slow rungs

def slow_rungs(d, recs, tables):
    """Every drive rung at or under 20% (breakaway block and grid) through a
    ripple tracker sized for the low band, in the shape fusion_step2.track
    emits so medium_series reads them. `ok` needs the ripple's cycles per
    linearized count within 3% of the table's anchor."""
    fs = d.board.tick_hz
    out = []
    for cap, df, meta in recs:
        bias = df[df.seg == 0].current_raw.mean()
        lut, c_lin, _ = tables[cap]
        smap = {m["seg"]: m for m in session.segment_map(meta)}
        for seg, g in df[df.seg > 0].groupby("seg"):
            m = smap[seg]
            if m["kind"] != "drive" or m["block"] not in ("breakaway", "grid"):
                continue
            duty = g.cmd_duty_q15.iloc[0] * 100 / Q15
            if abs(duty) > 20 or g.cmd_duty_q15.nunique() > 1:
                continue
            pos = g.pos.to_numpy()
            n = len(g)
            r = {"cap": cap, "seg": int(seg), "duty": int(round(duty)), "n": n, "i0": int(g.index[0]),
                 "pos": pos, "cur": g.current_raw.to_numpy() - bias,
                 "va": g.vmotor_a.to_numpy().astype(float), "vb": g.vmotor_b.to_numpy().astype(float),
                 "dq": g.duty_q15.to_numpy(), "valid": g.window_valid.to_numpy(),
                 "s0": governed.settled_from(g, fs), "ok": False, "block": m["block"]}
            out.append(r)
            lin = lut.counts(pos)
            a = int(n * 0.4)
            if abs(lin[-1] - lin[a]) < MIN_TRAVEL:
                continue
            v_lin = abs(np.polyfit(np.arange(n - a) / fs, lin[a:], 1)[0])
            f_seed = c_lin * v_lin
            x = r["cur"].astype(float)
            f, X = ripple.line_spectrum(x[a:], fs)
            seed, _ = ripple.interp_peak(f, X, max(50.0, 0.7 * f_seed), min(ripple.F_HI, 1.3 * f_seed))
            if f_seed < 450:
                rt = ripple.ridge_track(x, seed, fs, win=SLOW_WIN, hop=SLOW_HOP, fmin=SLOW_FMIN, nfft=SLOW_NFFT)
                floor = SLOW_FLOOR
            else:
                rt = ripple.ridge_track(x, seed, fs)
                floor = 250.0
            good = ((rt["amp"] / rt["noise"] >= 2.0) & (rt["f"] >= floor)).to_numpy()
            bad = np.flatnonzero(~good)
            rt = rt.iloc[bad.max() + 1:] if len(bad) else rt
            if len(rt) < 2:
                continue
            cyc = ripple.demod_cycles(x, rt["n"].to_numpy(), rt["f"].to_numpy(), fs)
            n0 = int(rt["n"].iloc[0]) + 100
            if n - n0 < M0 + BOX:
                continue
            cpc = (cyc[-1] - cyc[n0]) / abs(lin[-1] - lin[n0])
            r.update(cyc=cyc, n0=n0, accept=(n - n0) / n, cpc=cpc, f_line=float(rt["f"].median()),
                     ok=abs(cpc / c_lin - 1) < s2.CPC_TOL)
    return out


# ---------------------------------------------------------------- the model

def rail_point(r, board, rail_v, r_vpc, dt):
    """fusion_step2.settled_point with D x V_rail - R i for the applied EMF."""
    a = r["s0"]
    if a is None or r["n0"] > a:
        return None
    sg = np.sign(r["duty"])
    D = np.abs(r["dq"][a:]) / Q15
    # the crest shunt sample is a magnitude in either direction (window::i_from_frame signs it)
    i = r["cur"][a:]
    f = (r["cyc"][-1] - r["cyc"][a]) / ((len(r["pos"]) - 1 - a) * dt)
    return {"dir": int(sg), "y": (D * rail_v[r["cap"]] - r_vpc * i * board.v_term_per_count).mean(), "f": f}


def fit_model(core, board, rail_v, r_vpc, dt):
    """{dir: (Ke V per Hz, V0 V)} on the session's core rungs, R pinned at the servo's."""
    pts = pd.DataFrame([p for r in core for p in [rail_point(r, board, rail_v, r_vpc, dt)] if p])
    return {sg: tuple(np.polyfit(g.f, g.y, 1)) for sg in (1, -1) for g in [pts[pts.dir == sg]]}


def held(x, valid, seg):
    """Last valid sample carried forward inside a segment, 0 before the first."""
    s = pd.Series(np.where(valid, x, np.nan))
    return s.groupby(seg).ffill().fillna(0.0).to_numpy()


def extra_series(df, ms, rail, r_vpc, ke, g_servo, board, lut, dt):
    """What step 3 adds to fusion_step2.medium_series: the model boxcars, the
    pot in Q4 (table and identity), the held current and the duty, on the
    same medium grid."""
    n = len(df)
    j = ms["j"]
    dq = df.duty_q15.to_numpy()
    sg = np.sign(dq).astype(int)
    seg = df.seg.to_numpy()
    bias = df[df.seg == 0].current_raw.mean()
    i_valid = (df.window_valid.to_numpy() == 1) & (sg != 0)
    # the crest shunt sample is a magnitude; the firmware signs it by the drive
    # direction (window::i_from_frame) before the observer sees it
    mag = df.current_raw.to_numpy() - bias
    i_hold = held(mag, i_valid, seg)
    i_fc = np.where(i_valid, mag, g_servo["fric_fc_counts"])
    D = np.abs(dq) / Q15
    ke_s = np.where(sg > 0, ke[1][0], ke[-1][0])
    v0_s = np.where(sg > 0, ke[1][1], ke[-1][1])
    out = {"dq": dq[j], "sgn_j": sg[j], "pos_q4": lut.q4(df.pos.to_numpy())[j],
           "raw_q4": (df.pos.to_numpy().astype(np.int64) << 4)[j],
           "i_hold": (sg * np.round(i_hold))[j].astype(np.int64), "i_valid": i_valid[j]}
    driven = sg != 0
    cd = np.r_[0, np.cumsum(~driven)]
    chg = np.r_[0, ((sg[1:] != sg[:-1]) | (seg[1:] != seg[:-1])).astype(int)]
    cchg = np.r_[0, np.cumsum(chg)]
    out["model_valid"] = ((cd[j + 1] - cd[j + 1 - BOX]) == 0) & ((cchg[j + 1] - cchg[j + 2 - BOX]) == 0)
    for name, i in (("hold", i_hold), ("fc", i_fc)):
        w = sg * (D * rail - r_vpc * i * board.v_term_per_count - v0_s) / ke_s / ms["c_lin"]
        cw = np.r_[0, np.cumsum(np.where(driven, w, 0.0))]
        out[f"model {name}"] = (cw[j + 1] - cw[j + 1 - BOX]) / BOX
    # the servo's own table: vbus_counts is the rail in tap counts, ke_vpc_q in
    # vcounts per linearized c/s, r_q12 in vcounts per ccount, no V0 anywhere
    w = sg * (D * rail / board.v_term_per_count - r_vpc * i_fc) / (g_servo["ke_vpc_q"] / 4096.0)
    cw = np.r_[0, np.cumsum(np.where(driven, w, 0.0))]
    out["model servo"] = (cw[j + 1] - cw[j + 1 - BOX]) / BOX
    # the terminal floor gate on the measured boxcar: window_valid carries the
    # current floor only, 64 ticks on rev 2A, and tap B sampled in brake below
    # 160 ticks is not a terminal reading
    above = np.abs(dq) >= V_FLOOR_Q15
    ca = np.r_[0, np.cumsum(~above)]
    out["meas_valid"] = ms["valid"] & ((ca[j + 1] - ca[j + 1 - BOX]) == 0)
    W4 = 80
    out["v_pot4"] = np.full(len(j), np.nan)
    out["v_ref4"] = np.full(len(j), np.nan)
    m = j >= W4
    pos_lin = lut.counts(df.pos.to_numpy())
    cyc = ms["cyc"]
    out["v_pot4"][m] = (pos_lin[j[m]] - pos_lin[j[m] - W4]) / (W4 * dt)
    out["v_ref4"][m] = (cyc[j[m]] - cyc[j[m] - W4]) / (W4 * dt) / ms["c_lin"]
    out["v_ref20c"] = np.full(len(j), np.nan)
    hi = np.minimum(j + 190, n - 1)
    m = (j >= 210) & (hi > j)
    out["v_ref20c"][m] = (cyc[hi[m]] - cyc[j[m] - 210]) / ((hi[m] - j[m] + 210) * dt) / ms["c_lin"]
    raw = df.pos.to_numpy().astype(float)
    out["v_raw20"] = np.full(len(j), np.nan)
    m = j >= 400
    out["v_raw20"][m] = (raw[j[m]] - raw[j[m] - 400]) / (400 * dt)
    return out


def medium_with_cyc(df, rungs, lut, c_lin, ke, r_ohm, board, dt):
    """fusion_step2.medium_series plus the ripple angle on the fast grid and
    the table's anchor, which the extra series need."""
    ms = s2.medium_series(df, rungs, lut, c_lin, ke, r_ohm, board, dt)
    n = len(df)
    cyc = np.full(n, np.nan)
    for r in rungs:
        if r["ok"]:
            a, b = r["i0"] + r["n0"], r["i0"] + r["n"]
            cyc[a:b] = r["cyc"][r["n0"]:] * np.sign(r["duty"])
    ms["cyc"] = cyc
    ms["c_lin"] = c_lin
    ms["t0"] = np.full(len(ms["j"]), -1)       # medium ticks since the rung's first tracked sample
    for r in rungs:
        if r["ok"]:
            sel = (ms["j"] >= r["i0"] + r["n0"]) & (ms["j"] < r["i0"] + r["n"])
            ms["t0"][sel] = (ms["j"][sel] - (r["i0"] + r["n0"])) // MED
    return ms


def observer_series(df, ms, ex, rungs, gains, key, gate=False):
    """The fw observer over every scored rung's segment, per segment from its
    first medium tick, on `key` (pos_q4 or raw_q4). Returns omega in c/s on the
    medium grid, nan outside the segments run."""
    out = np.full(len(ms["j"]), np.nan)
    seg = ms["seg"]
    for s in sorted({r["seg"] for r in rungs if r["ok"]}):
        sel = np.flatnonzero(seg == s)
        if len(sel) == 0:
            continue
        out[sel] = fw_observer(ex["i_hold"][sel], ex[key][sel], gains,
                               i_valid=ex["i_valid"][sel] if gate else None) / 65536.0
    return out


# ---------------------------------------------------------------- scoring, fusion_step2's formulas on step 3's bands

def score(est, ref, valid, ms, duties):
    m = (ms["rung"] >= 0) & valid & np.isfinite(ref) & np.isfinite(est)
    d = duties[ms["rung"][m]]
    err = est[m] - ref[m]
    t = pd.DataFrame({"band": pd.cut(np.abs(d), BANDS, labels=BAND_NAMES),
                      "err": err, "err_dir": err * np.sign(d), "v": np.abs(ref[m])})
    g = t.groupby("band", observed=True)
    return pd.DataFrame({"rms": np.sqrt(g.err.apply(lambda e: np.mean(e ** 2))) / g.v.mean() * 100,
                         "bias": g.err_dir.mean() / g.v.mean() * 100, "ticks": g.size()})


def score_by_duty(est, ref, valid, ms, duties, blocks):
    """Per (block, duty): the breakaway rungs are 200 ms and still accelerating,
    the grid's 10% rung is 1.5 s of settled travel under the floor."""
    m = (ms["rung"] >= 0) & valid & np.isfinite(ref) & np.isfinite(est)
    d = duties[ms["rung"][m]]
    err = (est[m] - ref[m]) * np.sign(d)
    t = pd.DataFrame({"block": blocks[ms["rung"][m]], "duty": np.abs(d), "err": err, "v": np.abs(ref[m])})
    g = t.groupby(["block", "duty"])
    return pd.DataFrame({"bias": g.err.mean() / g.v.mean() * 100,
                         "rms": np.sqrt(g.err.apply(lambda e: np.mean(e ** 2))) / g.v.mean() * 100,
                         "ticks": g.size()})


def settle(est, ref, valid, ms, duties, lo=5, hi=13):
    """Bias by time since the rung's first tracked sample, rungs lo..hi% duty."""
    m = (ms["rung"] >= 0) & valid & np.isfinite(ref) & np.isfinite(est) & (ms["t0"] >= 0)
    d = duties[ms["rung"][m]]
    keep = (np.abs(d) >= lo) & (np.abs(d) <= hi)
    t = pd.DataFrame({"t_ms": ms["t0"][m][keep] * MED / 20.0,
                      "err_dir": (est[m] - ref[m])[keep] * np.sign(d[keep]), "v": np.abs(ref[m][keep])})
    t["bin"] = pd.cut(t.t_ms, SETTLE_BINS_MS, right=False)
    g = t.groupby("bin", observed=True)
    return pd.DataFrame({"bias": g.err_dir.mean() / g.v.mean() * 100,
                         "rms": np.sqrt(g.err_dir.apply(lambda e: np.mean(e ** 2))) / g.v.mean() * 100,
                         "ticks": g.size()})


def flat(sc):
    out = {}
    for b in BAND_NAMES:
        if b in sc.index and sc.loc[b, "ticks"] > 0:
            out[f"rms {b}"] = sc.loc[b, "rms"]
            out[f"bias {b}"] = sc.loc[b, "bias"]
    return out


def settle_ms(cs, bar=1.0):
    ok = (cs.bias.abs() < bar).to_numpy()
    for i in range(len(ok)):
        if ok[i:].all():
            return cs.index[i].left
    return np.nan


# ---------------------------------------------------------------- the whole thing

def analyse(label, cfg, cache, out_dir, tracked):
    d = oscnb.datasets.load(cfg["key"])
    board = d.board
    fs = board.tick_hz
    dt = 1.0 / fs
    dt_med = MED * dt
    r_ohm = d.servo.measured["r_winding_measured"].value
    recs = s2.recordings(d)
    frames = {c: df for c, df, meta in recs}
    rail = {c: board.rail_from_vbus_counts(m["vbus_counts"]) for c, _, m in recs}
    r_q12 = {m["drive"]["r_q12"] for _, _, m in recs if "drive" in m}
    if not r_q12:
        # fw 64 free captures carry no drive block: the servo ran the same set as the governed session
        r_q12 = {m["drive"]["r_q12"] for _, _, m in s2.recordings(oscnb.datasets.load(SETS[cfg["partner"]]["key"])) if "drive" in m}
    assert len(r_q12) == 1, r_q12
    r_vpc = r_q12.pop() / 4096.0
    g_servo = SERVO_GAINS[board.key]
    core = tracked[label]
    tables = s2.loo_tables(core, cfg["core"])
    c_lin = float(np.mean([t[1] for t in tables.values()]))
    cp = cache / f"{s2.slug(label)}-slow.pkl" if cache else None
    if cp and cp.exists():
        rungs = pickle.loads(cp.read_bytes())
    else:
        print(f"tracking the slow rungs of {label} ...", file=sys.stderr)
        rungs = slow_rungs(d, recs, tables)
        if cp:
            cp.write_bytes(pickle.dumps(rungs))
    duties = np.array([r["duty"] for r in rungs])
    blocks = np.array([r["block"] for r in rungs])
    ke_meas = s2.fit_ke(core, cfg["core"], board, dt, r_ohm)
    ke_model = fit_model(s2.core_of(core, cfg["core"]), board, rail, r_vpc, dt)

    print(f"\n{'=' * 100}\n{label}: {cfg['key']} on {board.key}, {len(recs)} captures, rail "
          f"{min(rail.values()):.2f} .. {max(rail.values()):.2f} V, servo R {r_vpc:.3f} vcounts/ccount "
          f"({r_vpc * board.v_term_per_count / board.a_per_count:.2f} ohm with the bridge), "
          f"table anchor {c_lin:.4f} ripple cycles per linearized count")
    print("servo gains:", g_servo, "-> re-synthesized per f_o:", {f: synth(g_servo["b_i_q313"] / 8192, f) for f in F_O_HZ})
    acc = pd.DataFrame([{"duty": abs(r["duty"]), "ok": r["ok"], "line": r.get("f_line", np.nan),
                         "cpc": r.get("cpc", np.nan) / c_lin} for r in rungs])
    g = acc.groupby("duty")
    print("slow rungs per duty (all / accepted, median ripple line Hz, cycles per count over the anchor):")
    print(pd.DataFrame({"n": g.size(), "ok": g.ok.sum(), "line_hz": g.line.median().round(0),
                        "cpc_ratio": g.cpc.median().round(3)}).T.to_string())
    print("model (D V_rail - R i - V0) / Ke, refit on the core rungs: " + ", ".join(
        f"{'fwd' if s > 0 else 'rev'} Ke {ke_model[s][0] * 1e3:.3f} mV/Hz V0 {ke_model[s][1] * 1e3:+.0f} mV" for s in (1, -1)))
    print("measured back-EMF model (fusion_step2 refit): " + ", ".join(
        f"{'fwd' if s > 0 else 'rev'} Ke {ke_meas[s][0] * 1e3:.3f} mV/Hz V0 {ke_meas[s][1] * 1e3:+.0f} mV" for s in (1, -1)))

    rows, by_duty, settles = [], {}, {}
    parts = {}

    def add(name, est, ref, valid, ms, cost):
        parts.setdefault(name, {"sc": [], "bd": [], "st": [], "cost": cost})
        parts[name]["sc"].append(score(est, ref, valid, ms, duties))
        parts[name]["bd"].append(score_by_duty(est, ref, valid, ms, duties, blocks))
        parts[name]["st"].append(settle(est, ref, valid, ms, duties))

    for c, df in frames.items():
        lut, cl, _ = tables[c]
        rs = [r if r["cap"] == c else dict(r, ok=False) for r in rungs]
        ms = medium_with_cyc(df, rs, lut, cl, ke_meas, r_ohm, board, dt)
        ex = extra_series(df, ms, rail[c], r_vpc, ke_model, g_servo, board, lut, dt)
        always = np.ones(len(ms["j"]), bool)
        add("ripple 1 ms vs its centred 20 ms mean (the truth's own scatter)", ms["v_ref"], ex["v_ref20c"], always, ms, "-")
        add("back-EMF 1 ms (measured)", ms["wb"], ms["v_ref"], ex["meas_valid"], ms, "today's boxcar")
        add("pot 4 ms", ex["v_pot4"], ex["v_ref4"], always, ms, "1 sub, ring of 80 pot words")
        add("pot 20 ms", ms["v_pot20"], ms["v_ref20"], always, ms, "1 sub, ring of 400 pot words")
        add("pot 20 ms raw", ex["v_raw20"], ms["v_ref20"], always, ms, "-")
        obs = {}
        for f_o in F_O_HZ:
            gains = g_servo if f_o == 15 else dict(synth(g_servo["b_i_q313"] / 8192, f_o), fric_fc_counts=g_servo["fric_fc_counts"])
            tag = "fw observer (servo gains, f_o 15)" if f_o == 15 else f"fw observer f_o {f_o}"
            obs[f_o] = observer_series(df, ms, ex, rs, gains, "pos_q4")
            add(tag, obs[f_o], ms["v_ref"], always, ms, "5 widening mul/medium tick, 3 states")
        add("fw observer raw pot (servo gains)", observer_series(df, ms, ex, rs, g_servo, "raw_q4"), ms["v_ref"], always, ms, "same")
        add("fw observer, predict gated on the current window", observer_series(df, ms, ex, rs, g_servo, "pos_q4", gate=True),
            ms["v_ref"], always, ms, "same, one branch")
        add("fw observer, no model (b_i 0)", observer_series(df, ms, ex, rs, dict(g_servo, b_i_q313=0), "pos_q4"),
            ms["v_ref"], always, ms, "4 widening mul, 2 states, no tau_d")
        add("switch (servo today)", omega_switch(ex["meas_valid"], ms["wb"], obs[15]), ms["v_ref"], always, ms, "today")
        for name in ("model hold", "model fc", "model servo"):
            add(name, ex[name], ms["v_ref"], ex["model_valid"], ms, "1 mul/fast tick + 2/medium tick")
        msm = {"valid": ex["model_valid"], "sgn": ex["sgn_j"], "wb": ex["model fc"], "pot": ms["pot"], "seg": ms["seg"]}
        for W in WINDOWS_MS:
            Wm = int(round(W * 1e-3 / dt_med))
            Ws = Wm * dt_med
            ok, dth, S = s2.window_terms(msm, Wm, dt_med)
            for form in ("scale", "offset"):
                for tau in TAUS_MS:
                    gg = s2.gain(form, tau, Ws, dt_med)
                    xs = s2.run_state(ok, dth, S, msm["sgn"], form, gg, Ws, 1.0 if form == "scale" else 0.0)
                    add(f"model fc + {form} W{W} tau{tau}", s2.estimate(form, xs, msm), ms["v_ref"], ex["model_valid"], ms,
                        "model + 3 mul (scale) / 2 (offset) per medium tick, two rings of W/0.5 ms words")

    for name, p in parts.items():
        sc = s2.pooled(p["sc"])
        rows.append(dict(est=name, cost=p["cost"], **flat(sc)))
        bd = pd.concat(p["bd"])
        w = bd.ticks
        gb = bd.groupby(level=[0, 1])
        by_duty[name] = pd.DataFrame({"bias": (bd.bias * w).groupby(level=[0, 1]).sum() / gb.ticks.sum(),
                                      "rms": np.sqrt((bd.rms ** 2 * w).groupby(level=[0, 1]).sum() / gb.ticks.sum())})
        settles[name] = s2.pooled(p["st"])
    R = pd.DataFrame(rows).set_index("est")
    cols = [c for b in BAND_NAMES for c in (f"rms {b}", f"bias {b}") if c in R]
    print(f"\n--- {label}: rms and bias in % of band speed against the 1 ms ripple (pot rows against the ripple over their own window)")
    main = [n for n in R.index if "+ scale" not in n and "+ offset" not in n]
    print(R.loc[main, cols].round(2).to_string())
    print(f"\n--- {label}: the model with the pot-steered correction, best three of each form by rms over 8-11% then |bias| 8-11%")
    for form in ("scale", "offset"):
        t = R[[f"+ {form}" in n for n in R.index]].copy()
        t["key"] = t["rms 8-11%"].round(1) + t["bias 8-11%"].abs() / 100
        print(t.sort_values("key").head(3)[cols].round(2).to_string())
    print(f"\n--- {label}: bias [%] by block and duty, the floor crossing (the terminal floor sits at 14% in these captures)")
    show = ["back-EMF 1 ms (measured)", "fw observer (servo gains, f_o 15)", "fw observer, predict gated on the current window",
            "fw observer f_o 5", "fw observer raw pot (servo gains)", "switch (servo today)", "model fc", "model servo", "pot 20 ms"]
    BD = pd.concat({n: by_duty[n].bias for n in show}, axis=1).sort_index()
    print(BD.round(2).to_string())
    print(f"\n--- {label}: rms [%] by duty")
    print(pd.concat({n: by_duty[n].rms for n in show}, axis=1).sort_index().round(2).to_string())
    print(f"\n--- {label}: bias [%] by time since the rung's first tracked sample, rungs 5-13% (under the floor); settle = first bin from which |bias| < 1%")
    sshow = show[1:] + ["model fc + offset W40 tau25", "model fc + scale W40 tau25"]
    ST = pd.concat({n: settles[n].bias for n in sshow}, axis=1)
    ST.loc["settle_ms"] = [settle_ms(settles[n]) for n in sshow]
    print(ST.round(2).to_string())
    if out_dir:
        R.to_csv(out_dir / f"{s2.slug(label)}-step3-scores.csv")
        pd.concat(by_duty, axis=1).to_csv(out_dir / f"{s2.slug(label)}-step3-by-duty.csv")
        pd.concat(settles, axis=1).to_csv(out_dir / f"{s2.slug(label)}-step3-settle.csv")
    return R, by_duty, settles


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sets", nargs="*", default=list(SETS), help="which of the named sets to score")
    ap.add_argument("--cache", type=Path, default=None, help="fusion_step2's tracker cache (optional)")
    ap.add_argument("--out", type=Path, default=None, help="directory for the score tables as CSV (optional)")
    a = ap.parse_args()
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 40)
    if a.out:
        a.out.mkdir(parents=True, exist_ok=True)
    tracked = {}
    for label in a.sets:
        cfg = SETS[label]
        cp = a.cache / f"{s2.slug(label)}.pkl" if a.cache else None
        if cp and cp.exists():
            tracked[label] = pickle.loads(cp.read_bytes())
            continue
        d = oscnb.datasets.load(cfg["key"])
        print(f"tracking {label} ({cfg['key']}) ...", file=sys.stderr)
        tracked[label] = s2.track(d, s2.recordings(d), cfg["f_floor"])
        if cp:
            cp.parent.mkdir(parents=True, exist_ok=True)
            cp.write_bytes(pickle.dumps(tracked[label]))
    for label in a.sets:
        analyse(label, SETS[label], a.cache, a.out, tracked)


if __name__ == "__main__":
    main()
