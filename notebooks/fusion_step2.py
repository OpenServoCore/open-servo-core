"""Fusion plan step 2: the slow pot-steered back-EMF correction, scored offline.

The fusion plan (bringup docs/fusion-plan.md) keeps the velocity source switch and
adds one slow state on top of the back-EMF boxcar, steered by the linearized pot:
over a window of tens of milliseconds, compare how far the pot moved with how far
the back-EMF says the shaft moved, and nudge the state so the two agree. Two forms
are on the table, and this script scores both the way notebook 09 section 8 scores
a speed estimate, against the commutation ripple:

  scale    omega_hat = k * omega_b          e = dtheta_pot - k * sum(omega_b) Ts
           k <- k + g e sign(omega_b)
  offset   omega_hat = omega_b - c          e = dtheta_pot - sum(omega_b - c) Ts
           c <- c - g e sign(omega_b)       (c is V0 / Ke, kept in counts per second)

The state updates once per medium tick (0.5 ms) over a trailing window that is a
whole multiple of 4 ms (the position channel's 250 Hz comb cancels), only while
every boxcar in the window was valid and of one sign. Otherwise it holds. That is
the plan's update rule, and it is what runs here, over each whole recording in
time order, so the state carries from rung to rung the way it would in the servo.

What is scored. On every ripple-tracked grid rung, from 20 ms after the first
tracked sample to the end (section 8's cut), the corrected 1 ms boxcar against the
ripple over the same 20 samples, in linearized counts per second, rms and bias in
percent of the band's mean speed. The pot goes through a grid table built from the
other captures of the same session, never the table the servo carried
(pos-table-refresh-policy). The back-EMF model is section 3's: R pinned at the
registry value, Ke and V0 per direction fitted on settled windows. Two fits are
scored: "stale", fitted on the partner session (the calibration a servo carries
into a new session), and "refit", fitted on the session itself.

The pass line (fusion-plan.md, step 2): no worse than the 1 ms back-EMF up to 50%
duty in rms, bias under 1% in every band, under 5% rms from 55 to 100% duty within
the length of a rung, and the time to settle reported. The governed sessions stop
at 45% (board D) and 30% (rev 2A), so the 55 to 100% line is judged on the free
board D session, the only whole-row data above 50%.

Run from notebooks/:  uv run python fusion_step2.py [--sets "D governed" ...]
                      [--cache DIR] [--out DIR]
"""

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import pandas as pd

import oscnb
from oscnb import governed, poslut, ripple, session

Q15 = 32767
M0 = 400                 # samples: 20 ms after the first tracked sample, section 8's cut
BOX = 20                 # samples: the 1 ms back-EMF boxcar (bemf.rs BOXCAR_TICKS)
MED = 10                 # fast ticks per medium tick (kernel DECIM_MED)
WINDOWS_MS = (20, 40, 80)
TAUS_MS = (10, 25, 50, 100, 250)
V_REF_CPS = 5000.0       # the shaft speed the scale gain's time constant is quoted at (about 30% duty)
MIN_ACCEPT = 0.90
CPC_TOL = 0.03
RAW = (0, (1 << poslut.ADC_BITS) - 1)
BANDS = [14, 25, 40, 50, 100]
BAND_NAMES = ["15-25%", "30-40%", "45-50%", "55-100%"]

SETS = {
    "D governed": dict(key="mg90-a__2s__limit", partner="D free", f_floor=250.0, core=(15, 50)),
    "D free": dict(key="mg90-a__2s", partner="D governed", f_floor=250.0, core=(15, 50)),
    "2A governed": dict(key="mg90-a__dev-v006-2A__2s__limit", partner="2A free", f_floor=200.0, core=(15, 30)),
    "2A free": dict(key="mg90-a__dev-v006-2A__2s__limit-1100", partner="2A governed", f_floor=200.0, core=(15, 50)),
}


# ---------------------------------------------------------------- data in

def complete(d):
    return [c for c in d.captures("session") if "slow" in {r.name for r in d.recordings("session", c)}]


def recordings(d):
    """(capture, frame, meta) of every slow session recording, whole rows checked."""
    out = []
    for c in complete(d):
        rec = [r for r in d.recordings("session", c) if r.name == "slow"][0]
        meta = rec.meta
        dropped = meta.get("rows_dropped")
        if dropped is None:
            # fw 64 did not count: the free board D set was shown whole up to 100% by the
            # frame-boundary step power (bringup docs/bemf-shift/findings.md, addendum)
            assert meta["fw"] == 64, rec
        else:
            assert dropped == 0, (rec, dropped)
        out.append((c, rec.frame(), meta))
    return out


def coupling_prior(recs, fs):
    """Ripple cycles per raw count over settled 20 to 30% rungs, to seed the tracker."""
    out = []
    for cap, df, meta in recs:
        bias = df[df.seg == 0].current_raw.mean()
        smap = {m["seg"]: m for m in session.segment_map(meta)}
        for seg, g in df[df.seg > 0].groupby("seg"):
            if smap[seg]["block"] != "grid" or smap[seg]["kind"] != "drive":
                continue
            if not 20 <= abs(g.cmd_duty_q15.iloc[0] * 100 / Q15) <= 30:
                continue
            st = governed.settled(g, fs)
            f, X = ripple.line_spectrum(st.current_raw.to_numpy() - bias, fs)
            out.append(ripple.settled_line(f, X)[0] / abs(ripple.pot_speed(st, fs, 1.0)))
    return float(np.median(out))


def track(d, recs, f_floor):
    """Every grid drive rung through the ripple tracker, notebook 09's acceptance.

    A rung carries the arrays the scoring needs and `i0`, its first row in the
    whole recording, so the replay over the recording can find it again.
    """
    fs = d.board.tick_hz
    cp = coupling_prior(recs, fs)
    out = []
    for cap, df, meta in recs:
        bias = df[df.seg == 0].current_raw.mean()
        df = df.assign(bias=bias)
        smap = {m["seg"]: m for m in session.segment_map(meta)}
        for seg, g in df[df.seg > 0].groupby("seg"):
            if smap[seg]["block"] != "grid" or smap[seg]["kind"] != "drive":
                continue
            duty = g.cmd_duty_q15.iloc[0] * 100 / Q15
            if abs(duty) < 12 or g.cmd_duty_q15.nunique() > 1:
                continue
            r = {"cap": cap, "seg": int(seg), "duty": int(round(duty)), "n": len(g), "i0": int(g.index[0]),
                 "pos": g.pos.to_numpy(), "cur": g.current_raw.to_numpy() - bias,
                 "va": g.vmotor_a.to_numpy().astype(float), "vb": g.vmotor_b.to_numpy().astype(float),
                 "dq": g.duty_q15.to_numpy(), "valid": g.window_valid.to_numpy(),
                 "s0": governed.settled_from(g, fs)}
            cyc, n0, _ = ripple.motor_angle(g, cp, fs, 1.0, f_floor=f_floor)
            if cyc is None or r["s0"] is None:
                r.update(ok=False)
                out.append(r)
                continue
            p = r["pos"]
            r.update(cyc=cyc, n0=n0, accept=(len(g) - n0) / len(g),
                     cpc=(cyc[-1] - cyc[n0]) / abs(p[-1] - p[n0]) if p[-1] != p[n0] else np.nan)
            out.append(r)
    med = np.median([r["cpc"] for r in out if "cyc" in r and 15 <= abs(r["duty"]) <= 50])
    for r in out:
        r["ok"] = "cyc" in r and r["accept"] >= MIN_ACCEPT and abs(r["cpc"] / med - 1) < CPC_TOL
    return out


# ---------------------------------------------------------------- the pot table and the model

def core_of(rungs, core):
    return [r for r in rungs if r["ok"] and core[0] <= abs(r["duty"]) <= core[1]]


def chunks(rs):
    return [(r["pos"][r["n0"]:], r["cyc"][r["n0"]:]) for r in rs]


def loo_tables(rungs, core):
    """Per capture: the grid table stitched from the other captures' core rungs, and
    its ripple cycles per linearized count over the band it is anchored on."""
    rs = core_of(rungs, core)
    band = poslut.well_covered(poslut.stitch(chunks(rs), *RAW))
    out = {}
    for c in sorted({r["cap"] for r in rungs}):
        s = poslut.stitch(chunks([r for r in rs if r["cap"] != c]), *RAW)
        b = (max(band[0], s.lo), min(band[1], s.hi))
        c_lin = (s.angle(b[1]) - s.angle(b[0])) / (b[1] - b[0])
        out[c] = (poslut.fit_grid(s, band=b), c_lin, b)
    return out


def settled_point(r, board, dt):
    a = r["s0"]
    if r["n0"] > a:
        return None
    sg = np.sign(r["duty"])
    D = np.abs(r["dq"][a:]) / Q15
    vd = (r["va"][a:] - r["vb"][a:]) * sg * board.v_term_per_count
    i = r["cur"][a:] * board.a_per_count
    f = (r["cyc"][-1] - r["cyc"][a]) / ((len(r["pos"]) - 1 - a) * dt)
    return {"dir": int(sg), "y": (D * vd).mean(), "i": i.mean(), "f": f}


def fit_ke(rungs, core, board, dt, r_ohm):
    """Section 3's fit: EMF on ripple frequency per direction, R pinned.
    Returns {dir: (Ke V per Hz, V0 V)}."""
    pts = pd.DataFrame([p for r in core_of(rungs, core) for p in [settled_point(r, board, dt)] if p])
    out = {}
    for sg in (1, -1):
        g = pts[pts.dir == sg]
        out[sg] = tuple(np.polyfit(g.f, g.y - r_ohm * g.i, 1))
    return out


# ---------------------------------------------------------------- the replay

def medium_series(df, rungs, lut, c_lin, ke, r_ohm, board, dt):
    """Everything the correction and the score read, on the medium-tick grid of one
    whole recording. Returns a dict of arrays indexed by medium tick t, whose boxcar
    closes on sample j = MED * t + MED - 1 and covers samples j - BOX + 1 .. j."""
    n = len(df)
    sg = np.sign(df.duty_q15.to_numpy()).astype(int)
    bias = df[df.seg == 0].current_raw.mean()
    D = np.abs(df.duty_q15.to_numpy()) / Q15
    vd = (df.vmotor_a.to_numpy().astype(float) - df.vmotor_b.to_numpy()) * sg * board.v_term_per_count
    i = (df.current_raw.to_numpy() - bias) * board.a_per_count
    ke_s = np.where(sg > 0, ke[1][0], ke[-1][0])
    v0_s = np.where(sg > 0, ke[1][1], ke[-1][1])
    # the per-sample model: ripple Hz over c_lin is linearized counts per second, signed
    w = sg * (D * vd - r_ohm * i - v0_s) / ke_s / c_lin
    valid = (df.window_valid.to_numpy() == 1) & (sg != 0)
    w = np.where(valid, w, 0.0)
    # the ripple truth, where a rung is accepted: its own angle in cycles from n0, signed
    cyc = np.full(n, np.nan)
    scored = np.full(n, -1)                     # index into rungs, from n0 + M0 to the end
    for idx, r in enumerate(rungs):
        if r["ok"]:
            a, b = r["i0"] + r["n0"], r["i0"] + r["n"]
            cyc[a:b] = r["cyc"][r["n0"]:] * np.sign(r["duty"])
            scored[r["i0"] + r["n0"] + M0:b] = idx
    pos_lin = lut.counts(df.pos.to_numpy())
    # cumulative sums with a leading zero, so a window [a, b) is c[b] - c[a]
    cw, cv = np.r_[0, np.cumsum(w)], np.r_[0, np.cumsum(~valid)]
    j = np.arange(MED - 1, n, MED)
    j = j[j >= BOX]
    wb = (cw[j + 1] - cw[j + 1 - BOX]) / BOX
    box_valid = (cv[j + 1] - cv[j + 1 - BOX]) == 0
    # the mean of 20 per-sample speeds against the angle swept by those 20 increments
    v_ref = (cyc[j] - cyc[j - BOX]) / (BOX * dt) / c_lin
    # the 20 ms pot speed, the reference row section 8 carries beside the boxcar
    W20 = 400
    v_pot20 = np.full(len(j), np.nan)
    v_ref20 = np.full(len(j), np.nan)
    m = j >= W20
    v_pot20[m] = (pos_lin[j[m]] - pos_lin[j[m] - W20]) / (W20 * dt)
    v_ref20[m] = (cyc[j[m]] - cyc[j[m] - W20]) / (W20 * dt) / c_lin
    return {"j": j, "wb": wb, "valid": box_valid, "sgn": sg[j], "pot": pos_lin[j],
            "v_ref": v_ref, "rung": scored[j], "v_pot20": v_pot20, "v_ref20": v_ref20,
            "seg": df.seg.to_numpy()[j]}


def window_terms(ms, Wm, dt_med):
    """Per medium tick: whether the trailing Wm-tick window is all valid, one sign
    and inside one TEL segment, the pot's move over it and the boxcar's integral
    over it. A capture is a chain of bursts, one per segment, with the seek and
    rest between them unrecorded (the tick restarts at 0 and the pot jumps), so a
    window across a segment boundary would read the seek as a scale error."""
    valid, sgn, wb, pot, seg = ms["valid"], ms["sgn"], ms["wb"], ms["pot"], ms["seg"]
    T = len(wb)
    cinv = np.r_[0, np.cumsum(~valid)]
    chg = np.r_[0, ((sgn[1:] != sgn[:-1]) | (seg[1:] != seg[:-1])).astype(int)]
    cchg = np.r_[0, np.cumsum(chg)]
    cw = np.r_[0, np.cumsum(np.where(valid, wb, 0.0))]
    t = np.arange(T)
    ok = t >= Wm
    a = np.maximum(t + 1 - Wm, 0)
    ok &= (cinv[t + 1] - cinv[a]) == 0
    # no sign or segment change at any u in [t - Wm + 1, t]: sgn and seg agree over [t - Wm, t]
    ok &= (cchg[t + 1] - cchg[np.minimum(a, t + 1)]) == 0
    dth = np.where(ok, pot[t] - pot[np.maximum(t - Wm, 0)], 0.0)
    S = (cw[t + 1] - cw[a]) * dt_med
    return ok, dth, S


def run_state(ok, dth, S, sgn, form, g, Ws, x0):
    """The recursion over medium ticks; the state after each tick, held where the
    window is not usable."""
    T = len(ok)
    xs = np.empty(T)
    x = x0
    ok_l, dth_l, S_l, sg_l = ok.tolist(), dth.tolist(), S.tolist(), sgn.tolist()
    if form == "scale":
        for t in range(T):
            if ok_l[t]:
                e = dth_l[t] - x * S_l[t]
                x += g * e * sg_l[t]
            xs[t] = x
    else:
        for t in range(T):
            if ok_l[t]:
                e = dth_l[t] - (S_l[t] - sg_l[t] * x * Ws)
                x -= g * e * sg_l[t]
            xs[t] = x
    return xs


def gain(form, tau_ms, Ws, dt_med):
    """g for a time constant tau: for the scale form at V_REF_CPS (its loop gain is
    the travel per window, so it tightens with speed), for the offset form at any
    speed (its loop gain is the window length)."""
    per_update = dt_med / (tau_ms * 1e-3)
    return per_update / (V_REF_CPS * Ws) if form == "scale" else per_update / Ws


def estimate(form, xs, ms):
    return xs * ms["wb"] if form == "scale" else ms["wb"] - ms["sgn"] * xs


# ---------------------------------------------------------------- scoring

def score(est, ms, rungs, duties):
    """rms and bias in percent of the band's mean reference speed, over the scored
    ticks of accepted rungs, per band."""
    m = (ms["rung"] >= 0) & ms["valid"] & np.isfinite(ms["v_ref"]) & np.isfinite(est)
    d = duties[ms["rung"][m]]
    err = est[m] - ms["v_ref"][m]
    t = pd.DataFrame({"band": pd.cut(np.abs(d), BANDS, labels=BAND_NAMES),
                      "err": err, "err_dir": err * np.sign(d), "v": np.abs(ms["v_ref"][m])})
    g = t.groupby("band", observed=True)
    return pd.DataFrame({"rms": np.sqrt(g.err.apply(lambda e: np.mean(e ** 2))) / g.v.mean() * 100,
                         "bias": g.err_dir.mean() / g.v.mean() * 100, "ticks": g.size()})


def score_by_duty(est, ms, rungs, duties):
    m = (ms["rung"] >= 0) & ms["valid"] & np.isfinite(ms["v_ref"]) & np.isfinite(est)
    d = duties[ms["rung"][m]]
    err = (est[m] - ms["v_ref"][m]) * np.sign(d)
    t = pd.DataFrame({"duty": np.abs(d), "err": err, "v": np.abs(ms["v_ref"][m])})
    g = t.groupby("duty")
    return g.err.mean() / g.v.mean() * 100


def pot20_score(ms, rungs, duties):
    m = (ms["rung"] >= 0) & np.isfinite(ms["v_ref20"]) & np.isfinite(ms["v_pot20"])
    d = duties[ms["rung"][m]]
    err = ms["v_pot20"][m] - ms["v_ref20"][m]
    t = pd.DataFrame({"band": pd.cut(np.abs(d), BANDS, labels=BAND_NAMES),
                      "err": err, "err_dir": err * np.sign(d), "v": np.abs(ms["v_ref20"][m])})
    g = t.groupby("band", observed=True)
    return pd.DataFrame({"rms": np.sqrt(g.err.apply(lambda e: np.mean(e ** 2))) / g.v.mean() * 100,
                         "bias": g.err_dir.mean() / g.v.mean() * 100, "ticks": g.size()})


SETTLE_BINS_MS = [0, 20, 40, 80, 120, 160, 1000]


def cold_start(ms, rungs, duties, Wm, form, g, Ws, dt_med):
    """Cold start on every scored rung: the state reset at the rung's first boxcar
    (k = 1, c = 0) and the recursion run over the rung alone, so the estimate's error
    against the ripple can be read by time since the rung's first valid boxcar. The
    time to settle is the first bin from which the bias stays under 1%. The carried
    replay above is what the servo would do; this is the worst case, a servo that
    forgot its state."""
    x0 = 1.0 if form == "scale" else 0.0
    parts = []
    for idx in np.unique(ms["rung"][ms["rung"] >= 0]):
        r = rungs[idx]
        if abs(r["duty"]) < 15:
            continue
        sel = np.flatnonzero((ms["j"] >= r["i0"]) & (ms["j"] < r["i0"] + r["n"]))
        if len(sel) < 2 * Wm:
            continue
        sub = {k: ms[k][sel] for k in ("valid", "sgn", "wb", "pot", "seg")}
        ok, dth, S = window_terms(sub, Wm, dt_med)
        xs = run_state(ok, dth, S, sub["sgn"], form, g, Ws, x0)
        est = estimate(form, xs, sub)
        first = int(np.flatnonzero(sub["valid"])[0])
        m = (ms["rung"][sel] == idx) & sub["valid"] & np.isfinite(ms["v_ref"][sel])
        ref = ms["v_ref"][sel][m]
        parts.append(pd.DataFrame({"t_ms": (np.flatnonzero(m) - first) * dt_med * 1e3,
                                   "err_dir": (est[m] - ref) * np.sign(r["duty"]), "v": np.abs(ref)}))
    t = pd.concat(parts)
    t["bin"] = pd.cut(t.t_ms, SETTLE_BINS_MS, right=False)
    gr = t.groupby("bin", observed=True)
    return pd.DataFrame({"bias": gr.err_dir.mean() / gr.v.mean() * 100,
                         "rms": np.sqrt(gr.err_dir.apply(lambda e: np.mean(e ** 2))) / gr.v.mean() * 100,
                         "ticks": gr.size()})


def settle_ms(cs):
    """The left edge of the first bin from which |bias| stays under 1%."""
    ok = (cs.bias.abs() < 1.0).to_numpy()
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
    recs = recordings(d)
    rungs = tracked[label]
    frames = {c: df for c, df, meta in recs}
    duties = np.array([r["duty"] for r in rungs])
    acc = pd.DataFrame([{"duty": abs(r["duty"]), "ok": r["ok"]} for r in rungs]).groupby("duty").ok.agg(["size", "sum"])
    print(f"\n{'=' * 100}\n{label}: {cfg['key']} on {board.key}, R pinned {r_ohm} ohm, "
          f"{len(recs)} captures, rail at the start {min(board.rail_from_vbus_counts(m['vbus_counts']) for _, _, m in recs):.2f} "
          f".. {max(board.rail_from_vbus_counts(m['vbus_counts']) for _, _, m in recs):.2f} V")
    print("rungs per duty (all, accepted):", {int(k): f"{v['size']}/{int(v['sum'])}" for k, v in acc.iterrows()})

    tables = loo_tables(rungs, cfg["core"])
    print("leave-one-out tables, band and ripple cycles per linearized count:",
          {c: (b, round(cl, 4)) for c, (_, cl, b) in tables.items()})

    models = {"refit": fit_ke(rungs, cfg["core"], board, dt, r_ohm)}
    pcfg = SETS[cfg["partner"]]
    models["stale"] = fit_ke(tracked[cfg["partner"]], pcfg["core"], board, dt, r_ohm)
    for name, ke in models.items():
        src = cfg["key"] if name == "refit" else pcfg["key"]
        print(f"model {name} ({src}): " + ", ".join(
            f"{'fwd' if s > 0 else 'rev'} Ke {ke[s][0] * 1e3:.3f} mV/Hz V0 {ke[s][1] * 1e3:+.0f} mV" for s in (1, -1)))

    rows, by_duty, settle = [], [], []
    for mname, ke in models.items():
        series = {c: medium_series(frames[c], [r if r["cap"] == c else dict(r, ok=False) for r in rungs],
                                   tables[c][0], tables[c][1], ke, r_ohm, board, dt) for c in frames}
        for est_name, est_of in (("back-EMF 1 ms", lambda ms: ms["wb"]),):
            parts = [score(est_of(ms), ms, rungs, duties) for ms in series.values()]
            sc = pooled(parts)
            rows.append(dict(model=mname, est=est_name, form="-", W_ms=0, tau_ms=0, **flat(sc)))
            by_duty.append(pd.concat([score_by_duty(est_of(ms), ms, rungs, duties) for ms in series.values()],
                                     axis=1).mean(axis=1).rename((mname, est_name)))
        if mname == "refit":
            sc = pooled([pot20_score(ms, rungs, duties) for ms in series.values()])
            rows.append(dict(model="-", est="pot 20 ms (LOO table)", form="-", W_ms=0, tau_ms=0, **flat(sc)))
        for W in WINDOWS_MS:
            Wm = int(round(W * 1e-3 / dt_med))
            Ws = Wm * dt_med
            for form in ("scale", "offset"):
                for tau in TAUS_MS:
                    g = gain(form, tau, Ws, dt_med)
                    x0 = 1.0 if form == "scale" else 0.0
                    parts, bd, st = [], [], []
                    for c, ms in series.items():
                        ok, dth, S = window_terms(ms, Wm, dt_med)
                        xs = run_state(ok, dth, S, ms["sgn"], form, g, Ws, x0)
                        est = estimate(form, xs, ms)
                        parts.append(score(est, ms, rungs, duties))
                        bd.append(score_by_duty(est, ms, rungs, duties))
                        st.append(cold_start(ms, rungs, duties, Wm, form, g, Ws, dt_med))
                    sc = pooled(parts)
                    rows.append(dict(model=mname, est=f"{form} W{W} tau{tau}", form=form, W_ms=W, tau_ms=tau, **flat(sc)))
                    by_duty.append(pd.concat(bd, axis=1).mean(axis=1).rename((mname, f"{form} W{W} tau{tau}")))
                    cs = pooled(st)
                    settle.append(dict(model=mname, form=form, W_ms=W, tau_ms=tau, settle_ms=settle_ms(cs),
                                       **{f"cold bias {b.left}-{b.right} ms": cs.loc[b, "bias"] for b in cs.index}))

    R = pd.DataFrame(rows)
    S = pd.DataFrame(settle)
    BD = pd.concat(by_duty, axis=1)
    verdict(label, R, S)
    print("\nbias by duty [%], refit model, baseline and the best of each form by the 50% bar")
    show = ["back-EMF 1 ms"] + [best(R, "refit", f) for f in ("scale", "offset")]
    print(BD["refit"][[c for c in show if c in BD["refit"]]].round(2).to_string())
    if "stale" in BD:
        print("\nbias by duty [%], stale model")
        show = ["back-EMF 1 ms"] + [best(R, "stale", f) for f in ("scale", "offset")]
        print(BD["stale"][[c for c in show if c in BD["stale"]]].round(2).to_string())
    if out_dir:
        R.to_csv(out_dir / f"{slug(label)}-scores.csv", index=False)
        S.to_csv(out_dir / f"{slug(label)}-settle.csv", index=False)
        BD.to_csv(out_dir / f"{slug(label)}-by-duty.csv")
    return R, S, BD


def pooled(parts):
    """Band scores pooled over captures, weighted by ticks."""
    t = pd.concat(parts)
    w = t.ticks
    g = t.groupby(level=0, observed=True)
    return pd.DataFrame({"rms": np.sqrt((t.rms ** 2 * w).groupby(level=0, observed=True).sum() / g.ticks.sum()),
                         "bias": (t.bias * w).groupby(level=0, observed=True).sum() / g.ticks.sum(),
                         "ticks": g.ticks.sum()})


def flat(sc):
    out = {}
    for b in BAND_NAMES:
        if b in sc.index:
            out[f"rms {b}"] = sc.loc[b, "rms"]
            out[f"bias {b}"] = sc.loc[b, "bias"]
    return out


def slug(s):
    return s.lower().replace(" ", "-")


def passes(row, base, bands):
    low = [b for b in bands if b != "55-100%"]
    rms_ok = all(row[f"rms {b}"] <= base[f"rms {b}"] + 1e-9 for b in low)
    bias_ok = all(abs(row[f"bias {b}"]) < 1.0 for b in bands)
    high_ok = row["rms 55-100%"] < 5.0 if "55-100%" in bands else np.nan
    return rms_ok, bias_ok, high_ok


def best(R, model, form):
    """The config of one form that passes the most of the bar, then the lowest
    worst-band |bias|, then the lowest rms up to 50%."""
    bands = [b for b in BAND_NAMES if f"rms {b}" in R]
    base = R[(R.model == model) & (R.est == "back-EMF 1 ms")].iloc[0]
    t = R[(R.model == model) & (R.form == form)].copy()
    p = t.apply(lambda r: passes(r, base, bands), axis=1, result_type="expand")
    t["score"] = p[0].astype(int) + p[1].astype(int) + p[2].fillna(0).astype(int)
    t["worst_bias"] = t[[f"bias {b}" for b in bands]].abs().max(axis=1)
    t["rms_low"] = t[[f"rms {b}" for b in bands if b != "55-100%"]].mean(axis=1)
    return t.sort_values(["score", "worst_bias", "rms_low"], ascending=[False, True, True]).iloc[0].est


def verdict(label, R, S):
    bands = [b for b in BAND_NAMES if f"rms {b}" in R]
    cols = [c for b in bands for c in (f"rms {b}", f"bias {b}")]
    for model in ("refit", "stale"):
        base = R[(R.model == model) & (R.est == "back-EMF 1 ms")].iloc[0]
        t = R[R.model.isin([model, "-"])].copy()
        p = t.apply(lambda r: passes(r, base, bands), axis=1, result_type="expand")
        t["rms<=base"], t["|bias|<1"], t["55-100<5"] = p[0], p[1], p[2]
        t = t.merge(S[S.model == model][["form", "W_ms", "tau_ms", "settle_ms"]],
                    on=["form", "W_ms", "tau_ms"], how="left")
        print(f"\n--- {label}, model {model}: rms and bias in % of band speed; pass columns against the plan's step 2 bar;"
              f" settle_ms = cold start, first bin from which |bias| < 1%")
        print(t.set_index("est")[cols + ["rms<=base", "|bias|<1", "55-100<5", "settle_ms"]].round(2).to_string())
        print(f"\ncold-start bias by time since the first valid boxcar [%], model {model}, best of each form")
        cb = [c for c in S.columns if c.startswith("cold bias")]
        s = S[S.model == model].assign(est=lambda t: t.form + " W" + t.W_ms.astype(str) + " tau" + t.tau_ms.astype(str))
        print(s[s.est.isin([best(R, model, f) for f in ("scale", "offset")])].set_index("est")[cb].round(2).to_string())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sets", nargs="*", default=list(SETS), help="which of the named sets to score")
    ap.add_argument("--cache", type=Path, default=None, help="directory for the tracker's cache (optional)")
    ap.add_argument("--out", type=Path, default=None, help="directory for the score tables as CSV (optional)")
    a = ap.parse_args()
    pd.set_option("display.width", 250)
    pd.set_option("display.max_columns", 40)
    if a.out:
        a.out.mkdir(parents=True, exist_ok=True)
    # every set's partner is tracked too: the stale model is fitted on it
    need = sorted({s for s in a.sets} | {SETS[s]["partner"] for s in a.sets})
    tracked = {}
    for label in need:
        cfg = SETS[label]
        cp = a.cache / f"{slug(label)}.pkl" if a.cache else None
        if cp and cp.exists():
            tracked[label] = pickle.loads(cp.read_bytes())
            continue
        d = oscnb.datasets.load(cfg["key"])
        print(f"tracking {label} ({cfg['key']}) ...", file=sys.stderr)
        tracked[label] = track(d, recordings(d), cfg["f_floor"])
        if cp:
            cp.parent.mkdir(parents=True, exist_ok=True)
            cp.write_bytes(pickle.dumps(tracked[label]))
    for label in a.sets:
        analyse(label, SETS[label], a.cache, a.out, tracked)


if __name__ == "__main__":
    main()
