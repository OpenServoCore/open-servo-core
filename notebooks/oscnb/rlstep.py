"""Autonomous R/L identification from a duty-toggle ladder.

WHAT THIS MEASURES

A brushed DC motor on a slow-decay H bridge, 20 kHz PWM, one telemetry sample
per PWM period taken at the ON centre (the crest). The shaft is free at mid
travel. The applied duty is toggled between two levels every 1 ms (20 samples)
or 2 ms (40 samples) in a long chain: 15, 25, 15, 25, ...

Per PWM period the winding sees D*V of drive and, in slow decay, a brake for
the rest, so the mean current obeys

    L di/dt = D*V - E - i*R_loop

with E the back EMF. The design premise was that the rotor's mechanical time
constant (tens of ms) keeps E the SAME constant in the level before the step
and the level after it, so that it cancels in the difference along with any
fixed bridge offset V0 (brush drop, dead-time volt-seconds), leaving a pure
RL transient between two settled levels:

    tau = L / R_loop          from the shape  (voltage free, V0 free)
    R_loop = d(D*V) / di      from the amplitude
    L = tau * R_loop

Nothing here needs a locked rotor, a bench supply, or a known V0. That is the
point: a servo can run this on itself.

MEASURED CAVEAT - THE PREMISE FAILS AT 1 ms STEPS. On sg90-a the output shaft
speed modulates 30 to 50% AT THE TOGGLE RATE (500 Hz): on a 15/25% chain the
pot advances 2.3 counts during a low step and 1.2 during a high step, and the
current keeps moving inside a step after the RL rise has settled (+4 counts
in high steps, -3 to -10 in low steps). The rotor and its backlashed, elastic
train respond at kHz rates, E toggles with the duty, and the differential R
reads 8 to 9 Ohm at 15-25% bias falling to 5.5 at 60-70% against 3.9 with the
rotor locked; tau reads ~130 us against 154. The pole is less affected than
the amplitude but not immune. The fix is a step the rotor cannot follow, a
few PWM periods long, which needs the firmware high-rate shunt burst; until
then this module characterises the bias, it does not identify the motor.

ALIGNMENT, AND THE ONE-SAMPLE SKEW

duty_q15 is the applied duty register read back with the sample. It flips one
sample BEFORE the current responds - the register is written at the top of the
period and the winding only sees the new pulse in the period after. So:

    k = 0   first sample whose duty_q15 reports the new duty.
            Its current, and its vmotor taps, still belong to the OLD level.
    k = 1   first sample carrying the new drive.

The fit therefore runs over k = 1..N, and the duty in force at sample k is
duty_q15[k-1]. Getting this backwards puts a settled OLD sample at the head of
the rise and biases tau long by roughly one tick.

The step edge is NOT the segment boundary. The recorder starts a new seg when
it writes the new duty, but the register read-back lands one to three samples
later, and segments come in runs of 19 to 21 samples. Everything below is
indexed off the observed duty change, and falls back to the segment start only
for a null ladder (same duty on both sides), where there is no change to find.

WHAT CANCELS AND WHAT DOES NOT

  cancels:  back EMF, bridge offset V0, shunt amplifier bias, and - in the
            free-intercept form of fit_r - the secular drift of the DC current
            level as the rotor spins up over the chain.
  does not: a multiplicative error in the volt-second scale (sampling the ON
            voltage at one point in a pulse whose amplitude is not flat), and
            any error in the current scale. Both land straight on R and L.

USE

    ds = datasets.load("sg90-a__dev-v006-D__usb")
    s  = rlstep.summarise(ds, "rlstep-c15-25", "capture-1")
    df = rlstep.campaign(ds)
"""

import numpy as np
import pandas as pd

from scipy.optimize import least_squares

Q15 = 32767.0

# last N samples of a level taken as its settled value. Six samples at 50 us is
# 300 us, which is about two electrical time constants wide and sits at the end
# of a 1 ms step, so the head of the window is already ~2.5 tau past the edge.
SETTLE = 6

# default pole-fit length. A 1 ms step is 20 samples, so 20 - 6 - 2 = 12 leaves
# the fit clear of the settled window; a 2 ms step would give 32, capped at 30.
FIT_CAP = 30


# --------------------------------------------------------------------------
# transitions
# --------------------------------------------------------------------------

def transitions(df, board, settle=SETTLE):
    """One row per duty step in the ladder.

    Walks each direction's chain separately (the recorder runs the whole
    schedule forward, then the whole schedule reversed) and cuts it at every
    observed duty change. For each cut it takes a settled level on both sides
    and carries the raw rise along in the row.

    Returned columns:
      dirn, step        direction and position in that direction's chain
      k0, n             edge index in the direction stream, samples in the step
      d_pre, d_post     duty as a fraction, from the register one sample early
      up, stepped       up-step flag; stepped is False for a null ladder
      i_pre, i_post, di settled currents and their difference, A
      vt_pre, vt_post   ON-state terminal volts, signed so drive is positive
      vr_pre, vr_post   ON-state rail volts from the direct vbus tap
      dvt, dvr          d(D*V) terminal referenced and rail referenced, V
      r_term, r_rail    dvt/di and dvr/di for this transition alone, ohm
      pos_pre, pos_post pot counts, for the drift check
      rise              array x_k, k = 0..n-1, normalised to this step
      gain              array (i_k - i_pre)/dvt, k = 0..n-1, siemens

    `rise` is normalised to its own step so transitions of different sign and
    size pool into one ensemble. `gain` is the same rise in conductance units,
    which is what exposes a plateau that droops (an EMF that did not hold
    still) rather than hiding it in the normalisation.
    """
    rest = df[df.seg == 0]
    if len(rest) == 0:
        raise ValueError("no seg 0 baseline: the shunt bias has nowhere to come from")
    bias = rest.current_raw.median()

    rows = []
    for dirn in (1, -1):
        s = df[(df.seg >= 1) & (df.dir == dirn)].reset_index(drop=True)
        if len(s) == 0:
            continue
        duty_q = s.duty_q15.to_numpy()
        d = np.abs(duty_q) / Q15
        i = board.amps(s.current_raw.to_numpy(), bias)
        # terminal difference is bias free by construction; the sign follows the
        # commanded direction so that "drive" is positive in both halves.
        vt = board.diff_v(s.vmotor_a.to_numpy(), s.vmotor_b.to_numpy()) * dirn
        vr = board.vsys_v(s.vbus_raw.to_numpy())
        pos = s.pos.to_numpy()
        valid = s.window_valid.to_numpy().astype(bool)
        seg = s.seg.to_numpy()

        edges = _edges(seg, duty_q)
        for j, k0 in enumerate(edges):
            lo = edges[j - 1] if j else 0
            hi = edges[j + 1] if j + 1 < len(edges) else len(s)
            # The first edge of a chain steps out of torque-off. There is no
            # settled previous level to difference against, so it is not a
            # transition in the sense this method means.
            if duty_q[k0 - 1] == 0 or k0 == 0:
                continue
            pre = slice(max(lo, k0 - settle + 1), k0 + 1)
            post = slice(max(k0, hi - settle), hi)
            if pre.stop - pre.start < 3 or post.stop - post.start < 3:
                continue
            if not (valid[pre].all() and valid[post].all()):
                continue

            d_pre, d_post = d[k0 - 1], d[k0]
            i_pre, i_post = np.median(i[pre]), np.median(i[post])
            vt_pre, vt_post = np.median(vt[pre]), np.median(vt[post])
            vr_pre, vr_post = np.median(vr[pre]), np.median(vr[post])
            dvt = d_post * vt_post - d_pre * vt_pre
            dvr = d_post * vr_post - d_pre * vr_pre
            di = i_post - i_pre
            stepped = d_post != d_pre
            rows.append(dict(
                dirn=dirn, step=j, k0=k0, n=hi - k0,
                d_pre=d_pre, d_post=d_post, up=d_post > d_pre, stepped=stepped,
                i_pre=i_pre, i_post=i_post, di=di,
                vt_pre=vt_pre, vt_post=vt_post, vr_pre=vr_pre, vr_post=vr_post,
                dvt=dvt, dvr=dvr,
                r_term=dvt / di if stepped and di else np.nan,
                r_rail=dvr / di if stepped and di else np.nan,
                pos_pre=np.median(pos[pre]), pos_post=np.median(pos[post]),
                rise=(i[k0:hi] - i_pre) / di if stepped and di else None,
                gain=(i[k0:hi] - i_pre) / dvt if stepped and dvt else None,
            ))
    return pd.DataFrame(rows)


def _edges(seg, duty_q):
    """Indices where a new level begins, one per segment.

    Inside a segment the register flips once, one to three samples in. Take
    that sample. A null ladder never flips, so fall back to the segment's own
    first sample, which is where the (identical) duty was rewritten.
    """
    out = []
    starts = np.flatnonzero(np.diff(seg)) + 1
    for a, b in zip(starts, list(starts[1:]) + [len(seg)]):
        flip = np.flatnonzero(duty_q[a:b] != duty_q[a - 1])
        out.append(a + flip[0] if len(flip) else a)
    return out


# --------------------------------------------------------------------------
# tau: the pole of the ensemble rise
# --------------------------------------------------------------------------

def _stack(tr, col, nfit):
    """Ragged rises into one (transitions x nfit+1) array, NaN padded."""
    rows = [r for r in tr[col] if r is not None]
    m = np.full((len(rows), nfit + 1), np.nan)
    for j, r in enumerate(rows):
        take = min(nfit + 1, len(r))
        m[j, :take] = r[:take]
    return m


def _pole(y, k):
    """Fit x_k = 1 - A*a^k and return (a, A, residual rms).

    A is free rather than pinned at 1. The voltage edge lands somewhere inside
    the tick between k=0 and k=1, not exactly on the sample instant, and a free
    A absorbs that sub-sample phase - it is a time origin, not a shape. Pinning
    A = 1 folds the phase into the pole and reads tau long.
    """
    ok = np.isfinite(y)
    if ok.sum() < 3:
        return np.nan, np.nan, np.nan

    def resid(p):
        return (1.0 - p[0] * p[1] ** k[ok]) - y[ok]

    r = least_squares(resid, [1.0, 0.7], bounds=([0.0, 0.05], [5.0, 0.995]))
    return r.x[1], r.x[0], float(np.sqrt(np.mean(r.fun ** 2)))


def _pole_free(y, k):
    """Same, with the asymptote C free: x_k = C - A*a^k.

    Cross-check on the normalisation. The post-step settled window sits about
    2.5 tau past the edge, so it still carries a percent or two of the
    transient; that pulls the normaliser low and the fitted pole with it. If C
    comes back near 1 the normalisation was honest, and if it comes back well
    above 1 the step never settled inside its window.
    """
    ok = np.isfinite(y)
    if ok.sum() < 4:
        return np.nan, np.nan, np.nan

    def resid(p):
        return (p[2] - p[0] * p[1] ** k[ok]) - y[ok]

    r = least_squares(resid, [1.0, 0.7, 1.0],
                      bounds=([0.0, 0.05, 0.5], [5.0, 0.995, 2.0]))
    return r.x[1], r.x[2], float(np.sqrt(np.mean(r.fun ** 2)))


def fit_tau(tr, tick_hz, nfit=None, boot=400, seed=0):
    """Pole fit on the ensemble of rises. Returns a dict of microseconds.

    The ensemble is a per-k MEDIAN across transitions, not a mean: a single
    transition that caught a commutator event or a rail glitch then moves the
    ensemble by nothing. Up-steps and down-steps enter the same ensemble, which
    is what makes the secular drift of the DC level cancel - it appears with
    opposite sign in the two, and the normalisation flips the down-steps back.

    Keys:
      tau_us, tau_lo_us, tau_hi_us   ensemble pole and its bootstrap bracket
      a, amp, rms                    fitted pole, amplitude, residual
      tau_free_us, asymptote         free-asymptote cross-check
      tau_med_us, tau_iqr_us, tau_p  per-transition distribution
      nfit, n                        fit length and transitions used
    """
    rises = [r for r in tr["rise"] if r is not None]
    if not rises:
        return dict(tau_us=np.nan, n=0)
    nmed = int(np.median([len(r) for r in rises]))
    if nfit is None:
        nfit = int(min(FIT_CAP, max(4, nmed - SETTLE - 2)))

    m = _stack(tr, "rise", nfit)
    if not np.isfinite(m).any():
        return dict(tau_us=np.nan, n=0)
    k = np.arange(nfit + 1)
    tick_us = 1e6 / tick_hz

    def tau_of(rows):
        med = np.nanmedian(rows, axis=0)
        a, amp, rms = _pole(med[1:], k[1:])
        return a, amp, rms, med

    a, amp, rms, med = tau_of(m)
    a_free, asym, _ = _pole_free(np.nanmedian(m, axis=0)[1:], k[1:])

    rng = np.random.default_rng(seed)
    taus = []
    for _ in range(boot):
        pick = rng.integers(0, len(m), len(m))
        ab = tau_of(m[pick])[0]
        if np.isfinite(ab) and 0 < ab < 1:
            taus.append(-tick_us / np.log(ab))
    lo, hi = (np.percentile(taus, [2.5, 97.5]) if taus else (np.nan, np.nan))

    per = np.array([_pole(row[1:], k[1:])[0] for row in m])
    per = per[np.isfinite(per) & (per > 0) & (per < 1)]
    per_tau = -tick_us / np.log(per) if len(per) else np.array([])

    return dict(
        tau_us=-tick_us / np.log(a) if 0 < a < 1 else np.nan,
        tau_lo_us=lo, tau_hi_us=hi, a=a, amp=amp, rms=rms,
        tau_free_us=-tick_us / np.log(a_free) if 0 < a_free < 1 else np.nan,
        asymptote=asym,
        tau_med_us=float(np.median(per_tau)) if len(per_tau) else np.nan,
        tau_iqr_us=float(np.subtract(*np.percentile(per_tau, [75, 25]))) if len(per_tau) else np.nan,
        tau_p=per_tau, curve=np.nanmedian(m, axis=0), nfit=nfit, n=len(m),
    )


def tau_vs_nfit(tr, tick_hz, lengths=(6, 8, 10, 12, 16, 20, 30)):
    """tau as the fit window grows. A pole that is really a pole does not care.

    A rise contaminated by a second, slower process - an EMF that is starting
    to move, a filter downstream of the shunt - reads a different tau at every
    window length, and the trend says which way the contamination runs.
    """
    have = [len(r) for r in tr["rise"] if r is not None]
    if not have:
        return {}
    out = {}
    nmax = int(np.median(have))
    for n in lengths:
        if n < nmax:
            out[n] = fit_tau(tr, tick_hz, nfit=n, boot=0)["tau_us"]
    return out


# --------------------------------------------------------------------------
# R: the amplitude of the step
# --------------------------------------------------------------------------

def _through_origin(x, y):
    """R for y = x/R. Returns nan rather than dividing by a null ladder."""
    den = float(np.sum(x * y))
    return float(np.sum(x * x)) / den if den else np.nan


def _with_intercept(x, y):
    """R and the offset for y = x/R + c.

    The intercept is not cosmetic. Over the chain the rotor spins up, the EMF
    grows, and every settled level drifts down by the same few mA per step
    regardless of which way the duty went. That drift is a constant added to
    every di, so it is exactly an intercept, and the slope is clean of it. When
    the free-intercept R and the origin R agree, the chain was not drifting
    enough to matter.
    """
    if len(x) < 3:
        return np.nan, np.nan
    m, c = np.linalg.lstsq(np.vstack([x, np.ones_like(x)]).T, y, rcond=None)[0]
    return (1.0 / m if m else np.nan), float(c)


def fit_r(tr, boot=400, seed=0):
    """Loop resistance from the step amplitude, both voltage references.

    delta_i against delta(D*V). Terminal referenced uses the motor taps, so it
    charges the winding and the wiring inside the taps but not the bridge;
    rail referenced uses the direct vbus tap, so it charges everything from the
    supply pin inward and reads a little higher.

    Keys per reference X in (term, rail):
      r_X_origin        pooled fit through the origin, ohm
      r_X_free          pooled fit with a free intercept, ohm
      r_X_intercept     that intercept, A - the per-step drift of the DC level
      r_X_up, r_X_down  origin fits on up-steps and down-steps separately
      r_X_med, r_X_iqr  per-transition distribution
      r_X_lo, r_X_hi    bootstrap bracket on the origin fit
    Plus n, and r_term_abs: the ABSOLUTE ratio D*V/i on the settled levels,
    which is what a locked-rotor ladder measures. It keeps V0 and the EMF in
    it, so it is a contrast, not a result.
    """
    t = tr[tr.stepped & tr.di.notna()]
    out = dict(n=len(t))
    if not len(t):
        return out
    rng = np.random.default_rng(seed)
    y = t.di.to_numpy()
    for ref, col in (("term", "dvt"), ("rail", "dvr")):
        x = t[col].to_numpy()
        out[f"r_{ref}_origin"] = _through_origin(x, y)
        r_free, c = _with_intercept(x, y)
        out[f"r_{ref}_free"] = r_free
        out[f"r_{ref}_intercept"] = c
        for lbl, mask in (("up", t.up.to_numpy()), ("down", ~t.up.to_numpy())):
            out[f"r_{ref}_{lbl}"] = _through_origin(x[mask], y[mask]) if mask.sum() > 2 else np.nan
        per = (x / y)
        per = per[np.isfinite(per)]
        out[f"r_{ref}_med"] = float(np.median(per)) if len(per) else np.nan
        out[f"r_{ref}_iqr"] = float(np.subtract(*np.percentile(per, [75, 25]))) if len(per) else np.nan
        bs = [_through_origin(x[p], y[p])
              for p in (rng.integers(0, len(x), len(x)) for _ in range(boot))]
        bs = [v for v in bs if np.isfinite(v)]
        out[f"r_{ref}_lo"], out[f"r_{ref}_hi"] = (
            np.percentile(bs, [2.5, 97.5]) if bs else (np.nan, np.nan))

    # absolute, for contrast with a locked-rotor ladder
    lev = np.concatenate([t.d_pre * t.vt_pre / t.i_pre, t.d_post * t.vt_post / t.i_post])
    lev = lev[np.isfinite(lev)]
    out["r_term_abs"] = float(np.median(lev)) if len(lev) else np.nan
    return out


def null_step(tr):
    """The control: what the method reports when nothing stepped.

    A null ladder rewrites the same duty every millisecond. Every mechanism
    that could fake a step - the segment marker, the register write, the
    recorder's own cadence - still fires; only the drive does not change. What
    comes back is the noise floor of di plus one step of chain drift.
    """
    t = tr[~tr.stepped]
    if not len(t):
        return dict(null_n=0)
    return dict(null_n=len(t),
                null_di_ma=float(np.median(t.di)) * 1e3,
                null_di_iqr_ma=float(np.subtract(*np.percentile(t.di, [75, 25]))) * 1e3,
                null_di_max_ma=float(np.max(np.abs(t.di))) * 1e3)


# --------------------------------------------------------------------------
# supply
# --------------------------------------------------------------------------

def source_impedance(df, board, floor_a=0.005):
    """Rail sag against crest current, over every driven sample.

    vsys_v(vbus_raw) is the direct rail tap, valid every tick including at
    rest, so the rest rail from seg 0 is on the same scale as the driven ones
    and the two can be differenced.

    Two numbers, and they answer different questions:
      z_fit_ohm   slope of a straight line through the driven cloud. This is
                  the INCREMENTAL sag, what one more amp costs right now.
      z_rest_ohm  median of (rest rail - driven rail)/i. This is referred to
                  the unloaded rail, so it also carries whatever fixed drop
                  the act of switching costs, and reads higher.
    Both are sampled at the ON centre, so they are the sag DURING the pulse,
    not the sag of the period average.
    """
    rest = df[df.seg == 0]
    bias = rest.current_raw.median()
    v_rest = float(board.vsys_v(rest.vbus_raw).median())
    d = df[(df.seg >= 1) & (df.window_valid == 1) & (df.duty_q15 != 0)]
    i = board.amps(d.current_raw.to_numpy(), bias)
    v = board.vsys_v(d.vbus_raw.to_numpy())
    ok = i > floor_a
    out = dict(v_rest=v_rest, n_sag=int(ok.sum()))
    if ok.sum() > 10:
        m, c = np.linalg.lstsq(np.vstack([i[ok], np.ones(ok.sum())]).T, v[ok], rcond=None)[0]
        out["z_fit_ohm"] = -float(m)
        out["v_intercept"] = float(c)
        out["z_rest_ohm"] = float(np.median((v_rest - v[ok]) / i[ok]))
    return out


# --------------------------------------------------------------------------
# drift
# --------------------------------------------------------------------------

def drift(df, tr, board):
    """How far the operating point moved while the method was not looking.

    The chain accelerates the rotor, the EMF grows, and the mean current per
    step slides down. That slide is the thing the differential method is meant
    to be blind to, so it is worth quoting next to the answer: if R comes out
    the same while the DC level moved by a third of the step size, the claim
    is earned rather than asserted.
    """
    rest = df[df.seg == 0]
    bias = rest.current_raw.median()
    d = df[(df.seg >= 1) & (df.duty_q15 != 0)].copy()
    d["i"] = board.amps(d.current_raw, bias)
    per = d.groupby(["dir", "seg"]).i.mean()
    out = {}
    slopes, spans = [], []
    for dirn, g in per.groupby(level=0):
        v = g.to_numpy()
        # every other step is the same level, so compare like with like
        for phase in (0, 1):
            w = v[phase::2]
            if len(w) > 3:
                slopes.append(np.polyfit(np.arange(len(w)), w, 1)[0])
                spans.append(w[-1] - w[0])
    out["drift_ma_per_step"] = float(np.median(slopes)) * 1e3 if slopes else np.nan
    out["drift_ma_span"] = float(np.median(spans)) * 1e3 if spans else np.nan
    trav = df[df.seg >= 1].groupby("dir").pos.agg(lambda s: s.iloc[-1] - s.iloc[0])
    out["pot_travel_counts"] = float(np.median(np.abs(trav)))
    if len(tr) and tr.di.notna().any():
        out["drift_frac_of_step"] = abs(out["drift_ma_per_step"]) / (
            float(np.median(np.abs(tr.di.dropna()))) * 1e3)
    return out


# --------------------------------------------------------------------------
# top level
# --------------------------------------------------------------------------

def summarise(ds, experiment, capture, name="ladder", nfit=None, boot=400):
    """Everything the method has to say about one capture, as a flat dict.

    L is quoted against the terminal-referenced R, because that is the one
    whose voltage is measured across the winding rather than across the
    winding plus the bridge.
    """
    df = ds.read(experiment, capture, name)
    b = ds.board
    rec = ds.recordings(experiment, capture)[0]
    meta = rec.meta
    tick_hz = meta.get("tick_hz", b.tick_hz)

    tr = transitions(df, b)
    t = fit_tau(tr, tick_hz, nfit=nfit, boot=boot)
    r = fit_r(tr, boot=boot)
    out = dict(experiment=experiment, capture=capture,
               window_ms=meta.get("window_ms"), decay=meta.get("decay"),
               supply=meta.get("supply", ds.supply),
               levels=_levels(meta), n_tr=len(tr), n_step=r.get("n", 0))
    out.update({k: v for k, v in t.items() if k not in ("tau_p", "curve")})
    out.update(r)
    out.update(null_step(tr))
    out.update(source_impedance(df, b))
    out.update(drift(df, tr, b))
    for ref in ("term", "rail"):
        R = out.get(f"r_{ref}_origin", np.nan)
        out[f"l_{ref}_mh"] = t["tau_us"] * 1e-6 * R * 1e3
    out["_tr"] = tr
    out["_tau"] = t
    return out


def _levels(meta):
    """The two duty levels of the schedule, as a label."""
    sched = [s.replace("then:", "") for s in meta.get("schedule", [])]
    seen = sorted({s for s in sched}, key=lambda s: float(s))
    return "-".join(seen)


CAMPAIGN_COLS = [
    "experiment", "capture", "window_ms", "levels", "n_step",
    "tau_us", "tau_lo_us", "tau_hi_us", "tau_free_us", "asymptote",
    "tau_med_us", "tau_iqr_us",
    "r_term_origin", "r_term_free", "r_term_intercept",
    "r_term_up", "r_term_down", "r_term_lo", "r_term_hi",
    "r_rail_origin", "r_rail_free", "r_term_abs",
    "l_term_mh", "l_rail_mh",
    "null_di_ma", "z_fit_ohm", "z_rest_ohm", "v_rest",
    "drift_ma_per_step", "drift_ma_span", "pot_travel_counts",
]


def campaign(ds, experiments=None, name="ladder", boot=200, cols=CAMPAIGN_COLS):
    """Every rlstep capture in a dataset, one row each.

    Missing captures are skipped rather than fatal: the ladder lands one
    capture at a time and this is meant to be re-run while it is still
    arriving.
    """
    if experiments is None:
        experiments = [e for e in ds.experiments if e.startswith("rlstep")]
    rows = []
    for exp in sorted(experiments):
        for cap in ds.captures(exp):
            try:
                rows.append(summarise(ds, exp, cap, name=name, boot=boot))
            except (KeyError, ValueError, OSError, EOFError) as err:
                rows.append(dict(experiment=exp, capture=cap, error=str(err)))
    df = pd.DataFrame(rows)
    keep = [c for c in cols if c in df.columns]
    if "error" in df.columns:
        keep = keep + ["error"]
    return df[keep]


def by_group(tr, tick_hz, key="dirn", boot=0):
    """Re-fit tau and R on subsets of one capture's transitions.

    `key` is any column of the transitions frame - dirn for the direction
    split, up for the up-step / down-step split, d_pre for the bias level.
    Splitting halves the ensemble, so read the brackets, not the digits.
    """
    rows = []
    for k, g in tr.groupby(key):
        t = fit_tau(g, tick_hz, boot=boot)
        r = fit_r(g, boot=boot)
        rows.append(dict(group=k, n=len(g), tau_us=t["tau_us"],
                         r_term=r.get("r_term_origin", np.nan),
                         r_rail=r.get("r_rail_origin", np.nan),
                         l_term_mh=t["tau_us"] * 1e-6 * r.get("r_term_origin", np.nan) * 1e3))
    return pd.DataFrame(rows)


def consistency(camp, by="experiment", cols=("tau_us", "r_term_origin", "r_rail_origin", "l_term_mh")):
    """Median and spread of the headline numbers, grouped.

    Group by experiment for the configuration sweep; the caller can add its
    own column (bias level, step size, direction) and group by that instead.
    """
    g = camp.groupby(by)
    out = g[list(cols)].agg(["median", "std", "count"])
    return out
