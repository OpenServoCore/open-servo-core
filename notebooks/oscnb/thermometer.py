"""The shipped winding thermometer on the bench: the firmware's own
`t_winding_cc` logged by `th` beside a K thermocouple on the can, and the
`osc ident thermal` hold.

A dataset of this kind holds one folder per capture, named in `captures.csv`
(capture, source, image, what, unix0, notes):

  oab/capture-N/      an out-and-back rep: oab.csv.gz (the `th oab` tick log),
                      rest.csv.gz (the torque-off log before it), post.csv.gz
                      (the torque-off log after), meter.csv.gz (the can
                      thermocouple), and the console files the bench kept
  thermal/capture-N/  an `osc ident thermal` hold: thermal.csv.gz (the rows the
                      fit read), anchor_snapshots.csv.gz (every poll of the
                      hold), anchor.csv.gz (every aggregate window), report.out
                      (the tool's console), rest.csv.gz, meter.csv.gz
  boot/capture-N/     a torque-off log from the reset on: boot.csv.gz, meter.csv.gz

  index(ds)                          the captures.csv index
  th_log(ds, capture, name)          a `th` log, with the derived columns below
  meter(ds, capture)                 the thermocouple in C, out-of-range packets dropped
  can_at(m, t, w)                    the can's median within +/- w s of each t
  rest_offset(rest, m, last_s)       can minus NTC over the rest log's last `last_s`
  bases(log)                         every TRACK rise inside a hold: the seat's reference
  exits(log)                         every TRACK fall inside a hold, with what moved
  gaps(log, m, offset, which)        the carry across each torque-off gap against the can
  ident_run(ds, capture)             the hold's rows, polls and windows on one unix clock
  calib_fields(text)                 the CALIB fields an `osc` console wrote or read
  fit_free(rows) / fit_g(rows, tau)  the ident thermal fit, ported (see THE FIT)

A `th` log row is one poll of the servo's telemetry span. The derived columns:
`t_w` and `t_ntc` (the firmware's winding and board NTC temperatures, C),
`i` (current over the tracked zero, counts), `track` (therm_flags TRACK bit).

THE TRUTH

With torque off for long enough the winding, can and board sit at one
temperature, and the can minus the board NTC there is the rest offset. A
winding that has been off for 240 s sits within a few tenths of the can
(the can-to-winding coupling is fast against 240 s), so the can less the
rest offset, T*, is what the NTC-based thermometer should read at the next
seat, with no resistance in it.

THE FIT

`osc ident thermal` holds the shaft at the low stop for 480 s and fits the
kernel's tracked excess x over the NTC against the power it logged. The rows
are cut into tracked stretches; each stretch loses its first BASE_SKIP_S and
its last EXIT_GUARD_S and is chunked into CHUNK_S pieces, each giving the
chunk's rate of x, mean P and mean x. The free form fits
rate = a P + b x by least squares (twice, the second time without chunks
over OUTLIER_SD sd), tau = -1/b, g = -a/b. The tau-fixed form fits only
rate + x / tau = a P, g = a tau. `fit_free` is the port of the tool's fit
as the bench ran it on the first hold; it reproduces that hold's printed
th_alpha_q24 and th_g_q016 exactly (tests).
"""

import gzip
import re
from pathlib import Path

import numpy as np
import pandas as pd

SLOW_HZ = 62.5                  # the kernel's SLOW rate, where the thermometer steps
TRACK, UNSET = 2, 1             # therm_flags bits (core estimator::thermal::flag)
COPPER_ZERO_C = 234.5           # copper: R is proportional to (234.5 + T), T in C
METER_RANGE = (10.0, 80.0)      # a thermocouple packet outside this is a link glitch
Q24, Q016 = 2.0 ** 24, 65536.0

BASE_SKIP_S, EXIT_GUARD_S, CHUNK_S, OUTLIER_SD = 3.0, 2.0, 2.0, 3.0


def index(ds):
    return pd.read_csv(ds.path / "captures.csv")


def _csv(path):
    with gzip.open(path, "rt") as fh:
        return pd.read_csv(fh)


def th_log(ds, capture, name):
    """`<capture>/<name>.csv.gz` with t_w, t_ntc (C), i (counts) and track."""
    h = _csv(ds.path / capture / f"{name}.csv.gz")
    h["t_w"] = h.t_winding_cc / 100
    h["t_ntc"] = h.t_ntc_cc / 100
    h["i"] = h.current - h.bias
    h["track"] = (h.therm_flags & TRACK) > 0
    return h


def meter(ds, capture):
    """The thermocouple's C packets inside METER_RANGE as (t_unix, can_c);
    `attrs['dropped']` counts the packets outside it."""
    d = _csv(ds.path / capture / "meter.csv.gz")
    d = d[d.func == "C"].copy()
    d["can_c"] = pd.to_numeric(d.value, errors="coerce")
    ok = (d.can_c > METER_RANGE[0]) & (d.can_c < METER_RANGE[1])
    m = d.loc[ok, ["t_unix", "can_c"]].reset_index(drop=True)
    m.attrs["dropped"] = int((~ok).sum())
    return m


def can_at(m, t, w=1.5):
    """Median of the packets strictly within +/- w s of each t; NaN where none."""
    tu, c = m.t_unix.to_numpy(), m.can_c.to_numpy()
    t = np.atleast_1d(np.asarray(t, float))
    lo = np.searchsorted(tu, t - w, side="right")
    hi = np.searchsorted(tu, t + w, side="left")
    out = np.array([np.median(c[a:b]) if b > a else np.nan for a, b in zip(lo, hi)])
    return out if out.size > 1 else float(out[0])


def rest_offset(rest, m, last_s=180.0):
    """Can minus the board NTC over the rest log's last `last_s` seconds:
    medians of both, the meter packets inside the same window."""
    t1 = rest.t_unix.max()
    t0 = t1 - last_s
    rw = rest[rest.t_unix >= t0]
    mw = m[(m.t_unix >= t0) & (m.t_unix <= t1)]
    can, ntc = float(mw.can_c.median()), float(rw.t_ntc_cc.median() / 100)
    return dict(t0=t0, t1=t1, can=can, can_sd=float(mw.can_c.std()), n=len(mw), ntc=ntc,
                t_w=float(rw.t_winding_cc.median() / 100), offset=can - ntc)


def holds(log):
    return [p for p in log.phase.unique() if p.startswith("hold")]


def bases(log):
    """One row per TRACK rise inside a hold: the hold, the row index, the time
    from the hold's first row, the reference R (r_hat_q12 at the rise), the
    temperature the base inherited, the step at the rise and the seat."""
    rows = []
    tr = log.track.to_numpy()
    for p in holds(log):
        g = log[log.phase == p]
        ts = g.t_unix.iloc[0]
        for k, i in enumerate(i for i in g.index if tr[i] and not tr[i - 1]):
            rows.append(dict(hold=p, n=k, idx=i, t_s=log.t_unix[i] - ts, t_unix=log.t_unix[i],
                             r_hat=int(log.r_hat_q12[i]), t_w=log.t_w[i], step=log.t_w[i] - log.t_w[i - 1],
                             t_ntc=log.t_ntc[i], i=int(log.i[i]), duty=int(log.duty_applied_q15[i]),
                             pos=int(log.pos[i]), seat=int(g.pos.median())))
    return pd.DataFrame(rows)


def exits(log, around=12):
    """One row per TRACK fall inside a hold: when, and what moved across it
    against the base it ended (position, current, the applied duty), with
    the carried temperature before the fall and at the next base."""
    tr = log.track.to_numpy()
    b = bases(log)
    rows = []
    for p in holds(log):
        g = log[log.phase == p]
        ts = g.t_unix.iloc[0]
        for j in (j for j in g.index[1:] if not tr[j] and tr[j - 1]):
            prev = b[(b.hold == p) & (b.idx < j)].iloc[-1]
            nxt = b[(b.hold == p) & (b.idx > j)]
            w0 = log.loc[max(j - around, g.index[0]):j - 1]
            w1 = log.loc[j:min(j + around, g.index[-1])]
            i_ref = log.i[prev.idx:prev.idx + 5].mean()
            moved = []
            if (w1.pos - prev.pos).abs().max() > 2:
                moved.append("pos")
            if abs(w1.i.mean() - i_ref) > i_ref / 8:
                moved.append("i")
            if w1.duty_applied_q15.iloc[-1] != w0.duty_applied_q15.iloc[0]:
                moved.append("duty")
            rows.append(dict(hold=p, t_s=log.t_unix[j] - ts, t_unix=log.t_unix[j], moved="+".join(moved) or "none",
                             i_before=w0.i.mean(), i_after=w1.i.mean(), i_ref=i_ref,
                             duty_before=int(w0.duty_applied_q15.iloc[0]), duty_after=int(w1.duty_applied_q15.iloc[-1]),
                             t_w_before=log.t_w[j - 1], t_w_next_base=nxt.t_w.iloc[0] if len(nxt) else g.t_w.iloc[-1]))
    return pd.DataFrame(rows)


def gaps(log, m, offset, which="first"):
    """The carry across each torque-off gap: the new hold's base (its `which`
    base, 'first' or 'last') against T* = can at the base (+/- 1.5 s median)
    less the rest offset. Also the same-seat ratio across the gap,
    (234.5 + T_old) R_new / R_old - 234.5, from the old hold's base of the
    same rule."""
    b = bases(log)
    pick = (lambda g: g.iloc[0]) if which == "first" else (lambda g: g.iloc[-1])
    hs = holds(log)
    rows = []
    for k in range(1, len(hs)):
        old, new = b[b.hold == hs[k - 1]], b[b.hold == hs[k]]
        if old.empty or new.empty:
            continue
        o, n = pick(old), pick(new)
        go = log[log.phase == hs[k - 1]]
        seek = log[log.phase == f"seek{k}"].t_unix.iloc[0]
        can = can_at(m, n.t_unix)
        t_star = can - offset
        truth = (COPPER_ZERO_C + o.t_w) * n.r_hat / o.r_hat - COPPER_ZERO_C
        rows.append(dict(gap=k, gap_s=round(seek - go.t_unix.iloc[-1]), t_unix=n.t_unix, r_old=o.r_hat, r_new=n.r_hat,
                         carried=n.t_w, can=can, t_star=t_star, carry_m_t=n.t_w - t_star,
                         ratio=truth, ratio_m_t=truth - t_star, t_ntc=n.t_ntc, step=n.step))
    return pd.DataFrame(rows)


def ident_run(ds, capture, unix0):
    """An `osc ident thermal` run's three files on the unix clock: the fit's
    rows (t_s from the run's start), the polls (host_ms) and the aggregate
    windows (t_ms), each with a `t_unix` column. `unix0` is the run
    directory's name, the run's start."""
    p = ds.path / capture
    th = _csv(p / "thermal.csv.gz")
    sn = _csv(p / "anchor_snapshots.csv.gz")
    win = _csv(p / "anchor.csv.gz")
    th["t_unix"] = unix0 + th.t_s
    sn["t_unix"] = unix0 + sn.host_ms / 1000
    sn["i"] = sn.current - sn.current_bias_counts
    win["t_unix"] = unix0 + win.t_ms / 1000
    return th, sn, win


FIELD = re.compile(r"^\s*([a-z][a-z0-9_]+)\s+(?:=\s+)?(-?\d+)\s+\(", re.M)


def calib_fields(text):
    """`name value` pairs from an osc console: the `wrote N fields` block
    (`  th_g_q016   1500  (...)`) and register reads (`r0_q12 = 4078  (addr`).
    A later line of the same name wins."""
    return {k: int(v) for k, v in FIELD.findall(text)}


def _stretches(t, flags):
    tracked = (flags & (TRACK | UNSET)) == TRACK
    edges = np.flatnonzero(np.diff(np.r_[0, tracked.astype(int), 0]))
    return list(zip(edges[::2], edges[1::2]))


def chunks(rows):
    """(rate of x, mean P, mean x) per CHUNK_S chunk of every tracked
    stretch; x in centi-C, rate per s. `rows` has t_s, x_cc, p, flags."""
    t, x, p, fl = (rows[c].to_numpy(float) for c in ("t_s", "x_cc", "p", "flags"))
    fl = fl.astype(int)
    out = []
    for a, b in _stretches(t, fl):
        st, sx, sp = t[a:b], x[a:b], p[a:b]
        keep = (st >= st[0] + BASE_SKIP_S) & (st <= st[-1] - EXIT_GUARD_S)
        st, sx, sp = st[keep], sx[keep], sp[keep]
        k = 0
        while k + 1 < len(st):
            s = k
            while k + 1 < len(st) and st[k] - st[s] < CHUNK_S:
                k += 1
            dt = st[k] - st[s]
            if dt < CHUNK_S:
                break
            h = np.diff(st[s:k + 1])
            ip = np.sum(h * (sp[s:k] + sp[s + 1:k + 1]) / 2) / dt
            ix = np.sum(h * (sx[s:k] + sx[s + 1:k + 1]) / 2) / dt
            out.append(((sx[k] - sx[s]) / dt, ip, ix))
    return np.array(out)


def _solve(c):
    r, p, x = c[:, 0], c[:, 1], c[:, 2]
    spp, spx, sxx, spy, sxy = p @ p, p @ x, x @ x, p @ r, x @ r
    det = spp * sxx - spx * spx
    return ((sxx * spy - spx * sxy) / det, (spp * sxy - spx * spy) / det)


def _sd(c, ab):
    e = c[:, 0] - ab[0] * c[:, 1] - ab[1] * c[:, 2]
    return float(np.sqrt(e @ e / (len(c) - 2)))


def fit_free(rows, w_per_unit):
    """The two-parameter fit: tau, g (centi-C per unit of p), their table
    encodings at SLOW_HZ, R_th in C/W through `w_per_unit` (watts per unit of
    p), the residual sd in centi-C per SLOW tick, the chunk counts."""
    c = chunks(rows)
    first = _solve(c)
    e = c[:, 0] - first[0] * c[:, 1] - first[1] * c[:, 2]
    kept = c[np.abs(e) <= OUTLIER_SD * _sd(c, first)]
    a, b = _solve(kept)
    tau, g = -1 / b, -a / b
    return dict(tau=tau, g=g, alpha_q24=round(Q24 / (tau * SLOW_HZ)), g_q016=round(g * Q016),
                r_th=g / (w_per_unit * 100), sd=_sd(kept, (a, b)) / SLOW_HZ,
                chunks=len(c), rejected=len(c) - len(kept))


def fit_g(rows, tau, w_per_unit):
    """The tau-fixed fit: rate + x / tau = a P over every chunk, g = a tau."""
    c = chunks(rows)
    a = float(((c[:, 0] + c[:, 2] / tau) @ c[:, 1]) / (c[:, 1] @ c[:, 1]))
    g = a * tau
    return dict(tau=tau, g=g, g_q016=round(g * Q016), r_th=g / (w_per_unit * 100), chunks=len(c))
