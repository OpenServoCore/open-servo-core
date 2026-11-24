"""The cruise step's fixture and its expected values: closed-loop Velocity
cruises of the mg90-d on dev-v006-2A-D, trimmed to the rows the cruise
fit reads, and the fit's numbers by a Python reference of the method.

Run read-only from a notebooks checkout, naming the bringup captures:

    cd notebooks && .venv/bin/python <this file> <bringup>/captures/friction-settle \\
        <bringup>/kb/scratch/friction-spins-vs-cruise/sitting/line.csv

Writes, beside this file:
- `legs.json.gz`: per leg of a nonzero goal in RUNS but LEFT_OUT, its
  run, segment, goal (counts/s), the leg's rows, the polled current bias
  after it, and the stretch of rows kept: from PRE rows before the first
  settled row inside its speed's window to the last one, column by
  column: the window flag as a string of 0 and 1; the raw pot and
  `duty_counts` each as the first value then the steps between rows;
  `current_raw` the same over the rows whose window was read only; and
  the tap difference `vmotor_a - vmotor_b` on one row in VDIFF_EVERY;
- `expected.json`: the set of record's table points and wide-code slots,
  the constants the fit reads, the leg's numbers and the lines by the
  reference below on the trimmed rows, the same on every row of the
  captures (`full`), on every leg of the runs, LEFT_OUT too
  (`every_leg`), and the analyst's line from the sitting's `line.csv`.

The reference is the cruise step's method (measure `fit::cruise`): the
rows a quarter into the leg and inside the middle third of the travel,
then inside the window both directions of the speed crossed; the reading
over the polled bias with the board's settle gain and the window term on
the tap difference's median; the all-tick estimate of the sitting's
`allticks.py` under 98 % of rows read; inertia from the table speed's
halves through the set's j_ff; the steadiness rule (the speed held, the
inertia correction bounded; the halves reported, not judged); the line weighted so
each speed counts once, each standard error the larger of the legs'
scatter and the speeds' means about the line.
"""
import gzip
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from scipy import signal

from oscnb import boards
from oscnb import windowterm as wt

HERE = Path(__file__).resolve().parent
PAIR = "mg90-d__dev-v006-2A-D"
BOARD = boards.BOARDS["dev-v006-2A-D"]
RUNS = ["K1-cl", "K2-cl-a", "K2-cl-b", "K3-cl", "H-cl", "R-c300"]
# legs left out for size: the 300 c/s run's last two, and the top
# speed's closing half leg
LEFT_OUT = {("R-c300", 4), ("R-c300", 5), ("H-cl", 7)}
TRAVEL = (110.0, 3730.0)         # the mg90-d's travel, linearized counts
CEILING = 4025.0                 # velocity_limit_cps of the set of record
PRE = 32
VDIFF_EVERY = 32
CPA = 1135.013429521035          # the set of record's counts/A

# the analyst's two lines in line.csv's `which` column: the valid
# 1500, 2150 and 4025 c/s legs, and those with the 300 c/s legs
ANALYST_LINE = "valid 1500/2150/4025"
ANALYST_WITH_LOW = "with 300 (report)"

# the method's constants, as measure fit::cruise names them
BAND_SHARE = 1.0 / 3.0
SETTLE_SHARE = 0.25
KEPT_MIN_S = 0.100
READ_SHARE_MIN = 0.98
TAU_US = (50, 70, 100, 130, 160, 200, 250)
LAGS = (0, 1)
HELD_SHARE = 0.05
INERTIA_MAX_MA = 1.5
# the comparison line without the slowest speed: legs from this share of
# the ceiling up
LOW_SHARE = 0.2
TICK_HZ = 20000.0
MEDIUM_HZ = TICK_HZ / 10


def read_gz(p):
    with gzip.open(p, "rt") as f:
        return pd.read_csv(f)


def band():
    lo, hi = TRAVEL
    t = hi - lo
    return lo + BAND_SHARE * t, hi - BAND_SHARE * t


def settle_rows(n):
    return int(SETTLE_SHARE * n)


def settled_span(leg, lo, hi):
    p, i = leg["lin"], leg["index"]
    m = (i >= settle_rows(leg["rows"])) & (p >= lo) & (p <= hi)
    return (float(p[m].min()), float(p[m].max())) if m.any() else None


def windows(legs):
    b = band()
    out = {}
    for goal in sorted({abs(l["goal"]) for l in legs}):
        by = {}
        for l in legs:
            if abs(l["goal"]) != goal:
                continue
            s = settled_span(l, *b)
            if s:
                d = by.setdefault(int(np.sign(l["goal"])), [s[0], s[1]])
                d[0], d[1] = min(d[0], s[0]), max(d[1], s[1])
        if not by:
            continue
        lo = max(v[0] for v in by.values())
        hi = min(v[1] for v in by.values())
        if lo < hi:
            out[goal] = (lo, hi)
    return out


def fit1(t, y):
    return np.polyfit(t, y, 1)[0]


def read_leg(l, window, fields, cpmr):
    s = int(np.sign(l["goal"]))
    p, idx = l["lin"], l["index"]
    m = (idx >= settle_rows(l["rows"])) & (p >= window[0]) & (p <= window[1])
    k = np.flatnonzero(m)
    out = {"run": l["run"], "seg": l["seg"], "goal": l["goal"]}
    if len(k) < KEPT_MIN_S * TICK_HZ:
        out["fail"] = "short"
        return out
    a, b = int(k[0]), int(k[-1]) + 1
    duty = l["duty"].astype(np.int64)
    h = np.abs(duty)
    sd = np.sign(duty) * s
    vd = l["vdiff"]
    wide = (h >= fields["v_window_min_ticks"]) & np.isfinite(vd)
    wide[:a] = False
    wide[b:] = False
    if not wide.any():
        out["fail"] = "no tap"
        return out
    v_on = float(np.median(vd[wide] * np.sign(duty[wide])))
    gq = fields["g_window_q016"] / 65536.0
    crest = (l["raw"] - l["bias"]) * BOARD.settle_gain(h)
    i = (crest - gq * (1 - h / BOARD.pwm_arr) * v_on) * sd
    valid = l["valid"]
    sl = slice(a, b)
    vk = valid[sl]
    read = float(vk.mean())
    i_read = float(i[sl][vk].mean())
    de = np.where(h >= BOARD.bridge_min_ticks, h, 0) * sd / BOARD.pwm_arr
    best = None
    for lag in LAGS:
        u = np.r_[np.zeros(lag), de[: len(de) - lag]]
        for tau in TAU_US:
            al = math.exp(-1e6 / TICK_HZ / tau)
            y = signal.lfilter([1 - al], [1, -al], u)
            X = np.c_[np.ones(vk.sum()), y[sl][vk]]
            coef, *_ = np.linalg.lstsq(X, i[sl][vk], rcond=None)
            mse = float(((i[sl][vk] - X @ coef) ** 2).mean())
            if best is None or mse < best[0]:
                best = (mse, lag, tau, coef, float(y[sl].mean()))
    mse, lag, tau, (c0, c1), ymean = best
    i_all = float(c0 + c1 * ymean)
    used = i_read if read >= READ_SHARE_MIN else i_all
    n = b - a
    t = np.arange(n) / TICK_HZ
    pk = p[sl]
    cps = s * fit1(t, pk)
    half = n // 2
    v1 = s * fit1(t[:half], pk[:half])
    v2 = s * fit1(t[half:], pk[half:])
    accel = (v2 - v1) / (n / 2 / TICK_HZ)
    i_acc = fields["j_ff_q88"] / 256.0 * accel / MEDIUM_HZ
    ma = 1e3 / CPA
    held = abs(cps / abs(l["goal"]) - 1)
    out.update({
        "kept_rows": n, "first_kept": int(idx[a]), "pos_lo": float(pk.min()), "pos_hi": float(pk.max()),
        "v_on_counts": v_on, "read_share": read, "i_read_ma": i_read * ma, "i_all_ma": i_all * ma,
        "tau_us": tau, "lag": lag, "i_used_ma": used * ma, "cps": float(cps),
        "rad_s": float(cps / cpmr), "halves": float(v2 / v1 - 1), "accel_cps2": float(accel),
        "i_inertia_ma": float(i_acc * ma), "i_ma": float((used - i_acc) * ma),
        "held": float(held),
    })
    why = []
    if held > HELD_SHARE:
        why.append("held")
    if abs(out["i_inertia_ma"]) > INERTIA_MAX_MA:
        why.append("inertia")
    out["valid"] = not why
    if why:
        out["fail"] = ",".join(why)
    return out


def line(legs, with_low):
    v = [l for l in legs if l.get("valid") and (with_low or abs(l["goal"]) / CEILING >= LOW_SHARE)]
    if len({abs(l["goal"]) for l in v}) < 2:
        return None
    n = pd.Series([abs(l["goal"]) for l in v]).value_counts().to_dict()
    x = np.array([l["rad_s"] for l in v])
    y = np.array([l["i_ma"] for l in v])
    w = np.array([1.0 / n[abs(l["goal"])] for l in v])
    X = np.c_[np.ones_like(x), x]
    A = X.T @ (w[:, None] * X)
    c = np.linalg.solve(A, X.T @ (w * y))
    r = y - X @ c
    s2 = float((w * r * r).sum() / (len(x) - 2))
    cov = s2 * np.linalg.inv(A)
    res, xg = {}, {}
    for g in sorted(n):
        res[str(g)] = float(np.mean([ri for ri, l in zip(r, v) if abs(l["goal"]) == g]))
        xg[str(g)] = float(np.mean([l["rad_s"] for l in v if abs(l["goal"]) == g]))
    # the speeds' means about the line, speeds less two degrees of freedom
    legs_se = (math.sqrt(cov[0, 0]), math.sqrt(cov[1, 1]))
    speeds_se = None
    if len(n) >= 3:
        Xg = np.c_[np.ones(len(n)), [xg[k] for k in res]]
        rg = np.array(list(res.values()))
        cg = float((rg * rg).sum() / (len(n) - 2)) * np.linalg.inv(Xg.T @ Xg)
        speeds_se = (math.sqrt(cg[0, 0]), math.sqrt(cg[1, 1]))
    pick = lambda k: (speeds_se[k], "speeds") if speeds_se and speeds_se[k] > legs_se[k] else (legs_se[k], "legs")
    (c_se, c_from), (v_se, v_from) = pick(0), pick(1)
    return {"coulomb_ma": float(c[0]), "coulomb_se_ma": c_se, "coulomb_se_from": c_from,
            "viscous_ma_s_per_rad": float(c[1]), "viscous_se": v_se, "viscous_se_from": v_from,
            "coulomb_se_legs_ma": legs_se[0], "viscous_se_legs": legs_se[1],
            "coulomb_se_speeds_ma": speeds_se and speeds_se[0], "viscous_se_speeds": speeds_se and speeds_se[1],
            "legs": len(v), "rad_s": [float(x.min()), float(x.max())], "residual_ma": res}


def legs_of(captures, left_out=LEFT_OUT):
    """Every leg of a nonzero goal but `left_out`, as arrays."""
    _, _, table, slots = wt.set_tables(PAIR)
    from oscnb import widecodes
    out = []
    for run in RUNS:
        tr = read_gz(captures / run / "track.csv.gz")
        polls = pd.read_csv(captures / run / "segments.csv").set_index("seg")
        for seg, g in tr.groupby("seg"):
            goal = int(g.goal.iloc[0])
            if goal == 0 or (run, seg) in left_out:
                continue
            g = g.reset_index(drop=True)
            out.append({
                "run": run, "seg": int(seg), "goal": goal, "rows": len(g),
                "bias": float(polls.loc[seg + 1, "current_bias_counts"]),
                "index": np.arange(len(g)), "pos": g.pos.to_numpy().astype(int),
                "lin": table.lin(widecodes.compensated(g.pos.to_numpy(), slots)),
                "raw": g.current_raw.to_numpy().astype(float),
                "duty": g.duty_counts.to_numpy().astype(int),
                "valid": g.window_valid.to_numpy() == 1,
                "vdiff": (g.vmotor_a - g.vmotor_b).to_numpy().astype(float),
            })
    return out


def trim(l, window):
    p, idx = l["lin"], l["index"]
    m = (idx >= settle_rows(l["rows"])) & (p >= window[0]) & (p <= window[1])
    k = np.flatnonzero(m)
    a, b = max(int(k[0]) - PRE, 0), int(k[-1]) + 1
    t = {key: (v[a:b] if isinstance(v, np.ndarray) else v) for key, v in l.items()}
    vd = t["vdiff"].copy()
    vd[(np.arange(len(vd)) % VDIFF_EVERY) != 0] = np.nan
    t["vdiff"] = vd
    t["first"] = a
    return t


def steps(v):
    v = np.asarray(v, np.int64)
    return [int(v[0])] + np.diff(v).astype(int).tolist()


def main():
    captures, analyst = Path(sys.argv[1]), Path(sys.argv[2])
    doc, fields, table, slots = wt.set_tables(PAIR)
    fields = dict(fields)
    fields["v_window_min_ticks"] = BOARD.v_window_min_ticks
    gear = doc["inputs"]["registry"]["servo"]["gear_ratio"]
    cpd = doc["scale"]["counts_per_deg"]
    cpmr = cpd * 180 / math.pi / gear
    full = legs_of(captures)
    win_full = windows(full)
    full_read = [read_leg(l, win_full[abs(l["goal"])], fields, cpmr) for l in full]
    every = legs_of(captures, set())
    win_every = windows(every)
    every_read = [read_leg(l, win_every[abs(l["goal"])], fields, cpmr) for l in every]
    trimmed = [trim(l, win_full[abs(l["goal"])]) for l in full]
    win = windows(trimmed)
    got = [read_leg(l, win[abs(l["goal"])], fields, cpmr) for l in trimmed]
    stored = []
    for l in trimmed:
        vd = l["vdiff"]
        stored.append({
            "run": l["run"], "seg": l["seg"], "goal": l["goal"], "rows": l["rows"], "first": l["first"],
            "bias_counts": l["bias"],
            "window_valid": "".join("1" if x else "0" for x in l["valid"]),
            "pos": steps(l["pos"]), "current_raw": steps(l["raw"][l["valid"]]), "duty_counts": steps(l["duty"]),
            "vdiff_every": VDIFF_EVERY,
            "vdiff": [int(x) for x in vd[::VDIFF_EVERY]],
        })
    blob = gzip.compress(json.dumps(stored, separators=(",", ":")).encode(), 9, mtime=0)
    (HERE / "legs.json.gz").write_bytes(blob)
    a = pd.read_csv(analyst)
    row = a[a.which.str.contains(ANALYST_LINE, regex=False)].iloc[0]
    rep = a[a.which.str.contains(ANALYST_WITH_LOW, regex=False)].iloc[0]
    sha = lambda p: hashlib.sha1(Path(p).read_bytes()).hexdigest()
    expected = {
        "sources": {
            "captures": {r: {"file": f"captures/friction-settle/{r}/track.csv.gz",
                             "sha1": sha(captures / r / "track.csv.gz")} for r in RUNS},
            "analyst_line": {"file": "kb/scratch/friction-spins-vs-cruise/sitting/line.csv", "sha1": sha(analyst)},
            "set": PAIR,
        },
        "points": [int(x) for x in table.points],
        "slots": list(slots),
        "lin_samples": [[r, float(table.lin(widecodes_c(r, slots)))] for r in (130, 1023, 1024, 1300, 2047, 2048, 2200, 3070, 3700)],
        "constants": {
            "counts_per_a": CPA, "g_window_q016": fields["g_window_q016"], "j_ff_q88": fields["j_ff_q88"],
            "pwm_arr": BOARD.pwm_arr, "tick_hz": TICK_HZ, "v_window_min_ticks": BOARD.v_window_min_ticks,
            "bridge_min_ticks": BOARD.bridge_min_ticks, "counts_per_deg": cpd, "gear_ratio": gear,
            "travel_counts": list(TRAVEL), "ceiling_cps": CEILING,
        },
        "windows": {str(k): list(v) for k, v in win.items()},
        "legs": got,
        "line": line(got, False),
        "line_with_low": line(got, True),
        "full": {"windows": {str(k): list(v) for k, v in win_full.items()},
                 "line": line(full_read, False), "line_with_low": line(full_read, True)},
        "every_leg": {"line": line(every_read, False), "line_with_low": line(every_read, True)},
        "analyst": {
            "line": {"coulomb_ma": row.intercept, "coulomb_se_ma": row.intercept_se,
                     "viscous_ma_s_per_rad": row.slope, "viscous_se": row.slope_se, "rad_s": [400, 1076]},
            "line_with_low": {"coulomb_ma": rep.intercept, "coulomb_se_ma": rep.intercept_se,
                              "viscous_ma_s_per_rad": rep.slope, "viscous_se": rep.slope_se},
        },
    }
    (HERE / "expected.json").write_text(json.dumps(expected, indent=1) + "\n")
    print(f"legs.json.gz {len(blob)} B, {sum(len(l['window_valid']) for l in stored)} rows")
    for k in ("line", "line_with_low"):
        print(k, "trimmed", expected[k])
        print(k, "full", expected["full"][k])
        print(k, "every leg", expected["every_leg"][k])
    print("analyst", expected["analyst"])


def widecodes_c(r, slots):
    from oscnb import widecodes
    return widecodes.compensated(np.asarray([r]), slots)[0]


if __name__ == "__main__":
    main()
