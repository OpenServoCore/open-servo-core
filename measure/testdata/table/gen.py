"""The table-fit fixtures and their expected values.

Run read-only from a notebooks checkout, the bringup checkout beside it:

    cd notebooks && uv run python <this file>

Writes, beside this file:
- `pass-1.sweep.csv.gz`, `pass-2.sweep.csv.gz`: the one-way 20 % passes of
  bringup `captures/ident-slice2/A` (segments 1 up and 3 down of each
  round trip), every row, the columns the table fit reads;
- `lookup-of-record.csv`: the camera lookup of record (stage 4), copied;
- `widecodes.json.gz`: the raw pot code histograms of the crawl and ladder
  hold rows (the stage 1 open-loop sets), even and odd chains apart;
- `expected.json`: what the reference rules below give on those rows;
- `sources.json`: where each came from, with sha1s.

The rules are the design note's offline scripts' (bringup
`kb/scratch/ident-table/offline/check_table.py`, `check_split.py`,
`check_wide.py`), copied, with one change the Rust port makes: the common
span is where both directions have samples (each direction's passes
pooled), not where all four passes do.
"""
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from oscnb import boards, postable, results, ripple, servos, motors, tel, widecodes

HERE = Path(__file__).resolve().parent
BRINGUP = Path.home() / "projects/OpenServoCore/osc-dev-v006-bringup"
CAP = BRINGUP / "captures/ident-slice2/A"
PAIR = postable.default_pair()
PAIR_DIR = results.ROOT / "telemetry" / PAIR
SERVO = servos.SERVOS["mg90-d"]
CYC_DEG = 360 / (motors.MOTORS[SERVO.motor].ripple_per_rev * SERVO.gear_ratio)
PASSES = [("pass-1", 1), ("pass-1", 3), ("pass-2", 1), ("pass-2", 3)]
COLUMNS = ["seg", "dir", "tick", "pos", "duty_counts", "current_raw"]
GRID, SUB, BOX = 16, 16, 16
ZONES = {"z1": (1570, 1650), "z2": (2200, 2250), "z3": (1890, 1975)}
SPLIT_ZONES = {"s1": (1600, 1650), "s2": (2200, 2250), "s3": (1890, 1975)}
STOPS = (110, 3730)


def sha1(path):
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()


def rms(x):
    x = np.asarray(x, float)
    return float(np.sqrt(np.mean(x ** 2)))


def samples(g, lo_row=2000, hi_cut=40):
    x = g["current_raw"].to_numpy(float)
    n = len(x)
    box = pd.Series(g["pos"].to_numpy(float)).rolling(BOX, center=True, min_periods=1).mean().to_numpy()
    mid = slice(n // 2 - 2000, n // 2 + 2000)
    f = ripple.line_freq(x[mid] - x[mid].mean(), lo=40, hi=3000)
    tc = np.asarray(ripple.tracked_crossings(x, lo_row, n - hi_cut, f))
    rows = np.arange(np.ceil(tc[0]), tc[-1], SUB)
    k = np.interp(rows, tc, np.arange(len(tc)))
    sign = np.sign(box[int(rows[-1])] - box[int(rows[0])])
    d = np.diff(tc)
    med = pd.Series(d).rolling(15, center=True, min_periods=3).median().to_numpy()
    odd = float(np.mean((d < 0.7 * med) | (d > 1.5 * med)))
    return box[rows.astype(int)], sign * k, dict(f=f, crossings=len(tc), odd=odd, sign=int(sign),
                                                  samples=len(rows), tc0=float(tc[0]), tc_last=float(tc[-1]))


def fit(passes, knots, w=1.0):
    K, P = len(knots), len(passes)
    rows_a, rhs, ns = [], [], 0
    for p, (x, y) in enumerate(passes):
        m = (x >= knots[0]) & (x <= knots[-1])
        x, y = x[m], y[m]
        ns += len(x)
        i = np.clip(((x - knots[0]) // GRID).astype(int), 0, K - 2)
        fr = (x - knots[i]) / GRID
        a = np.zeros((len(x), K + P - 1))
        a[np.arange(len(x)), i] = 1 - fr
        a[np.arange(len(x)), i + 1] = fr
        if p:
            a[:, K + p - 1] = 1
        rows_a.append(a)
        rhs.append(y)
    a = np.vstack(rows_a)
    y = np.concatenate(rhs)
    lam = w * ns / (K - 1)
    d = np.zeros((K - 2, K + P - 1))
    for j in range(K - 2):
        d[j, j:j + 3] = [1, -2, 1]
    sol = np.linalg.solve(a.T @ a + lam * d.T @ d, a.T @ y)
    return sol[:K], rms(y - a @ sol)


def line_out(knots, a):
    return a - np.polyval(np.polyfit(knots, a, 1), knots)


def slopes(xg, knots, a):
    return (np.interp(xg + 24, knots, a) - np.interp(xg - 24, knots, a)) / 48


def points(knots, m, lo_s, hi_s):
    sl = (m[-1] - m[0]) / (knots[-1] - knots[0])
    def ang(x):
        return np.where(x < knots[0], m[0] + (x - knots[0]) * sl,
                        np.where(x > knots[-1], m[-1] + (x - knots[-1]) * sl, np.interp(x, knots, m)))
    g = np.arange(257) * 16.0
    t = lo_s + (ang(g) - ang(lo_s)) * (hi_s - lo_s) / (ang(hi_s) - ang(lo_s))
    c = np.round(t - g)
    idx = np.arange(257)
    c[(idx <= (lo_s + 15) >> 4) | (idx >= hi_s >> 4)] = 0
    return c.astype(int)


def table_part(record_slots):
    per, src = {}, {}
    for name in ("pass-1", "pass-2"):
        df = pd.read_csv(CAP / name / "sweep.csv")
        keep = df[df["seg"].isin([1, 3])][COLUMNS]
        out = HERE / f"{name}.sweep.csv.gz"
        out.write_bytes(gzip.compress(keep.to_csv(index=False).encode(), mtime=0))
        src[out.name] = {"from": f"osc-dev-v006-bringup captures/ident-slice2/A/{name}/sweep.csv",
                         "sha1": sha1(CAP / name / "sweep.csv"),
                         "rows": "segments 1 and 3, every row", "columns": COLUMNS}
    for name, s in PASSES:
        df, meta = tel.load(CAP / name, wide_codes=True)
        assert list(widecodes.slots_of_meta(meta)) == record_slots
        g = df[df["seg"] == s].reset_index(drop=True)
        x, y, c = samples(g)
        per[f"{name}/{s}"] = (x, y, c)
    up = [k for k in per if per[k][2]["sign"] > 0]
    dn = [k for k in per if per[k][2]["sign"] < 0]
    lo = max(min(per[k][0].min() for k in up), min(per[k][0].min() for k in dn))
    hi = min(max(per[k][0].max() for k in up), max(per[k][0].max() for k in dn))
    k0, k1 = int(np.ceil(lo / GRID)) + 1, int(np.floor(hi / GRID)) - 1
    knots = np.arange(k0, k1 + 1) * GRID
    F, r_f = fit([per[k][:2] for k in up], knots)
    R, r_r = fit([per[k][:2] for k in dn], knots)
    M = (F + R) / 2
    trips = []
    for t in ("pass-1", "pass-2"):
        f1, _ = fit([per[k][:2] for k in up if k.startswith(t)], knots)
        r1, _ = fit([per[k][:2] for k in dn if k.startswith(t)], knots)
        trips.append((f1 + r1) / 2)
    xg = np.arange(knots[0] + 24, knots[-1] - 24 + 1, 4.0)
    lk = postable.lookup_of_record(PAIR)[0]
    g_l = (lk.angle(xg + 24) - lk.angle(xg - 24)) / 48
    e_s = g_l.mean() / g_l - 1
    zone = {}
    for zn, z in ZONES.items():
        m = (xg >= z[0]) & (xg <= z[1])
        zone[zn] = {"straight": float(np.mean(np.abs(e_s[m])))}
        for n, a in (("mean", M), ("up", F), ("down", R)):
            g = slopes(xg, knots, a)
            e = (g / g.mean()) / (g_l / g_l.mean()) - 1
            zone[zn][n] = float(np.mean(np.abs(e[m])))
    excess = {}
    for zn, z in SPLIT_ZONES.items():
        m = (xg >= z[0]) & (xg <= z[1])
        excess[zn] = {}
        for n, a in (("up", F), ("down", R)):
            g = slopes(xg, knots, a)
            excess[zn][n] = float(np.mean((g.mean() / g - 1)[m]))
    s1, s2 = slopes(xg, knots, trips[0]), slopes(xg, knots, trips[1])
    pts = points(knots, M, *STOPS)
    gain = 16 + np.diff(pts)
    sl = (M[-1] - M[0]) / (knots[-1] - knots[0])
    exp = {
        "cyc_deg": CYC_DEG,
        "passes": {k: v[2] for k, v in per.items()},
        "knots": [int(knots[0]), int(knots[-1])],
        "residual_deg": {"up": r_f * CYC_DEG, "down": r_r * CYC_DEG},
        "up_less_down_deg": {"rms": rms(line_out(knots, F - R)) * CYC_DEG,
                             "max": float(np.abs(line_out(knots, F - R)).max()) * CYC_DEG},
        "trip_repeat": {"line_out_deg": rms(line_out(knots, trips[0] - trips[1])) * CYC_DEG,
                        "slope_rms": rms(s1 / s2 - 1)},
        "zone_slope_error": zone,
        "excess": excess,
        "counts_per_deg": 1 / (sl * CYC_DEG),
        "stops": list(STOPS),
        "points_max_abs": int(np.abs(pts).max()),
        "gain_q4": [int(gain.min()), int(gain.max())],
        "mean": [float(v) for v in M],
    }
    lpath = postable.lookup_of_record(PAIR)[1]
    (HERE / "lookup-of-record.csv").write_bytes(lpath.read_bytes())
    src["lookup-of-record.csv"] = {"from": f"notebooks {results.rel(lpath)}", "sha1": sha1(lpath),
                                   "rows": "whole"}
    return exp, src


def hold_rows(folder):
    """oscnb's hold segments of a sweep (bringup ident-plan q2_widecodes_openloop.hold_rows)."""
    df, meta = tel.load(folder)
    board = boards.from_meta(meta)
    board.check_meta(meta, str(folder))
    seg = pd.read_csv(folder / "segments.csv")
    kick = any(str(x).startswith("then:") for x in meta["schedule"])
    keep = []
    for d in (1, -1):
        drv = seg[(seg["dir"] == d) & (seg["cmd_duty_q15"] != 0)]
        if kick:
            drv = drv.iloc[1:]
        keep += [int(x) for x in drv["seg"]]
    return df[df["seg"].isin(keep)]["pos"].to_numpy(np.int64)


def local_find(pos, floor=100, side=64, edge=15):
    pos = np.asarray(pos, np.int64)
    h = np.bincount(pos, minlength=4096)
    lo, hi = pos.min() + edge, pos.max() - edge
    out, thin = {}, {}
    for k in range(16):
        c = (k << 8) | 255
        if not (lo + side <= c and c + 16 + side <= hi):
            continue
        s = 0
        while h[c + 1 + s] == 0 and s < 40:
            s += 1
        ref = np.r_[h[c - side:c], h[c + 1 + s:c + 1 + s + side]]
        n = float(np.median(ref))
        if n < floor:
            thin[k] = n
            continue
        ratio = h[c] / n
        wide = s >= 3 and ratio >= 0.5 * (1 + s)
        out[k] = {"slot": 8 * s if wide else 0, "skipped": s, "ratio": float(ratio), "rows_ref": n}
    return {"carries": out, "thin": thin}


def wide_part():
    sets, src, exp = {}, {}, {}
    for name, base, index in (("crawl", PAIR_DIR / "pot-table" / "crawl-9", "chains.csv"),
                              ("ladder", PAIR_DIR / "ladder" / "low-1", "cycles.csv")):
        folders = [base / f for f in pd.read_csv(base / index)["folder"]]
        parts = [hold_rows(f) for f in folders]
        halves = {}
        for par, label in ((0, "even"), (1, "odd")):
            pos = np.concatenate([p for i, p in enumerate(parts) if i % 2 == par])
            lo, hi = int(pos.min()), int(pos.max())
            h = np.bincount(pos, minlength=4096)
            halves[label] = {"lo": lo, "hi": hi, "rows": [int(v) for v in h[lo:hi + 1]]}
        sets[name] = halves
        whole = np.concatenate(parts)
        exp[name] = {"whole": local_find(whole), "even": local_find(np.concatenate(parts[0::2])),
                     "odd": local_find(np.concatenate(parts[1::2]))}
        digest = hashlib.sha1()
        for f in folders:
            digest.update(sha1(f / "sweep.csv").encode())
        src[f"widecodes.json.gz: {name}"] = {
            "from": f"notebooks {results.rel(base)}, {index}'s {len(folders)} folders, the hold segments' raw pot codes",
            "sha1": sha1(base / index), "sweeps_sha1": digest.hexdigest(),
            "rows": "histograms, even and odd folders by their index apart"}
    (HERE / "widecodes.json.gz").write_bytes(gzip.compress(json.dumps(sets).encode(), mtime=0))
    return exp, src


def main():
    record_slots = json.loads((HERE.parent / "mg90-d/wide-codes.json").read_text())["slots"]
    t_exp, t_src = table_part(record_slots)
    w_exp, w_src = wide_part()
    (HERE / "expected.json").write_text(json.dumps({"table": t_exp, "widecodes": w_exp}, indent=1) + "\n")
    (HERE / "sources.json").write_text(json.dumps({**t_src, **w_src}, indent=1) + "\n")


if __name__ == "__main__":
    main()
