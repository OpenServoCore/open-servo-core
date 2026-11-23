"""The spin fixtures and their expected values, from the notebooks.

Run read-only from a notebooks checkout:

    cd notebooks && uv run python <this file> [--full]

Writes, beside this file:
- `ladder-*.sweep.csv.gz`: ladder cycles of mg90-d on dev-v006-2A-D, each
  spin's last SPIN_KEEP rows and each coast's first COAST_KEEP_LADDER rows,
  the torque-off baseline left out (its zeros are in `expected.json`);
- `runup-*.sweep.csv.gz`: the run-ups of `run-up/runup-open-1`, every
  spin row and each coast's first COAST_KEEP_RUNUP rows;
- `expected.json`: the scales, and what the reductions below give on the
  trimmed rows;
- with `--full`, `full.json`: the same on the notebooks' whole datasets
  (the ladder's 132 coasts, the run-up notebook's 24 run-ups), which the
  opt-in parity test reads against the checkout named by OSC_NOTEBOOKS.

The reductions are the notebooks' own: the ladder notebook's `coast_emf`,
`e0_line`, `coast_ke`, `spin_window` and `spin_speed` and its gate; the
run-up notebook's `run_entry`, `open_mask`, `count_run`, `read_count`, the
cross-run check and `j_from_fits` through the origin, with one change:
the count is seeded by the EMF alone (`seed="emf"`), never the pot; and
the friction notebook's coast route (`bins` and the weighted line), with
J the pooled J of the same run-ups. Copied from the notebooks at main
34add9d, trimmed to what the Rust port computes.
"""
import gzip
import json
import sys
import warnings
from pathlib import Path

import numpy as np
import pandas as pd

from oscnb import coast, cycles, motors, results, ripple, scales, servos, tel

warnings.simplefilter("ignore")
HERE = Path(__file__).resolve().parent
NB = Path.cwd()
SERVO = servos.SERVOS["mg90-d"]
MOTOR = motors.MOTORS["mg90-d"]
BOARD_KEY = "dev-v006-2A-D"
PAIR = f"{SERVO.key}__{BOARD_KEY}"
PAIR_DIR = NB / "telemetry" / PAIR
FS = 20_000
N_R = MOTOR.ripple_per_rev
SC = scales.load(BOARD_KEY)
A_PER_COUNT, V_PER_COUNT = SC.a_per_count, SC.v_per_count
TAP_OFFSET, I_DRIVE_OFFSET = SC.terminal_offset_counts, SC.drive_offset_counts
RES = results.read("3-servo", "resistance", pair=PAIR)
IND = results.read("3-servo", "inductance", pair=PAIR)
LADDER = results.read("3-servo", "ladder", pair=PAIR)
RUNUP = results.read("3-servo", "run-up", pair=PAIR)
NL_TYPE = results.read("2-motor", "no-load-current", pair=SERVO.type_motor)
R_LOCKED = RES["r_term_ohm"]["value"]
L_H = IND["l_mh"]["value"] * 1e-3
KE = LADDER["ke_mv_per_rev_s"]["value"] * 1e-3 / (2 * np.pi)
KT = KE
_drag = NL_TYPE["drive_drag_by_level_meter"]["5 V"]
I_DRAG0 = float(_drag["a_a"]["value"] + _drag["b_a"]["value"] * np.log(
    2 * np.pi * _drag["speed_range_rev_s"]["lo"] / (2 * np.pi * 100.0)))
PWM_ARR = 1200

LADDER_SETS = ["ref-open-1", "ref-close-1", "capture-1", "rail-1",
               "ref-open-6", "ref-close-6"]
RUNUP_SETS = ["runup-open-1", "runup-close-1", "runup-open-6",
              "runup-close-6"]
# (set, rail, duty, repeat) of the ladder cycles the fixture keeps: six
# whose coasts pass the gate on 3.7, 5 and 7.4 V, and one that fails it
LADDER_PICK = [("capture-1", 3.7, 60, 0), ("capture-1", 5.0, 30, 0),
               ("capture-1", 5.0, 40, 1), ("capture-1", 5.0, 60, 2),
               ("capture-1", 7.4, 30, 0), ("capture-1", 7.4, 40, 1),
               ("capture-1", 3.7, 15, 0)]
RUNUP_PICK = "runup-open-1"
SPIN_KEEP = 1000
COAST_KEEP_LADDER = 2400
COAST_KEEP_RUNUP = 6000


# the ladder notebook's coast route
def spin_window(sp):
    ticks = sp["duty_counts"].abs()
    half = float(ticks.iloc[len(sp) // 2:].median())
    held = sp[ticks == half]
    n_held = len(held)
    n = int(min(100 * FS / 1e3, n_held // 2))
    n = max(n, int(20 * FS / 1e3))
    n = min(n, n_held)
    return held.iloc[-n:], half


def e0_line(emf, start, f_seed, target_ms=10, order=1):
    f, i0, i1, n = ripple.read_window(emf, start, f_seed, target_ms=target_ms)
    if not np.isfinite(f) or n == 0:
        return None
    nbox = max(int(round(FS / f)), 1)
    smooth = np.convolve(emf, np.ones(nbox) / nbox, mode="same")
    t = np.arange(len(emf)) / FS
    p = np.polyfit(t[i0:i1], smooth[i0:i1], order)
    return dict(e_cut=float(np.polyval(p, 0.0)),
                dedt=float(np.polyval(np.polyder(p), 0.0)),
                e_win=float(emf[i0:i1].mean()), i0=int(i0), i1=int(i1),
                f=float(f), n=int(n))


def coast_ke_row(sp, co, d, zero_e):
    cut = coast.cut_row(co)
    emf_read = coast.emf_from_cut(co, d, cut, zero_e, v_per_count=V_PER_COUNT)
    win, half = spin_window(sp)
    row = {"dir": d, "level": half, "cut": cut,
           "e_first": float(coast.early_emf(emf_read))}
    row["turned"] = bool(row["e_first"] >= coast.TURN_V)
    row.update(open_rows=0, f_seed=None, stop=None, window=None,
               spin_rev_s=None, gated=False)
    if not row["turned"]:
        return row, None
    bad = coast.open_rows(emf_read)
    emf = coast.bridge(emf_read, bad)
    stop = coast.stop_row(emf)
    f0 = coast.seed_freq(emf)
    row.update(open_rows=int(bad.sum()), f_seed=float(f0), stop=stop)
    m = e0_line(emf, coast.COAST_FROM_ROWS, f0)
    raw = win["current_raw"].to_numpy(float)
    x20 = raw[-int(0.02 * FS):]
    f20, _ = ripple.count_cycles(x20 - x20.mean(), f0)
    row["spin_rev_s"] = None if not np.isfinite(f20) else float(f20 / N_R)
    if m is None:
        return row, emf
    w = 2 * np.pi * m["f"] / N_R
    win_ms = (m["i1"] - m["i0"]) / FS * 1e3
    decel = -m["dedt"] * win_ms / 1e3 / m["e_win"]
    row["window"] = {"i0": m["i0"], "i1": m["i1"], "cycles": m["n"],
                     "rev_s": m["f"] / N_R, "e_win": m["e_win"],
                     "e_cut": m["e_cut"], "dedt": m["dedt"],
                     "ke": m["e_win"] / w, "decel_frac": float(decel)}
    row["gated"] = bool(decel <= 0.35 and m["n"] >= 4)
    return row, emf


def pooled(v):
    v = np.asarray(v, float)
    return {"mean": float(v.mean()),
            "sd": float(v.std(ddof=1)) if len(v) > 1 else None, "n": len(v)}


def ke_pool(rows):
    g = [r for r in rows if r["gated"]]
    out = {"all": pooled([r["window"]["ke"] for r in g]), "by_dir": {},
           "by_level": {}}
    for key, name in (("dir", "by_dir"), ("level", "by_level")):
        for k in sorted({r[key] for r in g}):
            out[name][str(int(k))] = pooled(
                [r["window"]["ke"] for r in g if r[key] == k])
    return out


def ladder_rows(df):
    _, zero_e = tel.rest_zeros(df)
    return zero_e, [coast_ke_row(sp, co, d, zero_e)
                    for d, sp, co in coast.pairs(df)]


# the run-up notebook's J, the EMF seed
def run_entry(d, spin, co, zero_i):
    g = spin.iloc[tel.drive_start(spin):]
    counts = g["current_raw"].to_numpy(float)
    taps = (g["vmotor_a"] - g["vmotor_b"]).to_numpy(float) * d
    return {"dir": d, "i": (counts - zero_i - I_DRIVE_OFFSET) * A_PER_COUNT,
            "u": (taps - TAP_OFFSET) * V_PER_COUNT, "coast": co}


FIT_EDGE_S = 1e-12


def fit_table(tc, per_rev, means, dropped, half=ripple.FIT_HALF_S,
              step=ripple.FIT_STEP_S, fs=FS):
    """oscnb `ripple.fit_table` with its window's edges inclusive by
    FIT_EDGE_S: the first window opens on the first crossing exactly, and
    rounding keeps or drops it (dropped in 5 of the 24 run-ups; J moves
    by 0.01 % pooled, 0.18 % at most in one run)."""
    angle = np.arange(len(tc)) * (2 * np.pi / per_rev)
    cum = {k: np.r_[0, np.cumsum(x)] for k, x in means.items()}
    cum_open = np.r_[0, np.cumsum(dropped)]
    rows = []
    for tm in np.arange(tc[0] + half, tc[-1] - half, step):
        m = (tc >= tm - half - FIT_EDGE_S) & (tc <= tm + half + FIT_EDGE_S)
        if m.sum() < 2 * per_rev + 1:
            continue
        p = np.polyfit(tc[m] - tm, angle[m], 2)
        res = angle[m] - np.polyval(p, tc[m] - tm)
        a = int(round((tm - half) * fs))
        b = int(round((tm + half) * fs))
        row = {"t": tm, "w": p[1], "alpha": 2 * p[0], "n": int(m.sum()),
               "res_deg": np.degrees(res.std()),
               "open_rows": int(cum_open[b] - cum_open[a])}
        for k in cum:
            row[k] = (cum[k][b] - cum[k][a]) / (b - a)
        rows.append(row)
    return pd.DataFrame(rows)


def read_run(r, zero_e):
    op = tel.dropout_rows(r["i"], coast.local_mean(r["i"]))
    op[:40] = False
    op[-20:] = False
    rough, _ = ripple.rough_speed(r["i"], r["u"], np.zeros(len(r["i"])),
                                  R_LOCKED, L_H, KE, (0.015, 0.045),
                                  seed="emf")
    cnt = ripple.count_speed(r["i"], rough, I_DRAG0, N_R)
    means = {"i": r["i"]}
    rev = ripple.rev_table(cnt["tc"], N_R, r["i"], means, op)
    rev["ok"] = ripple.flag_windows(rev["t"].to_numpy(), rev["w"].to_numpy())
    fit = fit_table(cnt["tc"], N_R, means, op)
    fit["res_ok"] = fit["res_deg"] <= 2.0 * fit["res_deg"].median()
    cut = coast.cut_row(r["coast"])
    emf = coast.emf_from_cut(r["coast"], r["dir"], cut, zero_e,
                             v_per_count=V_PER_COUNT)
    tab, info = coast.coast_table(emf, KE, N_R)
    return {"dir": r["dir"], "cnt": cnt, "rev": rev, "fit": fit, "tab": tab,
            "info": info, "rows": len(r["i"]), "dropouts": int(op.sum())}


def cross_run(reads):
    grid = np.arange(0, 0.055, 0.0005)
    for d in (1, -1):
        curves = [np.interp(grid, x["rev"]["t"][x["rev"]["ok"]],
                            x["rev"]["w"][x["rev"]["ok"]], left=np.nan,
                            right=np.nan)
                  for x in reads if x["dir"] == d]
        ref = np.nanmedian(np.array(curves), axis=0)
        for x in reads:
            if x["dir"] != d:
                continue
            h = x["rev"]
            off = np.abs(h["w"] / np.interp(h["t"], grid, ref) - 1) > 0.06
            x["rev"].loc[off.to_numpy(), "ok"] = False


def run_j(x):
    fit = x["fit"].copy()
    bad_t = x["rev"].loc[~x["rev"]["ok"], "t"].to_numpy()
    fit["win_ok"] = True
    if len(bad_t):
        tm = fit["t"].to_numpy()[:, None]
        fit["win_ok"] = ~(np.abs(tm - bad_t[None, :])
                          <= ripple.FIT_HALF_S).any(axis=1)
    g = fit[fit["res_ok"] & fit["win_ok"] & (fit["alpha"] > 0)]
    fn, _ = coast.coast_decel_fn(x["tab"])
    y = g["alpha"].to_numpy() - fn(g["w"].to_numpy())
    xx = KT * g["i"].to_numpy()
    use = np.isfinite(y) & np.isfinite(xx) & (g["i"].to_numpy() > 0)
    xx, y = xx[use], y[use]
    j = float((xx * xx).sum() / (xx * y).sum()) if len(xx) >= 8 else None
    return {"dir": x["dir"], "j": j, "fits_kept": int(use.sum()),
            "windows_flagged": int((~x["rev"]["ok"]).sum())}


def friction(reads, d, j):
    rc = pd.concat([x["tab"] for x in reads if x["dir"] == d],
                   ignore_index=True)
    rc["rev_s"] = rc["w"] / (2 * np.pi)
    rc["decel"] = -rc["alpha"]
    edges = np.arange(0, 525, 25.0)
    rc["bin"] = np.digitize(rc["rev_s"], edges) - 1
    g = rc.groupby("bin").agg(rev_s=("rev_s", "mean"),
                              decel=("decel", "median"),
                              sd=("decel", "std"),
                              n=("decel", "size")).reset_index()
    g["se"] = g["sd"] / np.sqrt(g["n"].clip(lower=1))
    g = g[(g["n"] >= 5) & (g["rev_s"] < 480)]
    w = g["rev_s"].to_numpy() * 2 * np.pi
    T = j * g["decel"].to_numpy()
    s = j * g["se"].to_numpy()
    X = np.vander(w, 2, increasing=True)
    coef, *_ = np.linalg.lstsq(X / s[:, None], T / s, rcond=None)
    res = T - X @ coef
    return {"dir": d, "tc": float(coef[0]), "b": float(coef[1]),
            "a0": float(coef[0] / j), "a1": float(coef[1] / j),
            "rms": float(np.sqrt((res ** 2).mean())), "bins": len(g)}


def run_summary(x, rj):
    tc = x["cnt"]["tc"]
    return {**rj, "rows": x["rows"], "crossings": len(tc),
            "added": int(x["cnt"]["added"]),
            "removed": int(x["cnt"]["removed"]), "tc_first": float(tc[0]),
            "tc_last": float(tc[-1]), "dropouts": x["dropouts"],
            "windows": len(x["rev"]),
            "windows_ok_local": int(x["rev"]["ok_local"].sum()),
            "fits": len(x["fit"]), "fits_res_ok": int(x["fit"]["res_ok"].sum()),
            "coast_fits": len(x["tab"]), "e0": float(x["info"]["e0"]),
            "stop": int(round(x["info"]["stop_ms"] * FS / 1e3)),
            "coast_open_rows": int(x["info"]["open_rows_coast"])}


def inertia(runs):
    reads = [read_run(r, ze) for r, ze in runs]
    for x in reads:
        x["rev"]["ok_local"] = x["rev"]["ok"].copy()
    cross_run(reads)
    rjs = [run_j(x) for x in reads]
    js = [r["j"] for r in rjs if r["j"] is not None]
    jp = pooled(js)
    return {"runs": [run_summary(x, rj) for x, rj in zip(reads, rjs)],
            "j": jp,
            "by_dir": {str(d): pooled([r["j"] for r in rjs if r["dir"] == d
                                       and r["j"] is not None])
                       for d in (1, -1)},
            "friction": [friction(reads, d, jp["mean"]) for d in (-1, 1)]}


def runup_runs(df):
    zero_i, zero_e = tel.rest_zeros(df)
    return zero_i, zero_e, [(run_entry(d, sp, co, zero_i), zero_e)
                            for d, sp, co in coast.pairs(df)]


# fixtures
def trimmed(df, spin_keep, coast_keep):
    """Each spin's last `spin_keep` rows (all with None) and each coast's
    first `coast_keep`, as the sweep's own CSV lines."""
    keep = []
    for d, sp, co in coast.pairs(df):
        keep += [sp if spin_keep is None else sp.iloc[-spin_keep:],
                 co.iloc[:coast_keep]]
    return pd.concat(keep)


def write_fixture(name, rows):
    text = rows.to_csv(index=False, lineterminator="\n")
    (HERE / name).write_bytes(gzip.compress(text.encode(), mtime=0))
    return pd.read_csv(HERE / name)


def load_raw(folder):
    return pd.read_csv(folder / "sweep.csv")


def ladder_fixture():
    out = []
    for name, rail, duty, rep in LADDER_PICK:
        cy = pd.read_csv(PAIR_DIR / "ladder" / name / "cycles.csv")
        hit = cy[(cy["volts_set"] == rail) & (cy["duty_pct"] == duty)
                 & (cy["repeat"] == rep)].iloc[0]
        folder = PAIR_DIR / "ladder" / name / hit["folder"]
        raw = load_raw(folder)
        zero_i, zero_e = tel.rest_zeros(raw)
        fname = f"ladder-{name}-k{int(hit['k'])}.sweep.csv.gz"
        df = write_fixture(fname, trimmed(raw, SPIN_KEEP, COAST_KEEP_LADDER))
        rows = [coast_ke_row(sp, co, d, zero_e)[0]
                for d, sp, co in coast.pairs(df)]
        out.append({"file": fname, "from": str(folder.relative_to(NB)),
                    "zero_i": float(zero_i), "zero_e": float(zero_e),
                    "coasts": rows})
    allrows = [r for f in out for r in f["coasts"]]
    return {"files": out, "pooled": ke_pool(allrows)}


def runup_fixture():
    cy = cycles.load(PAIR_DIR / "run-up" / RUNUP_PICK)
    files, runs = [], []
    for _, sw in cy.iterrows():
        folder = cy.folder(sw["k"])
        raw = load_raw(folder)
        zero_i, zero_e = tel.rest_zeros(raw)
        fname = f"runup-{RUNUP_PICK}-k{int(sw['k'])}.sweep.csv.gz"
        df = write_fixture(fname, trimmed(raw, None, COAST_KEEP_RUNUP))
        files.append({"file": fname, "from": str(folder.relative_to(NB)),
                      "zero_i": float(zero_i), "zero_e": float(zero_e)})
        for d, sp, co in coast.pairs(df):
            runs.append((run_entry(d, sp, co, zero_i), zero_e))
    return {"files": files, **inertia(runs)}


def checks(lad):
    """The counters on one fixture's rows, against the Rust's."""
    k = ladder_k("capture-1", 5.0, 30, 0)
    name = f"ladder-capture-1-k{k}.sweep.csv.gz"
    df = pd.read_csv(HERE / name)
    d, sp, co = coast.pairs(df)[0]
    cut = coast.cut_row(co)
    win, _ = spin_window(sp)
    raw = win["current_raw"].to_numpy(float)
    line = ripple.line_freq(raw, *ripple.BAND_HZ)
    # seeded by the coast's line, as the ladder seeds its spin tach
    f0 = [f for f in lad["files"] if f["file"] == name][0]["coasts"][0]["f_seed"]
    tc = ripple.crossings(raw, f0)
    f, n = ripple.count_cycles(raw, f0)
    tr = ripple.tracked_crossings(raw, 0, len(raw), f0)
    rep, added, removed = ripple.repair(np.delete(tr, [10, 11]))
    return {"file": name,
            "current": "the first spin's held tail, current_raw",
            "line_hz": float(line), "seed_hz": float(f0),
            "crossings": tc.tolist(),
            "count_hz": float(f), "count_cycles": int(n),
            "tracked": tr.tolist(), "repaired_added": int(added),
            "repaired_removed": int(removed), "repaired": rep.tolist(),
            "cut": int(cut)}


def ladder_k(name, rail, duty, rep):
    cy = pd.read_csv(PAIR_DIR / "ladder" / name / "cycles.csv")
    return int(cy[(cy["volts_set"] == rail) & (cy["duty_pct"] == duty)
                  & (cy["repeat"] == rep)].iloc[0]["k"])


def scales_json():
    return {"counts_per_a": SC.counts_per_a,
            "drive_offset_counts": I_DRIVE_OFFSET,
            "counts_per_v": SC.terminal_counts_per_v,
            "terminal_offset_counts": TAP_OFFSET, "pwm_arr": PWM_ARR,
            "window_s": 0.0, "r_ohm": R_LOCKED, "l_h": L_H, "ke": KE,
            "i_drag0_a": I_DRAG0, "per_rev": N_R, "fs": FS}


def record():
    return {"ke_mv_per_rev_s": LADDER["ke_mv_per_rev_s"]["value"],
            "ke_coasts_gated": LADDER["ke_coasts_gated"]["value"],
            "j_kg_m2": RUNUP["j_kg_m2"]["value"]}


def full():
    lad = []
    for name in LADDER_SETS:
        cy = cycles.load(PAIR_DIR / "ladder" / name)
        for _, sw in cy.iterrows():
            folder = cy.folder(sw["k"])
            df = load_raw(folder)
            zero_i, zero_e = tel.rest_zeros(df)
            _, rows = ladder_rows(df)
            lad.append({"from": str(folder.relative_to(NB)),
                        "zero_i": float(zero_i), "zero_e": float(zero_e),
                        "coasts": [r for r, _ in rows]})
    runs, src = [], []
    for name in RUNUP_SETS:
        cy = cycles.load(PAIR_DIR / "run-up" / name)
        for _, sw in cy.iterrows():
            folder = cy.folder(sw["k"])
            zero_i, zero_e, rr = runup_runs(load_raw(folder))
            runs += rr
            src.append({"from": str(folder.relative_to(NB)),
                        "zero_i": float(zero_i), "zero_e": float(zero_e)})
    allrows = [r for f in lad for r in f["coasts"]]
    return {"scales": scales_json(), "record": record(),
            "ladder": {"files": lad, "pooled": ke_pool(allrows)},
            "runup": {"files": src, **inertia(runs)}}


def dump(path, obj):
    path.write_text(json.dumps(obj, indent=1) + "\n")


if __name__ == "__main__":
    exp = {"scales": scales_json(), "record": record(),
           "ladder": ladder_fixture(), "runup": runup_fixture()}
    exp["checks"] = checks(exp["ladder"])
    dump(HERE / "expected.json", exp)
    p = exp["ladder"]["pooled"]["all"]
    print(f"fixture Ke {p['mean'] * 2e3 * np.pi:.4f} mV per rev/s over "
          f"{p['n']} gated coasts; J {exp['runup']['j']['mean'] * 1e9:.4f}"
          f"e-9 over {exp['runup']['j']['n']} run-ups")
    if "--full" in sys.argv:
        f = full()
        dump(HERE / "full.json", f)
        p = f["ladder"]["pooled"]["all"]
        print(f"full Ke {p['mean'] * 2e3 * np.pi:.4f} mV per rev/s over "
              f"{p['n']} gated coasts; J {f['runup']['j']['mean'] * 1e9:.4f}"
              f"e-9 over {f['runup']['j']['n']} run-ups")
