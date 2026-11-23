"""The window-term fixtures and their expected values, from the stage 3
window-term notebook's own code, read only.

Run from a notebooks checkout:

    cd notebooks && MPLBACKEND=Agg uv run python -I \
        ../open-servo-core/measure/testdata/window/generate.py \
        ../open-servo-core/measure/testdata/window

It executes the notebook's method cells (never its result-writing ones),
picks the 24 onsets the plan's excitation names (session A, the first four
kept of each duty and direction), trims their rows out of the recorded
sweeps into gzipped osc-lab CSVs, and writes expected.json: the per-onset
tables, the head fit, the direction split, the bootstrap sd and the
checks, for the 24 and for all 105.
"""
import gzip
import hashlib
import json
import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

OUT = Path(sys.argv[1]).resolve()
ROOT = Path.cwd()
NB = ROOT / "stages/3-servo/window-term/window-term.ipynb"
WT = ROOT / "oscnb/windowterm.py"
# the notebook's method cells, in order; the first line of each is checked
CELLS = {1: "from pathlib import Path", 2: "SERVO = ", 4: "SCALES = ",
         6: "RES = ", 8: "AFTER = ", 11: "CAPS = ", 21: "def catalog",
         23: "def onset_set", 25: "def edges", 27: "def fit",
         35: "def bootstrap", 37: "def pinned", 41: "def by",
         43: "def lateness", 45: "R_Q12 = "}


def sha1(p):
    return hashlib.sha1(Path(p).read_bytes()).hexdigest()


cells = json.loads(NB.read_text())["cells"]
ns = {}
os.chdir(NB.parent)
for i, head in CELLS.items():
    src = "".join(cells[i]["source"])
    if not src.startswith(head):
        raise SystemExit(f"cell {i} is not {head!r}: the notebook moved")
    if i == 35:
        # the definition only: its draws run below, per set
        src = src.split("\n\n\nBOOTS")[0]
    exec(src, ns)
os.chdir(ROOT)

CAT, CAPS, wt = ns["CAT"], ns["CAPS"], ns["wt"]
INFO = pd.concat([CAT[n][1] for n in ("A", "brake")], ignore_index=True)
KEPT = INFO[INFO.ok]
SUB = pd.concat([g[g.session == "A"].head(4)
                 for _, g in KEPT.groupby(["cmd_q15", "sgn"])])
SUB_KEYS = set(zip(SUB.cap, SUB.onset))

# the subset as a session of its own, for onset_set
rows = pd.concat([CAT[n][0] for n in ("A", "brake")], ignore_index=True)
chosen = np.array([k in SUB_KEYS for k in zip(rows.cap, rows.onset)])
rows["ok"] = rows.ok.to_numpy() & chosen
CAT["sub"] = (rows, SUB, CAT["A"][2])
SUB_CAPS = sorted(set(SUB.cap))
CAPS["sub"] = [c for n in ("A", "brake") for c in CAPS[n]
               if f"{n}/{c['name']}" in SUB_CAPS]


def solved(names):
    s = ns["onset_set"](names)
    s.lags = ns["edges"](s.rail)
    s.F = ns["grid"](s)
    f = ns["fit"](s, s.y.mean(axis=0), s.v_on.mean())
    late = ns["lateness"](s, f) * 1e-3
    sgn = s.keys.sgn.to_numpy()
    split = {}
    for d, name in ((1, "forward"), (-1, "reverse")):
        free, pin = ns["by"](s, sgn == d, f)
        split[name] = {"n": int((sgn == d).sum()), "g_free_s": free * 1e-3,
                       "g_pinned_s": pin * 1e-3}
    per = ns["pinned"](s, f["L"], f["R"])
    boot = ns["bootstrap"](s) * 1e-3
    g_over = f["g"] + late
    field = round(g_over * 1e3 * ns["Q016_PER_MS"])
    keys = s.keys
    onsets = []
    for j in range(len(keys)):
        onsets.append({
            "cap": keys.cap[j], "seg": int(keys.onset[j]),
            "sgn": int(keys.sgn[j]),
            "y_counts": s.y[j, :ns["READ_PERIODS"]].tolist(),
            "v_on_v": float(s.v_on[j]), "g_pinned_s": float(per[j])})
    return {
        "onsets": onsets, "n": len(s.y), "rail_v": s.rail,
        "lags": list(s.lags), "h": s.h[:ns["READ_PERIODS"]].tolist(),
        "gain": s.gain[:ns["READ_PERIODS"]].tolist(),
        "g_s": float(f["g"]), "l_h": float(f["L"]), "r_ohm": float(f["R"]),
        "edge": bool(f["edge"]), "rms_a": float(f["rms"]),
        "v_on_v": float(s.v_on.mean()),
        "resid_a": f["res"][:ns["READ_PERIODS"]].tolist(),
        "winding_a": f["iw"][:ns["READ_PERIODS"]].tolist(),
        "lateness_s": late, "g_over_mean_s": g_over,
        "tau_periods": float(f["L"] / f["R"] * ns["HZ"]),
        "split": split, "boot_sd_s": float(pd.Series(boot).std()),
        "boot_draws": ns["BOOT"], "field": field,
        "fence": field * ns["R_Q12"]}


def gate_rows(info):
    return [{"cap": r.cap, "seg": int(r.onset), "sgn": int(r.sgn),
             "pre_rows": int(r.pre_rows),
             "pre_vd_max": None if np.isnan(r.pre_vd_max)
             else float(r.pre_vd_max),
             "moved": float(r.moved), "same_sign": bool(r.same_sign),
             "seq": list(r.seq), "ok": bool(r.ok)} for r in info.itertuples()]


# the trimmed sweeps: each chosen onset's segment from its first row to
# AFTER rows past the onset, the recorded lines verbatim
OUT.mkdir(parents=True, exist_ok=True)
caps = {}
for cap in SUB_CAPS:
    session, sweep = cap.split("/")
    folder = ns["SESSIONS"][session][int(sweep.split("-")[1]) - 1]
    src = folder / "sweep.csv"
    lines = src.read_text().splitlines()
    head, body = lines[0], lines[1:]
    seg_of = [int(line.split(",", 1)[0]) for line in body]
    segs = sorted(int(o) for c, o in SUB_KEYS if c == cap)
    keep = []
    for seg in segs:
        idx = [i for i, s in enumerate(seg_of) if s == seg]
        duty = [int(body[i].split(",")[8]) for i in idx]
        on = next(k for k, d in enumerate(duty) if d != 0)
        keep += [body[i] for i in idx[:on + ns["AFTER"]]]
    name = f"{session}-{sweep}.sweep.csv.gz"
    data = ("\n".join([head] + keep) + "\n").encode()
    (OUT / name).write_bytes(gzip.compress(data, mtime=0))
    meta = json.loads((folder / "meta.json").read_text())
    caps[cap] = {
        "file": name,
        "from": str(src.relative_to(ROOT)), "sha1": sha1(src),
        "segments": segs,
        "tick_hz": meta["tick_hz"], "pwm_period_ticks": meta["pwm_period_ticks"],
        "vbus_counts": meta["vbus_counts"], "r_q12": meta["drive"]["r_q12"]}

T = ns["TIMING"]
B = ns["BOARD"]
scales = {
    "counts_per_a": ns["CPA"], "terminal_counts_per_v": ns["VPV"],
    "terminal_offset_counts": ns["TAP_OFFSET"],
    "sample_ticks": ns["SAMPLE"], "r_brake_ohm": ns["R_BRAKE"],
    "v_window_min_ticks": B.v_window_min_ticks,
    "settle_start_ticks": B.i_settle_start_ticks,
    "settle_q15": list(B.i_settle_q15),
    "rail_line": [{"v": v, "tap_on": T[f"tap_on_edge_lag_{r}"]["value"],
                   "off": T[f"off_edge_lag_{r}"]["value"]}
                  for r, v in ns["scales"].RAIL_LINE]}

git = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                     capture_output=True, text=True).stdout.strip()
sources = {
    "method": "the stage 3 window-term notebook's cells "
              f"{sorted(CELLS)} executed as written, at notebooks {git}",
    "notebook_sha1": sha1(NB), "windowterm_sha1": sha1(WT),
    "subset": "session A, the first four kept onsets of each duty and "
              "direction in record order: the plan's 24",
    "scales": scales, "caps": caps}
expected = {
    "method": {k: ns[k] for k in ("AFTER", "TAP_GATE", "POT_GATE",
                                  "READ_PERIODS", "V_ROW_MAX", "BOOT")},
    "gates": gate_rows(INFO),
    "sub": solved(["sub"]),
    "full": solved(["A", "brake"]),
    "record": json.loads((NB.parent / "results/mg90-d__dev-v006-2A-D.json")
                         .read_text())["g_over_mean_ms"]["value"] * 1e-3}
(OUT / "sources.json").write_text(json.dumps(sources, indent=1) + "\n")
(OUT / "expected.json").write_text(json.dumps(expected, indent=1) + "\n")
print(f"sub {expected['sub']['n']} onsets: g_over_mean "
      f"{expected['sub']['g_over_mean_s'] * 1e3:.4f} mS; full "
      f"{expected['full']['n']}: {expected['full']['g_over_mean_s'] * 1e3:.4f}")
