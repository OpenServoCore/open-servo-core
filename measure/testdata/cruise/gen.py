"""The cruise fit's fixture and its expected values: the friction line of
record, stage 5 cruise-friction on mg90-d / dev-v006-2A-D, from the
notebook's own code.

Run read-only from a notebooks checkout:

    cd notebooks && uv run python <this file>

It runs the code cells of `stages/5-closed-loop/cruise-friction/
cruise-friction.ipynb` that build the legs and the lines (CELLS: the set,
the window field, the sittings, the captures, `reading`, `replay`, `leg`,
`line_ma`, `speeds`, `leg_numbers`, `legs`, the line of record, the
repeat, both sittings together and the joint line with `inertia_ma`),
unchanged, in the notebook's folder, then writes beside this file:
- `legs.json.gz`: every whole leg of both sittings (44 legs, 884000
  rows; the line of record takes 19 of the first's 20, the joint line 42):
  its sitting, capture, segment and goal, the polls before and after it (rail, winding estimate, bias),
  the sitting's first winding estimate, and the leg's rows column by
  column: the ticks as the first and the count (they are
  whole), the window flag as a string of 0 and 1, the raw pot and
  `duty_counts` as the first value then the steps between rows, and
  `current_raw` the same over the rows whose window was read only (the
  replay reads no other);
- `expected.json`: the sources with their sha1s, the set's table points
  and slots, the constants the replay reads, each sitting's fields, each
  leg's numbers by `leg_numbers` with `inertia_ma` and the steady mean,
  the line of record (`LINE`, its legs) and the joint line (`J`, its legs),
  unrounded.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import nbformat  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
NB_DIR = Path("stages/5-closed-loop/cruise-friction").resolve()
NB = NB_DIR / "cruise-friction.ipynb"
# the code cells that build the legs and the lines; the rest print, plot,
# check the reading against g1 or write the result file
CELLS = [1, 2, 4, 6, 8, 11, 13, 19, 23, 25, 26, 27, 29, 31, 33, 34, 38, 42, 46]


def sha1(path):
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()


def deltas(x):
    x = [int(v) for v in x]
    return [x[0]] + [b - a for a, b in zip(x, x[1:])] if x else []


def f(x):
    return None if x is None or not np.isfinite(x) else float(x)


def main():
    nb = nbformat.read(NB, as_version=4)
    ns = {}
    os.chdir(NB_DIR)
    for k in CELLS:
        exec(compile(nb.cells[k].source, f"cell {k}", "exec"), ns)
    caps, sitting, legs_of = ns["CAPS"], ns["SITTING"], ns["LEGS"]
    both, joint, fit_legs = ns["both"], ns["JOINT"], ns["FIT_LEGS"]
    stored, numbers = [], []

    def in_line(name, r):
        return name == "g3" and bool(((fit_legs.capture == r.capture)
                                      & (fit_legs.seg == r.seg)).any())

    def in_joint(name, r):
        return bool(((joint.run == name) & (joint.capture == r.capture)
                     & (joint.seg == r.seg)).any())

    for name in ns["RUNS"]:
        whole = legs_of[name][legs_of[name].whole]
        t_read = int(caps[name]["c300"]["polls"].loc[0, "t_winding_cc"])
        for _, r in whole.iterrows():
            cap = caps[name][r.capture]
            g = cap["rows"][cap["rows"]["seg"] == r.seg]
            tick = g["tick"].to_numpy()
            assert (np.diff(tick) == 1).all()
            valid = g["window_valid"].to_numpy() == 1
            p0, p1 = cap["polls"].loc[r.seg], cap["polls"].loc[r.seg + 1]
            poll = lambda p: {k: int(p[k]) for k in (  # noqa: E731
                "vbus_raw", "t_winding_cc", "current_bias_counts")}
            stored.append({
                "run": name, "capture": r.capture, "seg": int(r.seg),
                "goal": float(g["goal"].iloc[0]),
                "before": poll(p0), "after": poll(p1),
                "t_read_cc": t_read,
                "first_tick": int(tick[0]), "n": len(g),
                "window_valid": "".join("1" if v else "0" for v in valid),
                "pos": deltas(g["pos"]),
                "duty_counts": deltas(g["duty_counts"]),
                "current_raw": deltas(g["current_raw"].to_numpy()[valid]),
            })
            j = both[(both.run == name) & (both.capture == r.capture)
                     & (both.seg == r.seg)]
            assert len(j) == 1
            inertia = float(ns["inertia_ma"](j.iloc[0]))
            numbers.append({
                "run": name, "capture": r.capture, "seg": int(r.seg),
                "dir": int(r.dir), "goal": int(r.goal), "rows": int(r.rows),
                "leg_ms": f(r.leg_ms), "held_in_ms": f(r.held_in_ms),
                "cps": f(r.cps), "v2_v1_pct": f(r.v2_v1_pct),
                "phase": int(r.phase), "sd_in_ma": f(r.sd_in_ma),
                "sd_worst_ma": f(r.sd_worst_ma),
                "read_share": f(r.read_share), "held_ma": f(r.held_ma),
                "read_ma": f(r.read_ma), "line_ma": f(r.line_ma),
                "integ_ma": f(r.integ_ma), "inertia_ma": inertia,
                "steady_ma": f(r.held_ma - inertia),
                "in_line": in_line(name, r), "in_joint": in_joint(name, r),
            })

    def line(fit, n):
        return {"intercept_ma": float(fit["intercept"]),
                "slope_ma_per_kcps": float(fit["slope"]),
                "intercept_se_ma": float(fit["intercept_se"]),
                "slope_se_ma_per_kcps": float(fit["slope_se"]),
                "rms_ma": float(fit["rms"]), "legs": n,
                "viscous_ma_s_per_rad": float(fit["slope"]) / 1e3
                * ns["COUNTS_PER_MOTOR_RAD"],
                "resid_ma": [float(x) for x in fit["resid"]]}

    board = ns["BOARD"]
    fields = ns["FIELDS"]
    cap0 = caps["g3"]["c300"]
    expected = {
        "sources": {
            "notebook": {"file": "stages/5-closed-loop/cruise-friction/"
                         "cruise-friction.ipynb", "sha1": sha1(NB),
                         "cells": CELLS},
            "captures": {
                f"{name}/{s}": {
                    "file": str(Path("telemetry") / ns["PAIR"]
                                / "window-term" / name / s / "track.csv"),
                    "sha1": sha1(ns["RUNS"][name] / s / "track.csv")}
                for name in ns["RUNS"] for s in ns["SPEEDS"]},
            "set": ns["PAIR"],
        },
        "points": [int(p) for p in ns["TABLE"].points],
        "slots": list(ns["SLOTS"]),
        "constants": {
            "counts_per_a": float(ns["CPA"]),
            "i_kp_q88": int(fields["i_kp_q88"]),
            "i_ki_q412": int(fields["i_ki_q412"]),
            "ke_vpc_q": int(fields["ke_vpc_q"]),
            "j_ff_q88": int(fields["j_ff_q88"]),
            "counts_per_motor_rad": float(ns["COUNTS_PER_MOTOR_RAD"]),
            "pwm_arr": int(cap0["arr"]), "tick_hz": int(cap0["hz"]),
            "bridge_min_ticks": int(board.bridge_min_ticks),
            "settle_start_ticks": int(board.i_settle_start_ticks),
            "settle_q15": [int(q) for q in board.i_settle_q15],
            "copper_per_c": float(ns["COPPER"]),
        },
        "sittings": {name: {k: int(v) for k, v in s.items()
                            if k in ("g_window_q016", "r_hat_q12",
                                     "fric_fc_counts", "fric_fv_q016",
                                     "speed_ceiling_cps",
                                     "velocity_limit_cps")}
                     for name, s in sitting.items()},
        "legs": numbers,
        "line": line(ns["LINE"], int(len(fit_legs))),
        "joint": line(ns["J"], int(len(joint))),
        "words": list(ns["WORDS"]), "joint_words": list(ns["JOINT_WORDS"]),
    }
    raw = json.dumps(stored, separators=(",", ":")).encode()
    (HERE / "legs.json.gz").write_bytes(gzip.compress(raw, 9, mtime=0))
    (HERE / "expected.json").write_text(json.dumps(expected, indent=1) + "\n")
    print(f"{len(stored)} legs, {sum(s['n'] for s in stored)} rows; line "
          f"{expected['line']['intercept_ma']:.4f} mA + "
          f"{expected['line']['slope_ma_per_kcps']:.4f} per 1000 c/s; joint "
          f"{expected['joint']['intercept_ma']:.4f} + "
          f"{expected['joint']['slope_ma_per_kcps']:.4f}")


if __name__ == "__main__":
    main()
