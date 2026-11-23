"""The closed-loop self-test fixtures and their expected values, from the
notebooks' scorecard.

Run read-only from a notebooks checkout:

    cd notebooks && uv run python <this file> [--full]

Writes, beside this file:
- `steps-moves.csv.gz`: four moves of the `steps` captures of rung
  RUNG (mg90-d on dev-v006-2A-D), 2, 20 and 60 deg, both ways, the 60 deg
  one of run-2 a hold that hunts, each segment's rows whole, the columns
  the scores read: run, seg, tick, window_valid, the raw pot,
  current_raw and duty_counts;
- `expected.json`: the rung's position table points and wide-code slots,
  its counts per degree, current scale and tick rate, and per move its
  capture's current zero and what the scorecard's own functions give:
  `step_move`'s overshoot and settle, the rest error (`settled_pos`
  against the goal), the share of the last REST_MS driven
  (`landing_tail_driven`'s test) and the peak |i_ma| over the segment;
- with `--full`, `full.json`: the same for every step move of the rung's
  three runs, which the opt-in parity test reads against the checkout
  named by OSC_NOTEBOOKS.

The scores are `oscnb.scorecard`'s, at notebooks main 34add9d, on the
capture `scorecard.load` reads: the raw pot through the rung's wide-code
slots and table, the rolling median of MEDIAN_ROWS rows within a segment.
"""
import gzip
import json
import sys
from pathlib import Path

import numpy as np

from oscnb import scorecard as sc

HERE = Path(__file__).resolve().parent
NB = Path.cwd()
PAIR = "mg90-d__dev-v006-2A-D"
RUNG = "20a-holds-yield-pot-terms"
RUNG_DIR = NB / "telemetry" / PAIR / "scorecard" / RUNG
PICK = ((1, 2), (1, 7), (1, 10), (2, 12))
COLUMNS = ["run", "seg", "tick", "window_valid", "pos", "current_raw", "duty_counts"]


def move_scores(cap, seg):
    g = sc.seg_rows(cap, seg["seg"])
    over, settle, _ = sc.step_move(cap, seg)
    tail = sc.tail(g, sc.REST_MS)
    driven = float((tail["duty_counts"] != 0).mean())
    return {
        "seg": seg["seg"],
        "size_deg": seg["size_deg"],
        "direction": seg["direction"],
        "leg": seg["leg"],
        "goal_counts": seg["goal_counts"],
        "rows": len(g),
        "overshoot_deg": over,
        "settle_s": settle,
        "rest_error_deg": abs(seg["goal_counts"] - sc.settled_pos(cap, seg["seg"])) / cap["cpd"],
        "tail_driven_share": driven,
        "quiet": driven <= sc.TAIL_DRIVEN_SHARE,
        "i_peak_ma": float(np.nanmax(np.abs(g["i_ma"].to_numpy()))),
        "pos_mean_counts": float(g["pos"].mean()),
        "pos_f_last_counts": float(g["pos_f"].iloc[-1]),
    }


def run_entry(k):
    folder = RUNG_DIR / f"run-{k}" / "steps"
    cap = sc.load(folder)
    return folder, cap, {
        "folder": str(folder.relative_to(NB)),
        "zero_counts": sc.current_zero(cap["rows"]),
        "moves": [move_scores(cap, s) for s in sc.plan_segs(cap, "step")],
    }


def main():
    runs = {k: run_entry(k) for k in sorted({r for r, _ in PICK})}
    folder, cap, _ = runs[1]
    slots = sc.rung_wide_slots(folder)
    table = cap["table"]
    parts, moves = [], []
    for run, seg in PICK:
        c = runs[run][1]
        rows = c["rows"]
        keep = rows[rows["seg"] == seg].copy()
        keep["pos"] = keep["pos_raw"]
        keep["run"] = run
        parts.append(keep[COLUMNS].to_csv(index=False, header=not parts))
        s = {p["seg"]: p for p in sc.plan_segs(c, "step")}[seg]
        moves.append({"run": run, "zero_counts": sc.current_zero(rows), **move_scores(c, s)})
    (HERE / "steps-moves.csv.gz").write_bytes(gzip.compress("".join(parts).encode(), mtime=0))
    expected = {
        "source": f"telemetry/{PAIR}/scorecard/{RUNG}/run-K/steps",
        "points": [int(p) for p in table.points],
        "slots": list(slots) if slots is not None else None,
        "counts_per_deg": cap["cpd"],
        "counts_per_a": cap["counts_per_a"],
        "tick_hz": cap["tick_hz"],
        "constants": {"median_rows": sc.MEDIAN_ROWS, "settle_band_deg": sc.SETTLE_BAND_DEG,
                      "rest_ms": sc.REST_MS, "tail_driven_share": sc.TAIL_DRIVEN_SHARE},
        "moves": moves,
    }
    (HERE / "expected.json").write_text(json.dumps(expected, indent=1) + "\n")
    if "--full" in sys.argv:
        full = {"rung": f"telemetry/{PAIR}/scorecard/{RUNG}",
                "runs": [(runs.get(k) or run_entry(k))[2] for k in (1, 2, 3)]}
        (HERE / "full.json").write_text(json.dumps(full, indent=1) + "\n")


if __name__ == "__main__":
    main()
