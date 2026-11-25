"""The spins step's recorded runs and what the decided Ke pooling gives on
them, for `tests/spin.rs`.

Run from this directory with any Python 3, read-only on the bring-up
checkout:

    python3 gen.py

Writes, beside this file:
- `hunt-R/NN-raw.csv.gz`: the spins captures of one `osc-ident` run on
  the bench servo (mg90-d on dev-v006), every non-pilot spin's drive and
  coast, copied unchanged;
- `expected.json`: per run on file (the 12 the replay note reads, hunt-R
  among them), the run's scales and spins from its `results/spins.json`
  and the replay's variant C numbers by the trend line's compare (the
  note's `runs.csv`, columns `line_*`): one tap zero a run from both
  directions' coast tails, the first 2 spins each way a level, the top
  level pooled only when the line through every level's mean is within
  2 %. The opt-in replay test reads the other runs from the checkout
  named by OSC_BRINGUP.

The numbers come from `kb/scratch/spins-ke-pool/variants.py` (oscnb's
coast reduction), not from the Rust port.
"""
import csv
import hashlib
import json
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
BRINGUP = Path.home() / "projects/OpenServoCore/osc-dev-v006-bringup"
NOTE = BRINGUP / "kb/scratch/spins-ke-pool"
SERVO = "00000000ffffffff76e5bc220ef6abcd"
RUNS = {  # variants.py's RUNS, captures/ relative
    "spins-run1": f"ident-spins/run1/runs/{SERVO}/20261123T122128Z",
    "spins-run2": f"ident-spins/run2/runs/{SERVO}/20261123T123333Z",
    "spins-run3": f"ident-spins/run3/runs/{SERVO}/20261123T124320Z",
    "spins-run4": f"ident-spins/run4/runs/{SERVO}/20261123T124458Z",
    "tune-run3": f"ident-tune/run3/runs/{SERVO}/20261123T124912Z",
    "tune-run4": f"ident-tune/run4/runs/{SERVO}/20261123T125543Z",
    "cruise-b": f"ident-cruise/b/{SERVO}/20261124T225943Z",
    "conform-a": f"ident-conform-gate/a/{SERVO}/20261125T060517Z",
    "conform-b": f"ident-conform-gate/b/{SERVO}/20261125T060839Z",
    "hunt-R": f"ident-hunt/R/{SERVO}/20261125T082846Z",
    "hunt-run3": f"ident-hunt/run3/R/{SERVO}/20261125T090531Z",
    "hunt-run4": f"ident-hunt/run4/R/{SERVO}/20261125T194450Z",
}
KEPT = "hunt-R"
PER_REV = 6  # mg90-d's nameplate: 2 brushes times 3 rotor segments


def num(v):
    return None if v in ("", "nan") else float(v)


def main():
    rows = {r["run"]: r for r in csv.DictReader(open(NOTE / "runs.csv"))
            if r["var"] == "C"}
    out = {"source": "kb/scratch/spins-ke-pool/variants.py, runs.csv "
                     "variant C, the trend line's compare (line_*)",
           "kept": KEPT, "per_rev": PER_REV, "runs": []}
    for name, rel in RUNS.items():
        run = BRINGUP / "captures" / rel
        res = json.loads((run / "results/spins.json").read_text())
        r = rows[name]
        spins = [{"n": s["n"], "level": s["level_share"], "dir": s["dir"],
                  "captures": s["captures"]}
                 for s in res["spins"] if not s["pilot"]]
        entry = {
            "name": name, "from": f"captures/{rel}",
            "polarity": res["drive_polarity"]["driven"],
            "fs": json.loads((run / "captures/spins/01-raw.meta.json")
                             .read_text())["tick_hz"],
            "scales": res["fits"]["scales"], "spins": spins,
            "c": {"zero_counts": num(r["zero"]),
                  "ke_mv_per_rev_s": num(r["line_ke"]),
                  "top_pooled": r["line_top_pooled"] == "True",
                  "top_vs_lower": num(r["top_vs_lower"]),
                  "trend": num(r["line_trend"]), "sd": num(r["line_sd"]),
                  "ke_coasts": int(r["ke_coasts"]), "gated": int(r["gated"]),
                  "plan": r["line_plan"]},
        }
        if name == KEPT:
            d = HERE / name
            d.mkdir(exist_ok=True)
            files = {}
            for s in spins:
                for k in s["captures"]:
                    f = f"{k:02d}-raw.csv.gz"
                    src = run / "captures/spins" / f
                    shutil.copyfile(src, d / f)
                    files[f] = hashlib.sha1(src.read_bytes()).hexdigest()
            entry["files"] = files
        out["runs"].append(entry)
    (HERE / "expected.json").write_text(json.dumps(out, indent=1) + "\n")


if __name__ == "__main__":
    main()
