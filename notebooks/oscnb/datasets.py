"""Datasets: captures grouped by the hardware that produced them.

A dataset is one servo on one supply, recorded through one board under one
drive rule. Its dataset.toml declares what the recordings cannot say about
themselves: which servo was bolted on, which supply fed it, when it was
captured, and the drive rule and current limit every recording ran under. The
board is not declared. Every recording's meta.json carries a `sense` block read
from the control table at capture time, and the board is identified from that.

  telemetry/<dataset>/dataset.toml       {servo, supply, rule, current_limit_counts, captured, notes, [load], ...}
  telemetry/<dataset>/pos-lut.json       the position table the servo ran, once per dataset
  telemetry/<dataset>/pos-lut-built.json the table `osc lut build` made from the dataset
  telemetry/<dataset>/<experiment>/capture-N/<recording>.csv.gz
                                            /<recording>.meta.json

A recording's meta.json `plant` block names the position table the servo streamed
`pos_lin` through (`lut_state`, `lut_crc`), its plant stamp verdict and its
`data_flags`; `Dataset.pos_lut` is the table itself when one was LIVE.

A static load is measured again for each session where the board sees it,
because its cable and contacts change with the rig while the servo entry keeps
the load's own value. A `[load]` table in dataset.toml carries that reading
(`ohm`, `lo`, `hi`, `where`, `source`, optional `note`); `Dataset.load` returns
it, or the servo entry's `r_load` when the table is absent.

An experiment is a named measurement procedure - grid, bridge, breakaway,
stepcoast, ripple, reversal. A capture is one repeat of it. Some experiments
write several recordings per capture (bridge writes slow, fast and spindown).

The board comes from the dataset's first recording, and reading any recording
checks its own `sense` block against that board, so a dataset that mixes
captures from two boards fails at load rather than silently producing numbers
that are off by the ratio of two shunts.

The drive rule says how the drive was held. `free` datasets were captured
before the servo limited open-loop current: every rung steps straight to its
duty. `limit` datasets were captured under the servo's current limit, where a
rung climbs to its duty over tens of ms. Steady state does not know which rule
reached it, transients do. A meta.json with no `drive` block and a dataset.toml
with no `rule` are `free`, and reading a recording checks its rule against its
dataset's the same way it checks `sense`.
"""

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import gzip
import json
import tomllib

from . import boards
from .servos import SERVOS, Measured

ROOT = Path(__file__).resolve().parent.parent / "telemetry"


@dataclass(frozen=True)
class Recording:
    dataset: "Dataset"
    experiment: str
    capture: str
    name: str
    path: Path

    @cached_property
    def meta(self) -> dict:
        p = self.path.with_suffix("").with_suffix(".meta.json")
        if not p.exists():                      # single-recording experiments
            p = self.path.parent / "meta.json"
        return json.loads(p.read_text())

    @property
    def rule(self):
        """The drive rule the meta's `drive` block names; none is `free`."""
        return self.meta.get("drive", {}).get("rule", "free")

    def frame(self, check=True):
        import pandas as pd
        if check:
            self.dataset.board.check_meta(self.meta, str(self))
            if self.rule != self.dataset.rule:
                raise ValueError(
                    f"{self}: captured under drive rule {self.rule!r}, its dataset "
                    f"declares {self.dataset.rule!r}. A dataset holds one rule."
                )
        with gzip.open(self.path, "rt") as fh:
            return pd.read_csv(fh)

    def __str__(self):
        return f"{self.dataset.key}/{self.experiment}/{self.capture}/{self.name}"


@dataclass(frozen=True)
class Dataset:
    key: str
    path: Path
    decl: dict
    """What dataset.toml declares: servo, supply, rule, current_limit_counts,
    captured, notes, retired."""

    @property
    def supply(self):
        return self.decl["supply"]

    @property
    def rule(self):
        """`free` or `limit`; a dataset.toml with no `rule` is `free`."""
        return self.decl.get("rule", "free")

    @property
    def retired(self):
        return bool(self.decl.get("retired"))

    @property
    def load(self):
        """The load path this session measured, as a Measured; the servo
        entry's `r_load` when dataset.toml has no `[load]` table."""
        t = self.decl.get("load")
        if t is None:
            return self.servo.measured["r_load"]
        return Measured(t["ohm"], t["lo"], t["hi"], "Ohm", f"{t['where']}: {t['source']}",
                        t.get("note", ""))

    @cached_property
    def pos_lut(self):
        """pos-lut.json: the table the servo ran, in the image form `osc lut
        write` takes, tagged `lut_state` and `lut_crc`; None at the identity."""
        p = self.path / "pos-lut.json"
        return json.loads(p.read_text()) if p.exists() else None

    def first_recording(self):
        for e in self.experiments:
            for c in self.captures(e):
                for r in self.recordings(e, c):
                    return r
        return None

    @cached_property
    def board(self):
        r = self.first_recording()
        if r is None:
            raise LookupError(f"{self.key}: no recording to read the board from")
        b = boards.from_meta(r.meta)
        if b is None:
            raise LookupError(
                f"{r}: meta.json sense {r.meta['sense']} matches no entry in "
                f"boards.py. Add the board to boards.py rather than bending an "
                f"entry that does not describe this hardware."
            )
        return b

    @cached_property
    def servo(self):
        k = self.decl["servo"]
        if k not in SERVOS:
            raise KeyError(f"{self.key}: dataset.toml names servo {k!r}, absent from servos.py")
        return SERVOS[k]

    @cached_property
    def rig(self):
        from .rig import Rig
        return Rig(self.board, self.servo)

    @cached_property
    def experiments(self) -> list:
        return sorted(d.name for d in self.path.iterdir()
                      if d.is_dir() and any(d.glob("capture-*")))

    def captures(self, experiment) -> list:
        ds = (self.path / experiment).glob("capture-*")
        return sorted((d.name for d in ds if d.is_dir()),
                      key=lambda n: int(n.split("-")[1]) if n.split("-")[1].isdigit() else 0)

    def recordings(self, experiment, capture) -> list:
        d = self.path / experiment / capture
        return [Recording(self, experiment, capture, p.name[:-len(".csv.gz")], p)
                for p in sorted(d.glob("*.csv.gz"))]

    def read(self, experiment, capture, name=None, check=True):
        recs = self.recordings(experiment, capture)
        if name is not None:
            recs = [r for r in recs if r.name == name]
        if len(recs) != 1:
            have = [r.name for r in self.recordings(experiment, capture)]
            raise KeyError(f"{self.key}/{experiment}/{capture}: pick one of {have}")
        return recs[0].frame(check=check)

    def summary(self):
        import pandas as pd
        rows = [{"experiment": e,
                 "captures": len(self.captures(e)),
                 "recordings per capture": len(self.recordings(e, self.captures(e)[0])),
                 "names": ", ".join(r.name for r in self.recordings(e, self.captures(e)[0]))}
                for e in self.experiments]
        return pd.DataFrame(rows).set_index("experiment")


def find(root=ROOT) -> dict:
    out = {}
    for m in sorted(Path(root).glob("*/dataset.toml")):
        out[m.parent.name] = Dataset(m.parent.name, m.parent, tomllib.loads(m.read_text()))
    return out


def load(key, root=ROOT) -> Dataset:
    ds = find(root)
    if key not in ds:
        raise KeyError(f"no dataset {key!r}; have {sorted(ds)}")
    return ds[key]
