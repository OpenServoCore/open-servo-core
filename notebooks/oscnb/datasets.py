"""Datasets: captures grouped by the hardware that produced them.

A dataset is one board paired with one servo on one supply. Its manifest.json
names the board and servo, so a capture carries its own constants rather than
relying on whoever opens it to remember which rig it came off.

  telemetry/<dataset>/manifest.json      {board, servo, supply, ...}
  telemetry/<dataset>/<experiment>/capture-N/<recording>.csv.gz
                                            /<recording>.meta.json

An experiment is a named measurement procedure - grid, bridge, breakaway,
stepcoast, ripple, reversal. A capture is one repeat of it. Some experiments
write several recordings per capture (bridge writes slow, fast and spindown).

Reading a recording checks its meta.json `sense` block against the board the
manifest claims, so a dataset pointed at the wrong board fails at load rather
than silently producing numbers that are off by the ratio of two shunts.
"""

from dataclasses import dataclass
from functools import cached_property
from pathlib import Path

import gzip
import json

from .boards import BOARDS
from .servos import SERVOS

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

    def frame(self, check=True):
        import pandas as pd
        if check:
            self.dataset.board.check_meta(self.meta, str(self))
        with gzip.open(self.path, "rt") as fh:
            return pd.read_csv(fh)

    def __str__(self):
        return f"{self.dataset.key}/{self.experiment}/{self.capture}/{self.name}"


@dataclass(frozen=True)
class Dataset:
    key: str
    path: Path
    manifest: dict

    @property
    def supply(self):
        return self.manifest["supply"]

    @property
    def retired(self):
        return bool(self.manifest.get("retired"))

    @cached_property
    def board(self):
        k = self.manifest["board"]
        if k not in BOARDS:
            raise KeyError(
                f"{self.key}: manifest names board {k!r}, which has no entry in "
                f"boards.py. Add it rather than reusing a board that does not "
                f"describe this hardware."
            )
        return BOARDS[k]

    @cached_property
    def servo(self):
        k = self.manifest["servo"]
        if k not in SERVOS:
            raise KeyError(f"{self.key}: manifest names servo {k!r}, absent from servos.py")
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
    for m in sorted(Path(root).glob("*/manifest.json")):
        out[m.parent.name] = Dataset(m.parent.name, m.parent, json.loads(m.read_text()))
    return out


def load(key, root=ROOT) -> Dataset:
    ds = find(root)
    if key not in ds:
        raise KeyError(f"no dataset {key!r}; have {sorted(ds)}")
    return ds[key]
