"""Pick a dataset, and show what that choice means.

A Rig is one board paired with one servo. You do not normally choose those: a
dataset's manifest names them, because a capture is a fact about the hardware
that produced it. Selecting a dataset selects the rig.

  rig.pick()                       dropdown of datasets, for working in Jupyter
  rig.use("sg90-a__dev-v006-D__2s")            explicit, for a headless run

Overriding board or servo by hand is for REPROCESSING - re-reading old captures
under a corrected constants entry. It is deliberately awkward:

  rig.use(key, board="...", servo="...")

The picker only changes what the NEXT cells see, so change it and run the
notebook below. Under `nbconvert --execute` there is nobody to click, so the
default dataset applies and the table still prints into the committed output -
which is the point of printing it: a human reading the notebook on GitHub can
check the numbers without running anything.
"""

from dataclasses import dataclass

import pandas as pd

from .boards import BOARDS, DEFAULT_BOARD
from .servos import SERVOS, DEFAULT_SERVO


@dataclass(frozen=True)
class Rig:
    board: object
    servo: object

    @property
    def label(self):
        return f"{self.servo.label} on {self.board.label}"

    def check_meta(self, meta, where=""):
        """Guard a capture against this rig. Board parts must match the
        capture's own `sense` block."""
        self.board.check_meta(meta, where or self.label)

    def table(self) -> pd.DataFrame:
        b, s = self.board, self.servo
        rows = [
            ("board", "converter step", b.v_adc_per_count * 1e3, "mV per count"),
            ("board", "terminal divider ratio r", b.div_ratio, ""),
            ("board", "terminal difference", b.v_term_per_count * 1e3, "mV per count"),
            ("board", "terminal bias VB", b.vb_nom_v * 1e3, "mV"),
            ("board", "terminal bias VB", b.vb_nom_counts, "counts"),
            ("board", "driven-low terminal reads", b.vb_nom_counts * (b.div_ratio - 1) / b.div_ratio, "counts"),
            ("board", "motor current", b.a_per_count * 1e3, "mA per count"),
            ("board", "motor current", b.counts_per_a, "counts per A"),
            ("board", "shunt", b.shunt_mohm, "mOhm"),
            ("board", "amplifier gain", b.amp_gain, "x"),
            ("board", "rail tap ratio", b.vbus_ratio, ""),
            ("board", "sample rate", b.tick_hz / 1e3, "samples per ms"),
            ("board", "current settled from" + ("" if b.floors_verified else " (UNVERIFIED)"), b.floor_i_pct, "% duty"),
            ("board", "voltage settled from" + ("" if b.floors_verified else " (UNVERIFIED)"), b.floor_v_pct, "% duty"),
            ("servo", "pot intercept", s.pos_intercept, "counts"),
            ("servo", "pot slope", s.pos_per_deg, "counts per degree"),
            ("servo", "envelope low", s.travel_counts[0], f"counts ({s.travel_deg[0]:g} deg)"),
            ("servo", "envelope high", s.travel_counts[1], f"counts ({s.travel_deg[1]:g} deg)"),
            ("servo", "runway", s.runway_counts, "counts"),
            ("servo", "seek guard low", s.guard_counts[0], "counts"),
            ("servo", "seek guard high", s.guard_counts[1], "counts"),
        ]
        for name, m in s.measured.items():
            rows.append(("servo (measured)", f"{name} [{m.lo:g}, {m.hi:g}] {m.source}", m.value, m.unit))
        df = pd.DataFrame(rows, columns=["where", "quantity", "value", "unit"])
        return df.set_index(["where", "quantity", "unit"]).round(4)

    def measured_table(self):
        """Measured values, kept apart from the declared constants above.

        A declared constant is what the hardware IS. A measured value is what
        somebody found when they looked, and it carries a bracket, a source and
        any caveat that came with it. Mixing the two in one table is how a
        fitted number ends up being treated as a specification."""
        import pandas as pd
        rows = []
        for where, obj in (("board", self.board), ("servo", self.servo)):
            for name, m in getattr(obj, "measured", {}).items():
                rows.append({"where": where, "quantity": name.replace("_measured", ""),
                             "value": m.value, "unit": m.unit,
                             "bracket": f"[{m.lo:g}, {m.hi:g}]",
                             "source": m.source, "caveat": m.note})
        return pd.DataFrame(rows).set_index(["where", "quantity"])

    def show(self):
        print(self.label)
        for note, who in ((self.board.notes, "board"), (self.servo.notes, "servo")):
            if note:
                print(f"\n  {who}: {note}")
        print()
        try:
            from IPython.display import display
            display(self.table())
        except ImportError:
            print(self.table().to_string())
        return self


_dataset = None
_current = None


def use(key=None, board=None, servo=None, show=True):
    """Select a dataset. Board and servo come from its manifest unless
    overridden, which is only for reprocessing under corrected constants."""
    global _dataset, _current
    from . import datasets
    found = datasets.find()
    if key is None:
        live = [k for k, d in found.items() if not d.retired]
        if not live:
            raise RuntimeError("no dataset with a manifest under telemetry/")
        key = sorted(live)[0]
    _dataset = found[key] if key in found else datasets.load(key)
    b = BOARDS[board] if board else _dataset.board
    s = SERVOS[servo] if servo else _dataset.servo
    _current = Rig(b, s)
    if board or servo:
        print(f"OVERRIDE: reading {key} as {b.key} / {s.key}, "
              f"not the {_dataset.manifest['board']} / {_dataset.manifest['servo']} "
              f"its manifest names.\n")
    if show:
        _show()
    return _dataset


def _show():
    d, r = _dataset, _current
    print(f"dataset  {d.key}")
    print(f"supply   {d.supply}")
    print(f"rig      {r.label}\n")
    try:
        from IPython.display import display
        display(d.summary())
        print("\nderived constants (what the hardware is)")
        display(r.table())
        print("\nmeasured values (what someone found when they looked)")
        display(r.measured_table())
    except ImportError:
        print(d.summary().to_string()); print(r.table().to_string())
        print(r.measured_table().to_string())


def dataset():
    if _dataset is None:
        use(show=False)
    return _dataset


def current() -> Rig:
    if _current is None:
        use(show=False)
    return _current


def pick():
    """A dropdown of every dataset with a manifest. Change it, then run below.

    Falls back to the default selection when ipywidgets is missing, so a
    headless execute still produces the table rather than an empty cell."""
    from . import datasets
    found = {k: v for k, v in datasets.find().items() if not v.retired}
    try:
        import ipywidgets as w
        from IPython.display import display, clear_output
    except ImportError:
        print("ipywidgets not available, using the default dataset")
        return use(show=True)

    opts = [(f"{k}   ({v.manifest['servo']} on {v.manifest['board']}, {v.supply})", k)
            for k, v in found.items()]
    dd = w.Dropdown(options=opts, value=(_dataset.key if _dataset else opts[0][1]),
                    description="dataset:", layout=w.Layout(width="720px"),
                    style={"description_width": "70px"})
    out = w.Output()

    def redraw(_=None):
        with out:
            clear_output(wait=True)
            use(dd.value, show=True)

    dd.observe(redraw, names="value")
    display(w.VBox([dd, out]))
    redraw()
    return _dataset
