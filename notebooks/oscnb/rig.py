"""Pick a board and a servo, and show what that choice means.

A Rig is one board paired with one servo. Notebooks select a rig once, near the
top, then read every constant off it. Nothing downstream hardcodes a divider or
a shunt.

Two ways to select, and both end at the same place:

  rig.picker()              two dropdowns, for working in Jupyter
  rig.select("dev-v006-D", "sg90-a")    explicit, for a headless run

The picker only changes what the NEXT cells see, so change a dropdown and run
the notebook below it. Under `nbconvert --execute` there is nobody to click, so
the defaults apply and the table still prints into the committed output - which
is the point of printing it: a human reading the notebook on GitHub can check
the numbers without running anything.
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


_current = Rig(BOARDS[DEFAULT_BOARD], SERVOS[DEFAULT_SERVO])


def select(board=DEFAULT_BOARD, servo=DEFAULT_SERVO, show=True):
    """Set the rig every later cell will read."""
    global _current
    _current = Rig(BOARDS[board], SERVOS[servo])
    return _current.show() if show else _current


def current() -> Rig:
    return _current


def picker():
    """Two dropdowns. Change either, then run the cells below.

    Falls back to printing the default selection when ipywidgets is missing or
    when nothing is driving the UI, so a headless execute still produces the
    table rather than an empty cell."""
    try:
        import ipywidgets as w
        from IPython.display import display, clear_output
    except ImportError:
        print("ipywidgets not available, using defaults")
        return select(show=True)

    b = w.Dropdown(options=[(v.label, k) for k, v in BOARDS.items()],
                   value=_current.board.key, description="board:",
                   layout=w.Layout(width="640px"), style={"description_width": "60px"})
    s = w.Dropdown(options=[(v.label, k) for k, v in SERVOS.items()],
                   value=_current.servo.key, description="servo:",
                   layout=w.Layout(width="640px"), style={"description_width": "60px"})
    out = w.Output()

    def redraw(_=None):
        with out:
            clear_output(wait=True)
            select(b.value, s.value, show=True)

    b.observe(redraw, names="value")
    s.observe(redraw, names="value")
    display(w.VBox([b, s, out]))
    redraw()
    return _current
