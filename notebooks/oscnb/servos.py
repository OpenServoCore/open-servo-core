"""Servos under test.

One entry per physical servo. A servo is the motor, gearbox and pot: what is
being measured. The electronics doing the measuring live in boards.py.

Adding a servo is an entry here. The raw captures never change.

Keep DECLARED facts (geometry, travel, pot mapping) apart from MEASURED ones.
A measured value carries the notebook that produced it and its bracket, so a
number never loses its provenance by being copied into a constants file.
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Measured:
    """A value some notebook measured. `source` is which notebook, `lo`/`hi`
    the bracket. Never quote one of these without its bracket."""
    value: float
    lo: float
    hi: float
    unit: str
    source: str

    def __str__(self):
        return f"{self.value:g} {self.unit} [{self.lo:g}, {self.hi:g}] ({self.source})"


@dataclass(frozen=True)
class Servo:
    key: str
    label: str
    model: str
    # pot mapping, three point anchored: pos = intercept + per_deg * degrees
    pos_intercept: float
    pos_per_deg: float
    # the envelope rungs are driven inside, in degrees
    travel_deg: tuple
    # seek guard band in pot counts, inboard of the physical stops
    guard_counts: tuple
    measured: dict = field(default_factory=dict)
    notes: str = ""

    def deg_to_pos(self, deg):
        return self.pos_intercept + self.pos_per_deg * deg

    def pos_to_deg(self, pos):
        return (pos - self.pos_intercept) / self.pos_per_deg

    @property
    def travel_counts(self) -> tuple:
        return tuple(self.deg_to_pos(d) for d in self.travel_deg)

    @property
    def runway_counts(self) -> float:
        lo, hi = self.travel_counts
        return hi - lo


SERVOS = {
    "sg90-a": Servo(
        key="sg90-a",
        label="SG90 clone, unit A",
        model="SG90",
        pos_intercept=232.0,
        pos_per_deg=19.96,
        travel_deg=(10.0, 170.0),
        guard_counts=(432, 3625),
        measured={
            "r_winding": Measured(4.5, 4.3, 4.8, "Ohm", "nb03, full duty onset anchor"),
        },
        notes=(
            "Gears serviced and cleaned after a run of end stop crashes; back "
            "driving is smooth and the earlier one sided tooth skipping is "
            "gone. The 10 to 170 degree envelope is deliberately inboard of "
            "the stops: a rung is an unpolled burst and the window length is "
            "the only thing bounding travel."
        ),
    ),
}

DEFAULT_SERVO = "sg90-a"
