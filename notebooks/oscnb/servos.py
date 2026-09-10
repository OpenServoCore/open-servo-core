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
    """A value somebody measured, carried with its bracket and where it came
    from. Copied here so nobody has to hunt through memory, bringup docs and
    the mechanical package again - but a copy is only as good as its
    provenance, so `source` is mandatory and `note` carries the caveat that
    would otherwise be lost in the move.

    Never quote one of these without its bracket."""
    value: float
    lo: float
    hi: float
    unit: str
    source: str
    note: str = ""

    def __str__(self):
        s = f"{self.value:g} {self.unit} [{self.lo:g}, {self.hi:g}] ({self.source})"
        return s + (f" - {self.note}" if self.note else "")


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
        # DISPUTED. 19.96 is a three-point fit to EYEBALLED horn angles
        # (432=10, 2028=90, 3625=170 deg). The ripple tach says 18.29,
        # +/-1.2% over 20 measurements, anchored on counted teeth and a
        # confirmed 6/rev - see measured["pot_scale"]. Kept at 19.96 only
        # because the intercept was fitted jointly with it and there is no
        # absolute angle reference to re-anchor the offset yet. Anything
        # reported in DEGREES or rad/s from this servo is ~9% out; counts
        # and wiper-volts are unaffected.
        pos_per_deg=19.96,
        travel_deg=(10.0, 170.0),
        guard_counts=(432, 3625),
        measured={
            # --- gear train, counted not fitted ---
            "gear_ratio_measured": Measured(
                234.73, 234.73, 234.73, ":1", "mechanical/sg90/measurements.py",
                "(47*38*32*23)/(10*10*8*7), counted teeth on the donor - exact"),
            "ripple_order_measured": Measured(
                6, 6, 6, "per motor rev", "bringup isns-diff runs 25-27",
                "3-segment commutator x 2 brushes; optical tach agreed 0.7% at "
                "d40 and 2.0% at d65 - a parts count, not an inference"),
            # --- bare-can motor model card, bringup run28 FINAL ---
            "kv_measured": Measured(
                7960, 7880, 8040, "RPM/V", "bringup isns-diff run28",
                "bare can, +/-1% HSI scale; pinion stayed on the shaft so true "
                "bare-shaft Kv sits slightly ABOVE this, call it ~8000"),
            "kt_measured": Measured(
                1.20, 1.19, 1.21, "mN-m/A", "bringup isns-diff run28", ""),
            "r_motor_measured": Measured(
                4.2, 4.16, 4.24, "Ohm", "bringup isns-diff run28",
                "bare can. nb03 got 4.5 [4.3, 4.8] in-servo via a full-duty "
                "onset anchor - the two are not the same measurement"),
            "l_motor_measured": Measured(
                0.5, 0.45, 0.55, "mH", "bringup isns-diff run28",
                "locked-rotor burst, tau_e 102 us"),
            "stall_current_measured": Measured(
                1.5, 1.45, 1.55, "A", "bringup isns-diff run28", "= V/R confirmed"),
            "stall_torque_measured": Measured(
                1.8, 1.75, 1.85, "mN-m", "bringup isns-diff run28", ""),
            "free_run_current_measured": Measured(
                0.081, 0.078, 0.084, "A", "bringup isns-diff run28", "at duty 65"),
            # --- in-servo, this dataset ---
            "r_winding_measured": Measured(
                4.5, 4.3, 4.8, "Ohm", "nb03, full duty onset anchor", ""),
            "pot_scale_measured": Measured(
                18.29, 18.07, 18.49, "counts/deg",
                "ripple tach vs counted gear ratio, 2S grid 30-45%, n=20",
                "DISAGREES with the 19.96 declared above (eyeballed horn "
                "angles) by 9.1%, and with a Kv-and-coast-BEMF route that "
                "lands near 20-23. Unresolved - see notes"),
            "coast_bemf_slope_measured": Measured(
                0.197, 0.19, 0.204, "V per wiper V/s", "nb01, chain 2",
                "chain 2 had GND-returned taps, the defect the VB bias fixed - "
                "re-measure on chain 3 before trusting"),
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
