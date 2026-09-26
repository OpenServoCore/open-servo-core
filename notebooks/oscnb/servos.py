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
        # DISPUTED, but by less than it was. 19.96 is a three-point fit to
        # EYEBALLED horn angles (432=10, 2028=90, 3625=170 deg). The ripple
        # tach says 19.30, bracket [19.30, 19.43] across three independent 2S
        # sets, anchored on counted teeth and a confirmed 6/rev - see
        # measured["pot_scale_measured"]. The gap is 3.3%, down from the 9.1%
        # an earlier Welch-peak route claimed, which nb06 sec 5.5 showed was a
        # final speed divided by a mean speed. Kept at 19.96 only because the
        # intercept was fitted jointly with it and there is no absolute angle
        # reference to re-anchor the offset yet. Anything reported in DEGREES
        # or rad/s from this servo is ~3% out; counts and wiper-volts are
        # unaffected.
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
                3.77, 3.68, 3.92, "Ohm",
                "nb02 sec 21 route (i): locked-rotor ladder on 2S, rungs 15-24% "
                "only, 4 captures passing the span gate",
                "V0 +0.455 [0.409, 0.484] V from the same fit, terminal "
                "referenced. SUPERSEDES 3.91 [3.85, 3.99] with V0 +0.413, which "
                "was fitted 12-24%: the 12% rung is 144 ticks under the 160-tick "
                "floor, window_valid clear, and on a resistor the crest current "
                "sample reads 10% low there (nb02 sec 20). Correcting that rung "
                "by the resistor factor instead gives 4.23 with V0 +0.34, "
                "rejected because the corrected point sits 88 mV, 3 rms, off the "
                "honest line. R and V0 trade off across a 0.16 to 0.33 A span so "
                "the pair moves together; the 2% hump correction on the honest "
                "rungs alone gives 3.86. The free-shaft onset route read 4.23 "
                "through a coasting shaft"),
            "tau_e_measured": Measured(
                154, 140, 171, "us", "nb02 part 2, sample-to-sample pole",
                "loop time constant L/R_eff at a rung onset; 146 us on USB. "
                "An independent ARX fit on the same data gave 142 +/- 9. "
                "Unchanged by the crest floor of nb02 sec 20: the pole is a ratio "
                "of samples at one duty and one phase, so a per-duty under-read "
                "cancels"),
            "l_motor_in_servo_measured": Measured(
                0.62, 0.53, 0.74, "mH",
                "nb02 sec 21, tau x R_eff with the route (i) R",
                "bracket is the R_eff choice (3.79 to 4.34 Ohm) times the tau "
                "capture scatter; was 0.64 [0.55, 0.75] on the 12-24% R. "
                "l_motor_measured above is the bare can, a different unit"),
            "pot_scale_measured": Measured(
                19.30, 19.30, 19.43, "counts/deg",
                "nb06 sec 5.5, travel-weighted ripple cycle counting against "
                "the counted gear ratio, 2S duty grid 30-45%, n=80 steps over "
                "10 captures, per-step CV 0.51%",
                "SUPERSEDES 18.29 (C = 265.5) from a Welch-peak-per-step route "
                "on the same population. Section 5.5 runs both routes paired "
                "on the identical 80 steps: the Welch peak sits at the SETTLED "
                "line while the counted mean ripple rate covers the whole step "
                "including spin-up, so the ratio is a final speed over a mean "
                "speed. The line sweeps 67% inside a rung, the paired frequency "
                "bias is +5.46% in 80 steps out of 80, and the sweep predicts "
                "it to 0.16 points. Trim the sweep away and the two routes "
                "agree to 0.08% on frequency. Finite spectral resolution is not "
                "the cause: an 8x finer bin moves the bias by 0.014 points. The "
                "USB captures reproduce the same sign and size of bias, so it "
                "belongs to the METHOD, not the supply. BRACKET is the span of "
                "the three independent 2S sets - grid 30-45% gives 19.30, "
                "steady steps 40 and 60% give 19.37, grid 50-55% gives 19.43 - "
                "and the carried value sits at the low edge because it is the "
                "set with the widest travel coverage and the tightest per-step "
                "spread, where the higher two are pulled up by short accepted "
                "windows. STANDING CAVEAT: the pot's local gain swings 5.8x "
                "with position (nb06 sec 6, 1.15 to 6.62 mV per ripple cycle "
                "in 20 mV bins), so any single number is a position-weighted "
                "average of a varying function. This one averages over 0.32 to "
                "2.95 wiper V, about 170 degrees of output travel, which is "
                "essentially the whole 10 to 170 degree envelope. Still "
                "DISAGREES with the 19.96 declared above by 3.3%, down from "
                "9.1%, and with a Kv-and-coast-BEMF route that lands near "
                "20-23"),
            "coast_bemf_slope_measured": Measured(
                0.197, 0.19, 0.204, "V per wiper V/s", "nb01, chain 2",
                "chain 2 had GND-returned taps, the defect the VB bias fixed - "
                "re-measure on chain 3 before trusting"),
            # THE Ke CLOSURE. Speed from the ripple line during the DRIVE
            # segment, BEMF from the coast that follows. The pot is never
            # touched, which is the whole point: against pot speed the same
            # captures give 13-28% low with CV 26%.
            "ke_measured": Measured(
                7.584, 7.44, 7.73, "mV per motor rev/s",
                "nb03 route R: ripple speed + coast BEMF over 2 whole ripple "
                "periods, 2S, n=30, CV 0.94%, flat across 20/30/40% and both dirs",
                "IN-SERVO, geared and enclosed. run28's 7.542 is a BARE CAN "
                "with pinion, by a two-point Kv slope that needs I*R to cancel "
                "between its points - an assumption gears would break. This "
                "route has no I*R term at all (coast means i=0). The -2.3% gap "
                "is NOT within-campaign heating: Ke drifts only +0.19% across "
                "5 captures, and the wrong way for warming. It is most likely "
                "just A DIFFERENT MOTOR: run28 ran a bare can on the encoder "
                "rig (and 6/rev came from tearing down a SIBLING can), while "
                "this is the can inside the servo. Spread in this family is "
                "wide - the M10 spec tab says Kv 6000 where run28 measured "
                "7960, 33% apart - so two cans agreeing to 2.3% is CLOSE, not "
                "a discrepancy. TREAT Ke AS PER-UNIT, never a family constant: "
                "measure it per servo, which is what this registry is for. "
                "Known bias in THIS "
                "number: drive ripple is an average while the shaft still "
                "accelerates, BEMF is near final speed, so it reads slightly "
                "HIGH - the wrong direction to explain the gap. SUPERSEDES an "
                "earlier 7.367 (CV 1.9%) from the same method: that one used a "
                "1400 Hz lower search band, which EXCLUDES the 20% rung whose "
                "line sits at 800 Hz, and averaged BEMF over a fixed sample "
                "count rather than 2 whole ripple periods. Use 600-5200 Hz."),
        },
        notes=(
            "Gears serviced and cleaned after a run of end stop crashes; back "
            "driving is smooth and the earlier one sided tooth skipping is "
            "gone. The 10 to 170 degree envelope is deliberately inboard of "
            "the stops: a rung is an unpolled burst and the window length is "
            "the only thing bounding travel."
        ),
    ),
    "mg90-a": Servo(
        key="mg90-a",
        label="Vorpal MG90 clone, unit A",
        model="MG90",
        # PROVISIONAL. No angle reference yet: 0 deg is put at the low hand
        # stop (pos 209) and the slope is a round 20 counts per degree, the
        # same class as sg90-a. Anything in degrees from this servo is a
        # placeholder until an indexed horn or dial read replaces it; counts
        # and wiper volts are unaffected.
        pos_intercept=209.0,
        pos_per_deg=20.0,
        # the guards below, expressed through the provisional mapping
        travel_deg=(15.55, 166.55),
        # 75+ counts inside the soft limits 432/3626 so a seek band never
        # meets the soft-limit clamp; hand stops at 209/3849
        guard_counts=(520, 3540),
        measured={},
        notes=(
            "All-metal four-stage train, ball-bearing output, plastic D-key "
            "coupling to the pot. Hand stops 209/3849 (low counts = left). "
            "Gear ratio (tooth count) and ripple order (commutator segments) "
            "are not yet known, so the notebooks that need them cannot run on "
            "this servo until they are. 2S windows come from its own pilot: "
            "v_ss = 0.2275*d - 0.845 counts/ms over 10-50%, about 0.83x "
            "sg90-a. Speed keeps rising to 100% (about 20 counts/ms at the "
            "end of a 140 ms rung, rail flat at 7.9 V), but spin-up takes "
            "50-60 ms, so the 30 ms allowance in the window rule leaves the "
            "high-duty rungs 5-8% short of the travel target."
        ),
    ),
    # Not a servo: the 4x4 grid of 3.3 ohm 0.5 W resistors wired in place of
    # the motor for the sensor-chain session. A static load has no pot, so the
    # position fields are identity placeholders and travel/guard are empty.
    "grid-3r7": Servo(
        key="grid-3r7",
        label="static load, 4x4 grid of 3.3 ohm",
        model="load",
        pos_intercept=0.0,
        pos_per_deg=1.0,
        travel_deg=(0.0, 0.0),
        guard_counts=(0, 0),
        measured={
            "r_load": Measured(3.7, 3.6, 3.8, "Ohm",
                               "DMM 200 ohm range at the grid header pins, probe offset 0.2 removed",
                               "cold; 10.7 W laps heat it, tempco unmeasured"),
        },
        notes="no motor, no pot: ladders run with osc sweep --static-load",
    ),
}


DEFAULT_SERVO = "sg90-a"
