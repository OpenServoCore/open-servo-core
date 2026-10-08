"""Measurement boards.

One entry per physical board. A board is the measurement chain: shunt,
amplifier, dividers, ADC. It says nothing about which motor is bolted to it -
that lives in servos.py.

The raw captures never change. Swapping a board means adding an entry here and
selecting it, not reprocessing anything.

AUTHORITY: every capture's meta.json carries a `sense` block read from the
servo at capture time. That block is the truth. The values here are declared so
a human can read them, and check_meta() fails loudly when they disagree - never
edit a number here to silence that error, fix the entry or the selection.
"""

from dataclasses import dataclass, field

from .servos import Measured


@dataclass(frozen=True)
class Board:
    key: str
    label: str
    # current sense
    shunt_mohm: float
    gain_milli: int
    # motor terminal taps: top to the terminal, bottom returned to the VB bias
    vmotor_div_top: float
    vmotor_div_bot: float
    bias_top: float          # VB network, 3V3 -> VB
    bias_bot: float          # VB -> GND
    # direct supply rail tap, no bias (VSYS is never negative)
    vbus_div_top: float
    vbus_div_bot: float
    # converter
    vdd_mv: int = 3300
    adc_counts: int = 4096
    # timing
    tick_hz: int = 20_000
    pwm_half_ticks: int = 1200
    # shortest drive window with a settled sample, in timer ticks
    floor_i_ticks: int = 160
    floor_v_ticks: int = 160
    floors_verified: bool = False
    measured: dict = field(default_factory=dict)
    notes: str = ""

    # --- derived: volts ---
    @property
    def v_adc_per_count(self) -> float:
        return (self.vdd_mv / 1e3) / self.adc_counts

    @property
    def div_ratio(self) -> float:
        """r in the tap formula Vterm = r*tap - (r-1)*VB."""
        return (self.vmotor_div_top + self.vmotor_div_bot) / self.vmotor_div_bot

    @property
    def v_term_per_count(self) -> float:
        """Terminal volts per count of tap DIFFERENCE (vA - vB is bias free)."""
        return self.v_adc_per_count * self.div_ratio

    @property
    def vbus_ratio(self) -> float:
        return (self.vbus_div_top + self.vbus_div_bot) / self.vbus_div_bot

    @property
    def vb_nom_counts(self) -> float:
        return self.adc_counts * self.bias_bot / (self.bias_top + self.bias_bot)

    @property
    def vb_nom_v(self) -> float:
        return (self.vdd_mv / 1e3) * self.bias_bot / (self.bias_top + self.bias_bot)

    # --- derived: amps ---
    @property
    def shunt_ohm(self) -> float:
        return self.shunt_mohm / 1e3

    @property
    def amp_gain(self) -> float:
        return self.gain_milli / 1e3

    @property
    def a_per_count(self) -> float:
        return self.v_adc_per_count / (self.amp_gain * self.shunt_ohm)

    @property
    def counts_per_a(self) -> float:
        return 1.0 / self.a_per_count

    # --- derived: drive window floors as a duty percentage ---
    @property
    def floor_i_pct(self) -> float:
        return self.floor_i_ticks * 100.0 / self.pwm_half_ticks

    @property
    def floor_v_pct(self) -> float:
        return self.floor_v_ticks * 100.0 / self.pwm_half_ticks

    # --- conversions ---
    def term_v(self, tap_counts, vb_counts):
        """Absolute terminal volts from one tap. A driven-LOW terminal does not
        read zero: it reads (r-1)/r * VB, about 520 counts on a biased board."""
        r = self.div_ratio
        return (r * tap_counts - (r - 1) * vb_counts) * self.v_adc_per_count

    def diff_v(self, tap_a_counts, tap_b_counts):
        """Terminal-to-terminal volts. Bias free by construction (single VB
        node), so no VB term is needed."""
        return (tap_a_counts - tap_b_counts) * self.v_term_per_count

    def amps(self, current_raw, rest_bias):
        return (current_raw - rest_bias) * self.a_per_count

    def vsys_v(self, vbus_raw):
        """Volts from the DIRECT rail tap `vbus_raw` (TEL bit 9, register 0x254).
        Unbiased, valid at rest, every tick.

        DO NOT pass `vbus_counts` (register 0x232, and the `vbus_counts` field in
        meta.json) to this. That is a different quantity in different units and
        you will get an answer ~18% low that still looks plausible. Use
        `rail_from_vbus_counts` for it. This has caught three people."""
        return vbus_raw * self.v_adc_per_count * self.vbus_ratio

    def rail_from_vbus_counts(self, vbus_counts):
        """Volts from `vbus_counts` (register 0x232, meta.json `vbus_counts`).

        The firmware's estimator already did the divider conversion: it takes an
        EWMA of vbus_raw and rescales it by scale_q15 into VMOTOR-TAP units, so
        undervolt, bemf and the current loop all speak one unit. So this value is
        already in tap counts and needs the TERMINAL ratio, not the rail one.
        Sending it through the rail divider a second time is the mistake above."""
        return vbus_counts * self.v_term_per_count

    # --- guard ---
    @property
    def sense(self) -> dict:
        """The meta.json `sense` block this board must produce."""
        return {
            "shunt_r_mohm": self.shunt_mohm,
            "vmotor_div_top": self.vmotor_div_top,
            "vmotor_div_bot": self.vmotor_div_bot,
            "gain_milli": self.gain_milli,
            "vdd_mv": self.vdd_mv,
        }

    def check_meta(self, meta, where=""):
        """Fail loudly when a recording was captured on different parts than
        this entry declares. Silence here is how a 1.8x current error hides."""
        got = {k: meta["sense"][k] for k in self.sense}
        if got != self.sense:
            raise ValueError(
                f"{where}: meta.json sense {got} != board {self.key} {self.sense}"
            )


BOARDS = {
    "dev-v006-D": Board(
        key="dev-v006-D",
        label="osc-dev-v006 board D (bodged toward rev-2A)",
        shunt_mohm=60,
        gain_milli=15_000,
        vmotor_div_top=6_800,
        vmotor_div_bot=3_300,
        bias_top=200,
        bias_bot=47,
        vbus_div_top=15_000,
        vbus_div_bot=10_000,
        floor_i_ticks=160,
        floor_v_ticks=160,
        floors_verified=False,
        measured={
            "vb_bias_measured": Measured(
                781, 775, 783, "counts", "Hi-Z rest baseline, 2S grid + stepcoast",
                "nominal 779 from the 200/47 network, so 0.3%. Read it from the "
                "Hi-Z rest tap directly; the brake-low route (VB = tap*r/(r-1)) "
                "needs settled brake samples and reads ~6% low on transients"),
            "rail_droop_usb_measured": Measured(
                -3.5, -4.0, -3.0, "% over 20-100% duty", "2S vs USB grid, 5 captures each",
                "USB also dips -21.8% transiently at 0.76 A; 2S is flat (+0.1%) "
                "and dips only -5.9% at 1.70 A. ~1.3 ohm vs ~0.27 ohm source Z"),
            "opa_noise_floor_measured": Measured(
                2.2, 2.0, 2.4, "counts RMS", "bringup isns-diff run29",
                "arm-B external network at G15; the internal PGA arms ran 10-15"),
            "esd_clamp_skew_measured": Measured(
                1.8, 1.7, 1.9, "V bemf", "bringup isns-diff findings",
                "above this the A-side ESD clamp skews the divider split - use "
                "the positive terminal, and distrust coast BEMF above it"),
            "vdd_measured": Measured(
                3.28, 3.27, 3.29, "V", "nb10 sec 2, DMM at the 3V3 test point, grid session",
                "vdd_mv stays 3300 so the sense block every capture carries is "
                "stable; apply this only where absolute volts or amps are quoted, "
                "ratios cancel it (0.6%)"),
            "terminal_chain_measured": Measured(
                0.0, -0.7, 0.7, "% vs DMM", "nb10 sec 4.1, 4x4 grid at 1.7 A",
                "both directions inside the meter's band or within 0.7% of its "
                "midpoint; VB bias, 6k8/3k3 ratio and rail-from-tap are right"),
            "current_chain_scale_measured": Measured(
                1.2, -1.5, 3.9, "% high vs grid route", "nb10 sec 4.2, 2S DC laps",
                "no gross scale error; the bracket is the grid's 0.1 ohm last "
                "digit plus meter class, no series ammeter (10 A jack dead). "
                "nb10 reads +1.13% with the measured 0.031 V lead drop; 1.2 "
                "took the session log's 0.04 V"),
            "crest_floor_measured": Measured(
                15, 15, 17, "% duty on 2S", "nb10 sec 5 and 6, static-load ladders + bursts",
                "honest crest current from 15% on 2S (amplifier edge ~3-4 us); "
                "USB needs about 30% (island-cap charge sharing on the soft rail)"),
        },
        notes=(
            "Temporary hacked rig, not the settled design. 60 mOhm shunt, "
            "6k8/3k3 terminal taps with 150 pF, returned to a VB bias from "
            "200/47. VSNS stays 15k/10k unbiased. The VB bias is what makes a "
            "coast back-EMF reading valid: with the bottoms on GND the "
            "floating winding splits its EMF and the negative terminal cannot "
            "be read. FLOORS UNVERIFIED: 120, 160 and 210 ticks all appear in "
            "different places for v_window_min_ticks. Confirm against the "
            "flashed build before trusting floor_v_pct."
        ),
    ),
    "dev-v006-2A": Board(
        key="dev-v006-2A",
        label="osc-dev-v006 rev 2A, board #1",
        shunt_mohm=60,
        gain_milli=14_884,
        vmotor_div_top=6_400,
        vmotor_div_bot=1_600,
        bias_top=430,
        bias_bot=100,
        vbus_div_top=6_400,
        vbus_div_bot=1_600,
        floor_i_ticks=64,
        floor_v_ticks=93,
        floors_verified=True,
        measured={
            "rest_current_noise_measured": Measured(
                1.28, 1.22, 1.33, "counts RMS", "nb10 sec 3, grid 2S ladder rest baselines",
                "the whole chain at rest with the bridge off, one value per capture; "
                "board D's grid baseline reads 1.7 the same way. Not the "
                "amplifier-alone floor board D's opa_noise_floor_measured quotes"),
            "crest_floor_measured": Measured(
                7, 7, 10, "% duty on 2S", "nb10 sec 5 and 8, static-load ladder + bursts",
                "honest to 3% from 7%, and the 7% edge is tap A's sample, not the "
                "current: the current alone reads 97% of the 100% rung at 5% and "
                "is inside 1% from about 10%, in high-speed mode. Normal mode "
                "reads about 95.5% at 5%, the same from 10%"),
            "shunt_settle_measured": Measured(
                156, 150, 156, "ticks from the ON compare to 1% of the 100% rung",
                "nb10 sec 8, 2S grid bursts folded at 4 ticks, registered on the ladder",
                "3.25 us, in high-speed mode. The current closes the last 3% on a "
                "slow tail, always from below, so an early sample reads low. The "
                "ladder's own 1% crossings (119 and 123 ticks of window, plus the "
                "31 to the sample) give the low end. Normal mode, from paired "
                "bringup captures/winding-r/c0 bursts with the shunt over the "
                "terminal difference: half way 48 ticks after the compare against "
                "44, a 13.1-tick exponential against 10.1, tail 3.3% against 2.6% "
                "at half way +48 and 0.7% in both at +96, so the 1% point holds"),
            "current_chain_scale_measured": Measured(
                0.5, -0.7, 2.3, "% high vs Rs1 pads", "Rs1 pads mV during a 4.2 s 100% grid lap",
                "meter 93-94 mV across Rs1 against 93.3-95.1 mV (median 94.0) "
                "from the chip's current times the nominal 60 mOhm, so the "
                "amplifier and ADC are inside ~1%; the shunt's own tolerance is "
                "not graded. The load route's +5.0% in nb10 sec 5.1 is the "
                "meter's ohms range at J4 reading the load low"),
        },
        notes=(
            "Rs1 60 mOhm; bare OPA with Rf 6k4 / Rg 430 (G 14.884), Cc 22 pF, "
            "Co1 DNP. Terminal taps Rv1/Rv3 6k4 over Rv2/Rv4 1k6 with "
            "Cv1/Cv2 100 pF, bottoms and caps returned to VB from Rb1 430 / "
            "Rb2 100 (773 counts nominal, about 81 ohm source, so VB rises "
            "with the leg current under drive). VSNS 6k4/1k6 to GND, unbiased, "
            "Cv3 100 pF. A 2A capture's sense block differs from board D's in "
            "gain_milli and both divider legs. Nothing in the block carries "
            "the bias or VSNS parts, or which 2A unit took the capture, so "
            "every 2A board shares this entry. Board #1 had Rv3 open for the "
            "grid-3r7 2S session: in that dataset tap B reads the VB node "
            "instead of terminal B, so only tap A, the rail and the current "
            "are graded there. Rv3 is repaired, and a dataset captured with it "
            "fitted reads both taps. Floors 64/93 ticks: the current at 3% "
            "with the settle gain on top (nb10 sec 8), the terminals from the "
            "grid ladder with tap B sampled 37 ticks before the crest (nb16 "
            "sec 6; reverse needs the tap swap, nb16 sec 7). Fast decay's "
            "terminals and the back-EMF observer keep 160."
        ),
    ),
}

DEFAULT_BOARD = "dev-v006-D"


def from_meta(meta):
    """Identify the board that recorded a capture, from its own `sense` block.
    Captures are self describing, so nothing needs a hand kept folder to
    hardware map. Returns None when no entry matches - add the board rather
    than bending an existing one to fit."""
    for b in BOARDS.values():
        if {k: meta["sense"][k] for k in b.sense} == b.sense:
            return b
    return None
