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
        """Direct rail tap. Unbiased, valid at rest, every tick."""
        return vbus_raw * self.v_adc_per_count * self.vbus_ratio

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
