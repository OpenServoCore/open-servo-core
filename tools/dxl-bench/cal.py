"""Master-side HSI calibration for the V006, per docs/dxl-hsi-calibration.md.

The math is §7: from a single CAL round-trip the host derives a discrete
HSITRIM step (biased toward "slow") and the sub-step µs residual (Q8.8) that
the slave should apply on top.

Tests / drivers compose a `Calibrator` over the existing pirate fixture; the
control-table addresses live in `comms` per `firmware/lib/core/src/regions/config.rs`.
"""

from __future__ import annotations

import struct
import time
from dataclasses import dataclass

from dxl_packet import build_calibrate, build_read, build_write, parse_status

# comms control-table offsets (CONFIG_BASE_ADDR=0 + identity (12 B) + comms fields).
CLOCK_TRIM_ADDR = 16             # i8
CLOCK_STEP_PPM_ADDR = 18         # u16 RO
CLOCK_FINE_TRIM_US_ADDR = 20     # i16 Q8.8 RW


# Status frame fixed overhead bytes around the count-zero payload:
# header(4) + id(1) + len(2) + instr(1) + err(1) + crc(2) = 11.
STATUS_OVERHEAD_BYTES = 11


@dataclass(frozen=True)
class CalMeasurement:
    """Per-shot CAL outcome — what the master observed on the wire."""
    count: int
    baud: int
    ticks_per_us: int
    req_tick: int
    first_tick: int
    last_tick: int

    @property
    def frame_bytes(self) -> int:
        # T_first is end-of-first-reply-byte, T_last is end-of-last-reply-byte;
        # the measurement spans the entire Status frame, not just the zero payload.
        return self.count + STATUS_OVERHEAD_BYTES

    @property
    def observed_us(self) -> float:
        delta = (self.last_tick - self.first_tick) & 0xFFFFFFFF
        return delta / self.ticks_per_us

    @property
    def nominal_us(self) -> float:
        # (M − 1) byte intervals between end-of-byte-1 and end-of-byte-M at 8N1.
        return (self.frame_bytes - 1) * 10 / self.baud * 1_000_000

    @property
    def drift_ppm(self) -> float:
        n = self.nominal_us
        return (self.observed_us - n) / n * 1_000_000


@dataclass(frozen=True)
class CalDerived:
    """Master-side math from §7 applied to one CalMeasurement."""
    drift_ppm: float
    ppm_per_step: int
    step: int                # signed; clock_trim += step
    residual_ppm: float      # always ≤ 0 after biased rounding
    residual_us: float       # always ≥ 0; what intercept compensation owes
    new_trim: int            # clamped clock_trim after applying `step`
    residual_q88: int        # i16 Q8.8 for clock_fine_trim_us

    @staticmethod
    def from_measurement(
        m: CalMeasurement,
        ppm_per_step: int,
        current_trim: int,
        n_target: int,
        baud_op: int,
        trim_min: int = -16,
        trim_max: int = 15,
    ) -> "CalDerived":
        # Sign convention: higher HSITRIM = faster HSI (per chip RM and
        # [[ch32v006-hsitrim-direction]]). drift_ppm > 0 means slow → step > 0
        # raises HSITRIM. §7.2 biased rounding: floor(drift/ppm_per_step)
        # always leaves residual ≥ 0 (chip stays on the slow side after trim)
        # so the fire-advance intercept (only advances, can't retard) absorbs it.
        import math
        drift_ppm = m.drift_ppm
        step = math.floor(drift_ppm / ppm_per_step)
        residual_ppm = drift_ppm - step * ppm_per_step
        new_trim = max(trim_min, min(trim_max, current_trim + step))
        # §7.3: drift residual contribution to fire floor, in µs at b_op.
        # residual_ppm ≥ 0 → residual_us ≥ 0 → intercept advances fire.
        byte_time_us = 10 / baud_op * 1_000_000
        residual_us = residual_ppm / 1_000_000 * n_target * byte_time_us
        residual_q88 = max(-32768, min(32767, round(residual_us * 256)))
        return CalDerived(
            drift_ppm=drift_ppm,
            ppm_per_step=ppm_per_step,
            step=step,
            residual_ppm=residual_ppm,
            residual_us=residual_us,
            new_trim=new_trim,
            residual_q88=residual_q88,
        )


class Calibrator:
    def __init__(self, pirate, dxl_id: int, baud: int,
                 n_target: int = 128, baud_op: int = 3_000_000):
        self.pirate = pirate
        self.id = dxl_id
        self.baud = baud
        self.n_target = n_target
        self.baud_op = baud_op
        self.ticks_per_us = pirate.hz_per_us()
        self._ppm_per_step = None

    @property
    def ppm_per_step(self) -> int:
        if self._ppm_per_step is None:
            self._ppm_per_step = self.read_clock_step_ppm()
        return self._ppm_per_step

    # ── control-table accessors ────────────────────────────────────────────

    def read_clock_trim(self) -> int:
        reply = self.pirate.xfer(build_read(self.id, CLOCK_TRIM_ADDR, 1), reply_us=200_000)
        assert reply, "no Status frame on clock_trim read"
        st = parse_status(reply)
        assert st.error == 0, f"clock_trim read err 0x{st.error:02X}"
        return struct.unpack("<b", st.params)[0]

    def read_clock_step_ppm(self) -> int:
        reply = self.pirate.xfer(build_read(self.id, CLOCK_STEP_PPM_ADDR, 2), reply_us=200_000)
        assert reply, "no Status frame on clock_step_ppm read"
        st = parse_status(reply)
        assert st.error == 0, f"clock_step_ppm read err 0x{st.error:02X}"
        return struct.unpack("<H", st.params)[0]

    def read_clock_fine_trim_us(self) -> int:
        reply = self.pirate.xfer(build_read(self.id, CLOCK_FINE_TRIM_US_ADDR, 2), reply_us=200_000)
        assert reply, "no Status frame on clock_fine_trim_us read"
        st = parse_status(reply)
        assert st.error == 0, f"clock_fine_trim_us read err 0x{st.error:02X}"
        return struct.unpack("<h", st.params)[0]

    def write_clock_trim(self, value: int) -> None:
        reply = self.pirate.xfer(
            build_write(self.id, CLOCK_TRIM_ADDR, struct.pack("<b", value)),
            reply_us=200_000,
        )
        assert reply, "no Status frame on clock_trim write"
        st = parse_status(reply)
        assert st.error == 0, f"clock_trim write err 0x{st.error:02X}"

    def write_clock_fine_trim_us(self, q88: int) -> None:
        reply = self.pirate.xfer(
            build_write(self.id, CLOCK_FINE_TRIM_US_ADDR, struct.pack("<h", q88)),
            reply_us=200_000,
        )
        assert reply, "no Status frame on clock_fine_trim_us write"
        st = parse_status(reply)
        assert st.error == 0, f"clock_fine_trim_us write err 0x{st.error:02X}"

    # ── CAL round-trip ─────────────────────────────────────────────────────

    def measure(self, count: int = 128) -> CalMeasurement:
        """One CAL trip: returns the (req, first, last) HSE timestamps. Also
        validates the reply payload is `count` zero bytes."""
        assert 1 <= count <= 128, f"count {count} out of [1, 128]"
        self.pirate.drain_stamps()
        reply = self.pirate.xfer(build_calibrate(self.id, count), reply_us=500_000)
        assert reply, "no Status frame on CAL"
        st = parse_status(reply)
        assert st.error == 0, f"CAL err 0x{st.error:02X}"
        assert len(st.params) == count, f"expected {count} reply bytes, got {len(st.params)}"
        assert all(b == 0 for b in st.params), f"non-zero reply byte: {st.params.hex()}"
        stamps = self.pirate.drain_stamps()
        rounds = [s for s in stamps if hasattr(s, "req")]
        assert len(rounds) == 1, f"expected 1 Round stamp, got {len(stamps)}: {stamps}"
        r = rounds[0]
        return CalMeasurement(
            count=count,
            baud=self.baud,
            ticks_per_us=self.ticks_per_us,
            req_tick=r.req,
            first_tick=r.first,
            last_tick=r.last,
        )

    def derive(self, m: CalMeasurement, current_trim: int | None = None) -> CalDerived:
        if current_trim is None:
            current_trim = self.read_clock_trim()
        return CalDerived.from_measurement(
            m, self.ppm_per_step, current_trim, self.n_target, self.baud_op,
        )

    def apply(self, d: CalDerived) -> None:
        """Write back both cal values. Chip applies after USART1 TC drains the
        ACKs; gives the apply ~50 ms to settle so subsequent xfers see the
        post-apply HSI."""
        if d.step != 0:
            self.write_clock_trim(d.new_trim)
            time.sleep(0.05)
        # Always write the fine trim — even step==0 cycles can refine residual.
        self.write_clock_fine_trim_us(d.residual_q88)
        time.sleep(0.05)
