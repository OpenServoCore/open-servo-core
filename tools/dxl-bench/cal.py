"""Slave-side HSI calibration for the V006, per docs/dxl-hsi-calibration.md.

The wire scheme is inverted from the original master-stamps variant: the
master streams `count` zero filler bytes, the slave times its own RX between
T_first (first RXNE) and T_last (IDLE-backdated end of last byte), and the
reply carries the (observed_ticks, nominal_ticks) pair so the host can derive
drift without any master-side timestamping. The pirate is just a passthrough
UART for this step (no Round-stamp dependence).

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


@dataclass(frozen=True)
class CalMeasurement:
    """Per-shot CAL outcome — slave-reported observed vs nominal tick counts
    for the master's just-sent CALIB payload. `applied_*` are what the slave
    already queued; both zero until the chip-side trim algorithm lands."""
    count: int
    baud: int
    observed_ticks: int
    nominal_ticks: int
    applied_trim_delta: int
    applied_fine_trim_us: int

    @property
    def drift_ppm(self) -> float:
        # Identical denominators in chip ticks: ratio is unit-less.
        # Positive drift ⇒ HSI fast (more chip ticks per wire byte than nominal).
        return (self.observed_ticks - self.nominal_ticks) / self.nominal_ticks * 1_000_000


@dataclass(frozen=True)
class CalDerived:
    """Master-side math from §7 applied to one CalMeasurement."""
    drift_ppm: float
    ppm_per_step: int
    step: int                # signed; clock_trim += step
    residual_ppm: float      # symmetric around 0 in [-ppm_per_step/2, +ppm_per_step/2]
    residual_us: float       # signed; intercept advances on +, retards on −
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
        # [[ch32v006-hsitrim-direction]]). drift_ppm > 0 means observed >
        # nominal, i.e. the chip's SysTick counted more ticks per fixed
        # wire-time → chip is fast → trim must drop. Hence step is the
        # *negative* of drift_ppm / ppm_per_step. Symmetric round-to-nearest
        # so the fine-trim residual stays bounded; the chip's signed Q8.8
        # FIRE_ADVANCE_FINE_TICKS absorbs the leftover with either sign.
        drift_ppm = m.drift_ppm
        step = -round(drift_ppm / ppm_per_step)
        residual_ppm = drift_ppm + step * ppm_per_step
        new_trim = max(trim_min, min(trim_max, current_trim + step))
        # §7.3: drift residual contribution to fire floor, in µs at b_op.
        # Sign: residual_ppm > 0 means chip HSI is fast → its `rdt_us * 48`
        # schedule undershoots wall-clock → already fires early → need
        # *negative* fine to push fire later. Chip semantics
        # ([[ch32-fire-advance-fine-ticks]]): + advances, − retards. So
        # the residual_us we ship has the *opposite* sign of residual_ppm.
        byte_time_us = 10 / baud_op * 1_000_000
        residual_us = -residual_ppm / 1_000_000 * n_target * byte_time_us
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
        """One CAL trip: master streams `count` filler bytes, slave times its
        own RX, reply carries (observed, nominal) chip-tick counts plus the
        already-applied trim deltas. Pirate is just the UART here."""
        assert 1 <= count <= 128, f"count {count} out of [1, 128]"
        # Reply payload is 11 bytes (observed u32 + nominal u32 + trim i8 +
        # fine i16) regardless of count; the request payload grows with count,
        # so cap reply timeout independent of it.
        reply = self.pirate.xfer(build_calibrate(self.id, count), reply_us=500_000)
        assert reply, "no Status frame on CAL"
        st = parse_status(reply)
        assert st.error == 0, f"CAL err 0x{st.error:02X}"
        assert len(st.params) == 11, (
            f"expected 11-byte measurement payload, got {len(st.params)}: {st.params.hex()}"
        )
        observed, nominal, applied_trim, applied_fine = struct.unpack(
            "<IIbh", st.params,
        )
        return CalMeasurement(
            count=count,
            baud=self.baud,
            observed_ticks=observed,
            nominal_ticks=nominal,
            applied_trim_delta=applied_trim,
            applied_fine_trim_us=applied_fine,
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
