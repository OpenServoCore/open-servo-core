"""Host-side HSI drift probe.

The chip owns its clock trim now — it watches inter-byte timing on every
non-Status packet and nudges HSITRIM toward zero drift autonomously (see
`firmware/lib/drivers/src/dxl/uart/clock.rs`). There is no CAL round-trip and
no readable trim register: the only signal the host can observe is how long the
chip takes to transmit a reply, measured against the pirate's stable clock.

`probe_drift_ppm` issues a long Read and reads the pirate's `Round` stamp for
that reply: `first`/`last` are the per-byte RXNE timestamps of the first and
last reply bytes. Their span covers `(N-1)` chip-transmitted byte-times; when
the chip's HSI is fast the span is short, slow the span is long. A long reply
(128 payload bytes → 138 byte-times of span) keeps the per-byte resync
quantization small relative to the span, and averaging a handful of probes
drives it down further.
"""

from __future__ import annotations

from dxl_packet import build_read, parse_status
from pirate import Pirate, PirateError, Round

# Mirror firmware/ch32/src/hal/rcc/v00x.rs::CLOCK_TRIM_PPM_PER_STEP
# (HSI_TRIM_STEP_HZ 60 kHz / HSI_HZ 24 MHz). One HSITRIM step ≈ 2500 ppm.
DRIFT_STEP_PPM = 2500

# Read the chip's config region (512 B from addr 0) — always present, RO, and
# long enough that the reply span dominates per-byte stamp quantization.
PROBE_ADDR = 0
PROBE_LEN = 128

# Full Status frame around an N-byte payload: 4 header + id + 2 len + instr +
# err + N params + 2 CRC.
_STATUS_OVERHEAD = 11


def _round_from(stamps: list) -> Round | None:
    rounds = [s for s in stamps if isinstance(s, Round)]
    return rounds[0] if len(rounds) == 1 else None


def _one_span(pirate: Pirate, dxl_id: int, addr: int, length: int) -> int:
    """One Read round-trip → the reply's `last - first` span in pirate ticks
    (modular u32). Raises on a missing/short reply or an ambiguous stamp."""
    pirate.drain_stamps()
    reply = pirate.xfer(build_read(dxl_id, addr, length), reply_us=200_000)
    if not reply:
        raise PirateError(f"no Status reply on drift Read @ {addr}")
    st = parse_status(reply)
    if st.error != 0:
        raise PirateError(f"drift Read err 0x{st.error:02X}")
    if len(reply) != _STATUS_OVERHEAD + length:
        raise PirateError(
            f"drift Read reply len {len(reply)} != {_STATUS_OVERHEAD + length}"
        )
    rnd = _round_from(pirate.drain_stamps())
    if rnd is None:
        raise PirateError("drift Read produced no single Round stamp")
    return (rnd.last - rnd.first) & 0xFFFFFFFF


def probe_drift_ppm(
    pirate: Pirate,
    dxl_id: int,
    baud: int,
    *,
    samples: int = 8,
    addr: int = PROBE_ADDR,
    length: int = PROBE_LEN,
    retries: int = 3,
) -> float:
    """Mean chip-TX drift in ppm over `samples` long-Read probes. Positive ⇒
    the chip transmits slow (HSI slow); negative ⇒ fast. Each ambiguous probe
    (no clean Round, short reply) is retried up to `retries` times."""
    n = _STATUS_OVERHEAD + length
    ticks_per_us = pirate.hz_per_us()
    # (N-1) byte-times of span, each 10 bits wide, in pirate ticks.
    nominal_ticks = (n - 1) * 10 / baud * ticks_per_us * 1_000_000

    spans: list[int] = []
    for _ in range(samples):
        for attempt in range(retries):
            try:
                spans.append(_one_span(pirate, dxl_id, addr, length))
                break
            except PirateError:
                if attempt == retries - 1:
                    raise
    mean_span = sum(spans) / len(spans)
    return (mean_span / nominal_ticks - 1) * 1_000_000
