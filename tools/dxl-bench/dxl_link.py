"""Helpers for reading + clearing the TelemetryDxlLink fault counters via DXL.

Addresses are generated from `regions::telemetry::addr::link::*`.
"""

import struct

from dxl_packet import build_read, build_write, parse_status

LINK_BASE = 0x023C
LINK_FIELDS = [
    "illegal_transition",
    "unexpected_byte_count",
    "previous_slot_timeout",
    "slot_timing_miss",
    "crc_patch_deadline_miss",
    "dma_overrun",
    "parity_error",
    "framing_error",
    "noise_error",
]
LINK_LEN = 4 * len(LINK_FIELDS)
LINK_FMT = f"<{len(LINK_FIELDS)}I"
ZERO_BLOB = b"\x00" * LINK_LEN


def read_counters(pirate, osc_id):
    frame = pirate.xfer(build_read(osc_id, LINK_BASE, LINK_LEN), reply_us=200_000)
    assert frame, "no Status frame on counter read"
    st = parse_status(frame)
    assert st.error == 0, f"counter read error 0x{st.error:02X}"
    return dict(zip(LINK_FIELDS, struct.unpack(LINK_FMT, st.params)))


def clear_counters(pirate, osc_id):
    frame = pirate.xfer(build_write(osc_id, LINK_BASE, ZERO_BLOB), reply_us=200_000)
    assert frame, "no Status frame on counter clear"
    st = parse_status(frame)
    assert st.error == 0, f"counter clear error 0x{st.error:02X}"


def print_counters(label, counters):
    nonzero = {k: v for k, v in counters.items() if v != 0}
    if not nonzero:
        print(f"  {label}: all counters zero", flush=True)
        return
    print(f"  {label}: nonzero counters", flush=True)
    for name, val in nonzero.items():
        print(f"    {name:25s} = {val}", flush=True)
