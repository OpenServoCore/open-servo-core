"""Shared bench helpers for pirate-driven chip scripts.

Pirate USB autodetect, comms-CT accessors, baud-switching, and the slave-side
HSI cal step. Lives separately from any one script so the matrix / stress
loops can compose against the same primitives.
"""

from __future__ import annotations

import struct
import sys
import time

import serial.tools.list_ports

from drift import probe_drift_ppm  # noqa: F401 — re-exported for scripts
from dxl_packet import (
    BROADCAST_ID,
    build_ping,
    build_read,
    build_write,
    parse_status,
)
from pirate import Pirate, PirateError

PIRATE_VID = 0xC0DE
PIRATE_PID = 0xCAFE

# comms CT addresses (mirror config::addr::comms in
# firmware/lib/core/src/regions/config.rs).
BAUD_RATE_IDX_ADDR = 13
RETURN_DELAY_2US_ADDR = 14

BAUD_INDEX = {
    9600: 0, 57600: 1, 115200: 2,
    1_000_000: 3, 2_000_000: 4, 3_000_000: 5,
}

# Must not collide with --id or any other live chip on the bus: this ID is
# named as slot 2 of the First-position cells so the bus IDLEs after chip
# slot 1, giving a clean single Round stamp.
FOREIGN_ID = 99
# INJ slot ID for Last cells (pirate fires the INJ slot via ARM).
INJ_ID = 50


def autodetect_pirate() -> str:
    matches = [
        p.device for p in serial.tools.list_ports.comports()
        if p.vid == PIRATE_VID and p.pid == PIRATE_PID
    ]
    if len(matches) != 1:
        sys.exit(f"expected exactly 1 pirate, found {matches}")
    return matches[0]


# ── CT helpers ──────────────────────────────────────────────────────────────

def _xfer_ct(pirate: Pirate, packet: bytes) -> bytes:
    reply = pirate.xfer(packet, reply_us=200_000)
    if not reply:
        raise PirateError("no Status reply")
    st = parse_status(reply)
    if st.error != 0:
        raise PirateError(f"err 0x{st.error:02X}")
    return st.params


def read_ct_u8(pirate: Pirate, dxl_id: int, addr: int) -> int:
    return _xfer_ct(pirate, build_read(dxl_id, addr, 1))[0]


def read_ct_u16(pirate: Pirate, dxl_id: int, addr: int) -> int:
    return struct.unpack("<H", _xfer_ct(pirate, build_read(dxl_id, addr, 2)))[0]


def write_ct_u16(pirate: Pirate, dxl_id: int, addr: int, value: int) -> None:
    _xfer_ct(pirate, build_write(dxl_id, addr, struct.pack("<H", value & 0xFFFF)))


def _ping_with_retry(pirate: Pirate, dxl_id: int,
                     retries: int = 3, delay_s: float = 0.02) -> bool:
    for _ in range(retries):
        if pirate.xfer(build_ping(dxl_id), reply_us=200_000):
            return True
        time.sleep(delay_s)
    return False


def set_chip_baud(pirate: Pirate, dxl_id: int, target_bps: int) -> None:
    """Ensure both ends are at `target_bps`; mirrors conftest.ensure_chip_baud
    minus the pytest dependency."""
    if target_bps not in BAUD_INDEX:
        raise ValueError(f"unsupported baud {target_bps}")
    pirate.set_baud(target_bps)
    time.sleep(0.05)
    pirate.drain_stamps()
    if _ping_with_retry(pirate, dxl_id):
        return
    current = None
    for bps in sorted(BAUD_INDEX, reverse=True):
        if bps == target_bps:
            continue
        pirate.set_baud(bps)
        time.sleep(0.05)
        pirate.drain_stamps()
        if _ping_with_retry(pirate, dxl_id):
            current = bps
            break
    if current is None:
        raise PirateError(f"chip not found at any known baud (target {target_bps})")
    _xfer_ct(pirate, build_write(
        dxl_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[target_bps]])
    ))
    time.sleep(0.05)
    pirate.set_baud(target_bps)
    time.sleep(0.05)
    # USB-CDC baud reconfig can glitch the pirate's TX line; the chip's
    # USART frames the glitch as a phantom byte and bumps FE/NE. A throwaway
    # ping forces a real frame so the next caller's health snapshot starts
    # clean.
    if not _ping_with_retry(pirate, dxl_id):
        raise PirateError(f"chip not responding after baud switch to {target_bps}")


# ── discovery ────────────────────────────────────────────────────────────────

def discover_chip_id(pirate: Pirate, baud: int) -> int:
    """Sweep known bauds for a broadcast-Ping reply; return the chip's ID.
    The ID is UID-derived now (firmware/ch32/src/runtime/init.rs), so callers
    can't assume a fixed default."""
    candidates = [baud] + [b for b in sorted(BAUD_INDEX, reverse=True) if b != baud]
    for b in candidates:
        pirate.set_baud(b)
        time.sleep(0.05)
        pirate.drain_stamps()
        reply = pirate.xfer(build_ping(BROADCAST_ID), reply_us=200_000)
        if reply:
            return parse_status(reply).id
    raise PirateError("no chip responded to broadcast Ping at any known baud")


# ── HSI warm-up ──────────────────────────────────────────────────────────────

def warm_up_trim(pirate: Pirate, dxl_id: int, baud: int, pings: int = 40) -> float:
    """Drive plain Pings so the chip's autonomous HSI trim converges, then
    return the probed residual drift in ppm. Replaces the old master-side CAL:
    the chip owns its trim now (firmware/lib/drivers/src/dxl/uart/clock.rs)."""
    for _ in range(pings):
        pirate.xfer(build_ping(dxl_id), reply_us=200_000)
    return probe_drift_ppm(pirate, dxl_id, baud)
