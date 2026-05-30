"""Reboot + tinyboot round-trip — gates the firmware-upgrade path."""

import time

from conftest import BAUD_INDEX, BAUD_RATE_IDX_ADDR, DEFAULT_BAUD
from dxl_packet import (
    build_ping,
    build_read,
    build_reboot,
    build_write,
    parse_status,
)
from tinyboot_frame import (
    build_info_request,
    build_reset_request,
    parse_info_response,
)

BOOT_MODE_ADDR = 0x031C
BOOT_MODE_BOOTLOADER = 1
CONTROL_BASE_ADDR = 0x0300
TINYBOOT_MODE_BOOTLOADER = 0


def _expect_status(pirate, packet, osc_id, expected_err=0):
    reply = pirate.xfer(packet, reply_us=200_000)
    assert reply, "no DXL Status frame"
    s = parse_status(reply)
    assert s.id == osc_id
    assert s.error == expected_err, f"err=0x{s.error:02X}, want 0x{expected_err:02X}"
    return s


def test_reboot_clears_volatile_state(pirate, osc_id, baud):
    _expect_status(pirate, build_write(osc_id, CONTROL_BASE_ADDR, b"\x01"), osc_id)
    s = _expect_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1), osc_id)
    assert s.params == b"\x01", f"setup failed, torque_enable={s.params.hex()}"

    _expect_status(pirate, build_reboot(osc_id), osc_id)
    time.sleep(0.5)

    try:
        if baud != DEFAULT_BAUD:
            pirate.set_baud(DEFAULT_BAUD)

        _expect_status(pirate, build_ping(osc_id), osc_id)

        s = _expect_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1), osc_id)
        assert s.params == b"\x00", f"reboot didn't clear volatile state: {s.params.hex()}"

        if baud != DEFAULT_BAUD:
            _expect_status(
                pirate,
                build_write(osc_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[baud]])),
                osc_id,
            )
            time.sleep(0.05)
    finally:
        pirate.set_baud(baud)


def test_tinyboot_round_trip(pirate, osc_id, baud, tinyboot_baud):
    try:
        _expect_status(
            pirate, build_write(osc_id, BOOT_MODE_ADDR, bytes([BOOT_MODE_BOOTLOADER])), osc_id,
        )
        _expect_status(pirate, build_reboot(osc_id), osc_id)

        time.sleep(0.3)
        pirate.set_baud(tinyboot_baud)

        reply = pirate.xfer(build_info_request(), reply_us=500_000)
        assert reply, "no Info reply from bootloader"
        info = parse_info_response(reply)
        assert info.mode == TINYBOOT_MODE_BOOTLOADER, (
            f"expected mode=bootloader(0), got {info.mode} — chip is not in tinyboot"
        )
        assert info.capacity > 0 and info.erase_size > 0, f"bogus InfoData: {info}"

        pirate.xfer(build_reset_request(bootloader=False), reply_us=50_000)
        time.sleep(0.5)

        pirate.set_baud(DEFAULT_BAUD)
        _expect_status(pirate, build_ping(osc_id), osc_id)

        if baud != DEFAULT_BAUD:
            _expect_status(
                pirate,
                build_write(osc_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[baud]])),
                osc_id,
            )
            time.sleep(0.05)
    finally:
        pirate.set_baud(baud)
