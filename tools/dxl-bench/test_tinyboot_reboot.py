"""Bootloader round-trip — gates the firmware-upgrade path."""

import time

from conftest import BAUD_INDEX, BAUD_RATE_IDX_ADDR, DEFAULT_BAUD
from dxl_packet import (
    build_ping,
    build_read,
    build_reboot,
    build_write,
    parse_status,
    read_status_frame,
)
from tinyboot_frame import (
    build_info_request,
    build_reset_request,
    read_info_response,
)

BOOT_MODE_ADDR = 0x031C
BOOT_MODE_BOOTLOADER = 1
CONTROL_BASE_ADDR = 0x0300
TINYBOOT_MODE_BOOTLOADER = 0


def _expect_status(port, osc_id, expected_err=0):
    frame = read_status_frame(port.ser, timeout_s=0.1)
    assert frame is not None, "no DXL Status frame"
    s = parse_status(frame)
    assert s.id == osc_id
    assert s.error == expected_err, f"err=0x{s.error:02X}, want 0x{expected_err:02X}"
    return s


def test_reboot_clears_volatile_state(port, osc_id, baud):
    # Reboot wipes RAM config back to defaults (incl. baud), so any non-default
    # test baud needs host+chip both restored before the session continues.
    port.writePort(build_write(osc_id, CONTROL_BASE_ADDR, b"\x01"))
    _expect_status(port, osc_id)
    port.writePort(build_read(osc_id, CONTROL_BASE_ADDR, 1))
    s = _expect_status(port, osc_id)
    assert s.params == b"\x01", f"setup failed, torque_enable={s.params.hex()}"

    port.writePort(build_reboot(osc_id))
    _expect_status(port, osc_id)
    time.sleep(0.5)

    try:
        if baud != DEFAULT_BAUD:
            port.setBaudRate(DEFAULT_BAUD)
        port.ser.reset_input_buffer()

        port.writePort(build_ping(osc_id))
        _expect_status(port, osc_id)

        port.writePort(build_read(osc_id, CONTROL_BASE_ADDR, 1))
        s = _expect_status(port, osc_id)
        assert s.params == b"\x00", f"reboot didn't clear volatile state: {s.params.hex()}"

        if baud != DEFAULT_BAUD:
            port.writePort(build_write(osc_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[baud]])))
            _expect_status(port, osc_id)
            time.sleep(0.05)  # let TC ISR apply pending BRR before host retunes
    finally:
        if port.getBaudRate() != baud:
            port.setBaudRate(baud)


def test_tinyboot_round_trip(port, osc_id, baud, tinyboot_baud):
    try:
        port.writePort(build_write(osc_id, BOOT_MODE_ADDR, bytes([BOOT_MODE_BOOTLOADER])))
        _expect_status(port, osc_id)

        port.writePort(build_reboot(osc_id))
        _expect_status(port, osc_id)

        time.sleep(0.3)
        port.setBaudRate(tinyboot_baud)
        port.ser.reset_input_buffer()

        port.writePort(build_info_request())
        info = read_info_response(port.ser, timeout_s=0.5)
        assert info.mode == TINYBOOT_MODE_BOOTLOADER, (
            f"expected mode=bootloader(0), got {info.mode} — chip is not in tinyboot"
        )
        assert info.capacity > 0 and info.erase_size > 0, f"bogus InfoData: {info}"

        port.writePort(build_reset_request(bootloader=False))
        time.sleep(0.5)

        # App reboot drops baud to default; ping at default, then retune if needed.
        port.setBaudRate(DEFAULT_BAUD)
        port.ser.reset_input_buffer()
        port.writePort(build_ping(osc_id))
        _expect_status(port, osc_id)

        if baud != DEFAULT_BAUD:
            port.writePort(build_write(osc_id, BAUD_RATE_IDX_ADDR, bytes([BAUD_INDEX[baud]])))
            _expect_status(port, osc_id)
            time.sleep(0.05)  # let TC ISR apply pending BRR before host retunes
    finally:
        if port.getBaudRate() != baud:
            port.setBaudRate(baud)
