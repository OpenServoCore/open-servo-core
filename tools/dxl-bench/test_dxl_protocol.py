import time

from dxl_packet import (
    build_clear,
    build_control_table_backup,
    build_factory_reset,
    build_ping,
    build_read,
    build_reboot,
    build_write,
    parse_status,
    read_status_frame,
    slot_period_us,
)

BROADCAST_ID = 0xFE
CONTROL_BASE_ADDR = 0x0300
ERR_INSTRUCTION = 0x02
ERR_DATA_RANGE = 0x04


def _silent(port, baud: int, payload_len: int = 0):
    one_slot_us = slot_period_us(baud, payload_len)
    time.sleep((one_slot_us / 1_000_000.0) + 0.02)
    n = port.ser.in_waiting
    assert n == 0, f"expected silence, got {n} bytes: {port.ser.read(n).hex()}"


def _expect_status(port, osc_id, expected_err):
    frame = read_status_frame(port.ser, timeout_s=0.1)
    assert frame is not None, "expected a Status frame"
    s = parse_status(frame)
    assert s.id == osc_id, f"wrong id: {s.id}"
    assert s.error == expected_err, f"expected err 0x{expected_err:02X}, got 0x{s.error:02X}"
    return s


def test_bad_crc_silent(port, osc_id, baud):
    pkt = bytearray(build_ping(osc_id))
    pkt[-1] ^= 0xFF
    port.writePort(bytes(pkt))
    _silent(port, baud, payload_len=3)


def test_truncated_packet_silent(port, osc_id, baud):
    pkt = build_ping(osc_id)
    port.writePort(pkt[:-2])
    _silent(port, baud, payload_len=3)


def test_broadcast_write_applies_silently(port, osc_id, baud):
    try:
        port.writePort(build_write(BROADCAST_ID, CONTROL_BASE_ADDR, b"\x01"))
        _silent(port, baud)

        port.writePort(build_read(osc_id, CONTROL_BASE_ADDR, 1))
        s = _expect_status(port, osc_id, 0)
        assert s.params == b"\x01", f"broadcast Write didn't apply: {s.params.hex()}"
    finally:
        port.writePort(build_write(osc_id, CONTROL_BASE_ADDR, b"\x00"))
        read_status_frame(port.ser, timeout_s=0.1)


def test_read_past_end_returns_data_range(port, osc_id):
    port.writePort(build_read(osc_id, 0xFFF0, 1))
    _expect_status(port, osc_id, ERR_DATA_RANGE)


def test_factory_reset_returns_instruction_error(port, osc_id):
    port.writePort(build_factory_reset(osc_id, option=0xFF))
    _expect_status(port, osc_id, ERR_INSTRUCTION)


def test_clear_returns_instruction_error(port, osc_id):
    port.writePort(build_clear(osc_id))
    _expect_status(port, osc_id, ERR_INSTRUCTION)


def test_control_table_backup_returns_instruction_error(port, osc_id):
    port.writePort(build_control_table_backup(osc_id))
    _expect_status(port, osc_id, ERR_INSTRUCTION)
