"""DXL 2.0 instruction-handling coverage: happy paths + error returns.

Covers PING / READ / WRITE / REG_WRITE+ACTION (the supported instructions) and
the four error classes the chip emits (Access, DataRange, Instruction, plus
frame-level silence on bad CRC / truncation / foreign IDs / broadcasts).
"""

from dxl_packet import (
    BROADCAST_ID,
    build_action,
    build_clear,
    build_control_table_backup,
    build_factory_reset,
    build_ping,
    build_read,
    build_reg_write,
    build_write,
    parse_status,
    slot_period_us,
)

ERR_INSTRUCTION = 0x02
ERR_DATA_RANGE = 0x04
ERR_ACCESS = 0x07
FOREIGN_ID = 99
CONTROL_BASE_ADDR = 0x0300  # torque_enable @ +0, RW
CONTROL_MODE_ADDR = CONTROL_BASE_ADDR + 1  # mode @ +1, RW


def _xfer_status(pirate, packet, reply_us=200_000):
    reply = pirate.xfer(packet, reply_us=reply_us)
    assert reply, f"no Status frame for {packet[7]:#04x}"
    return parse_status(reply)


def _silent_us(baud: int, payload_len: int = 0) -> int:
    return slot_period_us(baud, payload_len) + 20_000


# ── happy path ──────────────────────────────────────────────────────────────

def test_ping_own_id(pirate, osc_id):
    s = _xfer_status(pirate, build_ping(osc_id))
    assert s.id == osc_id
    assert s.error == 0, f"status error 0x{s.error:02X}"


def test_read_model_number(pirate, osc_id):
    s = _xfer_status(pirate, build_read(osc_id, 0, 2))
    assert s.error == 0
    assert len(s.params) == 2


def test_write_torque_enable_round_trip(pirate, osc_id):
    try:
        s = _xfer_status(pirate, build_write(osc_id, CONTROL_BASE_ADDR, b"\x01"))
        assert s.error == 0, f"status error 0x{s.error:02X}"

        s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
        assert s.error == 0
        assert s.params == b"\x01", f"expected torque_enable=1, got {s.params.hex()}"
    finally:
        pirate.xfer(build_write(osc_id, CONTROL_BASE_ADDR, b"\x00"), reply_us=200_000)


def test_reg_write_then_action_commits(pirate, osc_id):
    try:
        s = _xfer_status(pirate, build_reg_write(osc_id, CONTROL_BASE_ADDR, b"\x01"))
        assert s.error == 0, f"RegWrite status error 0x{s.error:02X}"

        s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
        assert s.error == 0
        assert s.params == b"\x00", f"staged write applied early: {s.params.hex()}"

        pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)

        s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
        assert s.error == 0
        assert s.params == b"\x01", f"Action didn't commit: {s.params.hex()}"
    finally:
        pirate.xfer(build_write(osc_id, CONTROL_BASE_ADDR, b"\x00"), reply_us=200_000)


def test_action_with_no_pending_staging_is_noop(pirate, osc_id):
    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    before = s.params

    pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)

    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == before, f"empty Action mutated state: {before.hex()} → {s.params.hex()}"


def test_write_clears_pending_reg_write(pirate, osc_id):
    try:
        s = _xfer_status(pirate, build_reg_write(osc_id, CONTROL_BASE_ADDR, b"\x01"))
        assert s.error == 0

        s = _xfer_status(pirate, build_write(osc_id, CONTROL_MODE_ADDR, b"\x01"))
        assert s.error == 0

        pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)

        s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
        assert s.error == 0
        assert s.params == b"\x00", f"sync Write didn't wipe pending RegWrite: {s.params.hex()}"
    finally:
        pirate.xfer(build_write(osc_id, CONTROL_MODE_ADDR, b"\x00"), reply_us=200_000)


# ── silence ─────────────────────────────────────────────────────────────────

def test_ping_foreign_id_silent(pirate):
    reply = pirate.xfer(build_ping(FOREIGN_ID), reply_us=50_000)
    assert reply is None, f"OSC replied to foreign ping: {reply.hex()}"


def test_bad_crc_silent(pirate, osc_id, baud):
    pkt = bytearray(build_ping(osc_id))
    pkt[-1] ^= 0xFF
    reply = pirate.xfer(bytes(pkt), reply_us=_silent_us(baud, payload_len=3))
    assert reply is None, f"bad-CRC ping replied: {reply.hex()}"


def test_truncated_packet_silent(pirate, osc_id, baud):
    pkt = build_ping(osc_id)
    reply = pirate.xfer(pkt[:-2], reply_us=_silent_us(baud, payload_len=3))
    assert reply is None, f"truncated ping replied: {reply.hex()}"


def test_broadcast_write_applies_silently(pirate, osc_id, baud):
    try:
        reply = pirate.xfer(
            build_write(BROADCAST_ID, CONTROL_BASE_ADDR, b"\x01"),
            reply_us=_silent_us(baud),
        )
        assert reply is None, f"broadcast Write replied: {reply.hex()}"

        s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
        assert s.params == b"\x01", f"broadcast Write didn't apply: {s.params.hex()}"
    finally:
        pirate.xfer(build_write(osc_id, CONTROL_BASE_ADDR, b"\x00"), reply_us=200_000)


# ── error returns ───────────────────────────────────────────────────────────

def test_write_to_ro_returns_access_error(pirate, osc_id):
    s = _xfer_status(pirate, build_write(osc_id, 0, b"\x00"))
    assert s.error == ERR_ACCESS, f"expected Access Error, got 0x{s.error:02X}"


def test_write_to_unmapped_returns_data_range_error(pirate, osc_id):
    s = _xfer_status(pirate, build_write(osc_id, 0xFFFE, b"\x00"))
    assert s.error == ERR_DATA_RANGE, f"expected DataRange Error, got 0x{s.error:02X}"


def test_read_past_end_returns_data_range(pirate, osc_id):
    s = _xfer_status(pirate, build_read(osc_id, 0xFFF0, 1))
    assert s.id == osc_id
    assert s.error == ERR_DATA_RANGE, f"expected DataRange, got 0x{s.error:02X}"


def test_reg_write_to_ro_returns_access_error(pirate, osc_id):
    s = _xfer_status(pirate, build_reg_write(osc_id, 0, b"\xAA\xBB"))
    assert s.error == ERR_ACCESS, f"expected Access Error, got 0x{s.error:02X}"


def test_reg_write_invalid_value_returns_data_range_error(pirate, osc_id):
    s = _xfer_status(pirate, build_reg_write(osc_id, CONTROL_BASE_ADDR, b"\x02"))
    assert s.error == ERR_DATA_RANGE, f"expected DataRange Error, got 0x{s.error:02X}"

    pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)
    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == b"\x00", f"torque_enable mutated despite RegWrite reject: {s.params.hex()}"


def test_factory_reset_returns_instruction_error(pirate, osc_id):
    s = _xfer_status(pirate, build_factory_reset(osc_id, option=0xFF))
    assert s.error == ERR_INSTRUCTION, f"expected Instruction Error, got 0x{s.error:02X}"


def test_clear_returns_instruction_error(pirate, osc_id):
    s = _xfer_status(pirate, build_clear(osc_id))
    assert s.error == ERR_INSTRUCTION, f"expected Instruction Error, got 0x{s.error:02X}"


def test_control_table_backup_returns_instruction_error(pirate, osc_id):
    s = _xfer_status(pirate, build_control_table_backup(osc_id))
    assert s.error == ERR_INSTRUCTION, f"expected Instruction Error, got 0x{s.error:02X}"
