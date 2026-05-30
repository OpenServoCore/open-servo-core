from dxl_packet import (
    build_clear,
    build_control_table_backup,
    build_factory_reset,
    build_ping,
    build_read,
    build_write,
    parse_status,
    slot_period_us,
)

BROADCAST_ID = 0xFE
CONTROL_BASE_ADDR = 0x0300
ERR_INSTRUCTION = 0x02
ERR_DATA_RANGE = 0x04


def _silent_us(baud: int, payload_len: int = 0) -> int:
    return slot_period_us(baud, payload_len) + 20_000


def _expect_status(pirate, packet, osc_id, expected_err, reply_us=200_000):
    reply = pirate.xfer(packet, reply_us=reply_us)
    assert reply, f"no Status frame for {packet.hex()}"
    s = parse_status(reply)
    assert s.id == osc_id, f"wrong id: {s.id}"
    assert s.error == expected_err, f"expected err 0x{expected_err:02X}, got 0x{s.error:02X}"
    return s


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

        s = _expect_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1), osc_id, 0)
        assert s.params == b"\x01", f"broadcast Write didn't apply: {s.params.hex()}"
    finally:
        pirate.xfer(build_write(osc_id, CONTROL_BASE_ADDR, b"\x00"), reply_us=200_000)


def test_read_past_end_returns_data_range(pirate, osc_id):
    _expect_status(pirate, build_read(osc_id, 0xFFF0, 1), osc_id, ERR_DATA_RANGE)


def test_factory_reset_returns_instruction_error(pirate, osc_id):
    _expect_status(pirate, build_factory_reset(osc_id, option=0xFF), osc_id, ERR_INSTRUCTION)


def test_clear_returns_instruction_error(pirate, osc_id):
    _expect_status(pirate, build_clear(osc_id), osc_id, ERR_INSTRUCTION)


def test_control_table_backup_returns_instruction_error(pirate, osc_id):
    _expect_status(pirate, build_control_table_backup(osc_id), osc_id, ERR_INSTRUCTION)
