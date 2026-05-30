from dxl_packet import (
    BROADCAST_ID,
    build_action,
    build_ping,
    build_read,
    build_reg_write,
    build_write,
    parse_status,
)

ERR_DATA_RANGE = 0x04
ERR_ACCESS = 0x07
FOREIGN_ID = 99
CONTROL_BASE_ADDR = 0x0300  # torque_enable @ +0, RW
CONTROL_MODE_ADDR = CONTROL_BASE_ADDR + 1  # mode @ +1, RW (Mode enum)


def _xfer_status(pirate, packet, reply_us=200_000):
    reply = pirate.xfer(packet, reply_us=reply_us)
    assert reply, f"no Status frame for {packet[7]:#04x}"
    return parse_status(reply)


def test_ping_own_id(pirate, osc_id):
    s = _xfer_status(pirate, build_ping(osc_id))
    assert s.id == osc_id
    assert s.error == 0, f"status error 0x{s.error:02X}"


def test_ping_foreign_id_silent(pirate):
    reply = pirate.xfer(build_ping(FOREIGN_ID), reply_us=50_000)
    assert reply is None, f"OSC replied to foreign ping: {reply.hex()}"


def test_read_model_number(pirate, osc_id):
    s = _xfer_status(pirate, build_read(osc_id, 0, 2))
    assert s.error == 0, f"status error 0x{s.error:02X}"
    assert len(s.params) == 2


def test_write_torque_enable_round_trip(pirate, osc_id):
    s = _xfer_status(pirate, build_write(osc_id, CONTROL_BASE_ADDR, b"\x01"))
    assert s.error == 0, f"status error 0x{s.error:02X}"

    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == b"\x01", f"expected torque_enable=1, got {s.params.hex()}"

    pirate.xfer(build_write(osc_id, CONTROL_BASE_ADDR, b"\x00"), reply_us=200_000)


def test_write_to_ro_returns_access_error(pirate, osc_id):
    s = _xfer_status(pirate, build_write(osc_id, 0, b"\x00"))
    assert s.error == ERR_ACCESS, f"expected Access Error, got 0x{s.error:02X}"


def test_write_to_unmapped_returns_data_range_error(pirate, osc_id):
    s = _xfer_status(pirate, build_write(osc_id, 0xFFFE, b"\x00"))
    assert s.error == ERR_DATA_RANGE, f"expected Data Range Error, got 0x{s.error:02X}"


def test_reg_write_then_action_commits(pirate, osc_id):
    s = _xfer_status(pirate, build_reg_write(osc_id, CONTROL_BASE_ADDR, b"\x01"))
    assert s.error == 0, f"RegWrite status error 0x{s.error:02X}"

    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == b"\x00", f"expected torque_enable still 0 before Action, got {s.params.hex()}"

    # Action is broadcast: no reply expected.
    pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)

    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == b"\x01", f"expected torque_enable=1 after Action, got {s.params.hex()}"

    pirate.xfer(build_write(osc_id, CONTROL_BASE_ADDR, b"\x00"), reply_us=200_000)


def test_reg_write_to_ro_returns_access_error(pirate, osc_id):
    s = _xfer_status(pirate, build_reg_write(osc_id, 0, b"\xAA\xBB"))
    assert s.error == ERR_ACCESS, f"expected Access Error, got 0x{s.error:02X}"


def test_reg_write_invalid_value_returns_data_range_error(pirate, osc_id):
    s = _xfer_status(pirate, build_reg_write(osc_id, CONTROL_BASE_ADDR, b"\x02"))
    assert s.error == ERR_DATA_RANGE, f"expected Data Range Error, got 0x{s.error:02X}"

    # Action should commit nothing (stage was rewound on validator reject).
    pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)
    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == b"\x00", f"torque_enable mutated despite RegWrite reject, got {s.params.hex()}"


def test_action_with_no_pending_staging_is_noop(pirate, osc_id):
    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    before = s.params

    pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)

    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == before, f"empty Action mutated state: {before.hex()} → {s.params.hex()}"


def test_write_clears_pending_reg_write(pirate, osc_id):
    s = _xfer_status(pirate, build_reg_write(osc_id, CONTROL_BASE_ADDR, b"\x01"))
    assert s.error == 0

    s = _xfer_status(pirate, build_write(osc_id, CONTROL_MODE_ADDR, b"\x01"))
    assert s.error == 0

    pirate.xfer(build_action(BROADCAST_ID), reply_us=50_000)

    s = _xfer_status(pirate, build_read(osc_id, CONTROL_BASE_ADDR, 1))
    assert s.error == 0
    assert s.params == b"\x00", f"sync Write should have wiped pending RegWrite, got {s.params.hex()}"

    pirate.xfer(build_write(osc_id, CONTROL_MODE_ADDR, b"\x00"), reply_us=200_000)
