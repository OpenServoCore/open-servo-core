from dynamixel_sdk import COMM_SUCCESS

ERR_DATA_RANGE = 0x04
ERR_ACCESS = 0x07
FOREIGN_ID = 99
CONTROL_BASE_ADDR = 0x0300  # torque_enable @ +0, RW


def test_ping_own_id(port, packet_handler, osc_id):
    _model, comm, err = packet_handler.ping(port, osc_id)
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == 0, f"status error 0x{err:02X}"


def test_ping_foreign_id_silent(port, packet_handler):
    _model, comm, _err = packet_handler.ping(port, FOREIGN_ID)
    assert comm != COMM_SUCCESS, "OSC replied to ping for an ID it shouldn't own"


def test_read_model_number(port, packet_handler, osc_id):
    data, comm, err = packet_handler.read2ByteTxRx(port, osc_id, 0)
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == 0, f"status error 0x{err:02X}"
    assert isinstance(data, int)


def test_write_torque_enable_round_trip(port, packet_handler, osc_id):
    comm, err = packet_handler.write1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR, 1)
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == 0, f"status error 0x{err:02X}"

    data, comm, err = packet_handler.read1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR)
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == 0, f"status error 0x{err:02X}"
    assert data == 1, f"expected torque_enable=1, got {data}"

    # Restore so subsequent runs start from a known state.
    packet_handler.write1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR, 0)


def test_write_to_ro_returns_access_error(port, packet_handler, osc_id):
    # Address 0 is identity.model_number (RO).
    comm, err = packet_handler.write1ByteTxRx(port, osc_id, 0, 0)
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == ERR_ACCESS, f"expected Access Error, got 0x{err:02X}"


def test_write_to_unmapped_returns_data_range_error(port, packet_handler, osc_id):
    comm, err = packet_handler.write1ByteTxRx(port, osc_id, 0xFFFE, 0)
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == ERR_DATA_RANGE, f"expected Data Range Error, got 0x{err:02X}"
