from dynamixel_sdk import BROADCAST_ID, COMM_SUCCESS

ERR_DATA_RANGE = 0x04
ERR_ACCESS = 0x07
FOREIGN_ID = 99
CONTROL_BASE_ADDR = 0x0300  # torque_enable @ +0, RW
CONTROL_MODE_ADDR = CONTROL_BASE_ADDR + 1  # mode @ +1, RW (Mode enum)


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


def test_reg_write_then_action_commits(port, packet_handler, osc_id):
    comm, err = packet_handler.regWriteTxRx(port, osc_id, CONTROL_BASE_ADDR, 1, [1])
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == 0, f"RegWrite status error 0x{err:02X}"

    data, comm, err = packet_handler.read1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR)
    assert comm == COMM_SUCCESS
    assert err == 0
    assert data == 0, f"expected torque_enable still 0 before Action, got {data}"

    comm = packet_handler.action(port, BROADCAST_ID)
    assert comm == COMM_SUCCESS, f"Action send failed: {packet_handler.getTxRxResult(comm)}"

    data, comm, err = packet_handler.read1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR)
    assert comm == COMM_SUCCESS
    assert err == 0
    assert data == 1, f"expected torque_enable=1 after Action, got {data}"

    packet_handler.write1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR, 0)


def test_reg_write_to_ro_returns_access_error(port, packet_handler, osc_id):
    comm, err = packet_handler.regWriteTxRx(port, osc_id, 0, 2, [0xAA, 0xBB])
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == ERR_ACCESS, f"expected Access Error, got 0x{err:02X}"


def test_reg_write_invalid_value_returns_data_range_error(port, packet_handler, osc_id):
    comm, err = packet_handler.regWriteTxRx(port, osc_id, CONTROL_BASE_ADDR, 1, [2])
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == ERR_DATA_RANGE, f"expected Data Range Error, got 0x{err:02X}"

    # Action should commit nothing (stage was rewound on validator reject).
    packet_handler.action(port, BROADCAST_ID)
    data, comm, err = packet_handler.read1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR)
    assert comm == COMM_SUCCESS
    assert err == 0
    assert data == 0, f"torque_enable mutated despite RegWrite reject, got {data}"


def test_action_with_no_pending_staging_is_noop(port, packet_handler, osc_id):
    data_before, comm, err = packet_handler.read1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR)
    assert comm == COMM_SUCCESS
    assert err == 0

    packet_handler.action(port, BROADCAST_ID)

    data_after, comm, err = packet_handler.read1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR)
    assert comm == COMM_SUCCESS
    assert err == 0
    assert data_after == data_before, f"empty Action mutated state: {data_before} → {data_after}"


def test_write_clears_pending_reg_write(port, packet_handler, osc_id):
    comm, err = packet_handler.regWriteTxRx(port, osc_id, CONTROL_BASE_ADDR, 1, [1])
    assert comm == COMM_SUCCESS
    assert err == 0

    comm, err = packet_handler.write1ByteTxRx(port, osc_id, CONTROL_MODE_ADDR, 1)
    assert comm == COMM_SUCCESS
    assert err == 0

    packet_handler.action(port, BROADCAST_ID)

    data, comm, err = packet_handler.read1ByteTxRx(port, osc_id, CONTROL_BASE_ADDR)
    assert comm == COMM_SUCCESS
    assert err == 0
    assert data == 0, f"sync Write should have wiped pending RegWrite, but torque_enable={data}"

    packet_handler.write1ByteTxRx(port, osc_id, CONTROL_MODE_ADDR, 0)
