from dynamixel_sdk import COMM_SUCCESS

ERR_INSTRUCTION = 0x02
FOREIGN_ID = 99


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


def test_write_rejected_with_instruction_error(port, packet_handler, osc_id):
    comm, err = packet_handler.write1ByteTxRx(port, osc_id, 0, 0)
    assert comm == COMM_SUCCESS, f"comm failed: {packet_handler.getTxRxResult(comm)}"
    assert err == ERR_INSTRUCTION, f"expected Instruction Error, got 0x{err:02X}"
