import time

from dxl_packet import (
    build_fast_bulk_read,
    build_fast_sync_read,
    parse_fast_response,
    read_status_frame,
    slot_period_us,
)

FOREIGN_A = 99
FOREIGN_B = 100


def _silent_window_s(baud: int, max_slot_index: int, payload_len: int) -> float:
    delay_us = (max_slot_index + 1) * slot_period_us(baud, payload_len)
    return (delay_us / 1_000_000.0) + 0.02


def test_fast_sync_read_only(port, osc_id):
    port.writePort(build_fast_sync_read(addr=0, length=2, ids=[osc_id]))
    frame = read_status_frame(port.ser, timeout_s=0.1)
    assert frame is not None, "expected Fast Status frame for Only-slot SyncRead"
    slots = parse_fast_response(frame, slot_lengths=[2])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0
    assert len(slots[0].data) == 2


def test_fast_sync_read_silent_when_not_in_list(port, osc_id, baud):
    port.writePort(build_fast_sync_read(addr=0, length=2, ids=[FOREIGN_A, FOREIGN_B]))
    time.sleep(_silent_window_s(baud, max_slot_index=1, payload_len=2))
    n = port.ser.in_waiting
    assert n == 0, f"FastSyncRead w/o our ID should be silent, but {n} bytes queued: {port.ser.read(n).hex()}"


def test_fast_sync_read_zero_length_silent(port, osc_id, baud):
    port.writePort(build_fast_sync_read(addr=0, length=0, ids=[osc_id]))
    time.sleep(_silent_window_s(baud, max_slot_index=0, payload_len=0))
    n = port.ser.in_waiting
    assert n == 0, f"FastSyncRead length=0 should be silent, but {n} bytes queued: {port.ser.read(n).hex()}"


def test_fast_sync_read_oversize_silent(port, osc_id, baud):
    port.writePort(build_fast_sync_read(addr=0, length=200, ids=[osc_id]))
    time.sleep(_silent_window_s(baud, max_slot_index=0, payload_len=200))
    n = port.ser.in_waiting
    assert n == 0, f"FastSyncRead length>MAX_READ should be silent, but {n} bytes queued: {port.ser.read(n).hex()}"


def test_fast_sync_read_out_of_range_returns_error_slot(port, osc_id):
    port.writePort(build_fast_sync_read(addr=0xFFF0, length=2, ids=[osc_id]))
    frame = read_status_frame(port.ser, timeout_s=0.1)
    assert frame is not None, "expected Fast Status frame even on read error"
    slots = parse_fast_response(frame, slot_lengths=[2])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0x04, f"expected DataRange, got 0x{slots[0].error:02X}"
    assert slots[0].data == b"\x00\x00", f"expected zero-fill, got {slots[0].data.hex()}"


def test_fast_bulk_read_only(port, osc_id):
    port.writePort(build_fast_bulk_read([(osc_id, 0, 2)]))
    frame = read_status_frame(port.ser, timeout_s=0.1)
    assert frame is not None, "expected Fast Status frame for Only-slot BulkRead"
    slots = parse_fast_response(frame, slot_lengths=[2])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0
    assert len(slots[0].data) == 2


def test_fast_bulk_read_silent_when_not_in_list(port, osc_id, baud):
    port.writePort(build_fast_bulk_read([(FOREIGN_A, 0, 2), (FOREIGN_B, 0, 4)]))
    time.sleep(_silent_window_s(baud, max_slot_index=1, payload_len=4))
    n = port.ser.in_waiting
    assert n == 0, f"FastBulkRead w/o our ID should be silent, but {n} bytes queued: {port.ser.read(n).hex()}"


def test_fast_bulk_read_zero_length_silent(port, osc_id, baud):
    port.writePort(build_fast_bulk_read([(osc_id, 0, 0)]))
    time.sleep(_silent_window_s(baud, max_slot_index=0, payload_len=0))
    n = port.ser.in_waiting
    assert n == 0, f"FastBulkRead length=0 should be silent, but {n} bytes queued: {port.ser.read(n).hex()}"


def test_fast_bulk_read_out_of_range_returns_error_slot(port, osc_id):
    port.writePort(build_fast_bulk_read([(osc_id, 0xFFF0, 4)]))
    frame = read_status_frame(port.ser, timeout_s=0.1)
    assert frame is not None, "expected Fast Status frame even on read error"
    slots = parse_fast_response(frame, slot_lengths=[4])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0x04, f"expected DataRange, got 0x{slots[0].error:02X}"
    assert slots[0].data == b"\x00\x00\x00\x00", f"expected zero-fill, got {slots[0].data.hex()}"
