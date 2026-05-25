"""SyncRead / BulkRead — covers slot-position math and silent-when-absent.

These build raw packets so we can include foreign IDs in the list; the SDK's
group helpers would block waiting for those foreign IDs to respond.
"""

import time

from dxl_packet import (
    build_bulk_read,
    build_sync_read,
    parse_status,
    read_status_frame,
    slot_period_us,
)

FOREIGN_A = 99
FOREIGN_B = 100


def _silent_window_s(baud: int, max_slot_index: int, payload_len: int) -> float:
    delay_us = (max_slot_index + 1) * slot_period_us(baud, payload_len)
    return (delay_us / 1_000_000.0) + 0.02


def _read_status(port):
    return read_status_frame(port.ser, timeout_s=0.1)


def test_sync_read_solo(port, osc_id):
    port.writePort(build_sync_read(addr=0, length=2, ids=[osc_id]))
    frame = _read_status(port)
    assert frame is not None, "expected Status frame for solo SyncRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_sync_read_in_middle_slot(port, osc_id):
    port.writePort(build_sync_read(addr=0, length=2, ids=[FOREIGN_A, osc_id, FOREIGN_B]))
    frame = _read_status(port)
    assert frame is not None, "expected delayed Status frame for slot-1 SyncRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_sync_read_id_not_in_list_silent(port, osc_id, baud):
    port.writePort(build_sync_read(addr=0, length=2, ids=[FOREIGN_A, FOREIGN_B]))
    time.sleep(_silent_window_s(baud, max_slot_index=1, payload_len=2))
    n = port.ser.in_waiting
    assert n == 0, f"SyncRead w/o our ID should be silent, but {n} bytes queued: {port.ser.read(n).hex()}"


def test_bulk_read_solo(port, osc_id):
    port.writePort(build_bulk_read([(osc_id, 0, 2)]))
    frame = _read_status(port)
    assert frame is not None, "expected Status frame for solo BulkRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_bulk_read_in_middle_slot(port, osc_id):
    port.writePort(build_bulk_read([
        (FOREIGN_A, 0, 4),
        (osc_id, 0, 2),
        (FOREIGN_B, 0, 8),
    ]))
    frame = _read_status(port)
    assert frame is not None, "expected delayed Status frame for slot-1 BulkRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_bulk_read_id_not_in_list_silent(port, osc_id, baud):
    port.writePort(build_bulk_read([(FOREIGN_A, 0, 2), (FOREIGN_B, 0, 2)]))
    time.sleep(_silent_window_s(baud, max_slot_index=1, payload_len=2))
    n = port.ser.in_waiting
    assert n == 0, f"BulkRead w/o our ID should be silent, but {n} bytes queued: {port.ser.read(n).hex()}"
