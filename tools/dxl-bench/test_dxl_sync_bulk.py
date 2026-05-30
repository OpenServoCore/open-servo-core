"""SyncRead / BulkRead — covers slot-position math and silent-when-absent."""

from dxl_packet import (
    build_bulk_read,
    build_sync_read,
    parse_status,
    slot_period_us,
)

FOREIGN_A = 99
FOREIGN_B = 100


def _silent_us(baud: int, max_slot_index: int, payload_len: int) -> int:
    delay_us = (max_slot_index + 1) * slot_period_us(baud, payload_len)
    return delay_us + 20_000


def test_sync_read_solo(pirate, osc_id):
    frame = pirate.xfer(build_sync_read(addr=0, length=2, ids=[osc_id]), reply_us=200_000)
    assert frame, "expected Status frame for solo SyncRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_sync_read_in_middle_slot(pirate, osc_id, baud):
    frame = pirate.xfer(
        build_sync_read(addr=0, length=2, ids=[FOREIGN_A, osc_id, FOREIGN_B]),
        reply_us=_silent_us(baud, max_slot_index=2, payload_len=2),
    )
    assert frame, "expected delayed Status frame for slot-1 SyncRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_sync_read_id_not_in_list_silent(pirate, baud):
    reply = pirate.xfer(
        build_sync_read(addr=0, length=2, ids=[FOREIGN_A, FOREIGN_B]),
        reply_us=_silent_us(baud, max_slot_index=1, payload_len=2),
    )
    assert reply is None, f"SyncRead w/o our ID got reply: {reply.hex()}"


def test_bulk_read_solo(pirate, osc_id):
    frame = pirate.xfer(build_bulk_read([(osc_id, 0, 2)]), reply_us=200_000)
    assert frame, "expected Status frame for solo BulkRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_bulk_read_in_middle_slot(pirate, osc_id, baud):
    frame = pirate.xfer(
        build_bulk_read([
            (FOREIGN_A, 0, 4),
            (osc_id, 0, 2),
            (FOREIGN_B, 0, 8),
        ]),
        reply_us=_silent_us(baud, max_slot_index=2, payload_len=8),
    )
    assert frame, "expected delayed Status frame for slot-1 BulkRead"
    s = parse_status(frame)
    assert s.id == osc_id and s.error == 0
    assert len(s.params) == 2


def test_bulk_read_id_not_in_list_silent(pirate, baud):
    reply = pirate.xfer(
        build_bulk_read([(FOREIGN_A, 0, 2), (FOREIGN_B, 0, 2)]),
        reply_us=_silent_us(baud, max_slot_index=1, payload_len=2),
    )
    assert reply is None, f"BulkRead w/o our ID got reply: {reply.hex()}"
