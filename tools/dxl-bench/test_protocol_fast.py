"""DXL Fast SyncRead / BulkRead — solo + slot-position emission semantics.

Covers: solo replies, silence when our ID isn't listed, error slots
(zero-fill payload), and the position-conditional Status framing — slot 0
emits header+body, middle slots emit body only, last slot emits body + the
final CRC over the whole coalesced frame.

The cross-slave coalesce stress lives in test_timing_fast_coalesce.py.
"""

from dxl_packet import (
    BROADCAST_ID,
    HEADER,
    INSTR_STATUS,
    _crc16,
    build_fast_bulk_read,
    build_fast_sync_read,
    parse_fast_response,
    slot_period_us,
)

FOREIGN_A = 99
FOREIGN_B = 100


def _silent_us(baud: int, max_slot_index: int, payload_len: int) -> int:
    return (max_slot_index + 1) * slot_period_us(baud, payload_len) + 20_000


def _solo_reply(pirate, packet, reply_us=200_000) -> bytes:
    reply = pirate.xfer(packet, reply_us=reply_us)
    assert reply, "expected Fast Status frame"
    return reply


# ── FastSyncRead ────────────────────────────────────────────────────────────

def test_fast_sync_read_only(pirate, osc_id):
    frame = _solo_reply(pirate, build_fast_sync_read(addr=0, length=2, ids=[osc_id]))
    slots = parse_fast_response(frame, slot_lengths=[2])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0
    assert len(slots[0].data) == 2


def test_fast_sync_read_silent_when_not_in_list(pirate, baud):
    reply = pirate.xfer(
        build_fast_sync_read(addr=0, length=2, ids=[FOREIGN_A, FOREIGN_B]),
        reply_us=_silent_us(baud, max_slot_index=1, payload_len=2),
    )
    assert reply is None, f"FastSyncRead w/o our ID got reply: {reply.hex()}"


def test_fast_sync_read_zero_length_silent(pirate, osc_id, baud):
    reply = pirate.xfer(
        build_fast_sync_read(addr=0, length=0, ids=[osc_id]),
        reply_us=_silent_us(baud, max_slot_index=0, payload_len=0),
    )
    assert reply is None, f"FastSyncRead length=0 got reply: {reply.hex()}"


def test_fast_sync_read_oversize_silent(pirate, osc_id, baud):
    reply = pirate.xfer(
        build_fast_sync_read(addr=0, length=200, ids=[osc_id]),
        reply_us=_silent_us(baud, max_slot_index=0, payload_len=200),
    )
    assert reply is None, f"FastSyncRead length>MAX_READ got reply: {reply.hex()}"


def test_fast_sync_read_out_of_range_returns_error_slot(pirate, osc_id):
    frame = _solo_reply(pirate, build_fast_sync_read(addr=0xFFF0, length=2, ids=[osc_id]))
    slots = parse_fast_response(frame, slot_lengths=[2])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0x04, f"expected DataRange, got 0x{slots[0].error:02X}"
    assert slots[0].data == b"\x00\x00", f"expected zero-fill, got {slots[0].data.hex()}"


# ── FastBulkRead ────────────────────────────────────────────────────────────

def test_fast_bulk_read_only(pirate, osc_id):
    frame = _solo_reply(pirate, build_fast_bulk_read([(osc_id, 0, 2)]))
    slots = parse_fast_response(frame, slot_lengths=[2])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0
    assert len(slots[0].data) == 2


def test_fast_bulk_read_silent_when_not_in_list(pirate, baud):
    reply = pirate.xfer(
        build_fast_bulk_read([(FOREIGN_A, 0, 2), (FOREIGN_B, 0, 4)]),
        reply_us=_silent_us(baud, max_slot_index=1, payload_len=4),
    )
    assert reply is None, f"FastBulkRead w/o our ID got reply: {reply.hex()}"


def test_fast_bulk_read_zero_length_silent(pirate, osc_id, baud):
    reply = pirate.xfer(
        build_fast_bulk_read([(osc_id, 0, 0)]),
        reply_us=_silent_us(baud, max_slot_index=0, payload_len=0),
    )
    assert reply is None, f"FastBulkRead length=0 got reply: {reply.hex()}"


def test_fast_bulk_read_out_of_range_returns_error_slot(pirate, osc_id):
    frame = _solo_reply(pirate, build_fast_bulk_read([(osc_id, 0xFFF0, 4)]))
    slots = parse_fast_response(frame, slot_lengths=[4])
    assert len(slots) == 1
    assert slots[0].id == osc_id
    assert slots[0].error == 0x04, f"expected DataRange, got 0x{slots[0].error:02X}"
    assert slots[0].data == b"\x00\x00\x00\x00", f"expected zero-fill, got {slots[0].data.hex()}"


# ── slot-position emission ──────────────────────────────────────────────────

def test_fast_sync_read_first_emits_header_and_body_only(pirate, osc_id, baud):
    chunk = pirate.xfer(
        build_fast_sync_read(addr=0, length=2, ids=[osc_id, FOREIGN_A, FOREIGN_B]),
        reply_us=_silent_us(baud, max_slot_index=2, payload_len=2),
    )
    assert chunk is not None and len(chunk) == 12, f"expected 12 bytes, got {chunk and chunk.hex()}"
    assert chunk[:4] == HEADER, f"bad header: {chunk[:4].hex()}"
    assert chunk[4] == BROADCAST_ID, f"id field should be 0xFE, got 0x{chunk[4]:02X}"
    packet_length = chunk[5] | (chunk[6] << 8)
    assert packet_length == 3 + 3 * (2 + 2), f"LEN field = {packet_length}, want 15"
    assert chunk[7] == INSTR_STATUS
    assert chunk[8] == 0, f"err = 0x{chunk[8]:02X}"
    assert chunk[9] == osc_id, f"slot id = {chunk[9]}, want {osc_id}"


def test_fast_sync_read_middle_emits_body_only(pirate, osc_id, baud):
    chunk = pirate.xfer(
        build_fast_sync_read(addr=0, length=2, ids=[FOREIGN_A, osc_id, FOREIGN_B]),
        reply_us=_silent_us(baud, max_slot_index=2, payload_len=2),
    )
    assert chunk is not None and len(chunk) == 4, f"expected 4 bytes, got {chunk and chunk.hex()}"
    assert chunk[0] == 0, f"err = 0x{chunk[0]:02X}"
    assert chunk[1] == osc_id, f"slot id = {chunk[1]}, want {osc_id}"


def test_fast_sync_read_last_emits_body_and_self_crc(pirate, osc_id, baud):
    chunk = pirate.xfer(
        build_fast_sync_read(addr=0, length=2, ids=[FOREIGN_A, FOREIGN_B, osc_id]),
        reply_us=_silent_us(baud, max_slot_index=2, payload_len=2),
    )
    assert chunk is not None and len(chunk) == 6, f"expected 6 bytes, got {chunk and chunk.hex()}"
    assert chunk[0] == 0, f"err = 0x{chunk[0]:02X}"
    assert chunk[1] == osc_id, f"slot id = {chunk[1]}, want {osc_id}"
    body = chunk[:4]
    crc = chunk[4] | (chunk[5] << 8)
    expected = _crc16(body)
    assert crc == expected, f"CRC over body alone = 0x{crc:04X}, want 0x{expected:04X}"
