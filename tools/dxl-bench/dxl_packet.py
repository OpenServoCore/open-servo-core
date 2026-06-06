"""DXL 2.0 packet builders + parser for tests that need raw control.

The DynamixelSDK's GroupSyncRead/GroupBulkRead are convenient but block waiting
for every advertised ID to respond — useless for tests that intentionally
include foreign IDs to exercise slot-position math.
"""

from typing import Iterable

INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_WRITE = 0x03
INSTR_REG_WRITE = 0x04
INSTR_ACTION = 0x05
INSTR_FACTORY_RESET = 0x06
INSTR_REBOOT = 0x08
INSTR_CLEAR = 0x10
INSTR_CONTROL_TABLE_BACKUP = 0x20
INSTR_STATUS = 0x55
INSTR_SYNC_READ = 0x82
INSTR_BULK_READ = 0x92
INSTR_FAST_SYNC_READ = 0x8A
INSTR_FAST_BULK_READ = 0x9A
INSTR_CALIBRATE = 0xE0

HEADER = bytes([0xFF, 0xFF, 0xFD, 0x00])
BROADCAST_ID = 0xFE

# Mirrors firmware/lib/core/src/services/dxl/slot.rs.
_STATUS_OVERHEAD = 11
_SLOT_MARGIN = 4
_BITS_PER_BYTE = 10


def slot_period_us(baud: int, param_len: int) -> int:
    bytes_ = _STATUS_OVERHEAD + param_len + _SLOT_MARGIN
    return bytes_ * _BITS_PER_BYTE * 1_000_000 // baud


def _crc16(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = ((crc << 8) ^ _CRC_TABLE[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


def build_packet(id: int, instr: int, params: bytes = b"") -> bytes:
    length = len(params) + 3
    body = bytearray(HEADER)
    body.append(id)
    body.append(length & 0xFF)
    body.append((length >> 8) & 0xFF)
    body.append(instr)
    body.extend(params)
    crc = _crc16(bytes(body))
    body.append(crc & 0xFF)
    body.append((crc >> 8) & 0xFF)
    return bytes(body)


def build_ping(id: int) -> bytes:
    return build_packet(id, INSTR_PING)


def build_read(id: int, addr: int, length: int) -> bytes:
    params = bytes(
        [addr & 0xFF, (addr >> 8) & 0xFF, length & 0xFF, (length >> 8) & 0xFF]
    )
    return build_packet(id, INSTR_READ, params)


def build_write(id: int, addr: int, data: bytes) -> bytes:
    params = bytearray([addr & 0xFF, (addr >> 8) & 0xFF])
    params.extend(data)
    return build_packet(id, INSTR_WRITE, bytes(params))


def build_reg_write(id: int, addr: int, data: bytes) -> bytes:
    params = bytearray([addr & 0xFF, (addr >> 8) & 0xFF])
    params.extend(data)
    return build_packet(id, INSTR_REG_WRITE, bytes(params))


def build_action(id: int) -> bytes:
    return build_packet(id, INSTR_ACTION)


def build_reboot(id: int) -> bytes:
    return build_packet(id, INSTR_REBOOT)


def build_calibrate(id: int, count: int) -> bytes:
    """CALIB request: master streams `count` zero filler bytes after the
    count field. Slave times its own RX between T_first (first RXNE) and
    T_last (IDLE-backdated end of last byte) and replies with the measurement.
    Wire payload = count(2 LE) + count zeros; total frame = 12 + count bytes."""
    payload = bytes([count & 0xFF, (count >> 8) & 0xFF]) + bytes(count)
    return build_packet(id, INSTR_CALIBRATE, payload)


def build_factory_reset(id: int, option: int = 0xFF) -> bytes:
    return build_packet(id, INSTR_FACTORY_RESET, bytes([option & 0xFF]))


def build_clear(id: int, option: int = 0x01, key: bytes = b"CLR\0") -> bytes:
    return build_packet(id, INSTR_CLEAR, bytes([option & 0xFF]) + key)


def build_control_table_backup(
    id: int, option: int = 0x01, key: bytes = b"CTRL"
) -> bytes:
    return build_packet(id, INSTR_CONTROL_TABLE_BACKUP, bytes([option & 0xFF]) + key)


def build_sync_read(addr: int, length: int, ids: Iterable[int]) -> bytes:
    params = bytearray(
        [addr & 0xFF, (addr >> 8) & 0xFF, length & 0xFF, (length >> 8) & 0xFF]
    )
    params.extend(ids)
    return build_packet(BROADCAST_ID, INSTR_SYNC_READ, bytes(params))


def build_bulk_read(tuples: Iterable[tuple[int, int, int]]) -> bytes:
    """tuples: iterable of (id, address, length)."""
    params = bytearray()
    for (tid, addr, length) in tuples:
        params.extend([
            tid,
            addr & 0xFF, (addr >> 8) & 0xFF,
            length & 0xFF, (length >> 8) & 0xFF,
        ])
    return build_packet(BROADCAST_ID, INSTR_BULK_READ, bytes(params))


def build_fast_sync_read(addr: int, length: int, ids: Iterable[int]) -> bytes:
    params = bytearray(
        [addr & 0xFF, (addr >> 8) & 0xFF, length & 0xFF, (length >> 8) & 0xFF]
    )
    params.extend(ids)
    return build_packet(BROADCAST_ID, INSTR_FAST_SYNC_READ, bytes(params))


def build_fast_bulk_read(tuples: Iterable[tuple[int, int, int]]) -> bytes:
    params = bytearray()
    for (tid, addr, length) in tuples:
        params.extend([
            tid,
            addr & 0xFF, (addr >> 8) & 0xFF,
            length & 0xFF, (length >> 8) & 0xFF,
        ])
    return build_packet(BROADCAST_ID, INSTR_FAST_BULK_READ, bytes(params))


def build_fast_first_bytes(
    packet_length: int, err: int, slot_id: int, data: bytes
) -> bytes:
    """On-wire bytes for a FastSlot::First: DXL header + status overhead + this
    slot's body (err + id + data). No CRC — only FastSlot::Last carries the
    frame CRC, computed over everything that preceded it on the bus."""
    pkt = bytearray(HEADER)
    pkt.append(BROADCAST_ID)
    pkt.append(packet_length & 0xFF)
    pkt.append((packet_length >> 8) & 0xFF)
    pkt.append(INSTR_STATUS)
    pkt.append(err)
    pkt.append(slot_id)
    pkt.extend(data)
    return bytes(pkt)


class FastSlot(tuple):
    @property
    def error(self) -> int: return self[0]
    @property
    def id(self) -> int: return self[1]
    @property
    def data(self) -> bytes: return self[2]


def parse_fast_response(frame: bytes, slot_lengths: list[int]) -> list[FastSlot]:
    """Parse a Fast Sync/Bulk Status frame. slot_lengths: payload length per slot, in order."""
    if len(frame) < 11:
        raise ValueError(f"too short ({len(frame)} bytes): {frame.hex()}")
    if frame[:4] != HEADER:
        raise ValueError(f"bad header: {frame[:4].hex()}")
    if frame[4] != BROADCAST_ID:
        raise ValueError(f"Fast response id field should be 0xFE, got 0x{frame[4]:02X}")
    length = frame[5] | (frame[6] << 8)
    if 7 + length != len(frame):
        raise ValueError(f"length field={length} != actual={len(frame) - 7}")
    if frame[7] != INSTR_STATUS:
        raise ValueError(f"not a Status frame: instr=0x{frame[7]:02X}")
    crc_lo, crc_hi = frame[-2], frame[-1]
    expected = _crc16(frame[:-2])
    if (crc_lo | (crc_hi << 8)) != expected:
        raise ValueError(f"CRC mismatch: got {crc_hi:02x}{crc_lo:02x}, want {expected:04x}")
    body = frame[8:-2]
    slots = []
    i = 0
    for slot_len in slot_lengths:
        if i + 2 + slot_len > len(body):
            raise ValueError(f"body too short for slot {len(slots)} (len {slot_len})")
        slots.append(FastSlot((body[i], body[i + 1], bytes(body[i + 2 : i + 2 + slot_len]))))
        i += 2 + slot_len
    if i != len(body):
        raise ValueError(f"body has {len(body) - i} trailing bytes after {len(slot_lengths)} slots")
    return slots


class StatusPacket(tuple):
    @property
    def id(self) -> int: return self[0]
    @property
    def error(self) -> int: return self[1]
    @property
    def params(self) -> bytes: return self[2]


def parse_status(data: bytes) -> StatusPacket:
    if len(data) < 11:
        raise ValueError(f"too short ({len(data)} bytes): {data.hex()}")
    if data[:4] != HEADER:
        raise ValueError(f"bad header: {data[:4].hex()}")
    id_ = data[4]
    length = data[5] | (data[6] << 8)
    if 7 + length != len(data):
        raise ValueError(f"length field={length} ≠ actual={len(data) - 7}")
    if data[7] != INSTR_STATUS:
        raise ValueError(f"not a Status frame: instr=0x{data[7]:02X}")
    err = data[8]
    params = bytes(data[9 : 9 + length - 4])
    crc_lo, crc_hi = data[-2], data[-1]
    expected = _crc16(data[:-2])
    if (crc_lo | (crc_hi << 8)) != expected:
        raise ValueError(f"CRC mismatch: got {crc_hi:02x}{crc_lo:02x}, want {expected:04x}")
    return StatusPacket((id_, err, params))


_CRC_TABLE = [
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
]
