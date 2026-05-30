"""Mirror of tinyboot/lib/protocol/src/{frame,crc}.rs for Info + Reset only."""

from dataclasses import dataclass

SYNC = bytes([0xAA, 0x55])

CMD_INFO = 0x00
CMD_RESET = 0x04

STATUS_REQUEST = 0x00
STATUS_OK = 0x01

RESET_FLAG_BOOTLOADER = 0x01

HEADER_LEN = 10
CRC_INIT = 0xFFFF


def crc16(data: bytes, crc: int = CRC_INIT) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def _build(cmd: int, status: int, addr: int, flags: int, data: bytes = b"") -> bytes:
    body = bytearray(SYNC)
    body.append(cmd)
    body.append(status)
    body.append(addr & 0xFF)
    body.append((addr >> 8) & 0xFF)
    body.append((addr >> 16) & 0xFF)
    body.append(flags)
    body.append(len(data) & 0xFF)
    body.append((len(data) >> 8) & 0xFF)
    body.extend(data)
    crc = crc16(bytes(body))
    body.append(crc & 0xFF)
    body.append((crc >> 8) & 0xFF)
    return bytes(body)


def build_info_request() -> bytes:
    return _build(CMD_INFO, STATUS_REQUEST, 0, 0)


def build_reset_request(bootloader: bool = False) -> bytes:
    flags = RESET_FLAG_BOOTLOADER if bootloader else 0
    return _build(CMD_RESET, STATUS_REQUEST, 0, flags)


@dataclass
class InfoResponse:
    capacity: int
    erase_size: int
    boot_version: int
    app_version: int
    mode: int


def parse_info_response(frame: bytes) -> InfoResponse:
    sync_at = frame.find(SYNC)
    if sync_at < 0:
        raise ValueError("SYNC AA55 not found in frame")
    frame = frame[sync_at:]
    if len(frame) < HEADER_LEN:
        raise ValueError(f"frame too short: {len(frame)} bytes")
    header = frame[:HEADER_LEN]
    cmd, status = header[2], header[3]
    length = header[8] | (header[9] << 8)
    if cmd != CMD_INFO:
        raise ValueError(f"bootloader reply cmd=0x{cmd:02X}, want INFO")
    if status != STATUS_OK:
        raise ValueError(f"bootloader reply status=0x{status:02X}, want OK")
    if length != 12:
        raise ValueError(f"InfoData len={length}, want 12")
    if len(frame) < HEADER_LEN + length + 2:
        raise ValueError(f"frame truncated: {len(frame)} bytes")
    data = frame[HEADER_LEN:HEADER_LEN + length]
    crc_bytes = frame[HEADER_LEN + length:HEADER_LEN + length + 2]
    expected = crc16(header + data)
    actual = crc_bytes[0] | (crc_bytes[1] << 8)
    if expected != actual:
        raise ValueError(f"bootloader CRC mismatch: got 0x{actual:04X}, want 0x{expected:04X}")
    return InfoResponse(
        capacity=int.from_bytes(data[0:4], "little"),
        erase_size=int.from_bytes(data[4:6], "little"),
        boot_version=int.from_bytes(data[6:8], "little"),
        app_version=int.from_bytes(data[8:10], "little"),
        mode=int.from_bytes(data[10:12], "little"),
    )
