"""USB-CDC client for the dxl-pirate.

The pirate is the bench's sole bus actor: master TX, listener, and IDLE-stamp
ring all in one V203 firmware. See `tools/dxl-pirate/src/proto.rs` for the
ASCII grammar this class wraps.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial


def _hex(data: bytes) -> str:
    return data.hex()


@dataclass(frozen=True)
class Plain:
    tick: int
    head: int


@dataclass(frozen=True)
class Round:
    req: int
    first: int
    last: int
    head: int


IdleStamp = Plain | Round


class PirateError(RuntimeError):
    pass


class Pirate:
    def __init__(self, port_path: str, cdc_timeout_s: float = 0.5):
        # CDC's nominal 115200 is purely cosmetic; the wire baud is whatever
        # `BAUD <bps>` was last set to.
        self.ser = serial.Serial(port_path, 115200, timeout=cdc_timeout_s)
        self.port_path = port_path

    def close(self) -> None:
        self.ser.close()

    def _send(self, line: str) -> None:
        self.ser.reset_input_buffer()
        self.ser.write(line.encode() + b"\n")
        self.ser.flush()

    def _readline(self) -> str:
        line = self.ser.readline()
        if not line:
            raise TimeoutError(f"pirate read timed out on {self.port_path}")
        return line.decode(errors="replace").rstrip()

    def command(self, line: str) -> str:
        """Send one line, return one stripped reply. Use for OK/value/STAMP/etc.
        Don't use for XFER/RX (their REPLY hex stream may span multiple CDC
        packets and is concatenated transparently by readline())."""
        self._send(line)
        return self._readline()

    def expect_ok(self, line: str) -> None:
        reply = self.command(line)
        if reply != "OK":
            raise PirateError(f"{line!r} → {reply!r}")

    def set_baud(self, bps: int) -> None:
        self.expect_ok(f"BAUD {bps}")

    def tick(self) -> int:
        reply = self.command("TICK?")
        if not reply.startswith("TICK "):
            raise PirateError(f"TICK? → {reply!r}")
        return int(reply.split()[1])

    def bytes_count(self) -> int:
        reply = self.command("BYTES")
        if not reply.startswith("BYTES "):
            raise PirateError(f"BYTES → {reply!r}")
        return int(reply.split()[1])

    def hz_per_us(self) -> int:
        reply = self.command("HZ")
        if not reply.startswith("HZ "):
            raise PirateError(f"HZ → {reply!r}")
        return int(reply.split()[1])

    def master(self, data: bytes) -> None:
        self.expect_ok(f"MASTER bytes={_hex(data)}")

    def arm(self, data: bytes, after_idle_ticks: int) -> None:
        self.expect_ok(f"ARM bytes={_hex(data)} after_idle={after_idle_ticks}")

    def fire(self, data: bytes, at_tick: int) -> None:
        self.expect_ok(f"FIRE bytes={_hex(data)} at={at_tick}")

    def last_fired(self) -> int:
        """Low-32 SysTick from inject's `FIRED_TICK_LO`. After a FIRE, equals
        `at_tick & 0xFFFFFFFF` on the scheduled path or `now-on-device` if the
        TIM4-OPM schedule was missed (immediate-fire fallback)."""
        reply = self.command("LAST?")
        if not reply.startswith("LAST "):
            raise PirateError(f"LAST? → {reply!r}")
        return int(reply.split()[1])

    def xfer(self, data: bytes, reply_us: int) -> bytes | None:
        """Fire `data` as master and wait up to `reply_us` for the slave's
        end-of-frame IDLE. Returns the slave's reply bytes, or None on
        timeout (no reply within the window)."""
        self._send(f"XFER bytes={_hex(data)} reply_us={reply_us}")
        reply = self._readline()
        if reply == "NOREPLY":
            return None
        if reply.startswith("REPLY "):
            hex_str = reply[6:]
            return bytes.fromhex(hex_str) if hex_str else b""
        if reply.startswith("ERR "):
            raise PirateError(f"XFER error: {reply}")
        raise PirateError(f"unexpected XFER response: {reply!r}")

    def rx_range(self, from_addr: int, length: int) -> bytes:
        """Pull `length` raw bytes from the RX DMA ring starting at absolute
        byte-count address `from_addr`. Caller is responsible for staying
        within the last RX_BUF_LEN (=256) bytes of the current head."""
        if length == 0:
            return b""
        self._send(f"RX from={from_addr & 0xFFFFFFFF} len={length}")
        reply = self._readline()
        if reply.startswith("REPLY "):
            hex_str = reply[6:]
            return bytes.fromhex(hex_str) if hex_str else b""
        if reply.startswith("ERR "):
            raise PirateError(f"RX error: {reply}")
        raise PirateError(f"unexpected RX response: {reply!r}")

    def drain_stamps(self) -> list[IdleStamp]:
        out: list[IdleStamp] = []
        while True:
            reply = self.command("DRAIN")
            if reply == "EMPTY":
                return out
            parts = reply.split()
            tag = parts[0]
            if tag == "STAMP" and len(parts) == 3:
                out.append(Plain(tick=int(parts[1]), head=int(parts[2])))
            elif tag == "ROUND" and len(parts) == 5:
                out.append(Round(
                    req=int(parts[1]), first=int(parts[2]),
                    last=int(parts[3]), head=int(parts[4]),
                ))
            else:
                raise PirateError(f"unexpected DRAIN entry: {reply!r}")

    def wait_quiet(self, window_s: float = 0.025) -> None:
        """Block until the bus has been silent (no new RX bytes) for
        `window_s`. Used between tests to absorb stragglers from a previous
        reply that arrived after that test's read window closed."""
        deadline = time.monotonic() + 10 * window_s
        prev = self.bytes_count()
        silent_since = None
        while time.monotonic() < deadline:
            time.sleep(0.005)
            now = self.bytes_count()
            if now != prev:
                prev = now
                silent_since = None
            elif silent_since is None:
                silent_since = time.monotonic()
            elif time.monotonic() - silent_since >= window_s:
                return
