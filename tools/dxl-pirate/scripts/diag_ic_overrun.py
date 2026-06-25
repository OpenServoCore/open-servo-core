#!/usr/bin/env python3
"""Reproduce tool-hsi-conv's ic_overrun and dump BTRACE.

Reads a 128-byte block from servo id=165 at 1Mbaud, then drains BTRACE to
see the walker's per-call workload (edges_consumed/bytes_emitted/duration).
Walker duration = tim2_cnt_exit - tim2_cnt_entry in 144MHz ticks (1 tick =
~7ns). Used to validate "PLL classifier too slow" theory.
"""
from __future__ import annotations

import argparse
import struct
import sys
import time
from dataclasses import dataclass

import serial


def crc_umts(buf: bytes) -> int:
    crc = 0
    for b in buf:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def build_read(id_: int, addr: int, length: int) -> bytes:
    # DXL 2.0 Read frame:
    # FF FF FD 00 ID LEN_L LEN_H 0x02 ADDR_L ADDR_H LEN_L LEN_H CRC_L CRC_H
    pkt_len = 7  # instr(1) + addr(2) + len(2) + crc(2)
    body = bytes([
        0xFF, 0xFF, 0xFD, 0x00,
        id_,
        pkt_len & 0xFF, (pkt_len >> 8) & 0xFF,
        0x02,  # Read
        addr & 0xFF, (addr >> 8) & 0xFF,
        length & 0xFF, (length >> 8) & 0xFF,
    ])
    crc = crc_umts(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_reboot(id_: int) -> bytes:
    pkt_len = 3  # instr + crc
    body = bytes([0xFF, 0xFF, 0xFD, 0x00, id_, pkt_len & 0xFF, (pkt_len >> 8) & 0xFF, 0x08])
    crc = crc_umts(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


@dataclass
class Trace:
    phase: int
    intfr_post_clear: int
    cnt_entry: int
    cnt_exit: int
    pending: int
    edges: int
    bytes_: int
    falling_total: int
    rx_total: int

    @property
    def duration(self) -> int:
        return (self.cnt_exit - self.cnt_entry) & 0xFFFF

    @property
    def phase_str(self) -> str:
        return {0: "IDLE", 1: "RX_HT", 2: "RX_TC", 3: "IC_HT", 4: "IC_TC"}.get(self.phase, f"?{self.phase}")


def readline(ser: serial.Serial, timeout: float = 1.0) -> bytes:
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            return bytes(buf)
        if b == b"\r":
            continue
        buf += b
    raise TimeoutError(f"readline timeout, partial={buf!r}")


def cmd(ser: serial.Serial, line: str, timeout: float = 1.0) -> str:
    ser.write((line + "\n").encode())
    return readline(ser, timeout).decode("ascii", errors="replace")


class DesyncError(RuntimeError):
    pass


def drain_bbatch(ser: serial.Serial, count: int = 64) -> list[tuple[int, int, int]]:
    """Send BBATCH; parse binary frame. Returns [(tick, byte, flags)]."""
    ser.write(f"BBATCH {count}\n".encode())
    # Header: 0xA5 0x5A count:u16LE
    hdr = ser.read(4)
    if len(hdr) < 4 or hdr[0] != 0xA5 or hdr[1] != 0x5A:
        # Maybe ERR — read rest of line
        rest = bytearray(hdr)
        while not rest.endswith(b'\n'):
            b = ser.read(1)
            if not b:
                break
            rest += b
        if b"desync" in rest:
            raise DesyncError(rest.decode(errors="replace").strip())
        raise RuntimeError(f"BBATCH bad header: {bytes(rest)!r}")
    n = struct.unpack("<H", hdr[2:4])[0]
    body = ser.read(n * 6)
    if len(body) != n * 6:
        raise RuntimeError(f"BBATCH short body: want {n*6}, got {len(body)}")
    out = []
    for i in range(n):
        off = i * 6
        tick = struct.unpack("<I", body[off:off+4])[0]
        out.append((tick, body[off+4], body[off+5]))
    return out


def parse_btrace_line(line: str) -> Trace:
    parts = line.split()
    # BTRACE phase intfr cnt_entry cnt_exit pending edges bytes falling_total rx_total
    assert parts[0] == "BTRACE"
    return Trace(
        phase=int(parts[1]),
        intfr_post_clear=int(parts[2]),
        cnt_entry=int(parts[3]),
        cnt_exit=int(parts[4]),
        pending=int(parts[5]),
        edges=int(parts[6]),
        bytes_=int(parts[7]),
        falling_total=int(parts[8]),
        rx_total=int(parts[9]),
    )


def drain_btrace(ser: serial.Serial, max_records: int = 256) -> list[Trace]:
    out = []
    for _ in range(max_records):
        line = cmd(ser, "BTRACE")
        if line == "EMPTY":
            break
        out.append(parse_btrace_line(line))
    return out


def parse_status(line: str) -> tuple[int, int, int, str, int]:
    # STATUS <baud> <avail> <desynced 0/1> <cause> <last_tick>
    parts = line.split()
    assert parts[0] == "STATUS", line
    return int(parts[1]), int(parts[2]), int(parts[3]), parts[4], int(parts[5])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/cu.usbmodembench_11")
    ap.add_argument("--id", type=int, default=165)
    ap.add_argument("--length", type=int, default=128)
    ap.add_argument("--addr", type=int, default=0)
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--reboot", action="store_true")
    ap.add_argument("--repeats", type=int, default=1)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.5)
    try:
        # 1) Quiesce + set baud + clear trace
        print(cmd(ser, "RESET"))
        print(cmd(ser, f"BAUD {args.baud}"))
        time.sleep(0.05)
        print(cmd(ser, "BTRACECLEAR"))

        # 2) Drain pre-existing stamps
        drained = drain_bbatch(ser, 255)
        print(f"pre-drain: {len(drained)} stamps")
        # Drain again to be sure
        while drain_bbatch(ser, 255):
            pass

        if args.reboot:
            print("rebooting chip...")
            rb = build_reboot(args.id)
            print(cmd(ser, f"MASTER bytes={rb.hex()}"))
            time.sleep(0.6)
            # Drain reboot reply
            while drain_bbatch(ser, 255):
                pass
            print(cmd(ser, f"BAUD {args.baud}"))
            time.sleep(0.05)
            while drain_bbatch(ser, 255):
                pass
            print(cmd(ser, "BTRACECLEAR"))

        # 3) Send long Read N times; race the walker
        rd = build_read(args.id, args.addr, args.length)
        print(f"sending Read addr={args.addr} len={args.length} x {args.repeats}")
        desynced_at = None
        for i in range(args.repeats):
            print(cmd(ser, f"MASTER bytes={rd.hex()}"))

            # 4) Poll STATUS + bbatch until silence or desync
            start = time.time()
            all_stamps: list[tuple[int, int, int]] = []
            last_status = None
            last_byte_t = time.time()
            while time.time() - start < 0.3:
                try:
                    batch = drain_bbatch(ser, 64)
                except DesyncError as e:
                    desynced_at = time.time() - start
                    print(f"!! DESYNC repeat={i} at t+{desynced_at*1000:.1f}ms via BBATCH: {e}")
                    break
                if batch:
                    all_stamps.extend(batch)
                    last_byte_t = time.time()
                else:
                    if time.time() - last_byte_t > 0.010:
                        break
                    time.sleep(0.001)
                st = cmd(ser, "STATUS")
                try:
                    baud, avail, des, cause, last_tick = parse_status(st)
                    last_status = (baud, avail, des, cause, last_tick)
                    if des and desynced_at is None:
                        desynced_at = time.time() - start
                        print(f"!! DESYNC repeat={i} at t+{desynced_at*1000:.1f}ms: "
                              f"cause={cause} avail={avail} last_tick={last_tick}")
                        break
                except Exception:
                    pass

            # Verify echo bytes match the request
            echo_ok = True
            if len(all_stamps) >= len(rd):
                for j, expected in enumerate(rd):
                    if all_stamps[j][1] != expected:
                        echo_ok = False
                        print(f"  repeat={i}: ECHO MISMATCH at byte {j}: got 0x{all_stamps[j][1]:02X}, expected 0x{expected:02X}")
                        break
            head_bytes = bytes(s[1] for s in all_stamps[:min(20, len(all_stamps))]).hex()
            print(f"  repeat={i}: {len(all_stamps)} stamps echo_ok={echo_ok} head={head_bytes}")
            if desynced_at is not None:
                break

        # 5) Dump BTRACE
        traces = drain_btrace(ser, 256)
        print(f"\n{len(traces)} BTRACE records:")
        print(f"  {'phase':6s} {'pend':4s} {'edges':5s} {'bytes':5s} {'dur_us':7s} {'cyc/byte':9s} {'ft':8s} {'rt':8s} {'intfr':5s}")
        for t in traces:
            cyc_per_byte = (t.duration / t.bytes_) if t.bytes_ > 0 else 0
            us = t.duration / 144.0
            print(f"  {t.phase_str:6s} {t.pending:4d} {t.edges:5d} {t.bytes_:5d} {us:7.2f} {cyc_per_byte:9.1f} {t.falling_total:8d} {t.rx_total:8d} {t.intfr_post_clear:5d}")

        # Summary
        if traces:
            total_dur = sum(t.duration for t in traces)
            total_bytes = sum(t.bytes_ for t in traces)
            total_edges = sum(t.edges for t in traces)
            print(f"\nSummary: total walker time {total_dur/144.0:.1f}us, "
                  f"bytes={total_bytes}, edges={total_edges}")
            if total_bytes:
                print(f"  avg cyc/byte: {total_dur/total_bytes:.1f}")
            # Peak per-call duration
            longest = max(traces, key=lambda t: t.duration)
            print(f"  longest call: {longest.duration/144.0:.1f}us "
                  f"(phase={longest.phase_str} edges={longest.edges} bytes={longest.bytes_})")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
