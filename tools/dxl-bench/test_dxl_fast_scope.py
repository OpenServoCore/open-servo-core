"""One-shot scope-capture test for FastSyncRead with rev_b as the last slave.

Bench setup expected (per session notes):
  - rev_b on the DXL bus via FT232H
  - V203 dxl-pirate on the same bus, acting as slot 0
  - Scope ch1 on the data line, ch2 on rev_b's dbg pin
  - Scope trigger: ch2 rising edge, Single

Sequence (basic variant — no straggle, no DMA wrap):
  1. Clear TelemetryDxlLink counters.
  2. Print "ARM SCOPE NOW" and sleep 5 s.
  3. Drain pending bytes and injector IDLE stamps so the upcoming fire is
     the next bus event.
  4. ARM the injector to emit slot 0 (header + INJ body) 250 µs after the
     host request ends (one DXL RDT).
  5. Send FastSyncRead [INJ_ID, osc_id] with 4 B payload. rev_b detects
     itself as Last, snoops slot 0, fires slot 1 (own body + frame CRC).
  6. Read counters, print non-zero entries.
  7. Clear counters.

Invoke with `-s` so the "ARM SCOPE NOW" print is not buffered:
  pytest tools/dxl-bench/test_dxl_fast_scope.py -s

Remove this file once the scope sweep is done.
"""

import time

from dxl_link import clear_counters, print_counters, read_counters
from dxl_packet import (
    build_fast_first_bytes,
    build_fast_sync_read,
    parse_fast_response,
    read_status_frame,
)

INJ_ID = 50
LENGTH = 4
INJ_DATA = b"\x10\x11\x12\x13"
# Status overhead: instr(1) + N_slots * (err + id + data) + crc(2).
PACKET_LENGTH = 1 + 2 * (2 + LENGTH) + 2


def _drain_all_stamps(injector):
    out = []
    while True:
        reply = injector.command("DRAIN")
        if reply == "EMPTY":
            return out
        assert reply.startswith("STAMP "), f"unexpected DRAIN reply: {reply!r}"
        _, tick_s, head_s = reply.split()
        out.append((int(tick_s), int(head_s)))


def test_fast_sync_dut_last_scope_capture(port, osc_id, injector, baud):
    clear_counters(port, osc_id)

    print("\n" + "=" * 60, flush=True)
    print(f"variant=basic  baud={baud}  INJ_ID={INJ_ID}  DUT_ID={osc_id}  length={LENGTH}", flush=True)
    print("ARM SCOPE NOW (ch2 rising edge, Single). Firing in 5 s...", flush=True)
    print("=" * 60, flush=True)
    time.sleep(5.0)

    while port.ser.in_waiting:
        port.ser.read(port.ser.in_waiting)
    _drain_all_stamps(injector)

    inj_bytes = build_fast_first_bytes(
        packet_length=PACKET_LENGTH, err=0, slot_id=INJ_ID, data=INJ_DATA,
    )
    reply = injector.command(f"ARM bytes={inj_bytes.hex()} after_idle={250 * 18}")
    assert reply == "OK", f"injector ARM rejected: {reply!r}"

    port.writePort(build_fast_sync_read(addr=0, length=LENGTH, ids=[INJ_ID, osc_id]))

    frame = read_status_frame(port.ser, timeout_s=0.2)
    assert frame is not None, "no coalesced Fast Status frame on the wire"

    slots = parse_fast_response(frame, slot_lengths=[LENGTH, LENGTH])
    assert len(slots) == 2
    assert slots[0].id == INJ_ID and slots[0].error == 0
    assert slots[0].data == INJ_DATA, (
        f"INJ slot 0 data corrupted: got {slots[0].data.hex()}, want {INJ_DATA.hex()}"
    )
    assert slots[1].id == osc_id and slots[1].error == 0
    assert len(slots[1].data) == LENGTH

    stamps = _drain_all_stamps(injector)
    print(f"\nframe ({len(frame)} B): {frame.hex()}", flush=True)
    print(f"INJ slot data: {slots[0].data.hex()}", flush=True)
    print(f"DUT slot data: {slots[1].data.hex()}", flush=True)
    print(f"IDLE stamps observed by injector ({len(stamps)}):", flush=True)
    for tick, head in stamps:
        print(f"  tick={tick:10d}  head={head:5d}", flush=True)
    if len(stamps) == 2:
        print("  → only req-end + frame-end stamps; no intermediate IDLE (good).", flush=True)
    elif len(stamps) > 2:
        print(f"  → {len(stamps) - 2} extra IDLE(s) — bus went idle between slots.", flush=True)

    print_counters("basic", read_counters(port, osc_id))
    clear_counters(port, osc_id)
