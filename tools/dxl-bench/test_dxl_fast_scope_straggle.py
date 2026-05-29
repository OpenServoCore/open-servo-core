"""Scope capture for the straggle path — INJ slot spans the catch-up window.

FastBulkRead with INJ_len=128, DUT_len=4. At 3 Mbaud:
  - INJ slot = 8 (header) + 2 + 128 = 138 B, ~460 µs on the wire.
  - fire_us  = 250 (RDT) + 138 × 3.33 ≈ 710 µs.
  - CatchupArmed CMP at fire_us − 150 ≈ 560 µs from request_end.
  - INJ has emitted ~93 B by the catch-up CMP; the remaining ~45 B straggle
    arrive between CatchupArmed and fire CMP.

Probe 2 (dbg) should show:
  - A wide catch-up pulse during the CatchupArmed body (~93 B × 0.21 µs CRC
    + walk overhead).
  - The fire pulse from `fire_now` through `patch_crc` (post-fire straggle
    walk over ~45 bytes + CRC patch).

Sequence: same as test_dxl_fast_scope.py — clear counters, ARM scope, fire,
read counters, clear again.

Invoke with `-s`:
  pytest tools/dxl-bench/test_dxl_fast_scope_straggle.py -s --baud 3000000

Remove this file once the scope sweep is done.
"""

import time

from dxl_link import clear_counters, print_counters, read_counters
from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    parse_fast_response,
    read_status_frame,
)

INJ_ID = 50
INJ_LEN = 128
DUT_LEN = 4
INJ_DATA = b"\xAA" * INJ_LEN
# Status overhead: instr(1) + INJ_slot(2 + INJ_LEN) + DUT_slot(2 + DUT_LEN) + crc(2).
PACKET_LENGTH = 1 + (2 + INJ_LEN) + (2 + DUT_LEN) + 2


def _drain_all_stamps(injector):
    out = []
    while True:
        reply = injector.command("DRAIN")
        if reply == "EMPTY":
            return out
        assert reply.startswith("STAMP "), f"unexpected DRAIN reply: {reply!r}"
        _, tick_s, head_s = reply.split()
        out.append((int(tick_s), int(head_s)))


def test_fast_bulk_dut_last_straggle_scope_capture(port, osc_id, injector, baud):
    clear_counters(port, osc_id)

    print("\n" + "=" * 60, flush=True)
    print(
        f"variant=straggle  baud={baud}  INJ_ID={INJ_ID}  DUT_ID={osc_id}  "
        f"inj_len={INJ_LEN}  dut_len={DUT_LEN}",
        flush=True,
    )
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

    port.writePort(build_fast_bulk_read([(INJ_ID, 0, INJ_LEN), (osc_id, 0, DUT_LEN)]))

    frame = read_status_frame(port.ser, timeout_s=0.5)
    assert frame is not None, "no coalesced Fast Status frame on the wire"

    slots = parse_fast_response(frame, slot_lengths=[INJ_LEN, DUT_LEN])
    assert len(slots) == 2
    assert slots[0].id == INJ_ID and slots[0].error == 0
    assert slots[0].data == INJ_DATA, (
        f"INJ slot data corrupted: got {slots[0].data[:8].hex()}…, want {INJ_DATA[:8].hex()}…"
    )
    assert slots[1].id == osc_id and slots[1].error == 0
    assert len(slots[1].data) == DUT_LEN

    stamps = _drain_all_stamps(injector)
    print(f"\nframe ({len(frame)} B): {frame[:32].hex()}…{frame[-16:].hex()}", flush=True)
    print(f"INJ slot data first 8: {slots[0].data[:8].hex()}  last 4: {slots[0].data[-4:].hex()}", flush=True)
    print(f"DUT slot data: {slots[1].data.hex()}", flush=True)
    print(f"IDLE stamps observed by injector ({len(stamps)}):", flush=True)
    for tick, head in stamps:
        print(f"  tick={tick:10d}  head={head:5d}", flush=True)
    if len(stamps) == 2:
        print("  → only req-end + frame-end stamps; no intermediate IDLE (good).", flush=True)
    elif len(stamps) > 2:
        print(f"  → {len(stamps) - 2} extra IDLE(s) — bus went idle between slots.", flush=True)

    print_counters("straggle", read_counters(port, osc_id))
    clear_counters(port, osc_id)
