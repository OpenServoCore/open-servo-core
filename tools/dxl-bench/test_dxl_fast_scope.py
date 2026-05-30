"""Scope-capture FastSyncRead with V006 as the last slave.

The pirate is both bus master AND fake foreign slave (INJ slot 0): MASTER fires
the host's FastSyncRead request, then ARM emits the INJ-slot bytes 250 µs
(=1 RDT) after the master IDLE. V006 sees slot 0 end + coalesces its own slot 1.

Bench setup:
  - V203 dxl-pirate on the DXL bus + V006 DUT
  - Scope ch1 on the data line, ch2 on V006's debug pin
  - Scope trigger: ch2 rising edge, Single

Coalesce check: after the master IDLE is suppressed, every additional IDLE on
the bus produces one stamp. A clean coalesced [INJ+DUT] burst yields 1 stamp
(Round at burst end); a stragglier where the bus quiets between INJ and DUT
yields ≥2 stamps.

Invoke with `-s` so the "ARM SCOPE NOW" print is unbuffered:
  pytest tools/dxl-bench/test_dxl_fast_scope.py -s
"""

import time

from dxl_link import clear_counters, print_counters, read_counters
from dxl_packet import (
    build_fast_first_bytes,
    build_fast_sync_read,
    parse_fast_response,
)

INJ_ID = 50
LENGTH = 4
INJ_DATA = b"\x10\x11\x12\x13"
# Status overhead: instr(1) + N_slots * (err + id + data) + crc(2).
PACKET_LENGTH = 1 + 2 * (2 + LENGTH) + 2


def test_fast_sync_dut_last_scope_capture(pirate, osc_id, baud):
    clear_counters(pirate, osc_id)

    print("\n" + "=" * 60, flush=True)
    print(f"variant=basic  baud={baud}  INJ_ID={INJ_ID}  DUT_ID={osc_id}  length={LENGTH}", flush=True)
    print("ARM SCOPE NOW (ch2 rising edge, Single). Firing in 5 s...", flush=True)
    print("=" * 60, flush=True)
    time.sleep(5.0)

    pirate.drain_stamps()
    b0 = pirate.bytes_count()

    inj_bytes = build_fast_first_bytes(
        packet_length=PACKET_LENGTH, err=0, slot_id=INJ_ID, data=INJ_DATA,
    )
    req = build_fast_sync_read(addr=0, length=LENGTH, ids=[INJ_ID, osc_id])

    pirate.arm(inj_bytes, after_idle_ticks=250 * 18)
    pirate.master(req)

    time.sleep(0.05)
    b1 = pirate.bytes_count()
    all_rx = pirate.rx_range(b0, b1 - b0)
    # rx layout: [master TX echo] + [INJ slot echo from ARM] + [DUT slot from V006]
    frame_start = len(req)
    frame = all_rx[frame_start:]
    assert len(frame) >= 11, f"no coalesced Fast Status frame on the wire: {all_rx.hex()}"

    slots = parse_fast_response(frame, slot_lengths=[LENGTH, LENGTH])
    assert len(slots) == 2
    assert slots[0].id == INJ_ID and slots[0].error == 0
    assert slots[0].data == INJ_DATA, (
        f"INJ slot 0 data corrupted: got {slots[0].data.hex()}, want {INJ_DATA.hex()}"
    )
    assert slots[1].id == osc_id and slots[1].error == 0
    assert len(slots[1].data) == LENGTH

    stamps = pirate.drain_stamps()
    print(f"\nframe ({len(frame)} B): {frame.hex()}", flush=True)
    print(f"INJ slot data: {slots[0].data.hex()}", flush=True)
    print(f"DUT slot data: {slots[1].data.hex()}", flush=True)
    print(f"IDLE stamps observed by pirate ({len(stamps)}):", flush=True)
    for s in stamps:
        print(f"  {s}", flush=True)
    if len(stamps) == 1:
        print("  → single burst stamp; INJ + DUT coalesced (good).", flush=True)
    else:
        print(f"  → {len(stamps) - 1} extra IDLE(s) — bus went idle between slots.", flush=True)

    print_counters("basic", read_counters(pirate, osc_id))
    clear_counters(pirate, osc_id)
