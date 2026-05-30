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

Invoke with `-s`:
  pytest tools/dxl-bench/test_dxl_fast_scope_straggle.py -s --baud 3000000
"""

import time

from dxl_link import clear_counters, print_counters, read_counters
from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    parse_fast_response,
)

INJ_ID = 50
INJ_LEN = 128
DUT_LEN = 4
INJ_DATA = b"\xAA" * INJ_LEN
PACKET_LENGTH = 1 + (2 + INJ_LEN) + (2 + DUT_LEN) + 2


def test_fast_bulk_dut_last_straggle_scope_capture(pirate, osc_id, baud):
    clear_counters(pirate, osc_id)

    print("\n" + "=" * 60, flush=True)
    print(
        f"variant=straggle  baud={baud}  INJ_ID={INJ_ID}  DUT_ID={osc_id}  "
        f"inj_len={INJ_LEN}  dut_len={DUT_LEN}",
        flush=True,
    )
    print("ARM SCOPE NOW (ch2 rising edge, Single). Firing in 5 s...", flush=True)
    print("=" * 60, flush=True)
    time.sleep(5.0)

    pirate.drain_stamps()
    b0 = pirate.bytes_count()

    inj_bytes = build_fast_first_bytes(
        packet_length=PACKET_LENGTH, err=0, slot_id=INJ_ID, data=INJ_DATA,
    )
    req = build_fast_bulk_read([(INJ_ID, 0, INJ_LEN), (osc_id, 0, DUT_LEN)])

    pirate.arm(inj_bytes, after_idle_ticks=250 * 18)
    pirate.master(req)

    time.sleep(0.1)
    b1 = pirate.bytes_count()
    # RX_BUF is 256 B; req + inj_bytes + dut frame may exceed it. Pull in
    # chunks so we don't reach back past the wrap horizon.
    total = b1 - b0
    assert total <= 256, f"frame exceeds RX_BUF horizon: {total} B"
    all_rx = pirate.rx_range(b0, total)
    frame = all_rx[len(req):]
    assert len(frame) >= 11, f"no coalesced Fast Status frame: {all_rx.hex()}"

    slots = parse_fast_response(frame, slot_lengths=[INJ_LEN, DUT_LEN])
    assert len(slots) == 2
    assert slots[0].id == INJ_ID and slots[0].error == 0
    assert slots[0].data == INJ_DATA, (
        f"INJ slot data corrupted: got {slots[0].data[:8].hex()}…, want {INJ_DATA[:8].hex()}…"
    )
    assert slots[1].id == osc_id and slots[1].error == 0
    assert len(slots[1].data) == DUT_LEN

    stamps = pirate.drain_stamps()
    print(f"\nframe ({len(frame)} B): {frame[:32].hex()}…{frame[-16:].hex()}", flush=True)
    print(f"INJ slot data first 8: {slots[0].data[:8].hex()}  last 4: {slots[0].data[-4:].hex()}", flush=True)
    print(f"DUT slot data: {slots[1].data.hex()}", flush=True)
    print(f"IDLE stamps observed by pirate ({len(stamps)}):", flush=True)
    for s in stamps:
        print(f"  {s}", flush=True)
    if len(stamps) == 1:
        print("  → single burst stamp; INJ + DUT coalesced (good).", flush=True)
    else:
        print(f"  → {len(stamps) - 1} extra IDLE(s) — bus went idle between slots.", flush=True)

    print_counters("straggle", read_counters(pirate, osc_id))
    clear_counters(pirate, osc_id)
