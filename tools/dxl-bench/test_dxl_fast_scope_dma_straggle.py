"""Scope capture for the DMA + straggle path.

Repeats the straggle chain `REPS` times back-to-back. Each rep advances the
V006 snoop ring by ~170 B (host request + INJ slot + DUT slot); after ~3 reps
the ring (N=512) laps during a snoop window and DMA1_CH5 TC fires.

The TC handler's ring math is gated by `unexpected_byte_count` — that
counter must stay 0 for the chain CRC to be correct across the wrap.

Probe 2 (dbg) on any rep where the ring wraps should show an extra brief
pulse during the snoop window — the on_dma1_ch5_tc body bracketed by
dbg_high / dbg_low — in addition to the catchup-walk and fire-window
pulses already visible in the straggle test.

Invoke with `-s`:
  pytest tools/dxl-bench/test_dxl_fast_scope_dma_straggle.py -s --baud 3000000
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
REPS = 8


def test_fast_bulk_dut_last_dma_straggle_scope_capture(pirate, osc_id, baud):
    clear_counters(pirate, osc_id)

    print("\n" + "=" * 60, flush=True)
    print(
        f"variant=dma+straggle  baud={baud}  INJ_ID={INJ_ID}  DUT_ID={osc_id}  "
        f"inj_len={INJ_LEN}  dut_len={DUT_LEN}  reps={REPS}",
        flush=True,
    )
    print("ARM SCOPE NOW (ch2 rising edge, Single). Firing in 5 s...", flush=True)
    print("=" * 60, flush=True)
    time.sleep(5.0)

    inj_bytes = build_fast_first_bytes(
        packet_length=PACKET_LENGTH, err=0, slot_id=INJ_ID, data=INJ_DATA,
    )
    request = build_fast_bulk_read([(INJ_ID, 0, INJ_LEN), (osc_id, 0, DUT_LEN)])
    reply_lens = [INJ_LEN, DUT_LEN]

    total_extra_idle = 0
    bad_inj = 0
    for i in range(REPS):
        pirate.drain_stamps()
        b0 = pirate.bytes_count()
        pirate.arm(inj_bytes, after_idle_ticks=250 * 18)
        pirate.master(request)
        time.sleep(0.05)
        b1 = pirate.bytes_count()
        total = b1 - b0
        assert total <= 256, f"rep {i}: frame exceeds RX_BUF horizon ({total} B)"
        all_rx = pirate.rx_range(b0, total)
        frame = all_rx[len(request):]
        assert len(frame) >= 11, f"rep {i}: no Status frame"
        slots = parse_fast_response(frame, slot_lengths=reply_lens)
        assert len(slots) == 2
        if slots[0].data != INJ_DATA:
            bad_inj += 1
        stamps = pirate.drain_stamps()
        if len(stamps) > 1:
            total_extra_idle += len(stamps) - 1
        time.sleep(0.005)

    print(f"\n  reps={REPS}  bad_inj={bad_inj}  total_extra_idle={total_extra_idle}", flush=True)
    print_counters("dma+straggle", read_counters(pirate, osc_id))
    clear_counters(pirate, osc_id)
