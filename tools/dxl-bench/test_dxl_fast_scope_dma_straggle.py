"""Scope capture for the DMA + straggle path.

Repeats the straggle chain 8 times back-to-back. Each rep advances the
snoop ring by ~170 B (host request + INJ slot + DUT slot); after ~3 reps
the ring (N=512) laps during a snoop window and DMA1_CH5 TC fires.

The TC handler's ring math is gated by `unexpected_byte_count` — that
counter must stay 0 for the chain CRC to be correct across the wrap.

Probe 2 (dbg) on any rep where the ring wraps should show an extra brief
pulse during the snoop window — the on_dma1_ch5_tc body bracketed by
dbg_high / dbg_low — in addition to the catchup-walk and fire-window
pulses already visible in the straggle test.

Sequence: clear counters, ARM scope, fire `REPS` chains, read counters,
clear again.

Invoke with `-s`:
  pytest tools/dxl-bench/test_dxl_fast_scope_dma_straggle.py -s --baud 3000000

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
PACKET_LENGTH = 1 + (2 + INJ_LEN) + (2 + DUT_LEN) + 2
REPS = 8


def _drain_all_stamps(injector):
    out = []
    while True:
        reply = injector.command("DRAIN")
        if reply == "EMPTY":
            return out
        assert reply.startswith("STAMP "), f"unexpected DRAIN reply: {reply!r}"
        _, tick_s, head_s = reply.split()
        out.append((int(tick_s), int(head_s)))


def test_fast_bulk_dut_last_dma_straggle_scope_capture(port, osc_id, injector, baud):
    clear_counters(port, osc_id)

    print("\n" + "=" * 60, flush=True)
    print(
        f"variant=dma+straggle  baud={baud}  INJ_ID={INJ_ID}  DUT_ID={osc_id}  "
        f"inj_len={INJ_LEN}  dut_len={DUT_LEN}  reps={REPS}",
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
    request = build_fast_bulk_read([(INJ_ID, 0, INJ_LEN), (osc_id, 0, DUT_LEN)])
    reply_lens = [INJ_LEN, DUT_LEN]

    total_extra_idle = 0
    bad_inj = 0
    for i in range(REPS):
        _drain_all_stamps(injector)
        reply = injector.command(f"ARM bytes={inj_bytes.hex()} after_idle={250 * 18}")
        assert reply == "OK", f"injector ARM rejected on rep {i}: {reply!r}"

        port.writePort(request)
        frame = read_status_frame(port.ser, timeout_s=0.5)
        assert frame is not None, f"no Status frame on rep {i}"
        slots = parse_fast_response(frame, slot_lengths=reply_lens)
        assert len(slots) == 2
        if slots[0].data != INJ_DATA:
            bad_inj += 1
        stamps = _drain_all_stamps(injector)
        if len(stamps) > 2:
            total_extra_idle += len(stamps) - 2
        time.sleep(0.005)

    print(f"\n  reps={REPS}  bad_inj={bad_inj}  total_extra_idle={total_extra_idle}", flush=True)
    print_counters("dma+straggle", read_counters(port, osc_id))
    clear_counters(port, osc_id)
