"""Bench tests that lean on the dxl-bus-injector.

The injector gives us two capabilities the host alone can't match:

  * **Listen.** Every IDLE on the wire is timestamped (HCLK/8 = 18 MHz ticks)
    into a small SPSC ring the host drains via `DRAIN`. Sub-byte-time
    visibility into when the DUT actually starts transmitting; used by the
    solo-DUT jitter test to verify the <= 1 byte time spec.

  * **Inject.** `ARM bytes=<hex> after_idle=<ticks>` latches a raw byte
    payload and fires it `after_idle` ticks after the next IDLE event. Lets
    us play a real other-slave slot in a coalesced Fast Sync/Bulk Read,
    timed off request-end without an unreachable USB round-trip. Used by
    the DUT-as-Last coalesced tests.

Requires injector firmware with the ARM command (see
`tools/dxl-bus-injector/src/proto.rs`). Reflash with `cargo run --release`
from that directory after firmware changes.
"""

import time

from dxl_packet import (
    build_fast_bulk_read,
    build_fast_first_bytes,
    build_fast_sync_read,
    parse_fast_response,
    read_status_frame,
)

INJ_ID = 50  # arbitrary, distinct from osc_id and the FOREIGN_A/B sentinels


def _drain_all_stamps(injector) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    while True:
        reply = injector.command("DRAIN")
        if reply == "EMPTY":
            return out
        assert reply.startswith("STAMP "), f"unexpected DRAIN reply: {reply!r}"
        _, tick_s, head_s = reply.split()
        out.append((int(tick_s), int(head_s)))


def _hz_per_us(injector) -> int:
    reply = injector.command("HZ")
    assert reply.startswith("HZ "), f"unexpected HZ reply: {reply!r}"
    return int(reply.split()[1])


def _tick_delta(later: int, earlier: int) -> int:
    # SysTick CNT is u32 free-running (~239 s period at 18 MHz); a plain modular
    # subtract gives the elapsed ticks as long as `later` follows `earlier`
    # within that window — true for everything we measure here.
    return (later - earlier) & 0xFFFF_FFFF


def _head_delta(later: int, earlier: int) -> int:
    return (later - earlier) & 0xFFFF


def test_fast_sync_solo_observed_by_injector(port, osc_id, injector):
    """Scaffolding: the injector should see one IDLE for the host request and a
    second for the DUT's Fast response, with the response stamp's byte-count
    delta matching the on-wire frame length. Establishes the pipeline the
    jitter test below depends on."""
    _drain_all_stamps(injector)

    req = build_fast_sync_read(addr=0, length=2, ids=[osc_id])
    port.writePort(req)
    frame = read_status_frame(port.ser, timeout_s=0.1)
    assert frame is not None, "expected Fast Status frame"
    slots = parse_fast_response(frame, slot_lengths=[2])
    assert len(slots) == 1 and slots[0].id == osc_id

    time.sleep(0.01)  # let the trailing IDLE land in the stamp ring
    stamps = _drain_all_stamps(injector)
    assert len(stamps) == 2, (
        f"expected 2 IDLE stamps (req end + resp end), got {len(stamps)}: {stamps}"
    )

    req_tick, req_head = stamps[0]
    resp_tick, resp_head = stamps[1]
    resp_bytes = _head_delta(resp_head, req_head)
    assert resp_bytes == len(frame), (
        f"injector observed {resp_bytes} response bytes, "
        f"host received {len(frame)}: stamps={stamps} frame={frame.hex()}"
    )
    assert _tick_delta(resp_tick, req_tick) > 0, (
        f"response stamp not after request stamp: req={req_tick} resp={resp_tick}"
    )


def test_fast_sync_solo_slot0_jitter_under_byte_time(port, osc_id, injector, baud):
    """Slot-0 fire jitter must stay within one byte time across repeated
    requests at the same baud (reference_dxl_fast_sync_semantics). Measure
    response-start vs request-end across N trials and assert (max - min) of
    that latency is below one byte time."""
    n_trials = 30
    payload_len = 2
    req = build_fast_sync_read(addr=0, length=payload_len, ids=[osc_id])

    ticks_per_us = _hz_per_us(injector)
    byte_time_us = 10 * 1_000_000 / baud
    byte_time_ticks = round(byte_time_us * ticks_per_us)

    _drain_all_stamps(injector)
    for _ in range(n_trials):
        port.writePort(req)
        frame = read_status_frame(port.ser, timeout_s=0.1)
        assert frame is not None, "expected Fast Status frame"
        time.sleep(0.005)
    time.sleep(0.01)

    stamps = _drain_all_stamps(injector)
    assert len(stamps) == 2 * n_trials, (
        f"expected {2 * n_trials} stamps for {n_trials} trials, got {len(stamps)}"
    )

    latencies_ticks: list[int] = []
    for i in range(n_trials):
        req_tick, req_head = stamps[2 * i]
        resp_tick, resp_head = stamps[2 * i + 1]
        resp_bytes = _head_delta(resp_head, req_head)
        # Stamp fires at end-of-response; back-compute start-of-response.
        resp_end_latency = _tick_delta(resp_tick, req_tick)
        resp_start_latency = resp_end_latency - resp_bytes * byte_time_ticks
        latencies_ticks.append(resp_start_latency)

    jitter_ticks = max(latencies_ticks) - min(latencies_ticks)
    jitter_us = jitter_ticks / ticks_per_us
    min_us = min(latencies_ticks) / ticks_per_us
    max_us = max(latencies_ticks) / ticks_per_us
    print(
        f"\n[fast-sync-jitter] baud={baud} n={n_trials} "
        f"min={min_us:.2f}us max={max_us:.2f}us "
        f"jitter={jitter_us:.2f}us byte_time={byte_time_us:.2f}us"
    )
    assert jitter_us <= byte_time_us, (
        f"slot-0 fire jitter {jitter_us:.2f}us exceeds 1 byte time {byte_time_us:.2f}us "
        f"(latencies_ticks={latencies_ticks})"
    )


def _arm_first_slot(injector, packet_length: int, slot_id: int, data: bytes) -> None:
    """ARM the injector to emit a FastSlot::First (header + body) one RDT after
    the next IDLE — DXL 2.0 spec: T0 starts at host_request_end + RDT. Default
    RDT is 250 µs; injector tick is HCLK/8 = 18 ticks/µs."""
    inj_bytes = build_fast_first_bytes(
        packet_length=packet_length, err=0, slot_id=slot_id, data=data
    )
    reply = injector.command(f"ARM bytes={inj_bytes.hex()} after_idle={250 * 18}")
    assert reply == "OK", f"injector ARM rejected: {reply!r}"


def _check_coalesced_two_slot(frame: bytes, length: int, dut_id: int, inj_data: bytes):
    """Frame-level checks shared by the Fast Sync + Fast Bulk DUT-as-Last
    tests: 2-slot coalesced response, INJ data lands verbatim, DUT slot present
    with correct id/length, frame CRC validated by parse_fast_response."""
    slots = parse_fast_response(frame, slot_lengths=[length, length])
    assert len(slots) == 2, f"expected 2 slots, got {len(slots)}: {frame.hex()}"
    assert slots[0].id == INJ_ID, f"slot 0 id={slots[0].id}, want {INJ_ID}"
    assert slots[0].error == 0, f"slot 0 err=0x{slots[0].error:02X}"
    assert slots[0].data == inj_data, (
        f"slot 0 data={slots[0].data.hex()}, want {inj_data.hex()} "
        f"(injector bytes were corrupted on the wire?)"
    )
    assert slots[1].id == dut_id, f"slot 1 id={slots[1].id}, want {dut_id}"
    assert slots[1].error == 0, f"slot 1 err=0x{slots[1].error:02X}"
    assert len(slots[1].data) == length


def test_fast_sync_read_dut_last_in_coalesced_frame(port, osc_id, injector):
    """Real 2-slot Fast Sync Read [INJ, DUT]: injector emits FastSlot::First
    immediately on request-end, DUT detects itself as Last and emits its body
    after slot-period delay with a snoop-patched CRC covering both bodies.
    Verifies the DUT's snoop+CRC path against actual bus bytes (today only
    self-emitted Only frames exercise it)."""
    _drain_all_stamps(injector)

    length = 2
    inj_data = b"\xA1\xA2"
    # packet_length = instr(1) + N_slots * (err + id + data) + crc(2)
    packet_length = 1 + 2 * (2 + length) + 2

    _arm_first_slot(injector, packet_length, INJ_ID, inj_data)

    port.writePort(build_fast_sync_read(addr=0, length=length, ids=[INJ_ID, osc_id]))
    frame = read_status_frame(port.ser, timeout_s=0.2)
    assert frame is not None, "expected coalesced Fast Sync Status frame"
    _check_coalesced_two_slot(frame, length, osc_id, inj_data)


def test_fast_bulk_read_dut_last_in_coalesced_frame(port, osc_id, injector):
    """Mirror of the sync test for Fast Bulk Read: same 2-slot coalescing,
    but the request encodes per-slot (id, addr, length) tuples instead of a
    shared addr/length + id list."""
    _drain_all_stamps(injector)

    length = 2
    inj_data = b"\xB1\xB2"
    packet_length = 1 + 2 * (2 + length) + 2

    _arm_first_slot(injector, packet_length, INJ_ID, inj_data)

    port.writePort(build_fast_bulk_read([(INJ_ID, 0, length), (osc_id, 0, length)]))
    frame = read_status_frame(port.ser, timeout_s=0.2)
    assert frame is not None, "expected coalesced Fast Bulk Status frame"
    _check_coalesced_two_slot(frame, length, osc_id, inj_data)
