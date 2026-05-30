"""Probe 2: INJ fire timing at `after_idle=4500` ticks (250 µs = one DXL RDT).

Validates the SysTick CMP-scheduled fire path, which Probe 1 (`after_idle=0`,
immediate-fire branch) does not exercise. This is the path the FastSyncRead
test depends on.

Bench setup expected:
  - V203 dxl-pirate on the bus (USB-CDC)
  - FT232H (or other USB-UART) for the host side of the bus
  - Scope ch1 on the data line, ch2 on PA7 (injector debug pin)
  - Scope trigger: ch2 rising edge, Single

Sequence per run:
  1. Drain pending bytes + stamps; ARM the injector with after_idle=4500
     (this also resets PA7 LOW for a clean scope-arm window).
  2. Print "ARM SCOPE NOW" + sleep 5 s.
  3. Send one byte from the host (the trigger frame). USART3 IDLE fires
     after wire-end; PA7 goes high; SysTick CMP fires 4500 ticks later;
     PA7 goes low; INJ frame is transmitted. Scope sees a ~250 µs PA7
     high pulse followed by the INJ wire-start.
  4. Drain stamps + read LAST? — print idle.tick, last_fired_tick, diff.
"""

import time

INJ_PAYLOAD = b"\xaa"
TRIGGER = b"\x55"
AFTER_IDLE_TICKS = 250 * 18  # 250 µs at SysTick = 18 MHz


def _drain_stamps(injector):
    out = []
    while True:
        reply = injector.command("DRAIN")
        if reply == "EMPTY":
            return out
        assert reply.startswith("STAMP "), f"unexpected DRAIN reply: {reply!r}"
        _, tick_s, head_s = reply.split()
        out.append((int(tick_s), int(head_s)))


def test_inj_probe2_scope(port, injector, baud):
    while port.ser.in_waiting:
        port.ser.read(port.ser.in_waiting)
    _drain_stamps(injector)

    reply = injector.command(
        f"ARM bytes={INJ_PAYLOAD.hex()} after_idle={AFTER_IDLE_TICKS}"
    )
    assert reply == "OK", f"injector ARM rejected: {reply!r}"

    print("\n" + "=" * 60, flush=True)
    print(f"PROBE 2  baud={baud}  after_idle={AFTER_IDLE_TICKS} ticks "
          f"(250 µs RDT)", flush=True)
    print("Scope: ch1 = bus, ch2 = PA7. Trigger ch2 rising, Single.", flush=True)
    print("PA7 reset LOW by ARM. ARM SCOPE NOW. Firing in 5 s...", flush=True)
    print("=" * 60, flush=True)
    time.sleep(5.0)

    port.ser.write(TRIGGER)
    port.ser.flush()
    time.sleep(0.05)

    while port.ser.in_waiting:
        port.ser.read(port.ser.in_waiting)

    stamps = _drain_stamps(injector)
    last_reply = injector.command("LAST?")
    assert last_reply.startswith("LAST "), f"unexpected LAST? reply: {last_reply!r}"
    last_fired = int(last_reply.split()[1])

    print(f"\nstamps observed by injector ({len(stamps)}):", flush=True)
    for tick, head in stamps:
        print(f"  tick={tick:10d}  head={head:5d}", flush=True)
    print(f"last_fired_tick = {last_fired}", flush=True)
    if stamps:
        idle_tick = stamps[0][0]
        diff = (last_fired - idle_tick) & 0xFFFFFFFF
        print(f"\nLAST? − stamps[0].tick = {diff} SysTicks "
              f"= {diff / 18:.2f} µs at 18 MHz", flush=True)
        # stamps[0].tick is already backdated to wire-end, so on the CMP path
        # we expect diff = after_idle + small chain latency. When after_idle
        # < CHAR_TIME, fire_at lands in the past and the immediate-fire path
        # takes over → diff ≈ CHAR_TIME instead.
        char_time = {9600: 16875, 57600: 2812, 115200: 1406,
                     1_000_000: 162, 2_000_000: 81, 3_000_000: 54}.get(baud)
        cmp_residual = diff - AFTER_IDLE_TICKS
        print(f"CMP-path residual = diff − after_idle = {cmp_residual} ticks "
              f"({cmp_residual / 18:.2f} µs)", flush=True)
        if char_time is not None and AFTER_IDLE_TICKS <= char_time:
            imm_residual = diff - char_time
            print(f"after_idle ≤ CHAR_TIME({baud})={char_time}: immediate-fire "
                  f"path expected", flush=True)
            print(f"immediate-fire residual = diff − CHAR_TIME = "
                  f"{imm_residual} ticks ({imm_residual / 18:.2f} µs)",
                  flush=True)
