"""Probe 1: INJ fire-latency floor at `after_idle=0`.

Validates that the injector's listener→fire chain is small and
baud-independent. Any baud drift here localizes a timing bug to the INJ side.

Bench setup expected:
  - V203 dxl-bus-injector on the bus (USB-CDC)
  - FT232H (or other USB-UART) for the host side of the bus
  - Scope ch1 on the data line, ch2 on PA7 (injector debug pin)
  - Scope trigger: ch2 rising edge, Single

Sequence per run:
  1. Drain pending bytes + stamps; ARM the injector (this also resets PA7
     LOW so the scope's rising-edge trigger has a clean window to capture).
  2. Print "ARM SCOPE NOW" + sleep 5 s. User arms the scope during the
     sleep; PA7 stays LOW since the bus is quiet.
  3. Send one byte from the host (the trigger frame). USART3 IDLE fires
     after wire-end; PA7 goes high; injector fires immediately; PA7 goes
     low. Scope captures the rising/falling edges of PA7 and the INJ
     wire-start.
  4. Drain stamps + read LAST? — print idle.tick, last_fired_tick, diff.

Invoke with `-s` so prints aren't buffered:
  pytest tools/dxl-bench/test_inj_probe1_scope.py -s
"""

import time

INJ_PAYLOAD = b"\xaa"  # one-byte, easy to identify on the scope
TRIGGER = b"\x55"  # one-byte host frame whose IDLE arms the INJ fire


def _drain_stamps(injector):
    out = []
    while True:
        reply = injector.command("DRAIN")
        if reply == "EMPTY":
            return out
        assert reply.startswith("STAMP "), f"unexpected DRAIN reply: {reply!r}"
        _, tick_s, head_s = reply.split()
        out.append((int(tick_s), int(head_s)))


def test_inj_probe1_scope(port, injector, baud):
    while port.ser.in_waiting:
        port.ser.read(port.ser.in_waiting)
    _drain_stamps(injector)

    reply = injector.command(f"ARM bytes={INJ_PAYLOAD.hex()} after_idle=0")
    assert reply == "OK", f"injector ARM rejected: {reply!r}"

    print("\n" + "=" * 60, flush=True)
    print(f"PROBE 1  baud={baud}  after_idle=0", flush=True)
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
        print("(expected: CHAR_TIME_SYSTICKS(baud) + small chain latency)", flush=True)
        char_time = {9600: 16875, 57600: 2812, 115200: 1406,
                     1_000_000: 162, 2_000_000: 81, 3_000_000: 54}.get(baud)
        if char_time is not None:
            print(f"expected CHAR_TIME_SYSTICKS({baud}) = {char_time} "
                  f"({char_time / 18:.2f} µs)", flush=True)
            print(f"residual = {diff - char_time} ticks "
                  f"({(diff - char_time) / 18:.2f} µs) "
                  "← should be small + baud-independent", flush=True)
