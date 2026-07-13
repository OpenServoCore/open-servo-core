# uart-pirate

Firmware that turns a MuseLab nanoCH32V203 into a hardware-stamped UART
multi-tool. Three things it does that a plain USB-UART can't:

- **Inject** raw byte payloads onto the wire with sub-microsecond start
  jitter, scheduled at an absolute `tick32` value, a delay after the next
  idle on the wire, or immediately.
- **Stamp every byte** received on the wire with a 144 MHz `tick32` (TIM2
  low + TIM3 high, hardware-locked 32-bit counter). Per-byte records
  carry a value, start tick, and anomaly flags drained by the host.
- **Expose a deterministic ASCII line protocol** over USB-CDC so a host
  bench script can drive captures + injections in lockstep with the
  wire — no probe required.

`tools/bench` uses it to exercise high-baud chain timing and to calibrate
TX-comp at 3 Mbaud, but it's useful standalone for poking at any
half-duplex UART target.

## Hardware

You need:

- A MuseLab nanoCH32V203 (V203C8T6, LQFP48).
- A WCH-LinkE for flashing via SWD.

### Pin map

| Pin   | Role                                                |
| ----- | --------------------------------------------------- |
| PB10  | USART3 TX (AF push-pull) — wire                     |
| PB11  | USART3 RX (input pull-up) — wire (jumpered to PB10) |
| PA12  | USB DP                                              |
| PA11  | USB DM                                              |
| PA15  | User LED (active-low)                               |

PB10 and PB11 are jumpered externally so both halves of the USART block
sit on the same single wire. RX captures both our own TX echo and any
remote replies; TIM2_CC1 input capture taps the same wire (via TI1S XOR
routing through TIM2_CH3 on PB10) for falling-edge stamping.

For the hardware-timing architecture (time domain, peripheral
assignments, RX/TX pipelines) see [TIMING.md](TIMING.md).

## Flash

Build, then push it over SWD with `wlink`:

```
cargo build --release
wlink flash target/riscv32imc-unknown-none-elf/release/uart-pirate
```

After flashing, the board re-enumerates as a USB-CDC serial device
(VID=0xC0DE, PID=0xCAFE).

## Use it

Open the USB serial port (any baud — CDC ignores it) and send ASCII
lines terminated by `\n`. The full command list is at the top of
`src/proto.rs`; the ones you'll actually use:

```
TICK?                                # current tick32 value
HZ                                   # tick32 ticks per microsecond (= 144)
BAUD 3000000                         # retune wire baud (quiesce wire first)

BDRAIN                               # pop one per-byte stamp, or EMPTY
BBATCH <count>                       # binary frame of up to <count> stamps

SEND bytes=<hex>                     # send now (no precise timing)
SEND bytes=<hex> at=<u32>            # send at an absolute tick32 value
SEND bytes=<hex> after_idle=<u32>    # send N tick32 ticks after next idle
LAST?                                # tick32 of the last send kickoff
```

Replies are `OK`, `ERR <reason>`, or a typed value (`TICK 12345`,
`BSTAMP 12345 42 0`, …).

### Watch the wire, byte by byte

Poll `BDRAIN` in a loop, or amortize via `BBATCH 64` for high-throughput
captures. Each emitted record is `(tick32, byte_value, flags)`:

```
> BDRAIN
< BSTAMP 1234567 255 0     # 0xFF, no anomalies
> BDRAIN
< BSTAMP 1234711 253 0     # 0xFD, +144 ticks ≈ 1 µs = 10 bits @ 1Mbaud
> BDRAIN
< EMPTY
```

`BBATCH <count>` replies with a binary frame: `[0xA5 0x5A][count:u16 LE]`
then `count × (tick:u32 LE, byte:u8, flags:u8)`. No newline terminator;
framing is length-prefixed. Up to 64 records per call.

Flag bits (see `src/capture.rs`): `COUNT_UNDER` (bit 0). Zero means the
walker found a real IC edge inside the snap window and the emitted tick
is hardware-precise (sub-tick of the wire edge, after subtracting the
CC filter delay). `COUNT_UNDER` set means no edge landed in the snap
window — the emitted tick is the walker's predicted free-run value,
accurate to `±SNAP_BITS · bit_ticks`.

If the walker hits an `ic_overrun` (designed-impossible) or
`stamp_overflow` (host didn't drain) condition, every command returns
`ERR desync <cause>` until `RESET` (or `BAUD`, which implicitly resets)
clears the flag.

### Schedule a follow-up after the wire quiets

After the host sends a request, schedule a faked response one
round-trip-delay (≈ 250 µs = 36000 ticks @ 144 MHz) after the request
ends:

```
> SEND bytes=55ff00fdfd...crc... after_idle=36000
< OK
```

The pirate waits for USART3 IDLE (one char-time after wire-end), then
sends the payload after the delay.

## Caveats

- TX payloads max 1024 bytes per shot (DMA buffer size).
- `after_idle` is measured from when the USART IDLE flag fires, which is
  ~1 character time **after** the actual end-of-byte (≈ 3.3 µs at
  3 Mbaud). Subtract that if you want sub-byte alignment.
- `BAUD` assumes a quiet wire — USART3 briefly drops UE around the BRR
  write.
- Nightly Rust required (`feature(sync_unsafe_cell)`).
