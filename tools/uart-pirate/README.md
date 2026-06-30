# uart-pirate

Firmware that turns a MuseLab nanoCH32V203 into a host-driven multi-tool for
the Dynamixel 2.0 bus. Three things it does that a plain USB-UART can't:

- **Master** the bus from the host: send arbitrary request bytes.
- **Inject** raw byte payloads onto the bus with sub-microsecond start jitter,
  scheduled either at an absolute `tick32` value or a chosen delay after the
  next idle on the wire.
- **Stamp every byte** received on the wire with a 144 MHz `tick32` (TIM2
  low + TIM3 high, hardware-locked 32-bit counter). Per-byte records carry
  a value, start tick, and anomaly flags drained by the host.

`tools/dxl-bench` uses it to exercise DXL Fast Sync/Bulk Read coalescing at
3 Mbaud and to drive master-side HSI calibration, but it's useful standalone
for poking at any DXL servo.

## Hardware

You need:

- A MuseLab nanoCH32V203 (V203C8T6, LQFP48).
- A DXL half-duplex transceiver (e.g. the `firmware/boards/nano-v203-injector`
  adapter) to drive the shared data line.

The V203's vendor bootloader speaks ISP over USB, so flashing works straight
out of the box with `wchisp` — no SWD probe needed. A WCH-LinkE is still handy
if you want `defmt` logs or live debugging.

### Pin map

| Pin   | Role                                    |
| ----- | --------------------------------------- |
| PA2   | USART2 TX, HDSEL + TIM2_CH3 IC — DXL bus |
| PA12  | USB DP                                  |
| PA11  | USB DM                                  |
| PA15  | User LED (active-low)                   |

PA2 is the single bidirectional wire — USART2 drives it in HDSEL mode and
TIM2_CH3 input capture taps the same pad for falling-edge stamping.

For the hardware-timing architecture (time domain, peripheral assignments,
RX/TX pipelines) see [TIMING.md](TIMING.md).

## Flash

Build once, then push it over USB with `wchisp`:

```
cargo build --release
wchisp info                                   # confirm the chip is alive
wchisp flash target/riscv32imac-unknown-none-elf/release/uart-pirate
```

You may need to hold the BOOT button while plugging in (or pressing RESET) to
enter the ISP bootloader the first time. After flashing, the board
re-enumerates as a USB-CDC serial device.

If you have a WCH-LinkE wired up, `cargo run --release` flashes via probe-rs.

## Use it

Open the USB serial port (any baud — CDC ignores it) and send ASCII lines
terminated by `\n`. The full command list is at the top of `src/proto.rs`; the
ones you'll actually use:

```
TICK?                                # current tick32 value
HZ                                   # tick32 ticks per microsecond (= 144)
BAUD 3000000                         # retune wire baud (quiesce bus first)

BDRAIN                               # pop one per-byte stamp, or EMPTY
BBATCH <count>                       # binary frame of up to <count> stamps
BYTES                                # total RX bytes since boot

FIRE  bytes=<hex>  at=<u32>          # inject at an absolute tick32 value
ARM   bytes=<hex>  after_idle=<u32>  # inject N tick32 ticks after next idle
MASTER bytes=<hex>                   # fire now as bus master
LAST?                                # tick32 of the last fire kickoff
```

Replies are `OK`, `ERR <reason>`, or a typed value (`TICK 12345`,
`BSTAMP 12345 42 0`, …).

### Watch the bus, byte by byte

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

`BBATCH <count>` replies with `STREAM <n>\n` followed by `n × (u32 LE tick,
u8 byte, u8 flags)` raw, then `\n`. Up to 64 records per call.

Flags bits (see `src/capture.rs`): `COUNT_OVER` (1), `COUNT_UNDER` (2).
All zero means the stamp is hardware-precise and the byte's edge count
matched its LUT entry exactly; a non-zero bit signals a wire-quality
anomaly (the stamp itself is still hardware-precise — see TIMING.md §4).

If the walker ever hits a designed-impossible condition that could
desync the rings (multi-cadence walker entry, IC ring overrun), every
command returns `ERR desync: edge<->byte rings desynced, power-cycle`
until the pirate is power-cycled. See TIMING.md §3.5.

### Inject a synthetic slave response

After the host issues a multi-slave Fast Sync Read, fire a fake slot one
round-trip-delay (≈ 250 µs = 36000 ticks) after the request ends:

```
> ARM bytes=55ff00fdfd...crc... after_idle=36000
< OK
```

The injector waits for USART2 IDLE (one char-time after wire-end), then
fires the payload after the delay.

## Caveats

- TX payloads max 1024 bytes per shot (DMA buffer size).
- `after_idle` is measured from when the USART IDLE flag fires, which is
  ~1 character time **after** the actual end-of-byte (≈ 3.3 µs at
  3 Mbaud). Subtract that if you want sub-byte alignment.
- BAUD assumes a quiet bus — USART2 briefly drops UE around the BRR
  write.
- Nightly Rust required (`feature(sync_unsafe_cell)`).

## Tests

Pure-functional helpers (BRR divisor, hex decode) live in `src/parse.rs` with
host unit tests:

```
cargo test --lib --target=<your host triple>
```
