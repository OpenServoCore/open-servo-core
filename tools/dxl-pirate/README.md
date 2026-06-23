# dxl-pirate

Firmware that turns a MuseLab nanoCH32V203 into a host-driven multi-tool for
the Dynamixel 2.0 bus. Three things it does that a plain USB-UART can't:

- **Master** the bus from the host: send arbitrary request bytes and stamp the
  master's last-bit-out tick (`T_request_end`).
- **Inject** raw byte payloads onto the bus with sub-microsecond start jitter,
  scheduled either at an absolute SysTick value or a chosen delay after the
  next idle on the wire.
- **Listen** passively and timestamp every idle gap (144 MHz HCLK ticks), so
  you can see exactly when a servo starts and finishes transmitting.

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

| Pin   | Role                       |
| ----- | -------------------------- |
| PA2   | USART2 TX, HDSEL — DXL bus |
| PB11  | USART3 RX listen tap       |
| PA12  | USB DP                     |
| PA11  | USB DM                     |
| PA15  | User LED (active-low)      |

PA2 and PB11 land on the **same** wire — PA2 drives, PB11 listens.

For the hardware-timing architecture (time domain, peripheral assignments,
RX/TX pipelines) see [TIMING.md](TIMING.md).

## Flash

Build once, then push it over USB with `wchisp`:

```
cargo build --release
wchisp info                                   # confirm the chip is alive
wchisp flash target/riscv32imac-unknown-none-elf/release/dxl-pirate
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
TICK?                                # current SysTick value
HZ                                   # ticks per microsecond (= 144)
BAUD 3000000                         # retune both TX + RX (quiesce bus first)

DRAIN                                # pop one idle timestamp, or EMPTY
BYTES                                # total RX bytes since boot

FIRE  bytes=<hex>  at=<u64>          # inject at an absolute SysTick value
ARM   bytes=<hex>  after_idle=<u32>  # inject N ticks after the next idle
LAST?                                # tick when the last fire actually went
```

Replies are `OK`, `ERR <reason>`, or a typed value (`TICK 12345`,
`STAMP 12345 42`, …).

### Watch the bus

Poll `DRAIN` in a loop. Every gap ≥ 1 byte time on the wire produces a stamp:

```
> DRAIN
< STAMP 1234567 42      # tick at idle edge, running byte counter
> DRAIN
< STAMP 1240123 58      # 16 bytes between idles → previous frame was 16 bytes
> DRAIN
< EMPTY
```

### Inject a synthetic slave response

After the host issues a multi-slave Fast Sync Read, fire a fake slot one
round-trip-delay (≈ 250 µs = 36000 ticks) after the request ends:

```
> ARM bytes=55ff00fdfd...crc... after_idle=36000
< OK
```

The injector waits for the listener to see the request-end idle, then fires
the payload after the delay.

## Caveats

- TX payloads max 128 bytes per shot (DMA buffer size).
- `after_idle` is measured from when the USART IDLE flag fires, which is ~1
  character time **after** the actual end-of-byte (≈ 3.3 µs at 3 Mbaud).
  Subtract that if you want sub-byte alignment.
- BAUD assumes a quiet bus — both UARTs briefly drop UE around the BRR write.
- Nightly Rust required (`feature(sync_unsafe_cell)`).

## Tests

Pure-functional helpers (BRR divisor, hex decode) live in `src/parse.rs` with
host unit tests:

```
cargo test --lib --target=<your host triple>
```
