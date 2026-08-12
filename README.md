<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/logo-dark.svg">
    <img src="docs/logo.svg" alt="OpenServoCore logo" width="320">
  </picture>
</p>

# OpenServoCore

> An open platform for turning cheap servos into smart actuators.

OpenServoCore (OSC) is open hardware and firmware that drops a CH32V006 control board into a $2-3 cloned hobby servo (SG90 and friends) and turns it into a Dynamixel-class smart actuator with position feedback, current sensing, bus addressing, and programmability. The bus speaks the **osc-native protocol**, a break-framed wire protocol designed to run whole on sub-$0.20 MCUs ([design writeup](https://aaronqian.com/log/2026-08-08-servo-protocol-stream-processing/)).

The thesis is the price point. At mass-production volume, an OSC swap board should add **no more than ~$1 to the BOM** of a cloned servo. That is cheap enough that upgrading every servo in a robot to smart becomes the default instead of a premium decision.

If you are here for a board, jump to [Boards](#boards). The firmware and its progress are in [Firmware](#firmware), and the design writeups are in [Reading](#reading).

## Status

**In active development. Nothing here is shippable yet.** What exists today:

- The **OSC Dev CH32** dev board is validated (Rev B) and you can order one.
- The firmware's protocol and bus transport are implemented and bench-proven on real silicon. Control loops are next.
- **[tinyboot](https://github.com/OpenServoCore/tinyboot)**, the OSC bootloader, has shipped v0.4.0 as its own repo.

The servo swap board (the one that goes inside a servo) is designed but not spun yet.

## Boards

**OSC Dev CH32** (`hardware/boards/osc-dev-v006/`) is the firmware development platform. It accepts any gutted hobby servo, takes power from USB-C, 1S-2S LiPo, or a WCH-LinkE, and fans out every rail and signal to edge test points. Rev B is validated. The [board README](hardware/boards/osc-dev-v006/README.md) has full schematics, pinouts, jumper behaviour, and bringup notes.

Want one? The full KiCad design files live in `hardware/boards/osc-dev-v006/`, ready to send to the board house of your choice.

There is also an experimental BLDC variant (`osc-dev-m007`) in the tree. It is unvalidated, so don't fab it.

## Firmware

The firmware is written in Rust and lives in `firmware/`. It is organized as chip-agnostic library crates (protocol, drivers, control table, discrete-event integration tests), a CH32 chip crate, and board binaries. The chip-agnostic layers compile and unit-test on a desktop with no hardware attached, and a discrete-event simulation runs the production crates ([the architecture writeup](https://aaronqian.com/log/2026-08-01-chip-agnostic-architecture-bare-metal-rust/) covers how).

What works today is the osc-native protocol and the servo bus transport. On the bench that means 0.5-3 Mbaud multi-servo status chains, ~30 µs ping turnaround at 1 Mbaud, and hardware CRC in both directions. The register-table conventions (flat control table, staged writes, alert semantics) will feel familiar if you have used Dynamixel. Control loops, persistence, and safety features are in progress, and build instructions will appear as the firmware matures.

The bootloader, [tinyboot](https://github.com/OpenServoCore/tinyboot), is a separate repo. It is part of the OSC firmware stack but versioned and released independently, and it supports more chips than the OSC boards on purpose (V003 / V00x / V103).

## Reading

The deep-dive writeups live on the blog:

- **[Stream processing on the wire](https://aaronqian.com/log/2026-08-08-servo-protocol-stream-processing/)**: the osc-native protocol design, and how the servo replies in tens of microseconds on a $0.16 chip.
- **[A chip-agnostic architecture for bare-metal embedded Rust](https://aaronqian.com/log/2026-08-01-chip-agnostic-architecture-bare-metal-rust/)**: the full firmware architecture, layer by layer, with code.
- **[Using the SPI peripheral as a DMA-fed CRC engine](https://aaronqian.com/log/2026-07-25-spi-as-dma-crc-engine/)**: hardware CRC on a chip with no CRC peripheral.
- **[HSI trim: calibrating a crystalless MCU over the bus](https://aaronqian.com/log/2026-07-18-hsi-trim-crystalless-mcu/)**: fleet clock calibration with nothing but the bus wire.
- **[Dynamixel 2.0 servo side: RX timing](https://aaronqian.com/log/2026-07-04-dynamixel-servo-side-rx-timing/)** and **[Fast Sync/Bulk Read](https://aaronqian.com/log/2026-07-11-dynamixel-servo-side-fast-sync-bulk-read/)**: the reference pair if you are building a Dynamixel-compatible device.

There is also one design doc in the repo worth reading. [Control theory](docs/control-theory.md) lays out the control design for the servo, with the cascaded loops, the estimators, and the sensing tiers. It is the plan, written ahead of the implementation, and it is not implemented yet.

## AI Use

I use AI heavily in this project. Claude (Fable 5, Opus 5, and earlier models) does a lot of the code generation and most of the document drafting, including parts of this README.

That does not make this vibe-coded. I review every line before it lands, and the result has to survive checks that don't care how the code was written. CI builds and tests every crate and cross-builds every board image. A discrete-event simulation runs the production crates and caught over a dozen real bugs before they reached silicon. The transport numbers above were measured on the bench. And this project was not built in one prompt. The history spans multiple years and many revisions of both the hardware and the firmware.

The documentation is more uneven than the code, and even this README could be easier to consume. That will improve, but working firmware comes first.

## Contributing

This is early. The most useful thing right now is **following along and asking questions** rather than opening PRs.

- **Discussions:** [github.com/OpenServoCore/open-servo-core/discussions](https://github.com/OpenServoCore/open-servo-core/discussions) is where design questions, ideas, and "is this on the roadmap?" go.
- **Build journey:** posts at [aaronqian.com](https://aaronqian.com) document the design decisions, dead ends, and what shipped each week.
- **Issues:** open ones are scoped to specific work. For now, contributor scope is small.

## License

OSC is fully open source.

- **Firmware**: [MIT](LICENSE-MIT) **OR** [Apache-2.0](LICENSE-APACHE), at your option (Rust ecosystem convention).
- **Hardware** (schematics, layouts, board files): [CERN-OHL-P v2.0](LICENSE-HARDWARE).
