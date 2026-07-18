<p align="center">
  <picture>
    <source media="(prefers-color-scheme: dark)" srcset="docs/logo-dark.svg">
    <img src="docs/logo.svg" alt="OpenServoCore logo" width="320">
  </picture>
</p>

# OpenServoCore

> An open platform for turning cheap servos into smart actuators.

OpenServoCore (OSC) is open hardware and firmware that drops a CH32V006 control board into a $2-3 cloned hobby servo (SG90 and friends) and turns it into a Dynamixel-class smart actuator — position feedback, current sensing, bus-addressable, programmable. The bus speaks the **osc-native protocol**: OSC's own break-framed wire protocol, inspired by Dynamixel Protocol 2.0 but redesigned to run whole on sub-$0.20 MCUs ([spec](docs/osc-native-protocol.md)).

The thesis is the price point: at mass-production volume, an OSC swap board should add **no more than ~$1 to the BOM** of a cloned servo. Cheap enough that "upgrade every servo in a robot to smart" stops being a premium decision and starts being a default.

## Status

**In active development. Nothing here is shippable yet.** The firmware is being rewritten, the dev board is validated and integrating with firmware, and the swap board is designed but not spun.

- **OSC Dev CH32** (`osc-dev-v006`) — Rev B validated. Firmware integration ongoing.
- **OSC SG90 M007** (`osc-sg90-m007`) — schematic redesigned on the CH32M007; layout pending, not spun. Waiting on firmware v2 to be testable against.
- **Firmware v1** (`firmware-old/`) — legacy. First pass was vibe-coded and got poor Reddit feedback. Kept as historical reference; **not a target for new work**.
- **Firmware v2** (rewrite) — in progress. The osc-native protocol and the servo bus transport are implemented and bench-proven on silicon (0.5-3 Mbaud, multi-servo chains); control loops are next.
- **tinyboot** (OSC bootloader) — v0.4.0 shipped. Lives at [`OpenServoCore/tinyboot`](https://github.com/OpenServoCore/tinyboot).

## Repo map

```
open-servo-core/
├── hardware/
│   ├── boards/
│   │   ├── osc-dev-v006/             # OSC Dev CH32 — has its own README with pinouts, jumpers, bringup notes
│   │   ├── osc-sg90-m007/            # OSC SG90 M007 swap board (schematic done, layout pending)
│   │   ├── servo-dev-board-stm32f301/# Retired hobby-phase STM32 dev board (legacy)
│   │   ├── encoder-board/            # Optional quadrature encoder breakout for J8 experiments
│   │   └── motor-mount/              # 3D-printable test fixtures
│   ├── shared.kicad_sym / shared.pretty / shared.3dshapes  # Shared KiCad libraries
│   └── templates/                    # KiCad project templates
├── docs/                             # Design docs — protocol spec, transport, driver pattern, history
├── firmware/                         # Firmware v2 (Rust) — chip-agnostic libs, CH32 chip crate, board binaries
├── tools/                            # Host-side tooling — hardware test bench, `osc` operator CLI
└── firmware-old/                     # Legacy firmware (do not use)
```

The OSC bootloader, [`tinyboot`](https://github.com/OpenServoCore/tinyboot), is a separate repo. It's part of the OSC firmware stack but versioned and released independently — its chip-support matrix (V003 / V00x / V103) is broader than the OSC boards on purpose.

## Naming

OSC boards follow `OSC <Form> <ChipFamily>`:

- **OSC Dev CH32** — dev board, exposes every rail and signal for firmware bringup. Directory: `osc-dev-v006`.
- **OSC SG90 M007** — swap board that physically replaces the SG90 factory PCB. Directory: `osc-sg90-m007`.

Engineering SKUs (`osc-<form>-<chip>-rev-<letter>`) appear in BOMs and schematic title blocks; the names above are what you'll see in posts and docs.

## Hardware

OSC standardizes on the **CH32V006** — 48 MHz RISC-V, 62 KB flash, 8 KB RAM. Chosen because it's the chip that makes the ≤$1 BOM uplift work. No multi-chip roadmap; one chip, done well.

Each board has its own README with full schematics, pinouts, jumper behaviour, and bringup notes:

- **[OSC Dev CH32](hardware/boards/osc-dev-v006/README.md)** — accepts any gutted hobby servo, USB-C / 1S-2S LiPo / WCH-LinkE power, full edge test-point fanout. Rev B validated.
- **[OSC SG90 M007](hardware/boards/osc-sg90-m007/README.md)** — compact swap board, 10×12.5 mm, double-sided. Schematic done; layout pending; not yet spun.

## Firmware

The Rust firmware v2 lives in `firmware/`: chip-agnostic library crates (protocol, drivers, control table, discrete-event integration tests), a CH32 chip crate, and board binaries. It speaks the osc-native protocol — OSC's own break-framed bus protocol, inspired by Dynamixel Protocol 2.0. DXL 2.0 itself was implemented and tuned first, then replaced: its wire format (header hunting, byte stuffing, reply-grid timing) costs more than a $0.15 MCU should pay, and controlling both ends of the wire made those subsystems deletable outright — the story is in [design history](docs/design-history.md). The register-table conventions (flat control table, staged writes, alert semantics) keep the DXL flavor.

The bus transport is bench-proven on silicon: 0.5-3 Mbaud, ~30 us ping turnaround at 1 M, multi-servo status chains, hardware CRC both directions. Control loops, persistence, and safety features are in progress; build instructions will appear as the rewrite matures.

The legacy `firmware-old/` tree contains the original architecture (multi-crate workspace targeting STM32F301 and partly CH32V003) and is kept for reference only.

## Documentation

Design docs live in [`docs/`](docs/):

- **[osc-native protocol](docs/osc-native-protocol.md)** — the wire protocol spec: break framing, instruction set, management plane.
- **[Servo transport](docs/osc-servo-transport.md)** — the servo-side transport design: DMA ring, deadline pipeline, hardware CRC.
- **[Driver pattern](docs/driver-pattern.md)** — the firmware architecture: services / drivers / providers / HAL.
- **[Design history](docs/design-history.md)** — what I tried and abandoned, and what it taught.
- **[Testing](docs/testing.md)** — the test strategy.

## Contributing

This is early — the most useful thing right now is **following along and asking questions**, not opening PRs.

- **Discussions:** [github.com/OpenServoCore/open-servo-core/discussions](https://github.com/OpenServoCore/open-servo-core/discussions) — design questions, ideas, "is this on the roadmap?" go here.
- **Build journey:** posts at [aaronqian.com](https://aaronqian.com) document the design decisions, dead ends, and what shipped each week.
- **Issues:** open ones on this repo are scoped to specific work (README, LICENSE, board revisions). Pre-firmware-v2, contributor scope is small.

## Hardware sponsorship

Dev boards are fabricated and assembled by **[PCBWay](https://www.pcbway.com/)** — sponsor since Feb 2026.

### Rev A

Five PCBA boards delivered. Build and assembly quality clean across all five — no fabrication issues. Bring-up turned up design issues on my side (VDD/VSS swap, silkscreen errors), but those traced back to my own schematic, not the manufacturing. The process itself was painless. A late BOM swap (RS1 shunt `100 mΩ` → `10 mΩ`) was accepted without fuss; the pre-fab assembly review caught a pad-clearance concern before manufacturing.

Full spin + bring-up writeup: [CH32V006 dev board first spin](https://aaronqian.com/projects/open-servo-core/logs/2026-04-03-ch32v006-dev-board-first-spin/).

### Rev B

Five PCBA boards delivered, May 2026. Validated. PCBWay's pre-fab manufacturability review flagged nothing, and build/assembly quality was again clean. A small bug found in the sponsored boards (shared `nRST` / `OPN2` pin) didn't warrant another validation round — patched in-rev with a solder bridge. The published files include the patch.

Reference design available as a [PCBWay community project](https://www.pcbway.com/project/shareproject/OSC_Dev_V006_Rev_B_OpenServoCore_Development_Board_CH32V006_0f6621d7.html) for one-click ordering.

## License

OSC is fully open. No dual licensing, no commercial gates.

- **Firmware** — [MIT](LICENSE-MIT) **OR** [Apache-2.0](LICENSE-APACHE), at your option (Rust ecosystem convention).
- **Hardware** (schematics, layouts, board files) — [CERN-OHL-P v2.0](LICENSE-HARDWARE).
