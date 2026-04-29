# OSC SG90 CH32

> ⚠️ **Designed, not spun. Pre-fabrication, pre-bringup.**
> This board has never been fabbed. Numbers and behaviour described below are **design intent**, not measured. The schematic and layout exist; the firmware they're meant to run does not yet — firmware v2 is unstarted as of this writing. Don't fab this without doing your own review first, and expect changes when bringup finally happens.

OpenServoCore swap board for SG90-class hobby servos. Compact, double-sided, designed to physically replace the factory PCB inside an SG90 case. CH32V006-based, DXL-compatible half-duplex UART, drop-in mechanical fit.

## Overview

- **MCU** — CH32V006F8U6 (RISC-V, 48 MHz, 62 KB flash, 8 KB RAM).
- **Motor driver** — TI DRV8837 H-bridge (1.8 A peak, IN1/IN2 PWM).
- **LDO** — TPAP2210K-3.3, 3.3 V logic rail.
- **UART buffer** — SN74LVC1G07 open-drain for half-duplex DXL.
- **Current shunt** — 25 mΩ, 1 % on the low side.
- **Power input** — 3.3-8.4 V (1S-2S LiPo) via the servo cable.
- **Debug** — 4-pad SWD interface for WCH-LinkE.
- **Position feedback** — existing servo potentiometer wiper.
- **Form factor** — 10 × 12.5 mm, double-sided assembly.

## Connectors

### Servo cable — J1 (1×3, through-hole pads)

Standard hobby-servo 3-wire pinout, but the signal pin carries half-duplex UART instead of PWM:

| Pin | Net    | Notes                                   |
| --- | ------ | --------------------------------------- |
| 1   | `VIN`  | 3.3-8.4 V (1S-2S LiPo).                 |
| 2   | `DATA` | Half-duplex UART, baud TBD by firmware. |
| 3   | `GND`  |                                         |

### Motor — J2 (2 wire pads)

Solder directly to the SG90 motor leads.

| Pad    | Net              |
| ------ | ---------------- |
| `OUTA` | Motor terminal A |
| `OUTB` | Motor terminal B |

### Potentiometer — J3 (3 pads)

Connects to the existing servo potentiometer.

| Pad    | Net          |
| ------ | ------------ |
| `+3V3` | Pot top rail |
| `VPOS` | Wiper        |
| `GND`  | Pot ground   |

### Debug — J4 (4 pads, 1.27 mm pitch)

Single-wire SWD for WCH-LinkE. Programmed via `wlink` / `probe-rs`. Firmware build instructions land when firmware v2 starts; until then there's nothing to flash.

| Pad | Net     |
| --- | ------- |
| 1   | `+3V3`  |
| 2   | `GND`   |
| 3   | `SWDIO` |
| 4   | `NC`    |

## Design intent

These are the targets the board was laid out around. None are measured.

- **Power architecture** — VIN routes to the DRV8837 directly; 3.3 V logic rail off the TPAP2210K LDO. Bulk capacitance is 2× 10 µF on VIN. PGND/GND joined at a single net tie.
- **Position sensing** — 12-bit ADC on `VPOS` with an RC filter (10 kΩ + 100 nF, fc ≈ 160 Hz) to suppress wiper noise. Sub-degree resolution is the design goal; actual precision waits on bringup and firmware oversampling.
- **Voltage sensing** — 20 kΩ / 10 kΩ dividers (ratio 0.333) on `VINS`, `VSNA`, `VSNB`. 8.4 V → 2.8 V at the ADC.
- **Current sensing** — 25 mΩ low-side shunt, sized for the DRV8837's ~1.76 A peak via `R_shunt ≈ 44 mV / I_stall`, so the hardware stall-detect path trips near the driver's max output. Raw V_shunt = 25 mV at 1 A; firmware sets ADC sampling, gain path, and software overcurrent threshold.
- **Communication** — DXL-style half-duplex UART. The SN74LVC1G07 open-drain buffer enables bidirectional comms over the single `DATA` line. Baud rate TBD — left to firmware v2 to choose.
- **Protection** — PESD5V0L1UL TVS on `DATA`.

## Assembly

Double-sided assembly, no vias-in-pads. Suitable for hot-plate / hot-air / IR reflow, or JLCPCB SMT. The board is 8-layer for routing density at this size.

## Mechanical fit

Designed to drop into a standard SG90 servo case:

- Motor leads solder to `OUTA` / `OUTB`.
- Pot wiper connects to `J3`.
- Servo cable connects to `J1`.
- Debug pads remain accessible through the case opening for bringup / reflashing.

Pad spacing for motor leads and debug pitch are still **to be confirmed against a physical case** when first boards arrive.

## License

Hardware files (schematic, layout, board files in this directory) are licensed under [CERN-OHL-P v2.0](../../../LICENSE-HARDWARE). See the [top-level README](../../../README.md#license) for the full licensing picture.
