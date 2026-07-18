# OSC SG90 M007

> ⚠️ **Designed, not spun. Pre-fabrication, pre-bringup.**
> This board has never been fabbed. Numbers and behaviour described below are **design intent**, not measured. The schematic is the M007 redesign; the layout in this directory is still the earlier CH32V006 revision and is **stale** - it gets redone against this schematic before any fab. Don't fab this without doing your own review first.

OpenServoCore swap board for SG90-class hobby servos. Compact, double-sided, designed to physically replace the factory PCB inside an SG90 case. CH32M007-based: one IC carries the MCU, the 3-phase gate pre-driver, the HV regulator, and the current-sense amplifier - no motor-driver IC, no LDO, no bus buffer.

## Overview

- **MCU + driver** - CH32M007E8U7 (QFN-26, 3×3 mm): 48 MHz RISC-V, integrated gate pre-driver with hardware dead-time, internal HV regulator (VSYS → 5 V VDD), on-die OPA/PGA current sense, CMP overcurrent → TIM1 brake in silicon.
- **Bridge** - 2× MCC MCMNP2065A (dual complementary N+P, DFN2020-6L 2×2 mm; N 20 mΩ / P 44 mΩ @4.5 V), one package per half-bridge, gates driven directly from HO/LO (no gate resistors), 100 kΩ gate keepers. Gates are ±10 V rated: legal because gate drive is rail-limited below the driver's 9.5 V clamp floor - see the 2S pin below.
- **Current shunts** - 2× 75 mΩ 1 % 0.5 W 0805 (Yageo PE series), per-leg low-side, Kelvin taps via net-ties into the on-die PGA (G = 16 differential).
- **Power input** - 6.5-9 V, **2S only** - the bridge's ±10 V gate rating pins this board to 2S (the dev board's 6.5-15 V window needs the ±20 V SOT-23-6 part). Firmware must interlock PWM above ~9 V rail (VSYS reads through the phase dividers). The gate-driver UVLO sets the 6.5 V floor; 1S and USB-5V cannot drive the bridge. VHREG is strapped to VHV (no series resistor).
- **Bus** - single-wire half-duplex UART (osc-native protocol), direct USART HDSEL on PB3 - no buffer. 33 Ω series + PESD5V0L1BA on DATA. 0.5-3 Mbaud.
- **Temperature** - on-board 10 kΩ NTC, B25/50 = 3380 K (NCP15XH103, 0402). Note: the dev board's NTC is 3950 K - the B constant is board-seeded config, not shared firmware math.
- **Position feedback** - existing servo potentiometer wiper, 4.7 kΩ + 100 nF at the ADC pin.
- **Passives** - smallest-package policy: 10 µF and 2.2 µF are 0603, all other R/C are 0402, shunts are the 1206 exception. JLC-basic parts wherever one exists; every part carries `LCSC`, `MFPN`, `Datasheet`, and `JLC_Basic` fields.
- **Form factor** - 10 × 12.5 mm target, double-sided assembly.

## Connectors

### Servo cable - J1 (1×3, wire pads)

Standard hobby-servo 3-wire cable; the signal wire carries the osc-native bus instead of PWM.

Center-power, same order as the dev board's bus connectors - an offset misplug never puts VSYS on DATA.

| Pad | Net    | Notes                       |
| --- | ------ | --------------------------- |
| 1   | `DATA` | Half-duplex osc-native bus. |
| 2   | `VSYS` | 6.5-15 V (2S/3S).           |
| 3   | `GND`  |                             |

### Motor - J2 (2 wire pads)

Solder directly to the SG90 motor leads (phase A / phase B of the bridge).

### Potentiometer - J4 (3 pads)

| Pad   | Net   | Notes              |
| ----- | ----- | ------------------ |
| `+5V` | `+5V` | Pot top rail.      |
| `POT` | `POT` | Wiper, via 4.7 kΩ. |
| `GND` | `GND` | Pot ground.        |

### Debug - J3 (4 pads, 1.0 mm pitch)

Single-wire SWIO for WCH-LinkE (`wlink`).

| Pad | Net    |
| --- | ------ |
| 1   | `+5V`  |
| 2   | `GND`  |
| 3   | `SWIO` |
| 4   | `nRST` |

## Design intent

These are the targets the schematic was drawn around. None are measured.

- **Decoupling** - DS Fig 3-1-3 floor exactly: 10 µF + 0.1 µF (25 V) on VHV, 2.2 µF on VHREG, 10 µF + 0.1 µF on VDD. No bulk beyond that - the single VHV 10 µF (bias-derated ~5 µF at 8.4 V) is the entire on-servo reservoir. This assumes the hub/BEC end of the harness carries the real per-branch bulk; hot-plug onto a charged pack rings a ceramic-only input and is a first-article scope item.
- **Voltage sensing** - 20 kΩ / 10 kΩ dividers on both phases (VSNA/VSNB); 15 V → 5 V at the ADC. Rail voltage reads through the phase dividers with the bridge parked high.
- **Current sensing** - per-leg shunts into the on-die PGA; CMP2 trip window ~2.5-2.75 A between worst 2S stall and harm (range budget in the bringup repo's `osc-dev-m007-pinout.md`).
- **Protection** - PESD5V0X1BCSFYL on DATA (ultra-low-capacitance, DFN1006-2); hardware OCP internal to the M007; no reverse-polarity protection (captive harness).

## Assembly

Double-sided assembly, 0402-and-up, suitable for JLCPCB economic SMT (all jellybeans are basic parts; extended types: MCU, FETs, shunts, PESD, 2.2 µF, 25 V 0402 100 nF, NTC). Motor, pot, and servo-cable connections are hand-soldered wire pads during the servo transplant.

## Mechanical fit

Designed to drop into a standard SG90 servo case. Pad spacing for motor leads and debug pitch are still **to be confirmed against a physical case** when first boards arrive.

## License

Hardware files (schematic, layout, board files in this directory) are licensed under [CERN-OHL-P v2.0](../../../LICENSE-HARDWARE). See the [top-level README](../../../README.md#license) for the full licensing picture.
