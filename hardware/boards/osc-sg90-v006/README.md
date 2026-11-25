# OSC SG90 V006

> **Schematic and layout done, not fabricated yet.**
> The schematic now uses a standard SOT-23-5 LDO, a 1206 alloy shunt and JLCPCB basic 0402 passives. The layout file is on 4 layers with those footprints and nets in, at the old positions, and is not placed or rerouted yet: footprints overlap and the tracks are the earlier ones.
> Numbers and behavior below are design intent, not measured. The sense chain uses the dev board's (osc-dev-v006 Rev. 2B) topology with its own shunt and gain values, and firmware has never run on *this* board. Don't fab this without doing your own review first, and expect changes when bringup happens.

OpenServoCore swap board for SG90 and MG90 class hobby servos. It replaces the factory PCB inside the servo case. CH32V006 based, single-wire TTL servo bus ([osc-native protocol](../../../docs/osc-native-protocol.md)).

## Overview

- **MCU** - CH32V006F8U7 (RISC-V, 48 MHz, 62 KB flash, 8 KB RAM, UQFN-20, 105 C grade).
- **Motor driver** - TI DRV8837 (U3). H-bridge with IN1/IN2 PWM, 1.8 A peak, VM up to 11 V. The exposed pad sits on `PGND` with two vias into a small back-side pour.
- **Current sense** - 100 mOhm kelvin-connected low-side shunt (Rs1) feeding the MCU's on-chip op-amp in bare mode. An external four-resistor difference network sets G = 12000 / 1500 = 8. Rg1 / Rg2 (1K5) bring the shunt taps into the op-amp inputs, Rf2 (12K) is the feedback resistor, and Rf1 (12K) ties the + input to `VREF`, a 2K / 1K divider off `+3V3` (Rd1 / Rd2, about 1.09 V). Two pull-ups (Ri1 / Ri2, 4K7) lift the op-amp inputs to about 0.82 V. With the divider's source resistance and the pull-ups included, the chain works out to about 0.804 V/A (about 1 mA per count) with zero current at about 1.09 V, so both current directions are visible. The output saturates near +2.56 A and -1.32 A nominal. The network is 1 % parts, so the zero spreads about +/-0.2 V board to board and firmware has to measure it at boot.
- **LDO** - MaxLinear SPX3819M5-L-3-3 (U1), 500 mA, SOT-23-5, 2.5-16 V input (20 V absolute max), +/-1 % output, with thermal shutdown. `+3V3` is also the ADC reference, which is why the regulator is a 1 % part. EN is tied to `VSYS`, and Cp3 (10 nF) on the BYP pin lowers the output noise. Cp3 is optional, leave it out if space is short. The LDO runs straight off `VSYS`, so at 8.4 V each milliamp of `+3V3` costs about 5 mW in it, at 191 C/W junction to ambient. The `+3V3` budget is about 35 mA: the board itself draws about 14 mA, which leaves at least 15 mA for the J5 flex.
- **Servo bus** - single-wire half-duplex UART on `PC0`, wired straight to the MCU through a 22 Ohm series resistor (Rx1). No buffer. Dx1 (RClamp0521T) clamps the line at the connector, ahead of Rx1. Rx2 is a 100K idle keeper to `+3V3` on the MCU side, so `DATA` never floats with no host on the wire. It is too weak to be the bus pull-up, which lives at the host end.
- **Power input** - 2S Li-ion (up to 8.4 V) on the servo cable. There is no bulk capacitor, only about 10 uF of ceramic, so avoid hot-plugging into a stiff 2S supply until bringup has scoped it.
- **Debug** - WCH-LinkE over the CH32V006 1-wire SWDIO, on four pogo pads on the back (J3).
- **Position feedback** - potentiometer or analog magnetic encoder on J4 (one or two channels). The J5 flex landing also brings out two encoder inputs, but only for experiments (see J5).
- **Supply and terminal sense** - `VSYS` measured through its own divider, plus both motor terminals referenced to a common bias (`VB`) so back-EMF is readable while the bridge coasts.
- **Temperature** - onboard NTC (TH1, 10K, B25/50 3950) or an external NTC on J5, selected with JP2.
- **Board** - 13 x 10 mm, 4-layer 1.6 mm (JLCPCB JLC04161H-3313 stackup), ENIG. All parts on the front. The back only has wire pads, pogo pads and solder jumpers.

## MCU pinout

CH32V006F8U7, UQFN-20.

|Pin|Port|Net|Function used|
|---|---|---|---|
|1|PD7|`RST` (JP1 to `OPA_P`)|Op-amp + input (OPP1), once JP1 is closed|
|2|PA1|`VPOS1`|ADC A1, position channel 1|
|3|PA2|`VPOS2_NTC`|ADC A0, position channel 2 or NTC (JP2)|
|4|VSS|`GND`||
|5|PD0|`OPA_N`|Op-amp - input (OPN1)|
|6|VDD|`+3V3`||
|7|PC0|`DATA`|USART1 TX (Remap3), half-duplex bus|
|8|PC1|-|Not connected|
|9|PC2|-|Not connected|
|10|PC3|`DRV_EN`|DRV8837 nSLEEP, GPIO|
|11|PC4|`VSNS`|ADC A2, supply sense|
|12|PC5|`IN1`|TIM1 CH2 (Remap8), DRV8837 IN1|
|13|PC6|`IN2`|TIM1 CH3 (Remap8), DRV8837 IN2|
|14|PC7|-|Not connected|
|15|PD1|`SWDIO`|Debug|
|16|PD2|`ENCB`|ADC A3, encoder B|
|17|PD3|`ENCA`|ADC A4, encoder A|
|18|PD4|`OPA_OUT`|Op-amp output, ADC A7|
|19|PD5|`VSNA`|ADC A5, terminal A sense|
|20|PD6|`VSNB`|ADC A6, terminal B sense|
|EP|VSS|`GND`||

`IN1` / `IN2` sit on TIM1 CH2 / CH3 here, the reverse of the dev board. Firmware for this board has to swap `MOTOR_IN1_CH` and `MOTOR_IN2_CH` (in `firmware/servo-ch32/src/cfg/chip.rs`). There is no sg90 firmware config yet.

## Connectors

Everything is a solder pad. There are no through-hole parts.

### Servo cable - J1 (3 wire pads, front and back)

The front and back pads are tied together through via-in-pad, so a second servo can be chained off the back.

|Pin|Net|Notes|
|---|---|---|
|1|`GND`||
|2|`VSYS`|2S Li-ion, up to 8.4 V|
|3|`DATA`|Half-duplex UART, osc-native protocol|

### Motor - J2 (2 pads, back)

One pad at each left corner, where the SG90 motor tabs land.

|Pin|Net|
|---|---|
|1|`MOT_A`|
|2|`MOT_B`|

### Debug - J3 (4 pogo pads, 1.27 mm pitch, back)

|Pin|Net|
|---|---|
|1|`+3V3`|
|2|`GND`|
|3|`SWDIO`|
|4|`GND`|

### Position sensor - J4 (4 wire pads, 1.0 mm pitch, front)

|Pin|Net|Notes|
|---|---|---|
|1|`+3V3`|Pot top rail / sensor supply|
|2|`GND`||
|3|`POS1`|Wiper, or sensor channel 1|
|4|`POS2`|Sensor channel 2, only when JP2 is open|

Each channel goes through a 4K7 + 100 nF RC filter (fc about 340 Hz). Both filters match, so a sin/cos pair stays phase-matched.

### Flex - J5 (6 pads, back, left edge)

Landing for a flex that carries an external NTC, which works today. The two encoder inputs are experimental: there is no firmware for a motor-side encoder on this board, and the CH32V006 has no timers or CPU time left for one. A reflective IR pair at its benched 20 mA per LED draws about 40 mA, well past the `+3V3` budget at 8.4 V. For experiments, run the board from a lower supply (about 5 V or one cell), where the LDO drops much less.

|Pin|Net|Notes|
|---|---|---|
|1|`GND`||
|2|`+3V3`|Keep the load under about 15 mA, enough for two IR LEDs at about 4-5 mA each|
|3|`ENCA`||
|4|`ENCB`||
|5|`VNTC_EXT`|External NTC, needs its own pull-up on the flex|
|6|`GND`||

### Solder jumpers

- **JP1** - leave it open for the first flash, which writes `RST_MODE` so PD7 stops being the reset pin. Then close it. The op-amp + input shares PD7 with NRST.
- **JP2** - picks what ADC A0 reads. B-C is the onboard NTC, A-C is the external NTC on J5, and open leaves it on `POS2` from J4. It ships open.

## Design intent

These are the targets the board was laid out around. None are measured.

- **Ground split** - `PGND` (driver and motor current) and `GND` meet only at Rs1. The driver supply caps (Ch1-Ch3) return to `GND`, so the motor loop has to cross the shunt.
- **Current sense** - the shunt taps are kelvin net ties (NT1 / NT2) on the shunt pads, straight into the gain resistors. The front ground pour is kept off the op-amp network, with solid ground on In1 underneath.
- **Voltage sensing** - 10K / 2K2 dividers (ratio 0.180) on `VSNS`, `VSNA` and `VSNB`. 8.4 V reads 1.51 V at the ADC. The terminal divider bottoms return to `VB` (Rb1 1K / Rb2 220, about 0.60 V) instead of GND, so a terminal swinging below ground still reads.
- **Precision parts** - only two parts are picked with care. The shunt is a 1206 metal alloy part (JIERR JER1206F1R100, 1 %, 50 ppm, 1 W); if the 1206 does not fit, the backup is the 0805 Yageo PE0805FRF470R1L (C2935234, 1 %, 100 ppm, 0.5 W), and the LDO is the 1 % SPX3819 because it is the ADC reference. Every other resistor is a 1 % 100 ppm JLCPCB basic 0402 part. Firmware measures the current zero and `VB` at boot, and motor identification absorbs the fixed gain error of each board. The terminal filter caps are C0G.
- **Motor path** - the motor traces run on the back, 0.4 mm wide, with the In4 ground plane between them and the In2 signal layer.
- **Supply path** - `VSYS` runs from J1 to the driver as a pour on In3, next to the `+3V3` plane.

## Fabrication

- 4-layer JLCPCB stackup JLC04161H-3313, 1.6 mm, ENIG.
- 0.09 / 0.09 mm track and space, 0.3 / 0.45 mm vias.
- Via-in-pad is used under U2, U3 and on the connector pads. Order with epoxy filled and capped vias.
- Single-sided assembly. Every part is on the front.
- The design rules live in `osc-sg90-v006.kicad_dru`.

## Mechanical fit

Designed to drop into a standard SG90 / MG90 case:

- Motor tabs solder to J2.
- The pot or sensor wires solder to J4.
- The servo cable solders to J1.
- The debug pads stay reachable for bringup and reflashing.

Pad positions are still **to be confirmed against a physical case** when the first boards arrive.

## License

Hardware files (schematic, layout, board files in this directory) are licensed under [CERN-OHL-P v2.0](../../../LICENSE-HARDWARE). See the [top-level README](../../../README.md#license) for the full licensing picture.
