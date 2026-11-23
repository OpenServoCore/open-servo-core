# OSC SG90 V006

> **Schematic and layout done, not fabricated yet.**
> Numbers and behavior below are design intent, not measured. The sense chain uses the dev board's (osc-dev-v006 Rev. 2B) topology with its own shunt and gain values, and firmware has never run on *this* board. Don't fab this without doing your own review first, and expect changes when bringup happens.

OpenServoCore swap board for SG90 and MG90 class hobby servos. It replaces the factory PCB inside the servo case. CH32V006 based, single-wire TTL servo bus ([osc-native protocol](../../../docs/osc-native-protocol.md)).

## Overview

- **MCU** - CH32V006F8U7 (RISC-V, 48 MHz, 62 KB flash, 8 KB RAM, UQFN-20, 105 C grade).
- **Motor driver** - TI DRV8837 (U3). H-bridge with IN1/IN2 PWM, 1.8 A peak, VM up to 11 V. The exposed pad sits on `PGND` with two vias into a small back-side pour.
- **Current sense** - 100 mOhm kelvin-connected low-side shunt (Rs1) feeding the MCU's on-chip op-amp in bare mode. An external four-resistor difference network sets G = 6200 / 680 = 9.1. Rg1 / Rg2 (680) bring the shunt taps into the op-amp inputs, Rf2 (6K2) is the feedback resistor, and Rf1 (6K2) ties the + input to `VREF`, a 1K6 / 820 divider off `+3V3` (Rd1 / Rd2, about 1.12 V). Two pull-ups (Ri1 / Ri2, 1K5) lift the op-amp inputs to about 1.04 V. With the divider's source resistance and the pull-ups included, the chain works out to about 0.917 V/A with zero current at about 1.11 V, so both current directions are visible. The output saturates near +2.21 A and -1.18 A nominal.
- **LDO** - Broadchip BCT2014ELS33 (U1), 300 mA, 1 x 1 mm, 2.5-15 V input. The board itself draws about 20 mA from `+3V3`, which is about 0.1 W in the LDO at 8.4 V. Keep the J5 flex load under about 30 mA or the LDO runs hot.
- **Servo bus** - single-wire half-duplex UART on `PC0`, wired straight to the MCU through a 22 Ohm series resistor (Rx1). No buffer. Dx1 (RClamp0521T) clamps the line at the connector, ahead of Rx1. Rx2 is a 100K idle keeper to `+3V3` on the MCU side, so `DATA` never floats with no host on the wire. It is too weak to be the bus pull-up, which lives at the host end.
- **Power input** - 2S Li-ion (up to 8.4 V) on the servo cable. There is no bulk capacitor, only about 10 uF of ceramic, so avoid hot-plugging into a stiff 2S supply until bringup has scoped it.
- **Debug** - WCH-LinkE over the CH32V006 1-wire SWDIO, on four pogo pads on the back (J3).
- **Position feedback** - potentiometer or analog magnetic encoder on J4 (one or two channels), or an encoder on the J5 flex.
- **Supply and terminal sense** - `VSYS` measured through its own divider, plus both motor terminals referenced to a common bias (`VB`) so back-EMF is readable while the bridge coasts.
- **Temperature** - onboard NTC (TH1, 10K 3950) or an external NTC on J5, selected with JP2.
- **Board** - 12.1 x 9.6 mm, 6-layer 1.6 mm (JLCPCB JLC06161H-3313 stackup), ENIG. All parts on the front. The back only has wire pads, pogo pads and solder jumpers.

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

Landing for an encoder flex that can also carry an external NTC.

|Pin|Net|Notes|
|---|---|---|
|1|`GND`||
|2|`+3V3`|Keep the load under about 30 mA|
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
- **Voltage sensing** - 6K8 / 1K6 dividers (ratio 0.190) on `VSNS`, `VSNA` and `VSNB`. 8.4 V reads 1.60 V at the ADC. The terminal divider bottoms return to `VB` (about 0.62 V) instead of GND, so a terminal swinging below ground still reads.
- **Precision parts** - the gain, divider and `VREF` resistors are thin film, 0.1-0.5 % and 25 ppm. In the `VB` bias, Rb1 is thin film too, but Rb2 is 1 % thick film. The terminal filter caps are C0G.
- **Motor path** - the motor traces run on the back, 0.4 mm wide, with the In4 ground plane between them and the In2 signal layer.
- **Supply path** - `VSYS` runs from J1 to the driver as a pour on In3, next to the `+3V3` plane.

## Fabrication

- 6-layer JLCPCB stackup JLC06161H-3313, 1.6 mm, ENIG.
- 0.09 / 0.09 mm track and space, 0.3 / 0.45 mm vias.
- Via-in-pad is used under U2, U3, U1 and on the connector pads. Order with epoxy filled and capped vias.
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
