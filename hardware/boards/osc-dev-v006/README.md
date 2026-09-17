# OSC Dev V006 - Rev. 2A

> **Pre-fabrication / pre-bringup.**
> Rev. 2A has not been fabricated or validated yet. Everything below describes the design as it heads to fab. Pinouts, signal names, jumper behavior, and component values may still change during bringup.
> **Fabricate this at your own risk!**

See [CHANGELOG.md](CHANGELOG.md) for revision history.

This is the OpenServoCore firmware development and validation board. It is built around the CH32V006F8P6 and designed to accept any gutted hobby servo (SG90, MG90, and similar), so firmware can be brought up and characterized against real motor / pot / encoder hardware.

![Front](docs/front.webp)

The render predates the 2A refresh, so the connector set on it is one revision behind the schematic.

## Overview

- **MCU** - CH32V006F8P6 (RISC-V, 48 MHz, 62 KB flash, 8 KB RAM, TSSOP20). Every pin is used, including the reset pin.
- **Motor driver** - TI DRV8212PDSGR (U3). H-bridge with IN1/IN2 PWM. 4 A peak, 1.76 A RMS continuous, VM 1.65-11 V.
- **Current sense** - 60 mOhm kelvin-connected low-side shunt (Rs1) feeding the MCU's on-chip op-amp in bare mode. An external four-resistor difference network sets G = 14.88 against a 0.52 V bias, so both current directions are visible.
- **LDO** - HT7533-1 (U1) for the 3.3 V logic rail. The DRV8212P VCC (`+3V3_DRV`) hangs off the same `+3V3` as the MCU through Rh2, a fitted 0R that a ferrite can replace.
- **Servo bus** - single-wire half-duplex UART on `PC0`, wired straight to the MCU through a 33 Ohm series resistor. No buffer, no TX_EN. Direction turnaround and RX timing are firmware's job.
- **Telemetry** - no dedicated pin. Telemetry rides the `DATA` wire as bounded CRC'd bursts, so there is no second UART on the board.
- **Qwiic** - SH 1.0 mm 4P I2C connector (J3) for an encoder module, either magnetic (I2C) or a quadrature encoder breakout.
- **Power input** - 1S-2S LiPo (3.0-8.4 V) via JST-PH, or the WCH-LinkE 5 V rail. Either or both, OR'd through SS54 Schottkys.
- **Debug** - WCH-LinkE over the CH32V006 1-wire SWDIO, on a 1x03 header (J2).
- **Position feedback** - potentiometer or analog magnetic encoder on J5, I2C encoder on J3, or an ADC-sampled IR encoder on J6 (pins) or J11 (flex landing).
- **Supply and terminal sense** - `VSYS` measured directly through its own divider, plus both motor terminals referenced to a common bias so back-EMF is readable while the bridge coasts.
- **Temperature** - onboard NTC (TH1) or external NTC (J7), selected with JP2.
- **Board** - 50 x 40 mm, 6-layer 1.6 mm (JLCPCB JLC06161H-3313 stackup), ENIG, via-in-pad.

### Changed from Rev. B

Rev. 2A is a big respin. Here is what changed:

- The 74LVC2G241 UART buffer is deleted. The bus `DATA` line connects straight to the MCU through a 33 Ohm series resistor (Rx1), and RX timing moved fully into firmware.
- Current sense is reworked around the CH32V006's on-chip op-amp: kelvin shunt, external difference network, and a VREF bias for bidirectional current.
- Motor terminal sense divider bottoms return to a bias node (`VB`) instead of GND, so both terminals stay readable during coast.
- `VSNS` is back, on its own pin, as a direct divider off `VSYS`.
- USB-C and screw-terminal power inputs are dropped. Battery and LinkE 5 V remain.
- The PWM servo header is dropped. Qwiic (I2C) is added.
- The edge test-point pads are replaced by three GND probe points. Signals are probed at their silk-labeled component pads.
- 4 layers to 6 layers, and all connectors are renumbered.

## MCU pinout

CH32V006F8P6, TSSOP20. The "Function used" column is what the board wires the pin for.

|Pin|Port|Net|Function used|
|---|---|---|---|
|1|`PD4`|`OPA_OUT`|Op-amp output (`OPO0`). The same pin is ADC `A7`, which samples it.|
|2|`PD5`|`VSNA`|ADC `A5`, motor terminal A sense.|
|3|`PD6`|`VSNB`|ADC `A6`, motor terminal B sense.|
|4|`PD7`|`RST`|`nRST` from the factory. After the option byte flip it becomes the op-amp `+` input (`OPP1`) through JP1. See [First boot](#first-boot).|
|5|`PA1`|`VPOS1`|ADC `A1`, position channel 1: pot wiper or the first transducer output (RC-filtered).|
|6|`PA2`|`VPOS2_NTC`|ADC `A0`, position channel 2 for a dual-transducer sensor, or the NTC divider when JP2 is jumped.|
|7|-|`GND`|`VSS`.|
|8|`PD0`|`OPA_N`|Op-amp `-` input (`OPN1`), difference network summing node.|
|9|-|`+3V3`|`VDD`.|
|10|`PC0`|`DATA`|USART1 TX (remap 3), half-duplex (HDSEL) servo bus.|
|11|`PC1`|`SDA`|I2C1 SDA, Qwiic (J3).|
|12|`PC2`|`SCL`|I2C1 SCL, Qwiic (J3).|
|13|`PC3`|`DRV_EN`|GPIO to the DRV8212P nSLEEP. A 10K pulldown (Rh1) keeps the driver asleep at reset.|
|14|`PC4`|`VSNS`|ADC `A2`, `VSYS` supply divider.|
|15|`PC5`|`DRV_IN2`|TIM1 CH2 (remap 8), H-bridge PWM.|
|16|`PC6`|`DRV_IN1`|TIM1 CH3 (remap 8), H-bridge PWM.|
|17|`PC7`|`STAT`|STAT LED, active low. The only firmware-driven lamp.|
|18|`PD1`|`SWDIO`|1-wire debug, 100 Ohm series (Ru1) to J2.|
|19|`PD2`|`ENCB`|Encoder B. ADC `A3`, analog quadrature sampling.|
|20|`PD3`|`ENCA`|Encoder A. ADC `A4`, analog quadrature sampling.|

TIM1 runs center-aligned PWM on IN1/IN2. CCR4 is not wired out; it is the ADC sampling-phase knob.

## Connectors

All 2.54 mm pin headers are vertical through-hole.

### Battery - J1

JST-PH 2P horizontal. **1S-2S LiPo only (3.0-8.4 V).** The DRV8212P caps the safe motor rail at 11 V, so 3S is not supported. This input is OR'd into `VSYS` through an SS54 (Dp1), so it coexists with LinkE power.

### WCH-LinkE - J2

1x03 pin header (2.54 mm) for the WCH-LinkE programmer / debugger.

|Pin|Net|Notes|
|---|---|---|
|1|`GND`||
|2|`VPROG`|LinkE 5 V rail, OR'd into `VSYS` through an SS54 (Dp2).|
|3|`SWDIO`|1-wire debug, through a 100 Ohm series resistor (Ru1).|

Three pins is the whole header now. Telemetry rides the servo bus, so the LinkE's TX / RX are not connected to anything.

Plugging in the LinkE alone is enough to power the board. Very handy for firmware-only sessions.

### Qwiic - J3

SH 1.0 mm 4P (GND / +3V3 / SDA / SCL), standard [Qwiic](https://www.sparkfun.com/qwiic) pinout. 4K7 pull-ups (Rq1 / Rq2) sit on the MCU side, and 100 Ohm series resistors (Rq3 / Rq4) sit between the MCU and the connector.

This one is meant for an encoder module rather than generic sensors: an I2C magnetic encoder (MT6701 / AS5600 breakout) or a quadrature encoder adapter. The board has no hardware quadrature counting (see J6), so Qwiic is the flexible stand-in for it. I2C1 is polled.

### Motor - J4

1x02 pin header carrying `MOT_A` / `MOT_B` straight off the DRV8212P outputs. Through-hole, so the high-current path stays zero-via.

### Position - J5

1x04 pin header, `+3V3` / `GND` / `POS1` / `POS2`. It takes:

- The gutted servo's **potentiometer** on `POS1`. Excitation comes from `+3V3` and the wiper goes into `POS1`. Excitation and ADC reference share the same rail, so the reading is ratiometric and supply drift cancels out.
- An **analog-output magnetic encoder** (e.g. MT6701 in ratiometric analog mode) as a drop-in pot replacement on the same first three pins.
- A **dual-output transducer** (sin / cos style, the tier-2 sensor) on `POS1` and `POS2`. `POS2` shares its ADC pin with the NTC, so leave JP2 open in this mode; the on-board temperature reading is given up for the second channel. Both channels sit behind identical 4K7 / 100nF filters (Ra1 / Ca1 and Ra2 / Ca2) so the pair stays phase-matched.

With a pot or a single-output encoder, leave `POS2` unconnected and jump JP2 for the NTC. For a digital magnetic encoder use Qwiic (I2C) instead.

### Encoder - J6

2x02 pin header. `+3V3` / `GND` on one row, `ENCA` / `ENCB` on the other, going into ADC channels `A4` / `A3`.

This is a future expansion connector for an ADC-sampled custom IR quadrature encoder, in the style of [ServoProject](https://github.com/adamb314/ServoProject). The analog A/B phases get oversampled and interpolated for sub-count resolution. Hardware quadrature counting is not possible here since `PD2` / `PD3` don't carry TIM2 CH1/CH2, so I never considered it. A digital encoder belongs on Qwiic instead.

### Motor encoder flex - J11

Six bare pads at 1.0 mm pitch for the motor-shaft encoder flex, attached tin-and-iron: the flex fingers are pre-tinned, laid on the pads and pressed with an iron, then hot glue over the pad row takes the strain. The footprint pair (`Flex_Landing_1x06_P1.0mm` on the board, `Flex_Fingers_1x06_P1.0mm` on the flex) lives in the shared library, and the same landing goes on the swap board, so the dev board rehearses the real attach. Pin 1 is at the bracket mark.

|Pin|Net|Notes|
|---|---|---|
|1|`GND`|Outer ground, guards the pair.|
|2|`+3V3`|Emitter and phototransistor supply.|
|3|`ENCA`|IR sensor A, ADC `A4`. Same net as J6.|
|4|`ENCB`|IR sensor B, ADC `A3`. Same net as J6.|
|5|`VNTC_EXT`|Thermistor on the motor can, into the external leg of JP2. The divider resistor sits on the flex.|
|6|`GND`|Outer ground.|

J6 and J11 are the same encoder input in two shapes: pins for a wired breakout or the bench coupon, the landing for the flex. Populate one.

### External NTC - J7

1x03 pin header, `VNTC_EXT` / `+3V3` / `GND`, for an external NTC thermistor or temperature sensor. Select it with JP2. The `+3V3` pin is there so the external part can bring its own pull-up or supply: a bare thermistor divides against a resistor on the connector, a sensor with a driven output just takes the rail. Rn1 10K serves the internal TH1 leg only, so nothing on the board loads the external node.

### Servo TTL bus - J8 / J9 / J10

OpenServoCore talks over a single-wire half-duplex UART, the same electrical layer as a TTL servo bus. Three wires only (`GND` / `V+` / `DATA`), the same pin count as a hobby servo cable. The wire protocol on top is **osc-native**, OSC's own break-framed protocol built for sub-$0.20 MCUs ([spec](../../../docs/osc-native-protocol.md)).

Three connectors share the same nets. Pick whichever fits your wiring:

- **J8** - 1x03 pin header for breadboard / daisy-chain wiring.
- **J9, J10** - JST-PH 3P vertical for cables.

Pin order is `G` / `V` / `D` with `V` in the center, so a reversed bench header only swaps `DATA` and `GND`, never power.

The powered board feeds `VSYS` onto `V+` to power downstream boards over the same cable. On the `DATA` line the order is connector, then ESD clamp (Dx1, PESD5V0L1BA), then the 33 Ohm series resistor (Rx1), then the pin. Rx1 is 33 Ohm rather than the usual 100 Ohm because `DATA` is the one line where edge speed matters at 3 Mbps over a whole chain, and the clamp does the protection work.

Rx2 is a DNP 10K pull-up footprint. The bus wants exactly one pull-up, at the host end, and servos self-bias internally, so populate Rx2 only when this board is the end of the wire on the bench.

**`V+` is `VSYS` directly, unprotected.** Anything you daisy-chain to the bus must tolerate the upstream board's full input voltage (up to 8.4 V at 2S full charge).

## Jumpers

### Bootstrap - JP1 (solder jumper)

This jumper bridges the op-amp `+` input network (`OPA_P`) onto the shared `nRST` / `OPP1` pin (`PD7`). It ships open on fresh boards and gets closed as the last step of [first boot](#first-boot). Current sense is inactive until it is closed.

### NTC source select - JP2

1x03 pin header, silk `EX` / `NTC` / `IN`. The center pin is the common signal routed to ADC `A0` through Ra2 / Ca2. Jump it to either side, or leave it open:

- `NTC` to `IN` uses the onboard Rn1 / TH1 divider.
- `EX` to `NTC` uses the external thermistor on J7.
- Open: the pin is position channel 2 (`POS2` on J5) and there is no temperature reading.

## Current sense

Motor return current flows through Rs1 (60 mOhm, 1 %, 50 ppm, 1206) between `PGND` and `GND`. Rs1 is the only tie between the two grounds, so every amp of motor return has to cross it. Net-tie kelvin taps (NT1 on the `PGND` pad, NT2 on the `GND` pad) feed the MCU's on-chip op-amp, which runs in bare mode behind an external four-resistor difference network.

- **Gain.** Rf 6K4 / Rg 430 gives G = 14.88. With the 60 mOhm shunt that is 893 mV/A at the ADC, about 0.9 mA per LSB.
- **Bias.** The amplifier references `VREF` = 0.52 V (Rd1 1K6 / Rd2 300 from `+3V3`, decoupled by Cd1), so negative current from regen or reversal is visible too. The usable range is roughly -0.58 A to +3.1 A.
- **Compensation.** Cc1 / Cc2, 22 pF across each feedback arm, for a corner around 1.1 MHz.
- **Input filter.** Ci1, 100 pF across `OPA_P` / `OPA_N`.

Rk1 / Rk2 (fitted 0R) sit in series with the kelvin taps. They are swap points for gain-trim experiments: if they ever become real resistors, swap both to the same thin-film value or the arms go out of balance.

Precision money goes to the shunt (1 %, 50 ppm) and the Rf / Rg quad (0.5 %, 25 ppm thin film). Matching and drift are what limit accuracy there and firmware cannot compensate for either. Everything downstream is offset error and boot calibration eats it.

This is a swap-and-measure board, so the populated values are a starting point and a set of experiment footprints ship unpopulated (DNP):

|Ref|Value|Purpose when fitted|
|---|---|---|
|`Cc3` / `Cc4`|22 pF|Stack on the 22 pF comp caps for a slower, quieter corner (about 570 kHz).|
|`Ck1`|100 pF|Differential filter across the kelvin pair, ahead of the gain resistors.|
|`Co1`|100 pF|Load cap on the op-amp output. Layout insurance.|
|`Rd3`|300|Parallels Rd2, dropping `VREF` to 0.28 V for a near-unipolar range.|
|`Rx2`|10K|`DATA` bus pull-up for single-device bench setups.|

The shunt itself has no alternate footprint. Other values (22 to 150 mOhm, all 1206) swap onto Rs1's own pads, so the kelvin taps never move. A second shunt in parallel would split the current by pad and trace resistance and break the symmetric entry, so there is no pad for one.

The DRV8212P's own OCP / TSD is the first protection layer. V006 has no comparator units, so the rest is firmware: a kernel I2t limit drops `DRV_EN` and latches a stall fault that only the user can clear, IWDG covers hung firmware, and any reset kills the bridge through the Rh1 pulldown.

## Other sensing

### Motor terminal voltage

`MOT_A` / `MOT_B` go through Rv1 / Rv3 (6K4) into ADC `A5` / `A6`, with Rv2 / Rv4 (1K6) and Cv1 / Cv2 (100 pF) returning to `VB` instead of `GND`.

`VB` is a 0.623 V bias node (Rb1 430 / Rb2 100 from `+3V3`, bulk Cb1 10uF and Cb2 100nF). The divider ratio is 0.20, so a tap reads `0.2 x terminal + 0.8 x VB`:

- A 2S rail at 8.4 V taps about 2.18 V, well inside the ADC range.
- During coast, the free terminal sits a diode below `PGND` or a diode above `VSYS`. At -0.9 V the tap is still 0.32 V, and at 9.3 V it is 2.36 V, so neither phase clips.

That is the point of the bias: both terminals stay readable while the bridge coasts, so `vA - vB` is real back-EMF rather than a rail-clamped stub. The rail comes back as `5 x tap - 4 x VB`.

`VB` is read at boot with the driver parked. With the bridge Hi-Z no current flows in either leg, so both taps sit at `VB` itself and firmware recovers the bias with no extra pin. The bias also shifts with drive current, since the divider bottoms inject into the `VB` node against its own 81 Ohm source impedance: about +80 mV at full duty on a 2S rail, which the 0.20 ratio turns into 0.3 V on a single-terminal reading. Both terminals are measured, so the injected current is known and firmware corrects `VB` from the two taps with one multiply-add; `vA - vB` is immune either way.

### Supply voltage

`VSYS` has its own divider straight to `GND`: Rv5 6K4 / Rv6 1K6 with Cv3 100 pF, into ADC `A2`. Divide by 5, so 8.4 V lands at 1.68 V. Same reels and the same phase as the terminal dividers, so the two chains track each other and rail minus terminal resolves the bridge drop instead of disappearing into a ratio disagreement.

### Position

Two channels with the same filter. `POS1` goes through Ra1 4K7 and Ca1 100nF into ADC `A1`. `POS2` goes through Ra2 4K7 and Ca2 100nF into ADC `A0`, the node JP2 also drives, so it is either the second transducer output or the NTC, never both.

### Temperature

Rn1 10K pulls up against the onboard 10K / 3950 TH1 on the internal leg of JP2. JP2 routes that divider (or the external thermistor on J7) onto the shared `A0` node, where Ca2 100nF is the filter for every source; Ra2 is only in the `POS2` path.

## Reference designators

Passives are numbered by the net they serve, not by the sheet they are drawn on. One letter means one thing across R, C and D, so `Rv1` and `Cv1` are part of the same divider. U, J, JP, TP, NT, TH, H and LOGO keep plain numbers.

|Letter|Family|Examples|
|---|---|---|
|`f`|OPA feedback|Rf1, Rf2|
|`g`|OPA gain|Rg1, Rg2|
|`c`|OPA compensation|Cc1 - Cc4|
|`i`|OPA input|Ci1|
|`o`|OPA output|Co1|
|`d`|VREF divider|Rd1 - Rd3, Cd1|
|`k`|Kelvin taps|Rk1, Rk2, Ck1|
|`s`|Shunt|Rs1|
|`v`|Terminal and supply dividers|Rv1 - Rv6, Cv1 - Cv3|
|`b`|VB bias|Rb1, Rb2, Cb1, Cb2|
|`n`|NTC|Rn1|
|`p`|Power input and LDO|Cp1 - Cp3, Dp1, Dp2|
|`l`|Lamps (LEDs and their resistors)|Dl1 - Dl4, Rl1 - Rl4|
|`u`|MCU supply and debug|Cu1, Cu2, Ru1|
|`q`|Qwiic I2C|Rq1 - Rq4|
|`h`|H-bridge|Ch1 - Ch5, Rh1, Rh2|
|`a`|Angle sensor (pot)|Ra1, Ca1|
|`x`|One-wire bus|Rx1, Rx2, Dx1|

## LEDs

All four are deliberately dim, in the 0.1-0.3 mA class.

|LED|Ref|Color|Meaning|
|---|---|---|---|
|`VSYS`|Dl1|yellow|System rail present.|
|`3V3`|Dl2|green|Logic rail up.|
|`DAT`|Dl3|blue|Servo bus activity (active low). Passive lamp on the `DATA` line.|
|`STA`|Dl4|red|MCU-driven status (`PC7`, active low).|

## Test points & probing

Three GND probe points (TP1-TP3) are spread across the board for scope ground springs. The 4x M2 mounting holes (2.2 mm) are also tied to `GND`, handy for an alligator clip. Signals are probed at their component pads, and the nets of interest are silk-labeled.

## Power and grounding

Battery (`VBAT`) and LinkE 5 V (`VPROG`) OR into `VSYS` through Dp1 / Dp2. `VSYS` is the raw motor rail and also feeds the LDO. `+3V3` is logic and, through Rh2, the DRV8212P VCC (`+3V3_DRV`). Rh2 ships as 0R; it is the swap point for a ferrite (600 Ohm at 100 MHz) if driver noise ever shows on the MCU rail, which is also the ADC reference.

The bridge decoupling (Ch1 100nF on `+3V3_DRV`, Ch2 100nF plus Ch3 / Ch4 10uF plus Ch5 100uF on `VSYS`) returns to `GND`, not `PGND`. That is deliberate. At PWM timescales the motor current loops cap to bridge to motor to `PGND` and back to the cap. If those caps landed on the `PGND` side, the loop would close without crossing the shunt and the ADC would only see average draw instead of the real chopped current.

`PGND` is a tiny top-layer island: the driver's GND pins and thermal pad, Rh1, NT1, and the `PGND` side of the shunt. Nothing else.

## Programming

Use a **WCH-LinkE** on J2. The link talks the CH32V006's 1-wire protocol on `SWDIO`.

### First boot

The MCU's `nRST` and the op-amp `+` input share pin `PD7`. Fresh chips come out of the factory with `RST_MODE=10`, so the pin is reset, and the sense network parks it near GND, which would hold the MCU in reset if it were connected. JP1 breaks the link until the option byte is flipped:

1. Make sure JP1 is open (no solder bridging the pads).
2. Flash the board, then program the USER option bytes over the WCH-LinkE to set `RST_MODE=11` so `PD7` becomes a GPIO.
3. Close JP1 with a small solder blob.

Do not close JP1 early. On an unprovisioned chip the op-amp network holds the pin near GND and the chip sits in reset. Recovery is reopening the jumper.

After this, the board runs as designed and current sense is live.
