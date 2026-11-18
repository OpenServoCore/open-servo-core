# OSC Dev CH32 - Changelog

Hardware revisions of the OSC Dev CH32 (`osc-dev-v006`) board. Newest first.

To get files at a specific revision, check out the corresponding tag (e.g. `git checkout osc-dev-v006-rev-a`) or browse the merge commit linked below.

---

## Rev 2B - 2026-11-18

**Status:** Schematic and layout done, not fabricated yet. The sense chain is what Rev. 2A board #1 and the reworked bench boards already run, now in copper. Board silk reads `Dev V006 Rev. 2B`.

### Removed

- Every DNP experiment footprint: comp caps `Cc1` - `Cc4`, input cap `Ci1`, output cap `Co1`, kelvin filter `Ck1` and VREF shift `Rd3`. Board #1 measured its lowest current floor with the comp caps out, and 0 - 47 pF across the op-amp inputs made no difference.
- `Rk1` / `Rk2` 0R kelvin swap points. The taps run straight from the net ties to Rg1 / Rg2.
- `Rh2` 0R and the `+3V3_DRV` net. The DRV8212P VCC and Ch1 sit straight on `+3V3`.

### Added

- **Op-amp input pull-ups** `Ri1` / `Ri2`, 1K5 0.5 % 25 ppm thin film from `+3V3` to `OPA_P` and `OPA_N`. They lift the input common mode from about 0.06 V to about 0.75 V, off the ramp of the op-amp's offset curve near GND. The Rev. 2A bodges were 1 % thick film, and each 1 % of mismatch against Rg moves the current zero about 0.11 V, so these match the Rg family.

### Changed

- **VREF 0.52 V -> 1.09 V.** Rd2 300 -> 820, one part where board #1 stacked two 1K6. Range is about -1.2 A to +2.2 A at the measured 0.915 V/A.
- **Bus ESD clamp** PESD5V0L1BA (SOD-323, 75 pF) -> RCLAMP0521T-ES (DFN1006-2L, 0.35 pF), so 32 nodes add about 11 pF to the wire instead of about 2400 pF.
- `Rx1` 33 -> 22 Ohm, for faster `DATA` edges at 3 Mbps.
- `Rx2` 10K DNP -> 100K fitted, a weak idle keeper so `DATA` never floats with no host attached. The real pull-up stays at the host end.
- LED resistors picked so the four lamps look about as bright as each other: Rl1 20K -> 68K (`VSYS`, still visible on LinkE 5 V), Rl2 10K -> 20K, Rl3 4K7 -> 1K (the blue `DAT` lamp was too faint), Rl4 4K7 -> 10K.
- The 19 via-in-pad vias at 0.5 mm drill -> 0.4 mm drill, 0.8 mm size. JLCPCB left every 0.5 mm via on the Rev. 2A boards unfilled.
- GND via added at the VREF divider and Dx1's ground path shortened.
- Silk and title blocks take the revision from the `${REV}` project variable.

---

## Rev 2A - 2026-08-02

**Status:** Built and brought up. Board #1 runs an MG90 on 2S through calibration, identification and closed-loop verify; the encoder inputs, Qwiic and `POS2` are not exercised yet. Value changes that need no routing change stay Rev. 2A. Board silk reads `Dev V006 Rev. 2A`.

### Removed

- 74LVC2G241 UART bus buffer and its RX jumper. `DATA` wires directly to the MCU through a 33 Ohm series resistor (`Rx1`), and direction turnaround and RX timing are fully in firmware.
- USB-C and screw-terminal power inputs. Battery (JST-PH) and LinkE 5 V remain, still SS54-OR'd.
- PWM servo header.
- Edge test-point hook rails. Nine 1.0 mm probe pads, most of them in two rows next to the sense network, replace them: ground, op-amp output, both PWM inputs, the three divider taps and the two position channels.
- Second UART. Telemetry rides the `DATA` wire as bounded CRC'd bursts, so there is no telemetry pin, no LinkE TX / RX wiring and no TEL lamp.
- Ferrite bead on the driver logic rail. The DRV8212P VCC (`+3V3_DRV`) now hangs off the MCU's `+3V3` through Rh2, a fitted 0R that keeps the bead as a swap option.
- OPA to CMP2 hardware stall detection. V006 has no comparator units, so protection is layered instead: driver OCP / TSD, a kernel I2t limit that drops `DRV_EN` and latches a user-clearable stall fault, IWDG, and the `DRV_EN` pulldown that kills the bridge on any reset.

### Added

- **Qwiic connector** (I2C, 4K7 pull-ups, 100 Ohm series) for an encoder module, magnetic or quadrature breakout.
- **`VB` bias node** for the motor terminal dividers, so both terminals stay readable while the bridge coasts.
- **Motor encoder flex landing.** J11 is a row of six bare 1.0 mm pads (`GND` / `+3V3` / `ENCA` / `ENCB` / `VNTC_EXT` / `GND`) for the tier-3 IR encoder flex, soldered tin-and-iron. It parallels the J6 encoder pins and adds the on-can thermistor into the JP2 external leg.
- **Second analog position channel.** J5 is 1x04 (`+3V3` / `GND` / `POS1` / `POS2`) for a dual-output transducer; `POS2` shares ADC `A0` (`PA2`) with the NTC through JP2, behind the same 4K7 / 100nF filter as `POS1`. Jump JP2 for the NTC, leave it open for the second channel.
- **DNP experiment footprints** around the current sense block: comp-cap stack (`Cc3`/`Cc4`), kelvin filter (`Ck1`), output cap (`Co1`), VREF shift (`Rd3`), bus pull-up (`Rx2`), plus fitted 0R kelvin series swap points (`Rk1`/`Rk2`).
- ESD clamp (PESD5V0L1BA) on the bus `DATA` line, ahead of the series resistor.
- **Function-family reference designators.** Passives carry a letter for the net they serve rather than a flat number, so `Rv1` and `Cv1` belong to the same divider. See the README.
- QR code on the back silk linking to the board README.

### Changed

- **Current sense reworked for accuracy.** 60 mOhm kelvin-connected shunt (`Rs1`) with net-tie midpoint taps, the only tie between `PGND` and `GND`, and the on-chip op-amp running in bare mode behind an external four-resistor difference network. G = 14.88 (6K4/430) for 893 mV/A, biased at VREF = 0.52 V so both current directions are visible. Precision money goes to the shunt and the Rf / Rg quad; everything downstream is offset that boot calibration eats.
- **Motor terminal sense is now differential and coast-readable.** 6K4/1K6 dividers on both terminals, ratio 0.20, with their bottoms and filter caps returning to the 0.623 V `VB` node instead of GND. Neither the coast diode phase nor a 2S rail clips, so `vA - vB` is real back-EMF and the rail comes back as `5 x tap - 4 x VB`. `VB` is read at boot with the driver parked: with the bridge Hi-Z both taps sit at `VB` itself.
- **`VSNS` moved to its own pin.** `VSYS` is sensed directly through a 6K4/1K6 divider with a 100 pF cap into `A2` (`PC4`), instead of being inferred from a driven terminal. Ratio 0.20 and the same reels as the terminal dividers, so the two chains track and rail minus terminal resolves the bridge drop.
- Bridge decoupling returns to `GND`, not `PGND`, so the chopped motor loop has to cross the shunt instead of bypassing it.
- WCH-LinkE header is 1x03 (5 V / GND / SWDIO) instead of 2x03.
- Encoder input (J6) is ADC-only by design. `PD2`/`PD3` don't carry TIM2 CH1/CH2, so digital encoders go on Qwiic instead. J6 targets an ADC-sampled IR quadrature encoder.
- 4 layers -> 6 layers, 50 x 40 mm, JLCPCB JLC06161H-3313 stackup, ENIG, via-in-pad. The motor path is a zero-via top-layer corridor to a through-hole motor connector.
- `SB1` bootstrap solder bridge became `JP1`. Same one-time `RST_MODE=11` first-boot procedure, and the op-amp `+` input now shares the pin.
- All connectors renumbered. The LED set is now VSYS / 3V3 / DAT / STA.

---

## Rev B - 2026-04-28

**Status:** Validated. Merged in [PR #10](https://github.com/OpenServoCore/open-servo-core/pull/10). In-rev patch: `SB1` bootstrap solder bridge added for the shared `nRST` / `OPN2` pin. Version label unchanged - published files include the patch.

### Fixed (from Rev A)

- VDD / VCC rail label swap.
- Top-row test-point labels.
- Encoder connector labels.
- Battery connector polarity (JST PH 2P).
- TX_EN / UART buffer conflict - added a jumper for RX <-> buffer so UART works without firmware workaround.

### Added

- **Hardware-based stall detection / overcurrent protection** - internal OPA -> CMP2 path. No extra parts.
- **External NTC connector** with onboard / external selection jumper.
- **Traditional PWM servo header** (driven from IN1) for servo identification and measurement.
- **VSNS net** - VSYS measured directly instead of derived from VSNA / VSNB.
- **WCH-LinkE connector**: 5 V pin (replacing the unused RST pin) gated by an SS54 for safety.
- **`SB1` bootstrap solder bridge** (in-rev patch) - `nRST` and `OPN2` share a pin; the bridge breaks the path during `nRST -> GPIO` option-byte programming, then closes for normal current-sense operation. See [Fabrication](README.md#fabrication).

### Changed

- **MCU pin remap:**
  - `ENCA` / `ENCB` selectable between TIM2 and ADC.
  - `ISNS+` / `ISNS-` moved to OPP0 / OPN2 to feed the differential OPA.
  - `STAT` LED moved to a TIM1 channel.
  - `nRST` removed (USER option byte programmed at provisioning; pin freed for OPA / GPIO use).
- Reset button and its RC debounce network removed (nRST gone).
- Board renamed `servo-dev-board-ch32v006` -> `osc-dev-v006` to match the product naming convention.

---

## Rev A - 2026-03-01

**Status:** Built, validated. **Deprecated - do not fab.** Superseded by Rev B. Merged in [PR #5](https://github.com/OpenServoCore/open-servo-core/pull/5).

Initial CH32V006-based dev board. PCBWay-sponsored fab. Brought up and validated end-to-end before known issues and pin-mapping limits drove the Rev B respin.

### Known issues (fixed in Rev B)

- VDD / VCC rail label swap.
- TX_EN pull-up conflict on the half-duplex UART buffer.
- Top-row test-point labels.
- Encoder connector labels.
- Battery connector (JST PH 2P) polarity reversed.
