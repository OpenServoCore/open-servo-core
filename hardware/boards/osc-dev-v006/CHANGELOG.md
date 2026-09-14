# OSC Dev CH32 - Changelog

Hardware revisions of the OSC Dev CH32 (`osc-dev-v006`) board. Newest first.

To get files at a specific revision, check out the corresponding tag (e.g. `git checkout osc-dev-v006-rev-a`) or browse the merge commit linked below.

---

## Rev 2A - 2026-08-02

**Status:** Design in progress, not fabricated or validated. Board silk reads `Dev V006 Rev. 2A`.

### Removed

- 74LVC2G241 UART bus buffer and its RX jumper. `DATA` wires directly to the MCU through a 33 Ohm series resistor (`Rx1`), and direction turnaround and RX timing are fully in firmware.
- USB-C and screw-terminal power inputs. Battery (JST-PH) and LinkE 5 V remain, still SS54-OR'd.
- PWM servo header.
- Edge test-point pads. There are three GND probe points instead, and signals are probed at silk-labeled component pads.
- Second UART. Telemetry rides the `DATA` wire as bounded CRC'd bursts, so there is no telemetry pin, no LinkE TX / RX wiring and no TEL lamp.
- Ferrite-split `+3V3_DRV` driver logic rail. The DRV8212P VCC sits on the same `+3V3` as the MCU.
- OPA to CMP2 hardware stall detection. V006 has no comparator units, so protection is layered instead: driver OCP / TSD, a kernel I2t limit that drops `DRV_EN` and latches a user-clearable stall fault, IWDG, and the `DRV_EN` pulldown that kills the bridge on any reset.

### Added

- **Qwiic connector** (I2C, 4K7 pull-ups, 100 Ohm series) for an encoder module, magnetic or quadrature breakout.
- **`VB` bias node** for the motor terminal dividers, so both terminals stay readable while the bridge coasts.
- **Second analog position channel.** J5 is 1x04 (`+3V3` / `GND` / `POS1` / `POS2`) for a dual-output transducer; `POS2` shares ADC `A0` (`PA2`) with the NTC through JP2, behind the same 4K7 / 100nF filter as `POS1`. Jump JP2 for the NTC, leave it open for the second channel.
- **DNP experiment footprints** around the current sense block: comp-cap stack (`Cc3`/`Cc4`), kelvin filter (`Ck1`), output cap (`Co1`), VREF shift (`Rd3`), alternative shunt (`Rs2`), bus pull-up (`Rx2`), plus fitted 0R kelvin series swap points (`Rk1`/`Rk2`).
- ESD clamp (PESD5V0L1BA) on the bus `DATA` line, ahead of the series resistor.
- **Function-family reference designators.** Passives carry a letter for the net they serve rather than a flat number, so `Rv1` and `Cv1` belong to the same divider. See the README.
- QR code on the back silk linking to the board README.

### Changed

- **Current sense reworked for accuracy.** 60 mOhm kelvin-connected shunt (`Rs1`) with net-tie midpoint taps, the only tie between `PGND` and `GND`, and the on-chip op-amp running in bare mode behind an external four-resistor difference network. G = 14.88 (6K4/430) for 893 mV/A, biased at VREF = 0.52 V so both current directions are visible. Precision money goes to the shunt and the Rf / Rg quad; everything downstream is offset that boot calibration eats.
- **Motor terminal sense is now differential and coast-readable.** 6K4/1K6 dividers on both terminals, ratio 0.20, with their bottoms and filter caps returning to the 0.623 V `VB` node instead of GND. Neither the coast diode phase nor a 2S rail clips, so `vA - vB` is real back-EMF and the rail comes back as `(tap - 0.8 x VB) / 0.2`. `VB` is self-measured at boot in brake-low.
- **`VSNS` moved to its own pin.** `VSYS` is sensed directly through a 20K/10K divider into `A2` (`PC4`), instead of being inferred from a driven terminal.
- Bridge decoupling returns to `GND`, not `PGND`, so the chopped motor loop has to cross the shunt instead of bypassing it.
- WCH-LinkE header is 1x03 (GND / 5 V / SWDIO) instead of 2x03.
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
