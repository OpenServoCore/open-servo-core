# OSC Dev CH32 — Changelog

Hardware revisions of the OSC Dev CH32 (`osc-dev-v006`) board. Newest first.

To get files at a specific revision, check out the corresponding tag (e.g. `git checkout osc-dev-v006-rev-a`) or browse the merge commit linked below.

---

## Rev 2A - 2026-08-02

**Status:** Design complete, ready for fab (JLCPCB). Not fabricated or validated yet. Board silk reads `Dev V006 Rev. 2A`.

### Removed

- 74LVC2G241 UART bus buffer. `DATA` now wires directly to the MCU through a 33 Ω series resistor, and direction turnaround and RX timing are fully in firmware.
- USB-C and screw-terminal power inputs. Battery (JST-PH) and LinkE 5 V remain, still SS54-OR'd.
- PWM servo header.
- Edge test-point pads. There are three GND probe points instead, and signals are probed at silk-labeled component pads.
- `VSNS` direct VSYS sense. That ADC pin went to the telemetry UART.

### Added

- **Qwiic connector** (I2C, 4K7 pull-ups) for an encoder module, magnetic or quadrature breakout.
- **Dedicated telemetry UART**. A single MCU pin (USART2 TX) bridged to the LinkE TX / RX pins through series resistors, sized for raw 20 kHz kernel-rate streaming for system identification and control loop characterization.
- **DNP experiment footprints** around the current sense block: comp-cap stack (`Cc3`/`Cc4`), input filter (`Ck1`), VREF shift (`Rd3`), parallel shunt (`RS2`), bus pull-up (`R24`), plus fitted 0 Ω kelvin series swap points (`Rk1`/`Rk2`).
- ESD clamp (PESD5V0L1BA) on the bus `DATA` line.
- QR code on the back silk linking to the board README.

### Changed

- **Current sense reworked for accuracy.** 33 mΩ kelvin-connected shunt with net-tie midpoint taps, and the on-chip op-amp wired as a four-resistor difference amplifier. G = 14.88 (6K4/430), output biased at VREF = 0.52 V so both current directions are visible, and the entire network is 0.5 % / 25 ppm thin film.
- Motor terminal sense dividers are now precision 20K/10K (÷3), tapped at the motor connector.
- Encoder input (J6) is ADC-only by design. `PD2`/`PD3` don't carry TIM2 CH1/CH2, so digital encoders go on Qwiic instead. J6 targets an ADC-sampled IR quadrature encoder.
- 4 layers -> 6 layers, 50 × 40 mm, JLCPCB JLC06161H-3313 stackup, ENIG, via-in-pad. The motor path is a zero-via top-layer corridor to a through-hole motor connector.
- `SB1` bootstrap solder bridge became `JP1`. Same `nRST -> GPIO` first-boot procedure, and the op-amp `+` input now shares the pin.
- All connectors renumbered. The LED set is now VSYS / 3V3 / TEL / DAT / STA.

---

## Rev B — 2026-04-28

**Status:** Validated. Merged in [PR #10](https://github.com/OpenServoCore/open-servo-core/pull/10). In-rev patch: `SB1` bootstrap solder bridge added for the shared `nRST` / `OPN2` pin. Version label unchanged — published files include the patch.

### Fixed (from Rev A)

- VDD / VCC rail label swap.
- Top-row test-point labels.
- Encoder connector labels.
- Battery connector polarity (JST PH 2P).
- TX_EN / UART buffer conflict — added a jumper for RX ↔ buffer so UART works without firmware workaround.

### Added

- **Hardware-based stall detection / overcurrent protection** — internal OPA → CMP2 path. No extra parts.
- **External NTC connector** with onboard / external selection jumper.
- **Traditional PWM servo header** (driven from IN1) for servo identification and measurement.
- **VSNS net** — VSYS measured directly instead of derived from VSNA / VSNB.
- **WCH-LinkE connector**: 5 V pin (replacing the unused RST pin) gated by an SS54 for safety.
- **`SB1` bootstrap solder bridge** (in-rev patch) — `nRST` and `OPN2` share a pin; the bridge breaks the path during `nRST -> GPIO` option-byte programming, then closes for normal current-sense operation. See [Fabrication](README.md#fabrication).

### Changed

- **MCU pin remap:**
  - `ENCA` / `ENCB` selectable between TIM2 and ADC.
  - `ISNS+` / `ISNS−` moved to OPP0 / OPN2 to feed the differential OPA.
  - `STAT` LED moved to a TIM1 channel.
  - `nRST` removed (USER option byte programmed at provisioning; pin freed for OPA / GPIO use).
- Reset button and its RC debounce network removed (nRST gone).
- Board renamed `servo-dev-board-ch32v006` → `osc-dev-v006` to match the product naming convention.

---

## Rev A — 2026-03-01

**Status:** Built, validated. **Deprecated — do not fab.** Superseded by Rev B. Merged in [PR #5](https://github.com/OpenServoCore/open-servo-core/pull/5).

Initial CH32V006-based dev board. PCBWay-sponsored fab. Brought up and validated end-to-end before known issues and pin-mapping limits drove the Rev B respin.

### Known issues (fixed in Rev B)

- VDD / VCC rail label swap.
- TX_EN pull-up conflict on the half-duplex UART buffer.
- Top-row test-point labels.
- Encoder connector labels.
- Battery connector (JST PH 2P) polarity reversed.
