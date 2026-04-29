# OSC Dev CH32 — Changelog

Hardware revisions of the OSC Dev CH32 (`osc-dev-v006`) board. Newest first.

To get files at a specific revision, check out the corresponding tag (e.g. `git checkout osc-dev-v006-rev-a`) or browse the merge commit linked below.

---

## Rev B — 2026-04-28

**Status:** Routed, awaiting fab. Merged in [PR #10](https://github.com/OpenServoCore/open-servo-core/pull/10).

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
