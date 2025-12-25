# Open Servo Firmware Refactor Plan (Phased)

> Scope: `firmware/` in `open-servo-*` crates.  
> Style: small, compile-safe steps; each stage has clear acceptance criteria.  
> Key principle: **ServoCore owns system state + safety envelope. Controllers are pure control math.**

---

## Executive summary

### What’s wrong today (high level)
- **Timing assumptions are hardcoded** (10kHz fast, 10ms slow), but boards may differ → real math bugs (thermal, blanking windows, timeouts).
- **ControlLoop trait is a junk drawer**: algorithm step + setpoint storage + output limits + PID tuning all in one.
- **Setpoint ownership is split** (core clamps, controller stores, compute takes setpoint) → multiple sources of truth.
- **Stall/saturation detection is wrong under limiting** (compares against `Duty::MAX` instead of the active limit).
- **Fault handling + event propagation are inconsistent** (faults can be “raised on clone” or dropped).

---

## Bugs / Must-fixes (correctness & safety)

> Fix these *before* big architecture changes.

1) **Driver enable not re-applied on normal ticks**
- Symptom: once disabled, system may not re-enable even if outputs say it should.
- Fix: `App::on_control_tick()` must always call `hw.set_enable(outputs.motor_enable)` (and apply PWM/coast consistently).

2) **Fault raising is broken**
- Symptom: `raise_fault` operates on a clone or clears immediately → fault never latches.
- Fix: Add `ServoCore::raise_fault(kind)` (or `fault_state_mut`) and ensure faults latch until explicitly cleared.

3) **Fault output not acted upon**
- Symptom: `FastOutputs.fault` exists but `App` ignores it (no event/log).
- Fix: Either enqueue `Event::Fault(kind)` when seen, or remove `FastOutputs.fault` and rely on core fault state.

4) **Thermal model uses wrong dt**
- Symptom: thermal integrator hardcodes 10ms while slow tick is not guaranteed to be 100Hz.
- Fix: thermal update must accept dt (or derive from board slow tick Hz).

5) **Direction-change blanking assumes 10kHz**
- Symptom: blanking “10 ticks = 1ms at 10kHz” becomes wrong for other rates.
- Fix: store blanking as time and convert to ticks from board timing.

6) **Position-error timeout tick-domain mismatch**
- Symptom: config/docs say one rate, code executes at another.
- Fix: decide domain (fast vs medium) and enforce it with types/config conversion.

7) **Stall detection blind under compliance limiting**
- Symptom: saturation check uses `Duty::MAX`, so “saturated at active limit” isn’t detected.
- Fix: make controller/core provide `saturated` relative to active limits; stall detection uses that.

8) **Stale/broken tests**
- Symptom: tests construct cores with old APIs.
- Fix: update to new APIs or gate legacy tests; do not leave failing tests as “noise”.

---

# Stage 0 — Stabilize & make safe (minimal patch set)

### Goal
Fix correctness bugs while keeping architecture mostly intact.

### Tasks
- [ ] `App::on_control_tick`: always apply `motor_enable` to hardware each tick.
- [ ] Implement real fault latching:
  - [ ] Add `ServoCore::raise_fault(kind)` and use it in App.
  - [ ] Remove clone-raise patterns.
- [ ] Propagate faults:
  - [ ] Enqueue fault events OR remove redundant fault fields.
- [ ] Thermal dt: update thermal model function signature to accept dt (or accept slow_hz).
- [ ] Fix compliance blanking to be time-based (convert to ticks using current fast_hz or existing CONTROL_HZ for now).
- [ ] Update tests to compile.

### Acceptance
- Builds pass for default + all-features (as applicable).
- Faults latch correctly and are observable.
- Thermal model uses correct dt for current configuration.
- Tests compile (or are intentionally gated with rationale).

---

# Stage 1 — ControlLoop refactor (core owns state; controller is control math)

### Goal
Replace the current kitchen-sink `ControlLoop` with a minimal, non-generic interface and make ServoCore the single source of truth for setpoint and envelope.

### Non-negotiable constraints
- **No default implementations** in traits (explicit no-op implementations in controllers).
- **No blanket impl / trait alias hacks** to unify bounds across debug commands.
- **Non-generic** input/output types.
- **Output limits are per-tick inputs** (not set via setter methods).
- **PID tuning is separate** from base `ControlLoop` (capability trait).

## Stage 1 API (recommended)

### Types (non-generic)
```rust
pub struct DutyLimits {
    pub min: Duty,
    pub max: Duty,
}

pub struct ControlInput {
    pub setpoint: CentiDeg,
    pub position: CentiDeg,
    pub velocity: Option<DegPerSec10>,
    pub current: Option<MilliAmp>,
    pub bus_voltage: Option<MilliVolt>,
    pub limits: DutyLimits,
}

pub struct ControlOutput {
    pub duty: Duty,
    /// true if final duty is at limits (relative to `limits`)
    pub saturated: bool,
}
```

### Base trait (explicit)
```rust
pub trait ControlLoop {
    fn reset(&mut self);

    fn fast_tick(&mut self, input: &ControlInput) -> ControlOutput;
    fn medium_tick(&mut self, input: &ControlInput);
    fn slow_tick(&mut self, input: &ControlInput);
}
```

### PID tuning trait (separate)
```rust
pub trait PidTunable {
    fn pid_config(&self) -> &PidConfig;
    fn with_pid_config_mut<F: FnOnce(&mut PidConfig)>(&mut self, f: F);
}
```

## ServoCore responsibilities after Stage 1
- Owns:
  - `setpoint: Option<CentiDeg>`
  - engagement/disengagement
  - safety/compliance/thermal envelope
  - mode machine (Move/Hold/Yield)
  - fault state
- Computes per-tick `DutyLimits` and builds `ControlInput`.
- Calls controller only when engaged and setpoint exists:
  - if `setpoint.is_none()` → return safe outputs, **do not call controller**.
- Stall detection uses `ControlOutput.saturated` (relative to active limits), not `Duty::MAX`.

## Debug shell guidance (Stage 1)
- Use **split bounds**:
  - Most commands: `C: ControlLoop`
  - PID-only subcommands: `C: ControlLoop + PidTunable` in helper fns behind `#[cfg(feature="pid")]`
- Do **not** require `PidTunable` everywhere.

### Acceptance
- Project builds.
- ServoCore no longer calls controller setpoint/limit setters.
- `ControlLoop` contains only reset + fast/medium/slow ticks.
- Stall detection uses `saturated`.
- PID tuning still works via `PidTunable`.

---

# Stage 2 — Board-defined timing + true fast/medium/slow scheduling

### Goal
Eliminate hardcoded `CONTROL_HZ`, “10kHz”, “10ms” assumptions. Make timing board-provided and derive all tick-related constants.

### Tasks
- [ ] Add `BoardTimingConfig { fast_hz, medium_hz, slow_hz }` to `open-servo-hw`.
- [ ] BoardConfig exposes `timing_config()`.
- [ ] ServoCore uses timing to compute:
  - [ ] dt values used in velocity estimator
  - [ ] decimation factors (fast→medium, fast→slow)
  - [ ] timeouts (ms→ticks conversion)
- [ ] Thermal model takes dt or slow_hz; remove hardcoded 10ms.
- [ ] Compliance blanking is time-based and uses fast_hz conversion.
- [ ] Revisit safety timeouts and clearly define tick domain.

### Acceptance
- No timing constants hardcoded in core/math that affect behavior.
- Changing board timing changes behavior correctly (thermal, blanking, timeouts).
- Tests include at least 2 timing configs to validate conversion logic.

---

# Stage 3 — Units & limits overhaul (i32 position; layered bounds)

### Goal
Remove artificial position range limits and unify limit logic.

### Tasks
- [ ] Switch internal position representation to i32 centidegrees (or strong type).
- [ ] Implement 3-layer limit system:
  - [ ] `SensorLimits` (raw ADC/encoder range)
  - [ ] `MechanicalLimits` (physical travel in centideg i32)
  - [ ] `UserLimits` (soft limits inside mechanical)
- [ ] Centralize clamp logic in one module (kinematics/limits).
- [ ] Ensure protocol/wire format can remain i16 if desired via conversion boundary.

### Acceptance
- Core math uses i32 position.
- All clamps happen via unified limit code path.
- Limit violations produce consistent faults/events.

---

# Stage 4 — Simplify boundaries + build a real simulation/test harness

### Goal
Make future iteration safe and prevent regressions.

### Tasks
- [ ] Remove hardware effects from pure state modules (no driver calls inside `FaultState`).
- [ ] Add a lightweight sim harness:
  - feed sequences of inputs, inspect outputs and fault transitions
- [ ] Add regression tests for:
  - [ ] stall detection under limiting
  - [ ] thermal dt correctness
  - [ ] blanking windows across tick rates
  - [ ] setpoint clamp behavior and mode transitions

### Acceptance
- Tests cover key safety invariants.
- Adding new controllers does not require invasive ServoCore changes.

---

## Implementation notes & pitfalls

### Saturation definition (important)
- `saturated` should mean: **final duty equals min or max limit provided this tick**.
- This supports stall detection and avoids false negatives under compliance/thermal derating.

### Temperature in controller inputs?
- Default answer: **no**, temp affects envelope (limits) in core.
- If a future controller truly needs it for model compensation, add via telemetry/extra fields later (or a dedicated slow input).

### “No setpoint” behavior
- ServoCore should avoid calling controller without a valid setpoint.
- When engaging, ServoCore can set setpoint to current position if none, to avoid jumps.

---

## Suggested Claude work ordering (micro-steps)
> Keeps compilation stable and isolates risk.

1) Introduce new types + new ControlLoop trait (compile only).
2) Update PID controller to implement new trait (explicit medium/slow no-ops).
3) Update ServoCore to own setpoint and call controller via new input/output.
4) Update stall detection to use `output.saturated`.
5) Update debug shell with split bounds (PID handlers require `PidTunable`).
6) Update tests and mocks.

---

## Out-of-scope for Stage 1 (explicitly)
- Board timing refactor (Stage 2).
- Full thermal refactor (except for Stage 0 must-fix).
- New controller algorithms.

---
