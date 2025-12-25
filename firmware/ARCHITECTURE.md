# ARCHITECTURE

OpenServo-Core Firmware Architecture (pre-alpha)

This document describes the **intended architecture** of the OpenServo firmware stack and the division of responsibility between the **board crate** and the **core/control/math** crates. It is written to be approachable for contributors and to keep the control/safety path deterministic and portable across hardware.

---

## 1. Design goals

- **Hardware-portable core:** core logic is reusable across MCUs/boards via small hardware traits.
- **Deterministic control path:** the hard real-time path is bounded, predictable, and heap-free.
- **Time-based behavior:** policy/state-machine timing is based on measured time deltas, not “tick counts”.
- **Safety-first:** immediate protection lives in the fastest domain; additional supervision runs at lower rates.
- **Contributor-friendly:** readable modules with clear responsibilities; avoid clever frameworks when possible.
- **No new dependencies for core path:** minimize footprint and maximize embedded compatibility.

---

## 2. Key principle: no fixed tick-rate assumptions

OpenServo-Core defines **three logical loops**:

- **ControlFast** — “sample/observe + immediate safety + actuate”
- **ControlMedium** — “windowed estimation + policy/state machine + soft-limits”
- **System** — “housekeeping/supervision” (thermal, audits, debug work, etc.)

**Important:** OpenServo-Core makes **no assumptions about absolute frequencies** (10 kHz / 1 kHz / 100 Hz are typical examples only).
The **board crate** supplies scheduling cadence and measured time deltas.

All time-based logic in the core uses microsecond deltas from `TickCtx`:

- `dt_us`: elapsed microseconds since the previous tick in that domain
- (Optional/board-defined) a **window delta** for medium-tick windowed stats (e.g., derived from decimation)

This keeps behavior correct across different clock trees, ADC rates, and decimation ratios.

---

## 3. Crates and responsibilities

### 3.1 Board crate (e.g. `open-servo-stm32f301`, future boards)
Owns **hardware integration** and scheduling:

- Starts peripherals (ADC/DMA, timers, motor driver, sensors).
- Defines the **tick cadence** and calls into core:
  - ControlFast tick (often from ADC DMA completion / timer ISR)
  - ControlMedium tick (often from decimation / sample-aligned scheduling)
  - System tick (low-rate timer or main loop)
- Constructs `FastInputs` (sensor readings) and applies `FastOutputs` (PWM, enable).
- Provides board-specific conversions where needed (e.g., ADC → engineering units, LUTs).

### 3.2 `open-servo-core`
Owns **servo behavior orchestration**:

- Tick pipelines (fast/medium/system).
- Safety and fault latching semantics.
- Accumulators and window snapshots for medium tick.
- Debug shell / event plumbing (not on the hard real-time path).

### 3.3 `open-servo-control`
Owns the **control algorithm**:

- The `ControlLoop` trait and implementations.
- Control math and tuning parameters (PID, feed-forward, etc.).
- No direct hardware access.

### 3.4 `open-servo-math`
Owns **units, tick types, and reusable math**:

- Unit-safe newtypes (e.g., CentiDeg, MilliAmp, CentiC).
- `TickCtx` and tick-domain identifiers.
- Shared filters/models (PID helpers, thermal, etc.).

### 3.5 `open-servo-hw` / `open-servo-hw-utils`
Owns **hardware trait definitions and reusable drivers**:

- Minimal trait surfaces for motor drivers, sensors, debug IO.
- Utilities to support board crates.

---

## 4. Tick model and data flow

### 4.1 Tick context
Each tick handler receives a `TickCtx`:

- `domain`: which loop is executing
- `dt_us`: measured delta time for that domain
- `seq`: monotonic sequence counter for diagnostics and testing

The board is responsible for providing correct `dt_us` (and any additional derived window time, if used).

### 4.2 ControlFast pipeline (hard real-time)
**Intent:** bounded work, no allocations, no I/O, deterministic.

Pipeline shape:

1) **Observe**
- Read sensors from `FastInputs`
- Update fast accumulators (window stats)
- Capture `last_input` metadata

2) **Protect (immediate safety)**
- Validate sensor health (e.g., invalid position)
- Check hard limits (e.g., overcurrent)
- Latch faults and request motor disable if needed
- Apply **soft clamps** (limits/compliance) to setpoint/duty ranges

3) **Control**
- Run `controller.fast_tick(...)` (position/current/velocity loop step)

4) **Actuate**
- Apply final caps (e.g., yield max duty)
- Produce `FastOutputs` (PWM + enable)
- Update runtime snapshot

### 4.3 ControlMedium pipeline (policy + estimation)
**Intent:** decimated and sample-aligned; consumes accumulator snapshots.

Pipeline shape:

1) **Aggregate**
- Snapshot accumulator stats for the last window (e.g., avg/peak current, windowed velocity)
- Merge with `last_input` metadata

2) **Policy**
- Time-based state machine decisions (hold/yield/backdrive, etc.) using time deltas
- Select soft-limiter configurations (e.g., compliance move vs hold)
- Update derived thresholds/caps (e.g., `yield_max_duty`)

3) **Apply**
- Commit policy decisions into internal state
- Update limiter configuration/state

4) **Control**
- Run `controller.medium_tick(...)`

### 4.4 System loop (supervision / housekeeping)
**Intent:** low-frequency work that must not pollute the hard RT path.

Typical responsibilities:
- Thermal model update
- Supervisory fault checks (persistent position error, prolonged stall, etc.)
- Debug/event work and reporting
- Slow controller maintenance: `controller.slow_tick(...)`

---

## 5. ServoCore state model

ServoCore organizes state into **three envelopes** with **feature subtrees**:

### 5.1 CoreConfig (writable; future persisted)
- User-configurable limits and tuning knobs
- Soft-limiter configurations (e.g., compliance move/hold)
- Board/servo configuration parameters

**Limits are modeled with `kinematics.rs` as the canonical store**, including:
- Sensor limits (raw)
- Mechanical limits (physical)
- User limits (soft)
- Direction / angle domain rules

### 5.2 CoreRuntime (read-only telemetry snapshot)
- “Present” values: position, velocity, current, voltage, temperature, duty
- Status flags: engaged, mode, compliance_limited, fault summary
- Counters useful for debugging (e.g., `fast_seq`)

Runtime is updated only by ticks (primarily fast tick).

### 5.3 CoreInternal (bookkeeping; not user-writable)
- Timers and elapsed time for state machines (hold/yield/backdrive)
- Accumulators and window stats inputs
- Previous-error / previous-command state for supervision and heuristics
- Last input metadata and derived thresholds/caps

---

## 6. Safety and fault handling

Safety is split by urgency:

### 6.1 Immediate protection (ControlFast)
- Detect invalid sensor data
- Detect overcurrent / hard trip conditions
- Latch faults
- Disable motor output

**Only the fault path may hard-disable the motor.**
Other “features” (limits/compliance/yield) are **soft constraints**: they clamp setpoints/duty and set flags but do not hard-off the actuator directly.

### 6.2 Supervisory protection (System loop)
- Thermal model-based faults
- Persistent error / long-duration anomalies
- Additional audits that are too heavy for fast tick

---

## 7. Modules (high-level)

In `open-servo-core`, the following modules are expected to exist (names may evolve during refactors):

- `app`: board-facing orchestrator that wires ticks to ServoCore and hardware traits
- `servo_core`: core behavior, tick pipelines, and state envelopes
- `inputs` / `outputs`: `FastInputs` and `FastOutputs` types
- `accumulator`: windowed accumulation used by medium tick
- `fault`: fault types and latching structures
- `safety`: safety primitives, thresholds, and helpers
- `debug_shell`: REPL/debug interface (off the RT path)
- `event`: optional event reporting mechanism (off the RT path)
- `tick` / `timing`: tick domain and timing helpers

---

## 8. Debug shell and events (non-RT)

The debug shell and event system must not compromise the RT path:

- No printing or heavy formatting in ControlFast.
- Any debug output should be emitted from System loop or a board-owned task.
- The debug shell is a *consumer* of runtime/config state; it should not own core logic.

---

## 9. Testing philosophy

Core tests should focus on invariants, not MCU specifics:

- **dt/sample_count independence:** time-based policy must behave correctly across dt variations and decimation.
- **no heap / bounded work:** fast tick stays ISR-safe.
- **state machine correctness:** hold/yield/backdrive transitions should be testable with synthetic inputs.
- **fault semantics:** faults latch and disable outputs deterministically.

A test harness should provide:
- synthetic `TickCtx` sequences
- synthetic `FastInputs`
- capture of `FastOutputs` and runtime snapshots

---

## 10. Contributor guidelines

If you add or change behavior:

1) Decide which loop it belongs to:
- Must be immediate/hard safety? → ControlFast protect
- Needs windowed stats / timers? → ControlMedium policy
- Expensive / supervisory? → System loop

2) Put logic where it belongs:
- Configurable knobs → CoreConfig subtree
- Telemetry → CoreRuntime
- Timers/accums/history → CoreInternal

3) Keep RT path clean:
- Avoid heap allocations in fast/medium ticks
- Avoid logging/formatting in fast tick
- Prefer small, pure helper functions with explicit inputs/outputs

---

## 11. Roadmap notes (near-term)

- Registry-backed registers and REPL commands will consume the same **CoreConfig/CoreRuntime** surfaces.
- Persistence will store CoreConfig safely (explicit save/debounce, versioning, defaults).

(Those pieces are intentionally outside the scope of this document’s core tick model.)

---
