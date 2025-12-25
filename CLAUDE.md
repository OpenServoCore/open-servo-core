# CLAUDE.md — Open Servo Firmware Repo Guide (Strict)

You are working in a Rust monorepo for an embedded “open servo” project (core + math + control).
Optimize for correctness, determinism, and zero-alloc embedded constraints.

This repo is **pre-alpha**, so API breakage is allowed *only when explicitly requested by the user*.
Otherwise, keep diffs small and behavior stable.

---

## Operating rules (non-negotiable)

- Follow the user’s plan exactly unless you find a concrete bug or mismatch with the codebase.
- If you need to deviate, **STOP** and explain: what you found, why it conflicts, and propose the smallest correction.
- Prefer minimal diffs and surgical edits.
- No new dependencies unless explicitly requested.
- No heap allocation. No floats. No dynamic formatting in hot paths.
- Keep code `no_std` compatible.

### Hard safety rules
- **Never** introduce new behavior-affecting “defaults” in `open-servo-core`.
  - No `with_defaults()` on config types in core.
  - No `impl Default for *Config` in core unless under `#[cfg(test)]`.
  - Tuning values must come from the **board crate** (or test helpers under `#[cfg(test)]`).
- **Never** grow `servo_core/mod.rs` with logic “because it’s easier.”
  - `mod.rs` is an orchestrator, not where behavior lives.
  - Tick logic must be delegated into `fast.rs`, `medium.rs`, `slow.rs`, and `features/*`.

---

## Workflow (required)

### Stage 0 — Locate targets
Before editing, locate the exact code sites. Confirm:
- The file paths exist.
- The referenced types/functions exist.
- The current code matches the plan’s assumptions.

### Stage 1 — Plan with acceptance checks
When the task is a refactor or architecture change, you MUST write a short plan (2–6 commits) and include:
- Goal
- Files touched
- Greppable acceptance checks (see below)
- Verification commands to run

### Stage 2 — Implement with guardrails
Implement commit-by-commit, running tests after each commit if requested.

### Stage 3 — Verify and report
Run requested verification commands and report results.

If a test fails, fix forward with minimal changes.

---

## Greppable acceptance checks (use `rg`)

For architecture tasks, you MUST satisfy acceptance checks expressed as ripgrep queries.
Include them in the plan and ensure they pass at the end.

Examples:
- `rg "SafetyManager" firmware/open-servo-core` must return **zero matches** (when asked to delete it).
- `rg "with_defaults" firmware/open-servo-core/src` must return **zero matches** except `#[cfg(test)]`.
- `rg "impl Default for .*Config" firmware/open-servo-core/src` must return **zero matches** except `#[cfg(test)]`.
- `rg "take_snapshot\\(" firmware/open-servo-core/src/servo_core/mod.rs` must be **absent** (snapshot belongs in medium.rs/aggregate).
- `rg "run_fast_tick" firmware/open-servo-core/src/servo_core/mod.rs` must be **present** (thin delegation).

---

## Orchestrator discipline (critical)

### `servo_core/mod.rs` rules
- `servo_core/mod.rs` MUST contain only:
  - `ServoCore` struct
  - constructors/builders
  - small accessors
  - tick methods that **delegate** into pipeline modules
  - minimal glue code (fault latching, engage/disengage gating)
- Tick methods must be short:
  - `fast_tick`, `control_medium_tick`, `slow_tick` should each be **small** and mostly call `fast::run_fast_tick`, etc.
- No heavy logic in `mod.rs`:
  - No policy FSM logic
  - No thermal model update logic
  - No accumulator snapshotting logic
  - No compliance limiter math

### Pipeline module responsibilities
- `fast.rs`: observe → protect → control → actuate, returns outputs + optional fault
- `medium.rs`: takes accumulator snapshot, policy FSM decisions, applies policy, calls controller medium tick
- `slow.rs`: supervision / thermal update / slow checks, returns optional fault
- `features/*`: pure functions operating on `(State, Config, inputs, dt_us)` as needed

### Fault handling rules
- Pipeline modules **detect** faults and return `Option<FaultKind>` (or an enum result).
- `ServoCore` **latches** faults into `fault_state`.
- Only fault path hard-disables outputs. Other features are soft constraints (clamps + flags).

---

## Timing rules (critical)

- Core makes **no fixed tick-rate assumptions**. Board supplies cadence and `TickCtx.dt_us`.
- Medium tick policy timing MUST use `snap.window_dt_us` from accumulator snapshot.
- Do NOT store `dt_us` in config. Time deltas are inputs to functions, not configuration.

---

## Code style

- Prefer clear names over cleverness.
- Use saturating arithmetic for fixed-point and embedded safety.
- Avoid repeated computations in hot paths; cache if it improves clarity without changing behavior.
- Keep comments brief and factual; explain “why”, not “what”.

---

## Fixed-point / units conventions

- `CentiDeg` is i16-backed and must remain that way for public APIs and wire formats unless explicitly instructed.
- For internal widened math, use i32-backed types (e.g., `CentiDeg32`) and saturating conversions at boundaries.
- Avoid i16 overflow hazards:
  - Never use `i16::abs()`; use `saturating_abs()`.
  - Use `saturating_add()` when adding small constants to potentially-large magnitudes.

---

## Configuration rules (critical)

- **Core defines schemas; board provides values.**
- No behavior-affecting defaults in `open-servo-core`:
  - If tests need defaults, create `#[cfg(test)] fn test_*_config()` helpers in `test_support.rs`.
- If a config has magic numbers today, move them to the board crate:
  - `open-servo-stm32f301/src/config.rs` should own `fn core_config() -> CoreConfig`.

---

## Testing

- Add targeted regression tests for the specific bug/hazard being fixed.
- Prefer deterministic unit tests that directly hit the code path.
- Tests may call private functions by placing tests in the same module.

---

## Verification commands (run as applicable)

- `cargo fmt`
- `cargo test` - runs all tests
- `cargo test -p open-servo-core`
- `cargo test -p open-servo-math`
- `cargo test -p open-servo-control`
- `cargo check -p open-servo-core --no-default-features`
- `cargo check -p open-servo-stm32f301`
- `cargo check -p open-servo-stm32f301 --no-default-features`

---

## Commit message hygiene

- Do not include “Claude”, “AI”, “Generated-by”, “Co-authored-by”, or any self-tagging in commit messages.
- Use concise one-liners in imperative mood (e.g., “Fix …”, “Widen …”, “Clamp …”).
