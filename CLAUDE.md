# CLAUDE.md — Open Servo Firmware Repo Guide

You are working in a Rust monorepo for an embedded “open servo” project (core + math + control).  
Optimize for correctness, determinism, and zero-alloc embedded constraints.

## Operating rules

- Follow the user’s plan exactly unless you find a concrete bug or mismatch with the codebase.
- If you need to deviate, **stop and explain**: what you found, why it conflicts, and propose the smallest correction.
- Prefer minimal diffs and surgical edits.
- Keep public APIs stable unless explicitly instructed otherwise.
- No new dependencies unless explicitly requested.
- No heap allocation. No floats. No dynamic formatting in hot paths.
- Keep code `no_std` compatible.

## Workflow

1. **Stage 0:** Before editing, locate the exact code sites (line numbers may drift). Confirm all targets exist.
2. Make the smallest change that satisfies the acceptance criteria.
3. Run the requested verification commands.
4. If a test fails, fix forward with minimal changes.

## Code style

- Prefer clear names over cleverness.
- Use saturating arithmetic for fixed-point and embedded safety.
- Avoid repeated computations in hot paths; cache if it improves clarity without changing behavior.
- Keep comments brief and factual; explain “why”, not “what”.

## Testing

- Add targeted regression tests for the specific bug/hazard being fixed.
- Prefer deterministic unit tests that directly hit the code path (avoid flaky “integration-ish” tests).
- When testing private functions: tests in the same module may call private methods directly.

## Fixed-point / units conventions

- `CentiDeg` is i16-backed and must remain that way for public APIs and wire formats unless explicitly instructed.
- For internal widened math, use i32-backed types (e.g., `CentiDeg32`) and saturating conversions at boundaries.
- Avoid i16 overflow hazards:
  - Never use `i16::abs()`; use `saturating_abs()`.
  - Use `saturating_add()` when adding small constants to potentially-large magnitudes.

## Stage discipline (current project phases)

- **Stage 0:** Widen internal setpoint/position math to i32 while keeping public APIs unchanged.
- **Stage 1:** Dynamic output limits with integral anti-windup: ensure limit updates also clamp integral accumulator.
- **Stage 2:** Introduce `CentiDeg32` in `open-servo-math` and migrate core to use it internally.
- **Stage 3:** Remove remaining i16 overflow hazards (`abs`, adds) and improve semantic correctness.

Always keep stages isolated: don’t mix unrelated refactors.

## What to modify (typical)

- `firmware/open-servo-core/src/servo_core.rs`
- `firmware/open-servo-core/src/safety/mod.rs`
- `firmware/open-servo-core/src/safety/sensor_health.rs`
- `firmware/open-servo-math/src/units.rs`
- `firmware/open-servo-control/src/pid.rs`
- `firmware/open-servo-math/src/pid.rs`

## Verification commands (run as applicable)

- `cargo fmt`
- `cargo test -p open-servo-core`
- `cargo test -p open-servo-math`
- `cargo test -p open-servo-control`
- `cargo check -p open-servo-core --no-default-features`

## Commit message hygiene

- **Do not** include “Claude”, “AI”, “Generated-by”, “Co-authored-by”, or any self-tagging in commit messages.
- Commit messages should read as if written by the maintainer.
- Use concise one-liners in imperative mood (e.g., “Fix …”, “Widen …”, “Clamp …”).
