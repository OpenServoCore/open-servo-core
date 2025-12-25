//! Test harness utilities for ServoCore simulation tests.
//!
//! Provides runner functions and snapshot capture for testing safety invariants.

#![cfg(test)]

extern crate std;
use std::vec::Vec;

use crate::fault::FaultKind;
use crate::inputs::FastInputs;
use crate::outputs::FastOutputs;
use crate::servo_core::{ServoCore, ServoMode};
use crate::test_support::{make_core, make_inputs, repeat_inputs, MockController};
use open_servo_control::ControlLoop;
use open_servo_math::{CentiDeg, Duty, TickCtx, TickDomain};

/// Create a dummy TickCtx for testing.
fn test_ctx() -> TickCtx {
    TickCtx {
        domain: TickDomain::ControlFast,
        dt_us: 100,
        seq: 0,
    }
}

// TODO(Stage 5): Thermal invariant tests require ThermalModel from open-servo-math.
// Consider creating a dedicated thermal test module or integration test that can
// instantiate the full thermal pipeline.

/// Snapshot of servo state after a tick.
///
/// Captures observable state for test assertions without requiring
/// access to internal implementation details.
#[derive(Debug, Clone)]
pub struct Snapshot {
    /// Current position from input
    pub position: CentiDeg,
    /// Current setpoint (if any)
    pub setpoint: Option<CentiDeg>,
    /// PWM command from output
    pub pwm: Duty,
    /// Motor enable state from output
    pub motor_enable: bool,
    /// Whether the core is currently faulted
    pub faulted: bool,
    /// Fault kind from output (if any)
    pub fault_kind: Option<FaultKind>,
    /// Current compliance mode
    pub mode: ServoMode,
}

impl Snapshot {
    /// Create a snapshot from the current core state and last tick results.
    pub fn from_tick<C: ControlLoop>(
        core: &ServoCore<C>,
        inputs: &FastInputs,
        outputs: &FastOutputs,
    ) -> Self {
        Self {
            position: inputs.position,
            setpoint: core.get_setpoint(),
            pwm: outputs.pwm_command,
            motor_enable: outputs.motor_enable,
            faulted: core.is_faulted(),
            fault_kind: outputs.fault,
            mode: core.mode(),
        }
    }
}

/// Run a sequence of fast ticks and collect outputs with snapshots.
///
/// # Arguments
/// * `core` - The ServoCore instance to tick
/// * `inputs` - Iterator of FastInputs to feed to each tick
///
/// # Returns
/// Vector of (FastOutputs, Snapshot) pairs for each tick
pub fn run_fast_ticks<C: ControlLoop>(
    core: &mut ServoCore<C>,
    inputs: impl IntoIterator<Item = FastInputs>,
) -> Vec<(FastOutputs, Snapshot)> {
    let ctx = test_ctx();
    inputs
        .into_iter()
        .map(|input| {
            let outputs = core.fast_tick(&ctx, input);
            let snapshot = Snapshot::from_tick(core, &input, &outputs);
            (outputs, snapshot)
        })
        .collect()
}

/// Run fast ticks with periodic slow ticks interleaved.
///
/// # Arguments
/// * `core` - The ServoCore instance to tick
/// * `inputs` - Iterator of FastInputs to feed to each tick
/// * `slow_every` - Call slow_tick() every N fast ticks (0 = never)
///
/// # Returns
/// Vector of (FastOutputs, Snapshot) pairs for each tick
pub fn run_with_slow<C: ControlLoop>(
    core: &mut ServoCore<C>,
    inputs: impl IntoIterator<Item = FastInputs>,
    slow_every: usize,
) -> Vec<(FastOutputs, Snapshot)> {
    let fast_ctx = test_ctx();
    let slow_ctx = TickCtx {
        domain: TickDomain::System,
        dt_us: 10_000,
        seq: 0,
    };
    let mut results = Vec::new();
    let mut tick_count = 0usize;

    for input in inputs {
        let outputs = core.fast_tick(&fast_ctx, input);
        let snapshot = Snapshot::from_tick(core, &input, &outputs);
        results.push((outputs, snapshot));

        tick_count += 1;
        if slow_every > 0 && tick_count % slow_every == 0 {
            core.slow_tick(&slow_ctx);
        }
    }

    results
}

// ========== stall detection tests (using harness) ==========

#[test]
fn test_stall_triggers_at_timeout() {
    let mut core = make_core(MockController::new());
    let ctx = test_ctx();
    let timeout = core.safety_thresholds().stall_timeout_ticks as usize;

    core.engage(CentiDeg::from_cdeg(9000));
    core.set_setpoint(CentiDeg::from_cdeg(9100));

    // Initialize sensor health
    core.fast_tick(&ctx, make_inputs(9000));

    // Set BOTH output and saturation
    core.controller_mut().set_output(Duty::MAX);
    core.controller_mut().set_saturated(true);

    // Build input sequence: (timeout - 1) ticks at position 9000
    let results = run_fast_ticks(&mut core, repeat_inputs(make_inputs(9000), timeout - 1));

    // Verify no fault during first (timeout - 1) ticks
    for (outputs, snapshot) in &results {
        assert!(!snapshot.faulted, "faulted before timeout");
        assert!(outputs.fault.is_none());
    }

    // Final tick should trigger stall
    let outputs = core.fast_tick(&ctx, make_inputs(9000));
    assert!(core.is_faulted(), "should fault at timeout");
    assert_eq!(outputs.fault, Some(FaultKind::Stall));
}

#[test]
fn test_stall_counter_resets_on_movement() {
    let mut core = make_core(MockController::new());
    let ctx = test_ctx();
    let timeout = core.safety_thresholds().stall_timeout_ticks as usize;

    core.engage(CentiDeg::from_cdeg(9000));
    core.set_setpoint(CentiDeg::from_cdeg(9100));

    // Initialize
    core.fast_tick(&ctx, make_inputs(9000));

    // Set BOTH output and saturation
    core.controller_mut().set_output(Duty::MAX);
    core.controller_mut().set_saturated(true);

    // Run half timeout at position 9000
    let half_timeout = timeout / 2;
    let _ = run_fast_ticks(&mut core, repeat_inputs(make_inputs(9000), half_timeout));
    assert!(!core.is_faulted(), "should not fault at half timeout");

    // Move beyond tolerance to reset counter (safe i32 math)
    let tol = core.safety_thresholds().stall_position_tolerance.as_cdeg() as i32;
    let new_position: i16 = (9000i32 + tol + 1).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
    core.fast_tick(&ctx, make_inputs(new_position));

    // Run (timeout - 1) at new position - counter should have reset
    let results = run_fast_ticks(
        &mut core,
        repeat_inputs(make_inputs(new_position), timeout - 1),
    );

    for (outputs, snapshot) in &results {
        assert!(!snapshot.faulted, "counter should have reset");
        assert!(outputs.fault.is_none());
    }

    assert!(!core.is_faulted(), "should not fault after reset");
}

#[test]
fn test_no_stall_without_saturation() {
    let mut core = make_core(MockController::new());
    let ctx = test_ctx();
    let timeout = core.safety_thresholds().stall_timeout_ticks as usize;

    core.engage(CentiDeg::from_cdeg(9000));
    core.set_setpoint(CentiDeg::from_cdeg(9100));

    // Initialize sensor health
    core.fast_tick(&ctx, make_inputs(9000));

    // High duty but NOT saturated - stall should NOT trigger
    core.controller_mut().set_output(Duty::MAX);
    core.controller_mut().set_saturated(false);

    // Run timeout + 5 ticks at constant position
    let results = run_fast_ticks(&mut core, repeat_inputs(make_inputs(9000), timeout + 5));

    // Should never fault
    for (outputs, snapshot) in &results {
        assert!(!snapshot.faulted, "should not fault without saturation");
        assert!(outputs.fault.is_none());
    }

    assert!(!core.is_faulted(), "should not fault without saturation");
}
