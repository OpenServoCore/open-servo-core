//! CoreInternal - Mutable bookkeeping/accumulators owned by features.
//!
//! All mutable state that isn't in CoreRuntime (telemetry) lives here.

use crate::accumulator::FastAccumulator;
use crate::servo_core::features::{BackdriveState, SafetyState, ThermalState};
use crate::servo_core::medium::{BackdriveDetector, HoldConditionsTracker, SetpointTracker};
use crate::servo_core::ServoMode;
use open_servo_control::ControlInput;
use open_servo_math::{CentiDeg32, DegPerSec10};

/// Mutable internal state for the servo core.
///
/// Contains all bookkeeping and accumulators organized by feature.
#[derive(Debug, Clone)]
pub struct CoreInternal {
    /// Safety state (sensor health, stall/error counters)
    pub safety: SafetyState,
    /// Thermal model state
    pub thermal: ThermalState,
    /// Backdrive/yield window state
    pub backdrive: BackdriveState,

    /// Setpoint tracking (for settle time detection)
    pub setpoint_tracker: SetpointTracker,
    /// Hold entry conditions tracker
    pub hold_tracker: HoldConditionsTracker,
    /// Backdrive detection (PWM/error sign analysis)
    pub backdrive_detector: BackdriveDetector,

    /// Fast-tick accumulator for windowed statistics
    pub fast_accum: FastAccumulator,
    /// Cached input from last successful fast_tick
    pub last_input: Option<ControlInput>,

    /// Current setpoint (owned by internal for state management)
    pub setpoint: Option<CentiDeg32>,
    /// Current compliance mode
    pub mode: ServoMode,
    /// Measured velocity (from accumulator)
    pub measured_velocity: DegPerSec10,
    /// Fast tick sequence counter (monotonic, for telemetry)
    pub fast_seq: u32,
}

impl CoreInternal {
    /// Create a new CoreInternal with default state.
    pub fn new(safety: SafetyState, thermal: ThermalState) -> Self {
        Self {
            safety,
            thermal,
            backdrive: BackdriveState::new(),
            setpoint_tracker: SetpointTracker::new(),
            hold_tracker: HoldConditionsTracker::new(),
            backdrive_detector: BackdriveDetector::new(),
            fast_accum: FastAccumulator::new(),
            last_input: None,
            setpoint: None,
            mode: ServoMode::Move,
            measured_velocity: DegPerSec10::from_dps10(0),
            fast_seq: 0,
        }
    }

    /// Reset all state (called on fault clear).
    pub fn reset(&mut self) {
        self.backdrive.reset();
        self.setpoint_tracker.reset();
        self.hold_tracker.reset();
        self.backdrive_detector.reset();
        self.fast_accum.reset();
        self.last_input = None;
        self.mode = ServoMode::Move;
        self.measured_velocity = DegPerSec10::from_dps10(0);
        // Note: safety, thermal, setpoint, fast_seq intentionally NOT reset
    }
}
