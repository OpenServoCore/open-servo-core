//! Shared test utilities for open-servo-core tests.

#![cfg(test)]

use crate::{FastInputs, ServoCore};
use open_servo_control::{ControlInput, ControlLoop, ControlOutput};
use open_servo_hw::{BoardSafetyConfig, BoardThermalConfig};
use open_servo_math::{CentiDeg, ComplianceConfig, Duty};

/// Minimal mock controller for testing ServoCore.
pub struct MockController {
    pub output: Duty,
    pub saturated: bool,
    pub reset_called: bool,
}

impl MockController {
    pub fn new() -> Self {
        Self {
            output: Duty::ZERO,
            saturated: false,
            reset_called: false,
        }
    }

    pub fn set_output(&mut self, output: Duty) {
        self.output = output;
    }

    pub fn set_saturated(&mut self, saturated: bool) {
        self.saturated = saturated;
    }
}

impl ControlLoop for MockController {
    fn reset(&mut self) {
        self.reset_called = true;
        self.output = Duty::ZERO;
    }

    fn fast_tick(&mut self, _input: &ControlInput) -> ControlOutput {
        ControlOutput {
            duty: self.output,
            saturated: self.saturated,
        }
    }

    fn medium_tick(&mut self, _input: &ControlInput) {}
    fn slow_tick(&mut self, _input: &ControlInput) {}
}

/// Create a ServoCore with default test configuration.
pub fn make_core(controller: MockController) -> ServoCore<MockController> {
    let safety_config = BoardSafetyConfig {
        current_limit_ma: 800,
        mcu_temp_limit_cc: 8000,
        position_max_delta_cdeg: 500,
        sensor_fault_count: 10,
        position_min_cdeg: 0,
        position_max_cdeg: 18000,
        stall_timeout_ticks: 1000,
        stall_position_tolerance_cdeg: 10,
        position_error_limit_cdeg: 3000,
        position_error_timeout_us: 5000, // 5ms = 50 ticks @ 10kHz for fast test iterations
    };
    let thermal_config = BoardThermalConfig {
        resistance_mohm: 5000,       // 5.0 ohm
        thermal_resistance_cw: 1000, // 10 C/W
        thermal_capacity_cj: 1500,   // 15 J/C
    };
    let compliance_config = ComplianceConfig::new(600, 50, 3, 230, 3277);
    ServoCore::new(
        controller,
        safety_config,
        thermal_config,
        compliance_config.clone(),
        compliance_config,
    )
}

/// Create FastInputs with the given position.
pub fn make_inputs(position: i16) -> FastInputs {
    FastInputs {
        position: CentiDeg::from_cdeg(position),
        #[cfg(feature = "current-sense-bus")]
        current: None,
        bus_voltage: None,
        temperature: None,
    }
}

/// Create an iterator that repeats the given input N times.
pub fn repeat_inputs(input: FastInputs, n: usize) -> impl Iterator<Item = FastInputs> {
    core::iter::repeat(input).take(n)
}
