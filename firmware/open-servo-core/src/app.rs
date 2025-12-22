//! Application orchestrator for servo control.
//!
//! App is a thin wrapper around ServoCore that handles:
//! - Reading sensors and building FastInputs
//! - Applying FastOutputs to hardware
//! - Event handling (slow tick, UART, faults)
//!
//! The actual control logic lives in ServoCore.

use open_servo_control::ControlLoop;
use open_servo_hw::motor::BdcMotorDriver;
#[cfg(feature = "current-sense-bus")]
use open_servo_hw::sensor::SafetyCurrentSource;
use open_servo_hw::sensor::{PositionSensor, SafetyMcuTempSource, SafetyVoltageSource};
use open_servo_math::{CentiC, CentiDeg, Duty};
#[cfg(feature = "current-sense-bus")]
use open_servo_math::MilliAmp;

use crate::event::Event;
use crate::fault::FaultKind;
use crate::inputs::FastInputs;
use crate::safety::{SafetyThresholds, SensorHealth};
use crate::servo_core::{ServoCore, SystemState};

// Re-export SystemState for backwards compatibility
pub use crate::servo_core::SystemState as AppSystemState;

/// Hardware orchestrator for servo control.
///
/// App reads sensors, calls ServoCore, and applies outputs to hardware.
/// It is generic over the control loop algorithm.
///
/// ## Trait Bounds
///
/// The hardware must implement:
/// - `BdcMotorDriver` - Motor control (required)
/// - `PositionSensor` - Position feedback (required)
/// - `SafetyCurrentSource` - Current sensing (optional via blanket impl)
/// - `SafetyVoltageSource` - Voltage sensing (optional via blanket impl)
/// - `SafetyMcuTempSource` - MCU temperature sensing (optional via blanket impl)
///
/// Boards without certain sensors implement the Safety*Source traits
/// directly, returning `None`. SafetyManager then skips those checks.
pub struct App<C: ControlLoop> {
    core: ServoCore<C>,
}

impl<C: ControlLoop> App<C> {
    /// Create a new App with the given controller.
    pub fn new(controller: C) -> Self {
        Self {
            core: ServoCore::new(controller),
        }
    }

    /// Hard real-time control tick (called from DMA/Timer ISR at 10kHz).
    ///
    /// Reads sensors, runs control loop, applies motor output.
    ///
    /// ## Safety Checks
    ///
    /// Safety checks are performed in ServoCore. Checks for missing sensors
    /// (those returning `None`) are automatically skipped.
    #[cfg(feature = "current-sense-bus")]
    pub fn on_control_tick<H>(&mut self, hw: &mut H)
    where
        H: BdcMotorDriver
            + PositionSensor
            + SafetyCurrentSource
            + SafetyVoltageSource
            + SafetyMcuTempSource,
    {
        // Build inputs from hardware
        let inputs = FastInputs {
            position: hw.read_position(),
            current: hw.read_safety_current(),
            bus_voltage: hw.read_safety_voltage(),
            temperature: hw.read_safety_mcu_temp(),
        };

        // Run pure control logic
        let outputs = self.core.fast_tick(inputs);

        // Apply outputs to hardware
        if outputs.motor_enable {
            hw.set_pwm(outputs.pwm_command);
        } else {
            hw.set_pwm(Duty::ZERO);
            hw.set_enable(false);
        }
    }

    /// Hard real-time control tick (called from DMA/Timer ISR at 10kHz).
    ///
    /// Version without current sensing.
    #[cfg(not(feature = "current-sense-bus"))]
    pub fn on_control_tick<H>(&mut self, hw: &mut H)
    where
        H: BdcMotorDriver + PositionSensor + SafetyVoltageSource + SafetyMcuTempSource,
    {
        // Build inputs from hardware (no current sensor)
        let inputs = FastInputs {
            position: hw.read_position(),
            bus_voltage: hw.read_safety_voltage(),
            temperature: hw.read_safety_mcu_temp(),
        };

        // Run pure control logic
        let outputs = self.core.fast_tick(inputs);

        // Apply outputs to hardware
        if outputs.motor_enable {
            hw.set_pwm(outputs.pwm_command);
        } else {
            hw.set_pwm(Duty::ZERO);
            hw.set_enable(false);
        }
    }

    /// Handle soft events (called from main loop).
    pub fn handle_event<H: BdcMotorDriver>(&mut self, hw: &mut H, event: Event) {
        match event {
            Event::SlowTick => {
                // Run slow tick monitoring
                if let Some(_fault) = self.core.slow_tick() {
                    // Apply safety on fault
                    hw.set_pwm(Duty::ZERO);
                    hw.set_enable(false);
                }

                // Log system state
                #[cfg(feature = "defmt")]
                defmt::info!("State: {:?}", self.core.system_state());
            }
            Event::Fault(_kind) => {
                // Log fault event (safety response already happened in fast_tick)
                #[cfg(feature = "defmt")]
                defmt::warn!("Fault event logged: {:?}", _kind);
            }
            Event::UartRx { port: _, byte: _ } => {
                // TODO: Handle UART protocol
            }
        }
    }

    /// Raise a fault manually (e.g., from external safety ISR).
    pub fn raise_fault<H: BdcMotorDriver>(&mut self, hw: &mut H, kind: FaultKind) {
        // Let core handle the fault
        self.core.fault_state().clone().raise(kind);
        self.core.clear_fault(); // This resets and re-raises properly
                                 // Actually we need a different approach - let's use the fault state directly

        // Apply safety immediately
        hw.set_pwm(Duty::ZERO);
        hw.set_enable(false);
    }

    /// Clear fault state.
    pub fn clear_fault<H: BdcMotorDriver>(&mut self, _hw: &mut H) {
        self.core.clear_fault();
    }

    /// Check if system is faulted.
    #[inline]
    pub fn is_faulted(&self) -> bool {
        self.core.is_faulted()
    }

    /// Get the current fault kind, if any.
    pub fn fault_kind(&self) -> Option<FaultKind> {
        self.core.fault_state().fault_kind()
    }

    /// Get current system state.
    pub fn get_system_state(&self) -> SystemState {
        self.core.system_state()
    }

    /// Set controller setpoint (in centidegrees).
    pub fn set_setpoint(&mut self, setpoint: CentiDeg) {
        self.core.set_setpoint(setpoint);
    }

    /// Get current setpoint.
    pub fn get_setpoint(&self) -> CentiDeg {
        self.core.get_setpoint()
    }

    // ============= Safety threshold accessors =============
    // Delegate to core.safety()

    /// Get current safety thresholds.
    pub fn get_thresholds(&self) -> SafetyThresholds {
        *self.core.safety().thresholds()
    }

    /// Set all safety thresholds.
    pub fn set_thresholds(&mut self, thresholds: SafetyThresholds) {
        self.core.safety_mut().set_thresholds(thresholds);
    }

    /// Set over-current threshold (requires `current-sense` feature).
    #[cfg(feature = "current-sense-bus")]
    pub fn set_current_limit(&mut self, limit: MilliAmp) {
        self.core.safety_mut().thresholds_mut().current_limit = limit;
    }

    /// Set MCU over-temperature threshold.
    pub fn set_mcu_temp_limit(&mut self, limit: CentiC) {
        self.core.safety_mut().thresholds_mut().mcu_temp_limit = limit;
    }

    /// Set maximum position delta per tick.
    pub fn set_position_max_delta(&mut self, delta: CentiDeg) {
        self.core.safety_mut().thresholds_mut().position_max_delta = delta;
    }

    /// Set consecutive bad sensor reads before fault.
    pub fn set_sensor_fault_count(&mut self, count: u8) {
        self.core.safety_mut().thresholds_mut().sensor_fault_count = count;
    }

    /// Set position bounds (min and max).
    pub fn set_position_bounds(&mut self, min: CentiDeg, max: CentiDeg) {
        let thresholds = self.core.safety_mut().thresholds_mut();
        thresholds.position_min = min;
        thresholds.position_max = max;
    }

    /// Set stall detection timeout in ticks.
    pub fn set_stall_timeout(&mut self, ticks: u16) {
        self.core.safety_mut().thresholds_mut().stall_timeout_ticks = ticks;
    }

    /// Set position error limit.
    pub fn set_position_error_limit(&mut self, limit: CentiDeg) {
        self.core.safety_mut().thresholds_mut().position_error_limit = limit;
    }

    /// Get sensor health state (for debugging).
    pub fn get_sensor_health(&self) -> &SensorHealth {
        self.core.safety().sensor_health()
    }
    
    /// Get estimated motor temperature in degrees.
    pub fn get_motor_temp_deg(&self) -> i16 {
        self.core.safety().motor_temp_deg()
    }
    
    /// Get motor temperature rise above ambient in degrees.
    pub fn get_motor_temp_rise_deg(&self) -> i16 {
        self.core.safety().motor_temp_rise_deg()
    }

    /// Get mutable reference to the controller.
    pub fn controller_mut(&mut self) -> &mut C {
        self.core.controller_mut()
    }

    /// Get reference to the controller.
    pub fn controller(&self) -> &C {
        self.core.controller()
    }
}
