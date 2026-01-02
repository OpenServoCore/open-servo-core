//! Shared test utilities for open-servo-kernel tests.

#![cfg(test)]

use open_servo_hw::v2::capability::{MotorType, SensorCapabilities, ServoPosKind};
use open_servo_math::CentiDeg;

use crate::state::{KernelConfig, PidGains};

/// Create a KernelConfig with reasonable test defaults.
pub fn test_kernel_config() -> KernelConfig {
    KernelConfig {
        pos_pid: PidGains::default(),
        hold_deadband_cdeg: 50,
        effort_limit_raw: 16000,
        servo_pos_kind: ServoPosKind::Bounded {
            min: CentiDeg::from_cdeg(-9500),
            max: CentiDeg::from_cdeg(9500),
        },
        motor_type: MotorType::Bdc,
        sensor_caps: SensorCapabilities::empty(),
    }
}
