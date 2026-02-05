//! Velocity sensor traits.

use open_servo_math::DegPerSec10;

/// Velocity feedback (from encoder differentiation or dedicated sensor).
///
/// Used for velocity control loops and damping.
pub trait VelocitySensor {
    /// Read angular velocity in 0.1 deg/s (DegPerSec10).
    fn read_velocity(&self) -> DegPerSec10;
}
