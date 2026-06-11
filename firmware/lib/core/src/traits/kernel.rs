use osc_units::Effort;

use crate::{ConversionVariables, Sample};

/// What the kernel control loop reads from the board hardware.
pub trait Sensors {
    /// One full ADC/encoder frame, called from the kernel tick.
    fn sample(&mut self, vars: &ConversionVariables) -> Sample;
}

/// What the kernel control loop writes to the motor driver.
pub trait Motor {
    fn write(&mut self, cmd: MotorCmd);
}

/// Bag of chip-side capabilities the kernel needs. Splits into disjoint
/// sub-trait borrows so the kernel can hold both at once.
pub trait KernelIo {
    type Sensors: Sensors;
    type Motor: Motor;

    /// Servo-wide capability flags (e.g., HAS_MOTOR_ENCODER).
    fn caps(&self) -> Capabilities {
        Capabilities::default()
    }
    fn parts(&mut self) -> (&mut Self::Sensors, &mut Self::Motor);
}

#[derive(Copy, Clone, Debug)]
pub enum MotorCmd {
    /// `drv_en` LOW. Driver powered down; outputs hi-Z. Boot/fault state.
    Disabled,
    /// `drv_en` HIGH, both half-bridges hi-Z. Motor free-wheels.
    Coast,
    /// `drv_en` HIGH, both low-side FETs on. Short-circuit braking.
    Brake,
    /// `drv_en` HIGH, PWM drive with selectable off-window decay.
    Drive { duty: Effort, decay: DecayMode },
}

#[derive(Copy, Clone, Debug)]
pub enum DecayMode {
    /// Off-window = COAST (idle leg LOW). Low EMI, near-zero avg current on DRV8212P.
    Fast,
    /// Off-window = BRAKE (idle leg HIGH). DRV8212P-correct for usable torque.
    Slow,
}

bitflags::bitflags! {
    /// Mirrors `capability_flags` in `ConfigIdentity`. Protocol-relevant only.
    #[derive(Copy, Clone, Debug, Default)]
    pub struct Capabilities: u32 {
        const HAS_MOTOR_ENCODER = 1 << 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn motor_cmd_stays_small() {
        assert!(size_of::<MotorCmd>() <= 8, "got {}", size_of::<MotorCmd>());
    }

    #[test]
    fn capabilities_compose() {
        let none = Capabilities::default();
        assert!(none.is_empty());
        let with_enc = Capabilities::HAS_MOTOR_ENCODER;
        assert!(with_enc.contains(Capabilities::HAS_MOTOR_ENCODER));
        assert_eq!(with_enc.bits(), 1);
    }
}
