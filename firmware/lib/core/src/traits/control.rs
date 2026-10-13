use crate::Shared;

/// What the kernel control loop reads from the board hardware.
pub trait Sensors {
    /// Acquisition only: complete the scan's frame in device counts in the
    /// table's tick block (`Shared::frame`), at the tick's entry.
    fn land(&mut self, shared: &Shared);
}

/// What the kernel control loop writes to the motor driver.
pub trait Motor {
    fn write(&mut self, cmd: MotorCmd);
}

/// Bag of chip-side capabilities the control loop needs. Splits into
/// disjoint sub-trait borrows so the kernel can hold both at once.
pub trait ControlIo {
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
    /// `drv_en` HIGH, PWM drive: `ticks` of the period on (`window::drive_ticks`),
    /// `reverse` in the wiring's frame, selectable off-window decay. Zero
    /// ticks coast.
    Drive {
        ticks: u16,
        reverse: bool,
        decay: DecayMode,
    },
}

#[derive(Copy, Clone, Debug, Default)]
pub enum DecayMode {
    /// Off-window = COAST (idle leg LOW). Low EMI, near-zero avg current on DRV8212P.
    Fast,
    /// Off-window = BRAKE (idle leg HIGH). DRV8212P-correct for usable torque.
    #[default]
    Slow,
}

bitflags::bitflags! {
    /// Mirrors `capability_flags` in `ConfigCommon`. Protocol-relevant only.
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
    fn motor_cmd_fits_one_register() {
        assert_eq!(size_of::<MotorCmd>(), size_of::<u32>());
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
