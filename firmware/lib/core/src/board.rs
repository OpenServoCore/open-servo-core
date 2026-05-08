use osc_units::Effort;

pub trait Board {
    fn caps(&self) -> Capabilities;
    fn write_motor(&mut self, cmd: MotorCmd);
}

/// Commanded motor state. The chip-side `Board` impl maps these to the
/// H-bridge IO pattern + the driver chip's enable line.
#[derive(Copy, Clone, Debug)]
pub enum MotorCmd {
    /// `drv_en` LOW. Driver powered down; outputs hi-Z. Boot/fault state.
    Disabled,
    /// `drv_en` HIGH, both half-bridges hi-Z. Motor free-wheels.
    Coast,
    /// `drv_en` HIGH, both low-side FETs on. Short-circuit braking.
    Brake,
    /// `drv_en` HIGH, PWM drive with selected current decay during off-window.
    Drive { duty: Effort, decay: DecayMode },
}

impl MotorCmd {
    pub const ZERO: Self = Self::Disabled;
}

/// Current decay during the PWM off-window.
#[derive(Copy, Clone, Debug)]
pub enum DecayMode {
    /// Opposite leg floats during off-window. Faster current decay.
    Fast,
    /// Opposite leg actively shorts during off-window. Slower decay.
    Slow,
}

bitflags::bitflags! {
    /// Host-visible feature flags. Mirrors `capability_flags` in `ConfigIdentity`.
    /// Reports protocol-relevant features only — sensor type (pot, magnetic, …)
    /// is a `BoardCfg` concern; the host only sees `present_position` in microrads.
    #[derive(Copy, Clone, Debug, Default)]
    pub struct Capabilities: u32 {
        /// Motor-side encoder is wired. When set, firmware uses it for velocity
        /// feedback; when clear, velocity is estimated from the BEMF model.
        const HAS_MOTOR_ENCODER = 1 << 0;
    }
}
