use osc_units::Effort;

/// Stamped into `ControlTable.config` pre-IRQ; thereafter host-owned.
/// Future load order: flash → these → core's `const_new`.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConfigDefaults {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
    /// VDD-at-chip-pin in mV. v006 ADC reference is VDD (no AVDD pin); single
    /// DMM measurement turns ±2-3 % LDO tolerance into DMM-limited accuracy.
    /// 3300 mV is the spec-typical fallback if the board hasn't been cal'd.
    pub vdd_mv: u16,
}

pub trait Board {
    fn caps(&self) -> Capabilities;
    fn write_motor(&mut self, cmd: MotorCmd);
    /// Per-tick cadence indicator (e.g. status-LED toggle).
    fn pulse_tick_indicator(&mut self) {}
}

#[derive(Copy, Clone, Debug)]
pub enum MotorCmd {
    /// `drv_en` LOW. Driver powered down; outputs hi-Z. Boot/fault state.
    Disabled,
    /// `drv_en` HIGH, both half-bridges hi-Z. Motor free-wheels.
    Coast,
    /// `drv_en` HIGH, both low-side FETs on. Short-circuit braking.
    Brake,
    /// `drv_en` HIGH, PWM drive with selected off-window decay.
    Drive { duty: Effort, decay: DecayMode },
}

impl MotorCmd {
    pub const ZERO: Self = Self::Disabled;
}

#[derive(Copy, Clone, Debug)]
pub enum DecayMode {
    /// Opposite leg floats during off-window. Faster decay.
    Fast,
    /// Opposite leg actively shorts during off-window. Slower decay.
    Slow,
}

bitflags::bitflags! {
    /// Mirrors `capability_flags` in `ConfigIdentity`. Protocol-relevant only.
    #[derive(Copy, Clone, Debug, Default)]
    pub struct Capabilities: u32 {
        /// When set, firmware uses motor encoder for velocity; else BEMF model.
        const HAS_MOTOR_ENCODER = 1 << 0;
    }
}
