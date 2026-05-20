use osc_units::Effort;

use crate::{FrameInputs, SampleFrame};

/// Stamped into `ControlTable.config` pre-IRQ; thereafter host-owned.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConfigDefaults {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
    /// VDD-at-chip-pin in mV; the v006 ADC reference is VDD itself.
    pub vdd_mv: u16,
}

pub trait Board {
    fn caps(&self) -> Capabilities {
        Capabilities::default()
    }
    fn sample(&mut self, inputs: &FrameInputs) -> SampleFrame;
    fn write_motor(&mut self, cmd: MotorCmd);
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

#[derive(Copy, Clone, Debug)]
pub enum DecayMode {
    Fast,
    Slow,
}

bitflags::bitflags! {
    /// Mirrors `capability_flags` in `ConfigIdentity`. Protocol-relevant only.
    #[derive(Copy, Clone, Debug, Default)]
    pub struct Capabilities: u32 {
        const HAS_MOTOR_ENCODER = 1 << 0;
    }
}
