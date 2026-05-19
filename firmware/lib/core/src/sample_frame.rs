use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::Shared;

/// Snapshot taken at the top of the DMA TC ISR so the board never holds a
/// cross-domain reference into the `ControlTable`.
#[derive(Copy, Clone, Debug)]
pub struct FrameInputs {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
}

impl FrameInputs {
    /// Per-field `&raw const` reads — cross-domain discipline forbids forming `&ConfigRegs`.
    /// Naturally-aligned i32, so torn-write-safe vs concurrent single-field host writes.
    pub fn snapshot(shared: &Shared) -> Self {
        // SAFETY: per-field raw-pointer copy, no `&` into ConfigRegs/ConfigPosLimits.
        unsafe {
            let cfg = shared.table.config.get();
            Self {
                pos_min_phys_urad: (&raw const (*cfg).limits.pos.pos_min_phys_urad).read(),
                pos_max_phys_urad: (&raw const (*cfg).limits.pos.pos_max_phys_urad).read(),
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SampleFrame {
    pub tick: u32,
    pub pos: Microrads,
    pub current: Milliamps,
    pub temp: CentiCelsius,
    pub vbus: Millivolts,
    /// |V_A − V_B| during the ON window of the PWM period.
    pub vmotor: Millivolts,
    pub raw: RawSamples,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct RawSamples {
    pub pos: u16,
    /// Peak shunt count (ON-window centre).
    pub current: u16,
    pub temp: u16,
    pub vbus: u16,
    pub vmotor_a: u16,
    pub vmotor_b: u16,
}
