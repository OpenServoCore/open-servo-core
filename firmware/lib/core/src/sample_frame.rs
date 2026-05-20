use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::Shared;

/// Snapshot taken at the top of the DMA TC ISR so the board never holds a
/// cross-domain reference into the `ControlTable`.
#[derive(Copy, Clone, Debug)]
pub struct FrameInputs {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
    /// VDD-at-chip-pin in mV; host-writable cal knob, seeded from board defaults.
    pub vdd_mv: u16,
}

impl FrameInputs {
    /// Per-field `&raw const` reads — cross-domain discipline forbids forming `&ConfigRegs`.
    /// Naturally-aligned i32/u16, so torn-write-safe vs concurrent single-field host writes.
    pub fn snapshot(shared: &Shared) -> Self {
        // SAFETY: per-field raw-pointer copy, no `&` into ConfigRegs/ConfigPosLimits.
        unsafe {
            let cfg = shared.table.config.get();
            Self {
                pos_min_phys_urad: (&raw const (*cfg).limits.pos.pos_min_phys_urad).read(),
                pos_max_phys_urad: (&raw const (*cfg).limits.pos.pos_max_phys_urad).read(),
                vdd_mv: (&raw const (*cfg).calibration.vdd_mv).read(),
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct SampleFrame {
    pub tick: u32,
    pub pos: Microrads,
    /// Post-OPA peak (drive-phase): V_OPAOUT → mA. Production current reading.
    pub current: Milliamps,
    /// Post-OPA trough (brake-phase): diagnostic.
    pub current_post_trough: Milliamps,
    pub temp: CentiCelsius,
    pub vbus: Millivolts,
    /// |V_A − V_B| during the ON window of the PWM period.
    pub vmotor: Millivolts,
    pub raw: RawSamples,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct RawSamples {
    pub pos: u16,
    /// Peak post-OPA shunt count (ON-window centre, IN9/OpaOut).
    pub current: u16,
    /// Trough post-OPA shunt count (OFF-window centre).
    pub shunt_post_trough: u16,
    pub temp: u16,
    pub vbus: u16,
    /// Peak (drive-phase / TOP scan) raw of motor terminal A divider.
    pub vmotor_a: u16,
    /// Trough (brake-phase / BOTTOM scan) raw of motor terminal A divider.
    pub vmotor_a_trough: u16,
    /// Peak motor terminal B divider.
    pub vmotor_b: u16,
    /// Trough motor terminal B divider.
    pub vmotor_b_trough: u16,
    /// IN10 raw. AVDD-ratiometric drift sentinel; not a fixed reference.
    pub vcal: u16,
    /// EWMA-filtered Vcal — same units (raw LSB), much lower noise.
    pub vcal_lpf: u16,
}
