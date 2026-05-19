use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::Shared;

/// Per-tick snapshot of the inputs `build_sample_frame` needs from
/// `Shared`. Built once at the top of the DMA TC ISR; passed by ref to
/// the board's frame builder. Keeps cross-domain reads out of the hot
/// conversion path — the board never holds a `&Shared` and never sees
/// the `ControlTable` layout.
///
/// TODO: once flash persistence lands, replace this snapshot with a
/// read-only view that points directly at the active CONFIG page in
/// flash (and at CALIB for LUTs like `pot_lut[55]`, `temp_lut[64]`).
/// Torque-off is required for CONFIG writes, so reader/writer are
/// temporally exclusive — the view can be a couple of raw pointers,
/// no per-tick copy needed.
#[derive(Copy, Clone, Debug)]
pub struct FrameInputs {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
}

impl FrameInputs {
    /// Snap the fields needed by `build_sample_frame` out of `shared`.
    /// Uses `&raw const` per-field reads (regions/mod.rs cross-domain
    /// discipline: never form `&T` into another domain's region). Each
    /// field is naturally-aligned `i32`, so reads are torn-write-safe
    /// against a concurrent single-field host write.
    pub fn snapshot(shared: &Shared) -> Self {
        // SAFETY: cross-domain read. Per-field raw-pointer copy via
        // `&raw const` avoids forming `&ConfigRegs` / `&ConfigPosLimits`.
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
    /// Peak shunt count (ON-window centre under center-aligned PWM).
    pub current: u16,
    pub temp: u16,
    pub vbus: u16,
    pub vmotor_a: u16,
    pub vmotor_b: u16,
}
