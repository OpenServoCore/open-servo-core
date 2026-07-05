use osc_units::{CentiCelsius, Microrads, Milliamps, Millivolts};

use crate::Shared;
use control_table::RegionStorageRaw;

/// Per-frame snapshot of the live config slice the sample-conversion math
/// needs. Lifted from `ControlTable` regions (`pos_limits`, `calibration`)
/// via raw-pointer reads so the conversion never holds a `&` into the table.
#[derive(Copy, Clone, Debug)]
pub struct ConversionVariables {
    pub pos_min_phys_urad: i32,
    pub pos_max_phys_urad: i32,
    pub vdd_mv: u16,
}

impl ConversionVariables {
    pub fn snapshot(shared: &Shared) -> Self {
        // SAFETY: per-field raw-pointer copy, no `&` into ConfigRegs/ConfigPosLimits.
        unsafe {
            let t = shared.table.region_ptr();
            Self {
                pos_min_phys_urad: (&raw const (*t).config.pos_limits.pos_min_phys_urad).read(),
                pos_max_phys_urad: (&raw const (*t).config.pos_limits.pos_max_phys_urad).read(),
                vdd_mv: (&raw const (*t).config.calibration.vdd_mv).read(),
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Sample {
    pub tick: u32,
    pub pos: Microrads,
    pub current: Milliamps,
    pub current_post_trough: Milliamps,
    pub temp: CentiCelsius,
    pub vbus: Millivolts,
    pub vmotor: Millivolts,
    pub raw: RawSamples,
}

#[derive(Copy, Clone, Debug, Default)]
pub struct RawSamples {
    pub pos: u16,
    pub current: u16,
    pub shunt_post_trough: u16,
    pub temp: u16,
    pub vbus: u16,
    pub vmotor_a: u16,
    pub vmotor_a_trough: u16,
    pub vmotor_b: u16,
    pub vmotor_b_trough: u16,
    pub vcal: u16,
    pub vcal_lpf: u16,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::config::ConfigDefaults;

    #[test]
    fn snapshot_reads_back_seeded_defaults() {
        let shared = Shared::new();
        let defaults = ConfigDefaults {
            pos_min_phys_urad: -1_500_000,
            pos_max_phys_urad: 1_500_000,
            vdd_mv: 3275,
            ..Default::default()
        };
        shared.table.seed_config_defaults(&defaults);

        let vars = ConversionVariables::snapshot(&shared);
        assert_eq!(vars.pos_min_phys_urad, -1_500_000);
        assert_eq!(vars.pos_max_phys_urad, 1_500_000);
        assert_eq!(vars.vdd_mv, 3275);
    }
}
