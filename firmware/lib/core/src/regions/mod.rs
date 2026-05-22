//! Host-visible register state. Reserved slots have no SRAM mirror; regmap
//! returns AccessError. Per-region block descriptors translate protocol
//! addresses; flash save/load uses the same descriptors.
//!
//!   CONFIG    0x0000..=0x01FF  (512 B)  — persistent A/B
//!   TELEMETRY 0x0200..=0x02FF  (256 B)  — RO from host
//!   CONTROL   0x0300..=0x08FF  (1536 B) — RW volatile
//!   CALIB     0x0900..=0x18FF  (4096 B) — single-copy + per-block CRC
//!
//! Owners form transient `&mut RegionT` via `&mut *cell.get()`; cross-domain
//! readers use raw-pointer per-field reads, never form `&T`.

use core::cell::SyncUnsafeCell;

pub mod calib;
pub mod config;
pub mod control;
pub mod telemetry;

pub use calib::{BemfCalibBlock, CalibRegs, PotLutBlock};
pub use config::{
    ConfigCalibration, ConfigComms, ConfigControl, ConfigControlPosition, ConfigIdentity,
    ConfigLimits, ConfigPosLimits, ConfigRegs, ConfigSafety, ConfigStall, ConfigThermal,
};
pub use control::{ControlLifecycle, ControlRegs, ControlStreaming};
pub use telemetry::{
    TelemetryConverted, TelemetryFault, TelemetryIntermediaries, TelemetryRaw, TelemetryRegs,
};

use crate::board::ConfigDefaults;
use crate::page::PageHeader;

pub const PAGE_HEADER_SIZE: usize = core::mem::size_of::<PageHeader>();

pub const CONFIG_REGION_SIZE: usize = 512;
pub const CONFIG_BLOCK_SIZE: usize = 32;
pub const CONFIG_BLOCK_COUNT: usize = CONFIG_REGION_SIZE / CONFIG_BLOCK_SIZE;
pub const CONFIG_BODY_SIZE: usize = CONFIG_REGION_SIZE - PAGE_HEADER_SIZE;

pub const TELEMETRY_REGION_SIZE: usize = 256;
pub const TELEMETRY_BLOCK_SIZE: usize = 32;
pub const TELEMETRY_BLOCK_COUNT: usize = TELEMETRY_REGION_SIZE / TELEMETRY_BLOCK_SIZE;

pub const CONTROL_REGION_SIZE: usize = 1536;
pub const CONTROL_BLOCK_SIZE: usize = 32;
pub const CONTROL_BLOCK_COUNT: usize = CONTROL_REGION_SIZE / CONTROL_BLOCK_SIZE;

pub const CALIB_REGION_SIZE: usize = 4096;
pub const CALIB_BLOCK_SIZE: usize = 256;
pub const CALIB_BLOCK_COUNT: usize = CALIB_REGION_SIZE / CALIB_BLOCK_SIZE;
pub const CALIB_BLOCK_BODY_SIZE: usize = CALIB_BLOCK_SIZE - PAGE_HEADER_SIZE;

pub const CONFIG_BASE_ADDR: u16 = 0x0000;
pub const TELEMETRY_BASE_ADDR: u16 = 0x0200;
pub const CONTROL_BASE_ADDR: u16 = 0x0300;
pub const CALIB_BASE_ADDR: u16 = 0x0900;

#[repr(C)]
pub struct ControlTable {
    pub config: SyncUnsafeCell<ConfigRegs>,
    pub telemetry: SyncUnsafeCell<TelemetryRegs>,
    pub control: SyncUnsafeCell<ControlRegs>,
    pub calib: SyncUnsafeCell<CalibRegs>,
}

impl ControlTable {
    pub const fn const_new() -> Self {
        Self {
            config: SyncUnsafeCell::new(ConfigRegs::const_new()),
            telemetry: SyncUnsafeCell::new(TelemetryRegs::const_new()),
            control: SyncUnsafeCell::new(ControlRegs::const_new()),
            calib: SyncUnsafeCell::new(CalibRegs::const_new()),
        }
    }

    /// Soft limits init to physical limits per control-table doc.
    /// Caller must be sole writer (install-time, pre-IRQ).
    pub fn seed_config_defaults(&self, defaults: &ConfigDefaults) {
        crate::log::debug!(
            "seed CONFIG: phys=[{}, {}] urad  vdd_mv={}  dxl_id={}  baud_idx={}",
            defaults.pos_min_phys_urad,
            defaults.pos_max_phys_urad,
            defaults.vdd_mv,
            defaults.dxl_id,
            defaults.dxl_baud.as_idx(),
        );
        // SAFETY: install-time, pre-IRQ, sole writer.
        let cfg = unsafe { &mut *self.config.get() };
        cfg.limits.pos.pos_min_phys_urad = defaults.pos_min_phys_urad;
        cfg.limits.pos.pos_max_phys_urad = defaults.pos_max_phys_urad;
        cfg.limits.pos.pos_min_soft_urad = defaults.pos_min_phys_urad;
        cfg.limits.pos.pos_max_soft_urad = defaults.pos_max_phys_urad;
        cfg.calibration.vdd_mv = defaults.vdd_mv;
        cfg.comms.id = defaults.dxl_id;
        cfg.comms.baud_rate_idx = defaults.dxl_baud.as_idx();
    }

    /// Called once pre-PFIC-IRQ — sole writer.
    pub fn load_config_from_flash<F: embedded_storage::nor_flash::ReadNorFlash>(
        &self,
        _flash: &mut F,
        _page_a_addr: u32,
        _page_b_addr: u32,
    ) {
        todo!()
    }

    /// Called once pre-PFIC-IRQ — sole writer.
    pub fn load_calib_from_flash<F: embedded_storage::nor_flash::ReadNorFlash>(
        &self,
        _flash: &mut F,
        _calib_base_addr: u32,
    ) {
        todo!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn region_structs_fit_regions() {
        assert!(size_of::<ConfigRegs>() <= CONFIG_REGION_SIZE);
        assert!(size_of::<TelemetryRegs>() <= TELEMETRY_REGION_SIZE);
        assert!(size_of::<ControlRegs>() <= CONTROL_REGION_SIZE);
        assert!(size_of::<CalibRegs>() <= CALIB_REGION_SIZE);
    }

    #[test]
    fn control_table_sram_footprint() {
        assert!(size_of::<ControlTable>() <= 1024);
    }
}
