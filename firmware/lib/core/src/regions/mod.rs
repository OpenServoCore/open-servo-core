//! Host-visible register state. Reserved slots have no SRAM mirror; regmap
//! returns AccessError. Per-region block descriptors translate protocol
//! addresses; flash save/load uses the same descriptors.
//!
//!   CONFIG    0x0000..=0x01FF  (512 B)  — persistent A/B
//!   TELEMETRY 0x0200..=0x02FF  (256 B)  — RO from host
//!   CONTROL   0x0300..=0x08FF  (1536 B) — RW volatile
//!   CALIB     0x0900..=0x18FF  (4096 B) — single-copy + per-block CRC
//!
//! Owners go through `RegionStorage::with`/`with_mut`. Cross-domain readers
//! that must avoid forming `&T` (aliasing-sensitive paths) use raw-pointer
//! per-field reads directly on `cell.get()`.

use control_table::RegionStorage;
use core::cell::SyncUnsafeCell;

pub mod calib;
pub mod config;
pub mod control;
pub(crate) mod hooks;
pub mod locks;
pub mod telemetry;

pub use calib::{BemfCalibBlock, CalibRegs, PotLutBlock};
pub use config::{
    BaudRate, ConfigCalibration, ConfigComms, ConfigControlPosition, ConfigIdentity,
    ConfigPosLimits, ConfigRegs, ConfigStall, ConfigThermal, StallResponse, StatusReturnLevel,
};
pub use control::{BootMode, ControlLifecycle, ControlRegs, ControlStreaming, ControlSystem, Mode};
pub use telemetry::{
    TelemetryConverted, TelemetryDxlLink, TelemetryDxlTune, TelemetryFault,
    TelemetryIntermediaries, TelemetryRaw, TelemetryRegs,
};

use crate::regions::config::ConfigDefaults;
use control_table::Table;

pub const CONFIG_REGION_SIZE: usize = 512;
pub const TELEMETRY_REGION_SIZE: usize = 256;
pub const CONTROL_REGION_SIZE: usize = 1536;
pub const CALIB_REGION_SIZE: usize = 4096;

pub const CONFIG_BASE_ADDR: u16 = 0x0000;
pub const TELEMETRY_BASE_ADDR: u16 = 0x0200;
pub const CONTROL_BASE_ADDR: u16 = 0x0300;
pub const CALIB_BASE_ADDR: u16 = 0x0900;

#[repr(C)]
#[derive(Table)]
#[ct_table(max_sram = 1024, hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ControlTable {
    #[ct_region]
    pub config: SyncUnsafeCell<ConfigRegs>,
    #[ct_region]
    pub telemetry: SyncUnsafeCell<TelemetryRegs>,
    #[ct_region]
    pub control: SyncUnsafeCell<ControlRegs>,
    #[ct_region]
    pub calib: SyncUnsafeCell<CalibRegs>,
}

impl ControlTable {
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
        self.config.with_mut(|cfg| {
            cfg.pos_limits.pos_min_phys_urad = defaults.pos_min_phys_urad;
            cfg.pos_limits.pos_max_phys_urad = defaults.pos_max_phys_urad;
            cfg.pos_limits.pos_min_soft_urad = defaults.pos_min_phys_urad;
            cfg.pos_limits.pos_max_soft_urad = defaults.pos_max_phys_urad;
            cfg.calibration.vdd_mv = defaults.vdd_mv;
            cfg.comms.id = defaults.dxl_id;
            cfg.comms.baud_rate_idx = defaults.dxl_baud;
            cfg.comms.return_delay_2us = defaults.dxl_return_delay_2us;
            cfg.comms.clock_trim = defaults.clock_trim;
        });
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
