//! Host-visible register state as a flat 1024-byte map. Sections carve
//! contiguous byte ranges; every address in `[0, 1024)` reads back (reserved
//! and skip bytes read as zero). Writes to non-writable bytes fail with
//! `AccessError`; addresses past the map end fail with `DataRange`.
//!
//!   CONFIG    0x000..0x080  (128 B) — persistent, torque-gated
//!   CALIB     0x080..0x180  (256 B) — persistent, torque-gated
//!   CONTROL   0x180..0x200  (128 B) — RW volatile
//!   TELEMETRY 0x200..0x280  (128 B) — RO from host
//!   (reserved 0x280..0x400  384 B)
//!
//! Owners go through `RegionStorage::with`/`with_mut` on the storage cell.
//! Cross-domain readers that must avoid forming `&T` (aliasing-sensitive
//! paths) use raw-pointer reads via `RegionStorageRaw::region_ptr`.

pub mod calib;
pub mod config;
pub mod control;
pub(crate) mod hooks;
pub mod telemetry;

pub use calib::{BemfCalibBlock, CalibRegs, PotLutBlock};
pub use config::{
    BaudRate, ConfigCalibration, ConfigComms, ConfigControlPosition, ConfigIdentity,
    ConfigPosLimits, ConfigRegs, ConfigStall, ConfigThermal, StallResponse, StatusReturnLevel,
};
pub use control::{BootMode, ControlLifecycle, ControlRegs, ControlStreaming, ControlSystem, Mode};
pub use telemetry::{
    TelemetryConverted, TelemetryDxlLink, TelemetryFault, TelemetryIntermediaries, TelemetryRaw,
    TelemetryRegs,
};

use crate::regions::config::ConfigDefaults;
use control_table::{RegionStorage, Table};

pub const CONFIG_REGION_SIZE: u16 = 128;
pub const CALIB_REGION_SIZE: u16 = 256;
pub const CONTROL_REGION_SIZE: u16 = 128;
pub const TELEMETRY_REGION_SIZE: u16 = 128;

pub const CONFIG_BASE_ADDR: u16 = 0x000;
pub const CALIB_BASE_ADDR: u16 = 0x080;
pub const CONTROL_BASE_ADDR: u16 = 0x180;
pub const TELEMETRY_BASE_ADDR: u16 = 0x200;

#[repr(C)]
#[derive(Table)]
#[ct_table(size = 1024, hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ControlTable {
    pub config: ConfigRegs,
    pub calib: CalibRegs,
    pub control: ControlRegs,
    pub telemetry: TelemetryRegs,
    #[ct_table(skip)]
    _rsvd: [u8; 384],
}

impl ControlTableCell {
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
        self.with_mut(|t| {
            let cfg = &mut t.config;
            cfg.pos_limits.pos_min_phys_urad = defaults.pos_min_phys_urad;
            cfg.pos_limits.pos_max_phys_urad = defaults.pos_max_phys_urad;
            cfg.pos_limits.pos_min_soft_urad = defaults.pos_min_phys_urad;
            cfg.pos_limits.pos_max_soft_urad = defaults.pos_max_phys_urad;
            cfg.calibration.vdd_mv = defaults.vdd_mv;
            cfg.comms.id = defaults.dxl_id;
            cfg.comms.baud_rate_idx = defaults.dxl_baud;
            cfg.comms.return_delay_2us = defaults.dxl_return_delay_2us;
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
