//! Host-visible register state as a flat 1024-byte map. Sections carve
//! contiguous byte ranges; every address in `[0, 1024)` reads back (reserved
//! and skip bytes read as zero). Writes to non-writable bytes fail with
//! `AccessError`; addresses past the map end fail with `DataRange`.
//!
//!   CONFIG    0x000..0x080  (128 B) -- persistent via MGMT SAVE
//!   CALIB     0x080..0x180  (256 B) -- persistence deferred (unused today)
//!   CONTROL   0x180..0x200  (128 B) -- RW volatile
//!   TELEMETRY 0x200..0x280  (128 B) -- RO from host
//!   PROFILE   0x280..0x2C0  ( 64 B) -- read-profile span words (sec 5.2)
//!   (reserved 0x2C0..0x400  320 B)
//!
//! Owners go through `RegionStorage::with`/`with_mut` on the storage cell.
//! Cross-domain readers that must avoid forming `&T` (aliasing-sensitive
//! paths) use raw-pointer reads via `RegionStorageRaw::region_ptr`.

pub mod calib;
pub mod config;
pub mod control;
pub(crate) mod hooks;
pub mod profile;
pub mod telemetry;

pub use calib::{BemfCalibBlock, CalibRegs, PotLutBlock};
pub use config::{
    BaudRate, ConfigCalibration, ConfigCommon, ConfigControlPosition, ConfigPosLimits, ConfigRegs,
    ConfigStall, ConfigThermal, StallResponse,
};
pub use control::{BootMode, ControlLifecycle, ControlRegs, ControlStreaming, ControlSystem, Mode};
pub use profile::{PROFILE_SLOTS, ProfileRegs, ProfileSlots, SPANS_PER_SLOT};
pub use telemetry::{
    TelemetryCommon, TelemetryConverted, TelemetryIntermediaries, TelemetryMode, TelemetryRaw,
    TelemetryRegs,
};

use crate::regions::config::ConfigDefaults;
use control_table::{RegionStorage, Table};

pub const CONFIG_REGION_SIZE: u16 = 128;
pub const CALIB_REGION_SIZE: u16 = 256;
pub const CONTROL_REGION_SIZE: u16 = 128;
pub const TELEMETRY_REGION_SIZE: u16 = 128;
pub const PROFILE_REGION_SIZE: u16 = 64;

pub const CONFIG_BASE_ADDR: u16 = 0x000;
pub const CALIB_BASE_ADDR: u16 = 0x080;
pub const CONTROL_BASE_ADDR: u16 = 0x180;
pub const TELEMETRY_BASE_ADDR: u16 = 0x200;
pub const PROFILE_BASE_ADDR: u16 = 0x280;

#[repr(C)]
#[derive(Table)]
#[ct_table(size = 1024, hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ControlTable {
    pub config: ConfigRegs,
    pub calib: CalibRegs,
    pub control: ControlRegs,
    pub telemetry: TelemetryRegs,
    pub profile: ProfileRegs,
    #[ct_table(skip)]
    _rsvd: [u8; 320],
}

impl ControlTableCell {
    /// Soft limits init to physical limits per control-table doc.
    /// Caller must be sole writer (install-time, pre-IRQ).
    pub fn seed_config_defaults(&self, defaults: &ConfigDefaults) {
        crate::log::debug!(
            "seed CONFIG: phys=[{}, {}] urad  vdd_mv={}  id={}  baud_idx={}",
            defaults.pos_min_phys_urad,
            defaults.pos_max_phys_urad,
            defaults.vdd_mv,
            defaults.id,
            defaults.baud.as_idx(),
        );
        // SAFETY: install-time, pre-IRQ, sole writer.
        self.with_mut(|t| {
            let cfg = &mut t.config;
            cfg.pos_limits.pos_min_phys_urad = defaults.pos_min_phys_urad;
            cfg.pos_limits.pos_max_phys_urad = defaults.pos_max_phys_urad;
            cfg.pos_limits.pos_min_soft_urad = defaults.pos_min_phys_urad;
            cfg.pos_limits.pos_max_soft_urad = defaults.pos_max_phys_urad;
            cfg.calibration.vdd_mv = defaults.vdd_mv;
            cfg.common.id = defaults.id;
            cfg.common.baud_rate_idx = defaults.baud.as_idx();
            cfg.common.response_deadline_us = defaults.response_deadline_us;
        });
    }
}

#[cfg(test)]
mod tests {
    use osc_protocol::table;

    use super::config::addr as config_addr;
    use super::telemetry::addr as telemetry_addr;

    /// Pins the struct layout to the protocol sec 5.4 common register block:
    /// every generated field address equals its protocol const, and the
    /// model-specific space begins exactly at each block's end.
    #[test]
    fn common_register_block_offsets_match_the_protocol() {
        assert_eq!(config_addr::common::MODEL_NUMBER, table::MODEL_NUMBER);
        assert_eq!(
            config_addr::common::FIRMWARE_VERSION,
            table::FIRMWARE_VERSION
        );
        assert_eq!(
            config_addr::common::HARDWARE_REVISION,
            table::HARDWARE_REVISION
        );
        assert_eq!(
            config_addr::common::CAPABILITY_FLAGS,
            table::CAPABILITY_FLAGS
        );
        assert_eq!(config_addr::common::ID, table::ID);
        assert_eq!(config_addr::common::BAUD_RATE_IDX, table::BAUD_RATE_IDX);
        assert_eq!(
            config_addr::common::RESPONSE_DEADLINE_US,
            table::RESPONSE_DEADLINE_US
        );
        assert_eq!(
            config_addr::pos_limits::POS_MIN_PHYS_URAD,
            table::CONFIG_COMMON_END
        );

        assert_eq!(telemetry_addr::common::FAULT_FLAGS, table::FAULT_FLAGS);
        assert_eq!(telemetry_addr::common::STATUS_FLAGS, table::STATUS_FLAGS);
        assert_eq!(telemetry_addr::common::TRIM_STEPS, table::TRIM_STEPS);
        assert_eq!(
            telemetry_addr::common::CRC_FAIL_COUNT,
            table::CRC_FAIL_COUNT
        );
        assert_eq!(
            telemetry_addr::common::FRAMING_DROP_COUNT,
            table::FRAMING_DROP_COUNT
        );
        assert_eq!(
            telemetry_addr::mode::MODE_ACTIVE,
            table::TELEMETRY_COMMON_END
        );
    }
}
