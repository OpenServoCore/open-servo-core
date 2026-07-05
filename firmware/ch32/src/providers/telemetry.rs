//! Telemetry provider — wire-condition miss counters stored in the
//! control-table telemetry region so the host reads them back over the
//! bus. Both counters use volatile RMW into `SHARED` — same pattern as
//! the `sample_tick` increment in `runtime/isr.rs`. A concurrent host
//! clear (via DXL bus write to the counter register) can drop one update
//! per race window, which the region's `rw` declaration explicitly
//! accepts (`telemetry.rs:72-74`).

use osc_core::RegionStorageRaw;
use osc_drivers::traits::dxl;

use crate::runtime::statics::SHARED;

/// Production binding to the `SHARED` control-table telemetry region.
pub struct Telemetry;

impl dxl::Telemetry for Telemetry {
    fn record_crc_patch_deadline_miss(&mut self) {
        // SAFETY: as above — sole RMW writer at this PFIC priority.
        unsafe {
            let p = &raw mut (*SHARED.table.region_ptr())
                .telemetry
                .link
                .crc_patch_deadline_miss;
            p.write_volatile(p.read_volatile().wrapping_add(1));
        }
    }
}
