//! Telemetry provider — wire-condition miss counters stored in the
//! control-table telemetry region so the host reads them back over the
//! bus. Both counters use volatile RMW into `SHARED` — same pattern as
//! the `sample_tick` increment in `runtime/isr.rs`. A concurrent host
//! clear (via DXL bus write to the counter register) can drop one update
//! per race window, which the region's `rw` declaration explicitly
//! accepts (`telemetry.rs:72-74`).

use osc_drivers::traits::dxl;

use crate::runtime::statics::SHARED;

/// Production binding to the `SHARED` control-table telemetry region.
pub struct Telemetry;

impl dxl::Telemetry for Telemetry {
    fn record_edge_anchor_miss(&mut self) {
        // SAFETY: SHARED is the canonical chip-side control-table store;
        // writers other than this one fire from DXL-side ISRs at the same
        // PFIC priority, so the RMW is atomic w.r.t. itself.
        unsafe {
            let p = &raw mut (*SHARED.table.telemetry.get()).link.edge_anchor_miss;
            p.write_volatile(p.read_volatile().wrapping_add(1));
        }
    }

    fn record_crc_patch_deadline_miss(&mut self) {
        // SAFETY: as above — sole RMW writer at this PFIC priority.
        unsafe {
            let p = &raw mut (*SHARED.table.telemetry.get()).link.crc_patch_deadline_miss;
            p.write_volatile(p.read_volatile().wrapping_add(1));
        }
    }
}
