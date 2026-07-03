use std::cell::Cell;
use std::rc::Rc;

use osc_drivers::mocks::MockTelemetry;

/// Miss-counter accumulators for `MockTelemetry` — sim harnesses read
/// observed wire-condition telemetry out of the state.
#[derive(Clone, Default)]
pub struct TelemetryState {
    edge_anchor_misses: Rc<Cell<u32>>,
    patch_misses: Rc<Cell<u32>>,
}

impl TelemetryState {
    pub fn edge_anchor_miss_count(&self) -> u32 {
        self.edge_anchor_misses.get()
    }

    pub fn patch_miss_count(&self) -> u32 {
        self.patch_misses.get()
    }
}

pub fn mock_telemetry() -> (MockTelemetry, TelemetryState) {
    let state = TelemetryState::default();
    let mut m = MockTelemetry::new();
    {
        let c = state.edge_anchor_misses.clone();
        m.expect_record_edge_anchor_miss().returning_st(move || {
            c.set(c.get().wrapping_add(1));
        });
    }
    {
        let c = state.patch_misses.clone();
        m.expect_record_crc_patch_deadline_miss()
            .returning_st(move || {
                c.set(c.get().wrapping_add(1));
            });
    }
    (m, state)
}
