use control_table::StagedWrites;

use crate::Shared;

use super::dispatch::{Dispatcher, PendingWrite};

/// Per-servo dispatch session: owns the HOLD-write staging buffer that spans
/// a WRITE+HOLD … COMMIT sequence, plus the pending-write slot (the one write
/// staged ahead of its CRC verdict — the dispatch spine). Both outlive the
/// [`Dispatcher`], which the bus rebuilds each wake. The bus driver holds one
/// and asks it for a [`Dispatcher`] each time it has a decoded request to run.
pub struct Session {
    staged: StagedWrites,
    pending: Option<PendingWrite>,
}

impl Session {
    pub fn new() -> Self {
        Self {
            staged: StagedWrites::new(),
            pending: None,
        }
    }

    pub fn dispatcher<'a>(&'a mut self, shared: &'a Shared) -> Dispatcher<'a> {
        Dispatcher::new(shared, &mut self.staged, &mut self.pending)
    }
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}
