use control_table::StagedWrites;

use crate::Shared;

use super::dispatch::Dispatcher;

/// Per-servo dispatch session: owns the HOLD-write staging buffer that spans
/// a WRITE+HOLD … COMMIT sequence. The bus driver holds one and asks it for a
/// [`Dispatcher`] each time it has a decoded request to run.
pub struct Session {
    staged: StagedWrites,
}

impl Session {
    pub fn new() -> Self {
        Self {
            staged: StagedWrites::new(),
        }
    }

    pub fn dispatcher<'a>(&'a mut self, shared: &'a Shared) -> Dispatcher<'a> {
        Dispatcher::new(shared, &mut self.staged)
    }
}

impl Default for Session {
    fn default() -> Self {
        Self::new()
    }
}
