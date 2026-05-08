use crate::Shared;

/// Cooperative background services. Runs on the main thread; non-real-time
/// work like DXL parsing, flash commits, status indicators.
pub struct Services;

impl Services {
    pub const fn new() -> Self {
        Self
    }

    pub fn run(&mut self, _shared: &Shared) -> ! {
        loop {
            core::hint::spin_loop();
        }
    }
}
