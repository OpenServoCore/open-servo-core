use crate::Shared;

/// Main-thread cooperative background work (DXL parsing, flash commits, status).
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

impl Default for Services {
    fn default() -> Self {
        Self::new()
    }
}
