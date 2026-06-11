pub mod bus;
pub mod events;

pub use bus::Ch32Bus;
pub use events::Ch32Events;

use osc_core::ServicesIo;

pub struct Ch32ServicesIo {
    pub bus: Ch32Bus,
    pub events: Ch32Events,
}

impl Ch32ServicesIo {
    pub const fn new() -> Self {
        Self {
            bus: Ch32Bus::new(),
            events: Ch32Events::new(),
        }
    }
}

impl Default for Ch32ServicesIo {
    fn default() -> Self {
        Self::new()
    }
}

impl ServicesIo for Ch32ServicesIo {
    type Bus = Ch32Bus;
    type Events = Ch32Events;

    fn parts(&mut self) -> (&mut Ch32Bus, &mut Ch32Events) {
        (&mut self.bus, &mut self.events)
    }
}
