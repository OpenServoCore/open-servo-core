//! Field-write hook surface for the `ControlTable` derive.
//!
//! Lives inside `regions` because it's a control-table concern: the
//! macro-generated `dispatch_events` fires methods on this trait whenever a
//! committed write touches a tagged field. The single [`ControlTableHooks`]
//! translator then maps each new value to the matching protocol-level
//! [`crate::Event`] and forwards via [`crate::ServiceEvents::send`].
//!
//! Kept `pub(crate)`: chip impls only see `Event` / `ServiceEvents`; the
//! field→event wiring stays an internal concern of the table layer.

use crate::regions::config::BaudRate;
use crate::traits::{Event, ServiceEvents};

pub(crate) trait ControlTableHookEvents {
    fn on_baud_rate_idx_write(&mut self, value: BaudRate);
    fn on_clock_trim_write(&mut self, value: i8);
    fn on_clock_fine_trim_us_write(&mut self, value: i16);
}

pub(crate) struct ControlTableHooks<'a, E: ServiceEvents> {
    events: &'a mut E,
}

impl<'a, E: ServiceEvents> ControlTableHooks<'a, E> {
    pub(crate) fn new(events: &'a mut E) -> Self {
        Self { events }
    }
}

impl<E: ServiceEvents> ControlTableHookEvents for ControlTableHooks<'_, E> {
    fn on_baud_rate_idx_write(&mut self, value: BaudRate) {
        self.events.send(Event::SetDxlBaud(value));
    }
    fn on_clock_trim_write(&mut self, value: i8) {
        self.events.send(Event::SetClockTrim(value));
    }
    fn on_clock_fine_trim_us_write(&mut self, value: i16) {
        self.events.send(Event::SetClockFineTrimUs(value));
    }
}
