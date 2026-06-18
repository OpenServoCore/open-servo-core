use std::any::Any;

use crate::sim::{Effect, SimTime};

pub trait EventSource {
    fn next_event_time(&self) -> Option<SimTime>;
    fn advance(&mut self, t: SimTime) -> Vec<Effect>;
    fn receive_edge(&mut self, at: SimTime, rising: bool);
    fn reset(&mut self) {}
    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;

    /// Sync the device's internal time cursor to `t` without processing any
    /// events. The engine calls this on every device at the top of
    /// [`Sim::advance`](crate::sim::Sim::advance) so devices that didn't
    /// fire during the previous quiescence (a Host whose servo replied with
    /// nothing) still see current time when the next closure queues work.
    /// Default is a no-op for devices that don't cache a `now` cursor.
    fn tick(&mut self, _t: SimTime) {}
}
