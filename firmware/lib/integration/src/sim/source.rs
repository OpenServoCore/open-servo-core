use std::any::Any;

use crate::sim::{Effect, SimTime};

/// Conductor surface for a discrete-event actor. `Host` and `Servo` each
/// implement it; the registry holds them as boxed `dyn EventSource` and
/// [`Sim::settle`](crate::sim::Sim::settle) is the only caller.
///
/// `advance` returns [`Effect`]s rather than mutating shared state — the
/// conductor stamps the source `DeviceId` and routes them through the bus.
/// That signature is what lets the test surface stay one-shot
/// (`host.send_X(); host.wait_for_reply();`) without re-borrowing `Sim`
/// from inside an actor.
pub trait EventSource {
    fn next_event_time(&self) -> Option<SimTime>;
    fn advance(&mut self, t: SimTime) -> Vec<Effect>;
    fn receive_edge(&mut self, at: SimTime, rising: bool);

    /// Sync the actor's internal `now` cursor to `t` without running any
    /// state-machine work. Called by [`Sim::device_mut`](crate::sim::Sim::device_mut)
    /// (and its `host_mut`/`servo_mut` sugar) so a test queuing work via
    /// the returned handle sees current sim time instead of a stale cursor
    /// from the previous quiescence. Default is a no-op for actors that
    /// don't cache a `now` cursor.
    fn set_now(&mut self, _t: SimTime) {}

    fn as_any(&self) -> &dyn Any;
    fn as_any_mut(&mut self) -> &mut dyn Any;
}
