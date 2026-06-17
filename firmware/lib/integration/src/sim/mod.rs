//! Discrete-event simulator for the DXL bus.
//!
//! Each device owns its own clock and queue, advertises its next scheduled
//! event time, and on `advance` returns `Effect`s the orchestrator routes
//! back to peers. `Wire` is the delivery medium: it routes per-bit edges
//! between devices in [`DeviceRegistry`].

pub mod effect;
pub mod orchestrator;
pub mod registry;
pub mod source;
pub mod time;
pub mod wire;

pub use effect::Effect;
pub use orchestrator::Sim;
pub use registry::{DeviceId, DeviceRegistry};
pub use source::EventSource;
pub use time::{ClockRatio, SimTime};
pub use wire::Wire;
