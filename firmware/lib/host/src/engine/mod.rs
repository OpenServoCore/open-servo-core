//! The host engine: framer (ring walk), schedule (await modes + deadlines),
//! and the `HostBus` composite that runs one command at a time.

pub mod framer;

pub use framer::{Frame, Framer, Step};
