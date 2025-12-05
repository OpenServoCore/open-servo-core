#![no_std]
#![forbid(unsafe_code)]

pub mod app;
pub mod event;
pub mod fault;

pub use app::{App, SystemState};
pub use event::{Event, EventQueue, EventProducer, EventConsumer};
pub use fault::{FaultKind, FaultState};