#![no_std]
#![forbid(unsafe_code)]

pub mod app;
pub mod debug_shell;
pub mod event;
pub mod fault;

pub use app::{App, SystemState};
pub use debug_shell::DebugShell;
pub use event::{Event, EventConsumer, EventProducer, EventQueue};
pub use fault::{FaultKind, FaultState};
