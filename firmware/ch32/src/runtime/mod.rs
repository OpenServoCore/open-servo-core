#[cfg(feature = "defmt")]
pub mod diag;
pub mod init;
pub mod isr;
pub mod registry;
pub mod run;
pub mod statics;

pub use init::bringup;
pub use registry::Drivers;
