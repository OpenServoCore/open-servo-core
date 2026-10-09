#[cfg_attr(timer_v3, path = "v3.rs")]
mod family;
pub mod tim2;

pub use family::*;
