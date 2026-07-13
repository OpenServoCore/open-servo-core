#[cfg_attr(systick_rv2, path = "rv2.rs")]
mod family;

pub use family::*;
