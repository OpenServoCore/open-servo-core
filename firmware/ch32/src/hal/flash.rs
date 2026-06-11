#[cfg_attr(flash_v00x, path = "flash/v00x.rs")]
mod family;

pub use family::*;
