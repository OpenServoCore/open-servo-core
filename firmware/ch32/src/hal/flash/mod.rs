#[cfg_attr(flash_v00x, path = "v00x.rs")]
mod family;

pub use family::*;
