#![no_std]
#![allow(unexpected_cfgs)]

pub use ch32_metapac as pac;

mod generated {
    include!(concat!(env!("OUT_DIR"), "/generated.rs"));
}
pub use generated::Pin;
