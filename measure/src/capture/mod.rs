//! The capture layer: TEL rows and the shunt burst, decoded pure, armed
//! and read over the client, and kept through a [`crate::sink::Sink`] as
//! the notebooks' own files.

pub mod burst;
pub mod meta;
pub mod ripple;
pub mod rows;
pub mod tel;
