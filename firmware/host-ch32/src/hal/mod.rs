//! V305 peripheral primitives (register dance only -- no policy, no driver
//! state). Single chip variant, so files stay flat.

pub mod dma;
pub mod flash;
pub mod gpio;
pub mod pfic;
pub mod rcc;
pub mod systick;
pub mod usart;
pub mod usbhs;
