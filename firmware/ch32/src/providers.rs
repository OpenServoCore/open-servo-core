//! Provider layer — implements driver-owned interfaces (`osc_drivers::traits`)
//! over HAL primitives. Drivers depend on providers only via the type
//! parameter; this folder is the only place that talks to both layers.

pub mod dma;
pub mod gpio;
pub mod rcc;
pub mod systick;
pub mod usart;
