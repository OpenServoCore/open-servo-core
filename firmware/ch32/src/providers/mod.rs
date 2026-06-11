//! Provider layer — implements driver-owned interfaces (`osc_drivers::traits`)
//! over HAL primitives. Drivers depend on providers only via the type
//! parameter; this folder is the only place that talks to both layers.

pub mod clock_trim;
pub mod digital_out;
pub mod dma_ring;
pub mod monotonic;
pub mod usart_baud;
