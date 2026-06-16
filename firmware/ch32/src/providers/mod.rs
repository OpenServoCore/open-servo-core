//! Provider layer — implements driver-owned interfaces (`osc_drivers::traits`)
//! over HAL primitives. Drivers depend on providers only via the type
//! parameter; this folder is the only place that talks to both layers.

pub mod clock_trim;
pub mod digital_out;
pub mod dxl_crc;
pub mod dxl_tx_scheduler;
pub mod edge_dma;
pub mod fast_last_scheduler;
pub mod monotonic;
pub mod rx_dma;
pub mod usart_baud;
