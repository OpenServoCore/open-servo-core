//! Provider layer — implements driver-owned interfaces (`osc_drivers::traits`)
//! over HAL primitives. Drivers depend on providers only via the type
//! parameter; this folder is the only place that talks to both layers.

pub mod crc;
pub mod deadline;
pub mod digital_out;
pub mod line;
pub mod monotonic;
pub mod ring;
pub mod tx_wire;
pub mod usart_baud;
pub mod wake;
