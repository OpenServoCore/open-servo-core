//! Provider layer — implements consumer-owned interfaces (`osc_drivers::traits`
//! for the bus composite, `osc_core` ports like `ConfigStore`) over HAL
//! primitives. Consumers depend on providers only via their trait surface;
//! this folder is the only place that talks to both layers.

pub mod config_store;
pub mod crc;
pub mod deadline;
pub mod digital_out;
pub mod monotonic;
pub mod ring;
pub mod tx_wire;
pub mod usart_baud;
