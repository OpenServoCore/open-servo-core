//! Adapter layer — implements driver-owned interfaces (`drivers::traits`)
//! over HAL primitives for production, and recording mocks for unit tests.
//! Drivers depend on adapters only via the type parameter; this folder is
//! the only place that talks to both layers.

pub mod dma;
pub mod gpio;
pub mod rcc;
pub mod systick;
pub mod usart;

#[cfg(test)]
pub mod mocks;
