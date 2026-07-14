//! Provider layer -- implements the engine-owned `osc_host::traits`
//! interfaces over HAL primitives, plus the two system providers (clocks,
//! pins) called once from bringup.

pub mod clocks;
pub mod deadline;
pub mod pins;
pub mod ring;
pub mod tx_wire;
pub mod usart_baud;
