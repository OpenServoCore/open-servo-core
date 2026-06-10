//! DXL 2.0 transport: snoop ring + chain CRC + fire scheduler.
//!
//! Layout:
//! - [`statics`]: ring/TX buffers, baud-derived atomics, pending-write slots.
//! - [`calibration`]: hand-tuned per-path ISR entry tick counts.
//! - [`crc`]: chip-owned [`dxl_protocol::CrcUmts`] impl (software on CH32V006;
//!   would be the CRC peripheral on a chip that has one).
//! - [`state`]: FSM types, `STATE`/`DISPATCH`, phase guards, fault counters.
//! - [`scheduler`]: `start_plain_after`/`start_fast_after` arm path plus
//!   `predict_n_pred`, advance-tick math, and the per-baud catchup interval.
//! - [`isr`]: SysTick body, `arm_tx`/`fire_now`/`patch_crc`,
//!   the snoop accumulator, and `ring_crc`.

pub(crate) mod cal;
pub(crate) mod calibration;
pub(crate) mod crc;
pub(crate) mod isr;
pub(crate) mod scheduler;
pub(crate) mod state;
pub(crate) mod statics;
pub(crate) mod timing;
pub(crate) mod tx_activity;

pub use crc::Ch32DxlCrc;
pub use isr::on_systick;
pub use scheduler::{start_fast_after, start_plain_after};
pub use state::{
    cancel, report_dma_overrun, report_framing_error, report_noise_error, report_parity_error,
};
