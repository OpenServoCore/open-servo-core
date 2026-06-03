//! DXL 2.0 transport: snoop ring + chain CRC + fire scheduler.
//!
//! Layout:
//! - [`statics`]: ring/TX buffers, baud-derived atomics, pending-write slots.
//! - [`calibration`]: hand-tuned per-path ISR entry tick counts.
//! - [`state`]: FSM types, `STATE`/`DISPATCH`, phase guards, fault counters.
//! - [`scheduler`]: `start_plain_after`/`start_fast_after` arm path plus
//!   `predict_n_pred`, advance-tick math, and the per-baud catchup interval.
//! - [`isr`]: SysTick + USART1 RXNE bodies, `arm_tx`/`fire_now`/`patch_crc`,
//!   the snoop accumulator, and `ring_crc`.

pub(crate) mod calibration;
pub(crate) mod isr;
pub(crate) mod scheduler;
pub(crate) mod state;
pub(crate) mod statics;

pub use isr::{arm_tx, fire_now, on_rxne, on_systick};
pub use scheduler::{start_fast_after, start_plain_after};
pub use state::{
    cancel, report_dma_overrun, report_framing_error, report_noise_error, report_parity_error,
};
