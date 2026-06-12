//! DXL 2.0 transport: snoop ring + chain CRC + fire scheduler.
//!
//! Layout:
//! - [`statics`]: ring/TX buffers, baud-derived atomics, pending-write slots.
//! - [`calibration`]: hand-tuned per-path ISR entry tick counts.
//! - [`state`]: FSM types, `STATE`/`DISPATCH`, phase guards, fault counters.
//! - [`scheduler`]: `start_plain_after`/`start_fast_after` arm path plus
//!   `predict_n_pred`, advance-tick math, and the per-baud catchup interval.
//! - [`isr`]: SysTick body, `arm_tx`/`fire_now`/`patch_crc`,
//!   the snoop accumulator, and `ring_crc`.
//!
//! The CRC engine itself moved to `crate::providers::dxl_crc::DxlCrc` —
//! it's a role-shaped provider per driver-pattern §5.3, not a legacy
//! transport concern.

pub(crate) mod calibration;
pub(crate) mod isr;
pub(crate) mod scheduler;
pub(crate) mod state;
pub(crate) mod statics;
pub(crate) mod timing;
pub(crate) mod tx_activity;

pub use isr::on_systick;
pub use state::{
    cancel, report_dma_overrun, report_framing_error, report_noise_error, report_parity_error,
};
