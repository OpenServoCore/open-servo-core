//! DXL 2.0 instruction happy-path suite, hardware-driven.
//!
//! Structure mirrors `firmware/lib/integration/tests/protocol/` — one file
//! per instruction. See [`support`] for the shared `Bus`, env config, and
//! test-serialisation discipline.
//!
//! **Not tested here.** Reboot lives in `tests/tinyboot.rs` (its own
//! process). Error-return paths (access, data-range, instruction) belong to
//! a future negative-space suite. Coalesce-timing sensitivity lives with
//! the tune tools, not here.
//!
//! **Expected fragility.** The Fast Sync / Bulk per-position tests can
//! fail at 2M / 3M baud until the chip is tuned and the hardware-fired
//! TX-start path lands (task #134). Software fire injects wire gaps and
//! bus contention that put slot-end / slot-start close enough together
//! that the parser sometimes rejects the frame. Rerun at 1M baud, or
//! wait for #134, to distinguish real regressions from known timing
//! floor limitations.

#[path = "../support.rs"]
mod support;

mod bulk_read;
mod fast_bulk_read;
mod fast_injected;
mod fast_sync_read;
mod ping;
mod read;
mod reg_write_action;
mod silence;
mod sync_read;
mod write;
