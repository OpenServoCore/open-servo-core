//! Compile-time application-level caps for the DXL service layer.
//!
//! The two primary knobs are tunable per build via env vars so chips/boards
//! can shrink or stretch their memory footprint:
//!
//! - `OSC_MAX_SLAVE_COUNT` — fan-out cap for coordinated reads/writes.
//!   Defaults to 16.
//! - `OSC_MAX_CONTROL_RW` — per-op control-table byte range. Defaults to 128.
//!
//! Set them in `.cargo/config.toml`:
//! ```toml
//! [env]
//! OSC_MAX_SLAVE_COUNT = "16"
//! OSC_MAX_CONTROL_RW  = "128"
//! ```
//!
//! All derived constants (dispatcher input caps, stitch buffer, TX buffer)
//! flow from these two plus the protocol-level constants exported by
//! `dxl_protocol`.

use control_table::STAGE_DATA_CAP;
use dxl_protocol::wire::{CRC_BYTES, FAST_RESPONSE_SLOT0_BYTES, RESPONSE_HEADER_BYTES};

/// Worst-case slave count we participate in for Sync/Bulk/Fast coordinated
/// operations. Override with `OSC_MAX_SLAVE_COUNT`.
pub const MAX_SLAVE_COUNT: usize = env_usize(option_env!("OSC_MAX_SLAVE_COUNT"), 16);

/// Largest control-table byte range a single Read or Write can address.
/// Override with `OSC_MAX_CONTROL_RW`.
pub const MAX_CONTROL_RW: usize = env_usize(option_env!("OSC_MAX_CONTROL_RW"), 128);

/// Largest outbound reply we'll buffer. Status reply and Fast Only reply
/// both end with a CRC; Fast Only is one byte longer because it carries a
/// slave-id ahead of the data on top of `RESPONSE_HEADER_BYTES`.
pub const DXL_TX_MAX_BYTES: usize = const_max(
    RESPONSE_HEADER_BYTES + MAX_CONTROL_RW + CRC_BYTES,
    FAST_RESPONSE_SLOT0_BYTES + MAX_CONTROL_RW + CRC_BYTES,
);

// A staged WRITE must fit one max-length DXL Write data payload.
const _: () = assert!(
    STAGE_DATA_CAP >= MAX_CONTROL_RW,
    "STAGE_DATA_CAP must be >= MAX_CONTROL_RW; widen control-table or shrink OSC_MAX_CONTROL_RW",
);

const fn const_max(a: usize, b: usize) -> usize {
    if a > b { a } else { b }
}

const fn env_usize(value: Option<&'static str>, default: usize) -> usize {
    match value {
        Some(s) => parse_usize(s),
        None => default,
    }
}

const fn parse_usize(s: &str) -> usize {
    let bytes = s.as_bytes();
    let mut n: usize = 0;
    let mut i = 0;
    while i < bytes.len() {
        let b = bytes[i];
        assert!(
            b >= b'0' && b <= b'9',
            "OSC_MAX_* must be a decimal integer"
        );
        n = n * 10 + (b - b'0') as usize;
        i += 1;
    }
    n
}
