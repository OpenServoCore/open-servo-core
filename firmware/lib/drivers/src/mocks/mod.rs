//! mockall-generated mocks for the driver-owned interfaces. One per-role
//! child module per trait file (mirroring `traits/`), each declaring one
//! `mock!` block; associated constants are set inside the impl block per
//! mockall's syntax for traits with consts.
//!
//! `std` is pulled into the crate root behind the same cfg gate that
//! guards this module (see `lib.rs`), so submodules can use `std`-backed
//! types without making the whole crate `std`-bound.

pub mod digital_out;
pub mod monotonic;

pub use digital_out::MockDigitalOut;
pub use monotonic::MockMonotonic;
