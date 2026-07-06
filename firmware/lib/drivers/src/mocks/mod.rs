//! Test doubles for the driver-owned interfaces. Most are mockall-generated
//! (one per-role child module per trait file, mirroring `traits/`, each with
//! one `mock!` block and consts set inside the impl block per mockall syntax).
//! The `bus` composite's providers are hand-rolled instead — their stateful
//! cursor and borrow-returning `bytes()` fight mockall's lifetime model.
//!
//! `std` is pulled into the crate root behind the same cfg gate that
//! guards this module (see `lib.rs`), so submodules can use `std`-backed
//! types without making the whole crate `std`-bound.

pub mod bus;
pub mod digital_out;
pub mod monotonic;

pub use digital_out::MockDigitalOut;
pub use monotonic::MockMonotonic;
