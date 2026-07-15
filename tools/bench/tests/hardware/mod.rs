//! Hardware-in-the-loop bench suite: drives a flashed V006 through the
//! osc-adapter, exercising the osc-native happy path (`docs/osc-native-protocol.md`
//! sec 5) plus the turnaround metric. One file per instruction concern;
//! [`support`] owns the shared adapter wire, env config, and serialisation.
//!
//! Run with the bench attached, e.g.:
//!
//! ```text
//! cargo test --test hardware -- --test-threads=1
//! ```
//!
//! `#[serial]` already serialises access to the one adapter; `--test-threads=1`
//! is belt-and-suspenders. Config: `BENCH_BAUD` (default boot baud), `BENCH_ID`
//! (default 1). No gating -- with no bench attached the shared wire panics on
//! first use, by design.

mod support;

mod chain;
mod hold_commit;
mod hot_loop;
mod mgmt;
mod ping;
mod profile;
mod read;
mod rescue;
mod silence;
mod trim;
mod turnaround;
mod write;
