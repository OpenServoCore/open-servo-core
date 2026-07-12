//! Hardware-in-the-loop bench suite: drives a flashed V006 over the uart-pirate,
//! exercising the osc-native happy path (`docs/osc-native-protocol.md` §5) plus
//! the turnaround metric. One file per instruction concern; [`support`] owns the
//! shared pirate client, env config, and serialisation.
//!
//! Run with the bench attached, e.g.:
//!
//! ```text
//! BENCH_PORT=/dev/tty.usbmodemXXXX cargo test --test hardware -- --test-threads=1
//! ```
//!
//! `#[serial]` already serialises access to the one pirate; `--test-threads=1`
//! is belt-and-suspenders. Config: `BENCH_PORT` (empty ⇒ autodetect),
//! `BENCH_BAUD` (default boot baud), `BENCH_ID` (default 1). No gating — with no
//! bench attached the shared client panics on first use, by design.
//!
//! Out of scope this pass: FAST/group-chain timing (untuned). Single-servo
//! happy path + turnaround (ping/read/write) + ENUM/ASSIGN + SAVE/FACTORY +
//! the rescue break.

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
