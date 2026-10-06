//! Sans-io servo identification. Experiments emit wire commands and consume
//! telemetry; fits and gain synthesis are pure functions over the collected
//! samples. No transport, no clock, no filesystem - the CLI wrapper pumps a
//! USB pipe and the GUI pumps Web Serial into the same code, each stamping
//! every read with its own clock, so the crate compiles to
//! wasm32-unknown-unknown unchanged (CI gates this).
//!
//! Layout facts (register addresses, TEL frame shape) are mirrored from the
//! firmware, not imported: the descriptor cross-check test in [`regs`] and
//! the golden vectors in [`frame`] pin the mirror to the published ABI.

pub mod burst;
pub mod exp;
pub mod fitmath;
pub mod fits;
pub mod frame;
pub mod gains;
pub mod kinematics;
pub mod limits;
pub mod lut;
pub mod pot;
pub mod regs;
pub mod report;
pub mod ripple;
pub mod run;
pub mod runway;
pub mod slip;
pub mod sources;
pub mod thermometer;
pub mod units;
