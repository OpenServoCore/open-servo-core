//! Servo identification, sans io: what a run measured, the servo's
//! nameplate, the board's description and a policy, turned into the
//! servo's constant set by rules. Pure, so the same code runs in the CLI
//! and, built for wasm, in the browser.

pub mod board;
pub mod inputs;
pub mod nameplate;
pub mod policy;
pub mod rules;

pub use board::Board;
pub use inputs::{Inputs, Measured, Quantity, Source, ThermalInputs, Tuning};
pub use nameplate::Nameplate;
pub use policy::Policy;
pub use rules::{Class, Entry, Missing, Outcome, Refusal, Rule, Set, build};
