//! Servo identification, sans io: what a run measured, the servo's
//! nameplate, the board's description and a policy, turned into the
//! servo's constant set by rules; the set file, and the path that installs
//! it on a servo safely. Pure but for the bus, which is the client's, and
//! files, which are a [`sink::Sink`]'s, so the same code runs in the CLI
//! and, built for wasm, in the browser.

pub mod board;
pub mod capture;
pub mod envelope;
pub mod fit;
pub mod guard;
pub mod inputs;
pub mod install;
pub mod nameplate;
pub mod ntc;
pub mod policy;
pub mod pyjson;
pub mod rules;
pub mod set;
pub mod sink;
pub mod state;
pub mod steps;

pub use board::Board;
pub use inputs::{Inputs, Measured, Quantity, Source, ThermalInputs, Tuning};
pub use nameplate::Nameplate;
pub use policy::Policy;
pub use rules::{Class, Entry, Missing, Outcome, Refusal, Rule, Set, build};
