//! The policy's choices as the rules read them: the inputs crate's, where
//! a tuning policy resolves to them, and the forms six of them take.

pub use osc_ident_inputs::policy::*;
pub use osc_ident_inputs::tuning::{
    CurrentLoop, Observer, PotJump, StallRelease, StallSpeed, Yield,
};
