//! Logging facade -- forwards to `defmt` or no-op via the shared `osc-log`
//! crate; re-exported here so call sites keep using `crate::log::*`. ch32 is
//! embedded-only (no host `log` arm): without `defmt` it no-ops. ch32 *invokes*
//! the macros, so it keeps its own `defmt` dep and forwards `osc-log/defmt`
//! (see osc-log).

pub use osc_log::{debug, error, info, trace, warn};
