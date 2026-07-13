//! Logging facade -- forwards to `defmt`/`log`/no-op via the shared `osc-log`
//! crate; re-exported here so call sites keep using `crate::log::*`.

pub use osc_log::{debug, error, info, trace, warn};
