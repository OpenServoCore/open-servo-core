//! Logging facade — forwards to `defmt` when feature is on, else no-op.

#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, trace, warn};

#[cfg(not(feature = "defmt"))]
pub use noop::*;

#[cfg(not(feature = "defmt"))]
mod noop {
    // Tuple binding keeps referenced vars "used" so call sites stay warning-clean.
    #[doc(hidden)]
    #[macro_export]
    macro_rules! __osc_core_noop_log {
        ($($arg:tt)*) => {{
            let _ = ($($arg)*);
        }};
    }

    pub use crate::__osc_core_noop_log as debug;
    pub use crate::__osc_core_noop_log as error;
    pub use crate::__osc_core_noop_log as info;
    pub use crate::__osc_core_noop_log as trace;
    pub use crate::__osc_core_noop_log as warn;
}
