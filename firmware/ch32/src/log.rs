//! Logging facade — mirrors `osc_core::log`. Forwards to `defmt` or no-ops.

#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, trace, warn};

#[cfg(not(feature = "defmt"))]
pub use noop::*;

#[cfg(not(feature = "defmt"))]
mod noop {
    #[doc(hidden)]
    #[macro_export]
    macro_rules! __osc_ch32_noop_log {
        ($($arg:tt)*) => {{
            let _ = ($($arg)*);
        }};
    }

    pub use crate::__osc_ch32_noop_log as debug;
    pub use crate::__osc_ch32_noop_log as error;
    pub use crate::__osc_ch32_noop_log as info;
    pub use crate::__osc_ch32_noop_log as trace;
    pub use crate::__osc_ch32_noop_log as warn;
}
