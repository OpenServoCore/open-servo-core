//! Logging facade — forwards to `defmt` on embedded, to `log` on host, else no-op.

#[cfg(feature = "defmt")]
pub use defmt::{debug, error, info, trace, warn};

#[cfg(all(feature = "log", not(feature = "defmt")))]
pub use log::{debug, error, info, trace, warn};

#[cfg(not(any(feature = "defmt", feature = "log")))]
pub use noop::*;

#[cfg(not(any(feature = "defmt", feature = "log")))]
mod noop {
    // Tuple binding keeps referenced vars "used" so call sites stay warning-clean.
    #[doc(hidden)]
    #[macro_export]
    macro_rules! __osc_drivers_noop_log {
        ($($arg:tt)*) => {{
            let _ = ($($arg)*);
        }};
    }

    pub use crate::__osc_drivers_noop_log as debug;
    pub use crate::__osc_drivers_noop_log as error;
    pub use crate::__osc_drivers_noop_log as info;
    pub use crate::__osc_drivers_noop_log as trace;
    pub use crate::__osc_drivers_noop_log as warn;
}
