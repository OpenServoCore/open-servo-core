//! Logging facade -- forwards to `defmt` on embedded, to `log` on host, else a
//! no-op. Shared by the lib crates (`osc-servo-core`, `osc-servo-drivers`), each of which
//! re-exports it as its own `log` module so call sites read `crate::log::*`.
//!
//! NOTE: `defmt`'s macros expand to absolute `::defmt::...` paths, so any crate
//! that *invokes* a logging macro under the `defmt` feature must also carry its
//! own `defmt` dependency -- re-exporting from here is not enough. The `log` and
//! no-op backends are hygienic and need nothing. Crates that only forward the
//! facade (never invoke) can drop the dep entirely.

#![no_std]

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
    macro_rules! __osc_log_noop {
        ($($arg:tt)*) => {{
            let _ = ($($arg)*);
        }};
    }

    pub use crate::__osc_log_noop as debug;
    pub use crate::__osc_log_noop as error;
    pub use crate::__osc_log_noop as info;
    pub use crate::__osc_log_noop as trace;
    pub use crate::__osc_log_noop as warn;
}
