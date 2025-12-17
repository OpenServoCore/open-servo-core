//! Lightweight logging for embedded systems.
//!
//! A defmt-like logging API that outputs to the REPL using ufmt.
//!
//! ## Usage
//!
//! ```rust,ignore
//! use open_servo_log::{info, warn, error};
//!
//! info!("System initialized");
//! warn!("Queue full");
//! error!("DMA failed: {}", code);
//! ```
//!
//! ## Log Levels
//!
//! Controlled via feature flags (compile-time filtering):
//! - `log-error` - errors only
//! - `log-warn` - errors + warnings
//! - `log-info` - errors + warnings + info (default)
//! - `log-debug` - all except trace
//! - `log-trace` - everything

#![no_std]

use core::cell::RefCell;
use core::ptr::NonNull;
use critical_section::Mutex;
use open_servo_hw::DebugIo;

// Re-export for use in macros
#[doc(hidden)]
pub use heapless;
#[doc(hidden)]
pub use ufmt;

/// Log level.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Level {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
    Trace = 4,
}

impl Level {
    /// Get the prefix string for this log level.
    pub const fn prefix(self) -> &'static str {
        match self {
            Level::Error => "[ERROR] ",
            Level::Warn => "[WARN]  ",
            Level::Info => "[INFO]  ",
            Level::Debug => "[DEBUG] ",
            Level::Trace => "[TRACE] ",
        }
    }
}

/// Type-erased writer function pointer.
/// Takes a pointer to the DebugIo instance and the data to write.
type WriteFn = fn(NonNull<()>, &[u8]);

/// Wrapper around NonNull that implements Send.
/// Safety: We only access this within critical sections.
struct SendPtr(Option<NonNull<()>>);

// Safety: Access is protected by critical section
unsafe impl Send for SendPtr {}

/// Global logger state.
struct Logger {
    io_ptr: SendPtr,
    write_fn: Option<WriteFn>,
}

impl Logger {
    const fn new() -> Self {
        Self {
            io_ptr: SendPtr(None),
            write_fn: None,
        }
    }
}

static LOGGER: Mutex<RefCell<Logger>> = Mutex::new(RefCell::new(Logger::new()));

/// Initialize the logger with a DebugIo backend.
///
/// # Safety
///
/// The provided reference must remain valid for the lifetime of the program.
/// Typically this means using a static.
///
/// # Example
///
/// ```rust,ignore
/// static mut RTT_IO: Option<RttDebugIo> = None;
/// unsafe {
///     RTT_IO = Some(RttDebugIo::init());
///     open_servo_log::init(RTT_IO.as_mut().unwrap());
/// }
/// ```
pub fn init<D: DebugIo>(io: &'static mut D) {
    // Create a write function that casts the pointer back to the concrete type
    fn write_impl<D: DebugIo>(ptr: NonNull<()>, data: &[u8]) {
        // Safety: We know the pointer is valid and points to a D
        let io = unsafe { &mut *(ptr.as_ptr() as *mut D) };
        io.write(data);
    }

    let ptr = NonNull::new(io as *mut D as *mut ()).unwrap();

    critical_section::with(|cs| {
        let mut logger = LOGGER.borrow_ref_mut(cs);
        logger.io_ptr = SendPtr(Some(ptr));
        logger.write_fn = Some(write_impl::<D>);
    });
}

/// Write bytes to the logger output.
///
/// This is used internally by the logging macros.
#[doc(hidden)]
pub fn write_bytes(data: &[u8]) {
    critical_section::with(|cs| {
        let logger = LOGGER.borrow_ref(cs);
        if let (Some(ptr), Some(write_fn)) = (logger.io_ptr.0, logger.write_fn) {
            write_fn(ptr, data);
        }
    });
}

/// Log implementation macro. Not for direct use.
///
/// Note: Requires `ufmt` crate to be available in the calling crate.
#[macro_export]
#[doc(hidden)]
macro_rules! __log_impl {
    ($level:expr, $($arg:tt)*) => {{
        // Write level prefix
        $crate::write_bytes($level.prefix().as_bytes());

        // Format message to buffer and write
        // Note: ufmt must be available in calling crate
        let mut buf: $crate::heapless::String<128> = $crate::heapless::String::new();
        let _ = ufmt::uwrite!(buf, $($arg)*);
        $crate::write_bytes(buf.as_bytes());

        // Write newline
        $crate::write_bytes(b"\r\n");
    }};
}

// Compile-time log level based on features
#[cfg(feature = "log-trace")]
const MAX_LEVEL: Level = Level::Trace;
#[cfg(all(feature = "log-debug", not(feature = "log-trace")))]
const MAX_LEVEL: Level = Level::Debug;
#[cfg(all(feature = "log-info", not(feature = "log-debug")))]
const MAX_LEVEL: Level = Level::Info;
#[cfg(all(feature = "log-warn", not(feature = "log-info")))]
const MAX_LEVEL: Level = Level::Warn;
#[cfg(all(feature = "log-error", not(feature = "log-warn")))]
const MAX_LEVEL: Level = Level::Error;
#[cfg(not(feature = "log-error"))]
const MAX_LEVEL: Level = Level::Error; // Fallback, but errors disabled

/// Check if a log level is enabled (compile-time).
#[inline(always)]
pub const fn level_enabled(level: Level) -> bool {
    #[cfg(not(feature = "log-error"))]
    return false;
    #[cfg(feature = "log-error")]
    return (level as u8) <= (MAX_LEVEL as u8);
}

/// Log an error message.
#[macro_export]
macro_rules! error {
    ($($arg:tt)*) => {{
        if $crate::level_enabled($crate::Level::Error) {
            $crate::__log_impl!($crate::Level::Error, $($arg)*);
        }
    }};
}

/// Log a warning message.
#[macro_export]
macro_rules! warn {
    ($($arg:tt)*) => {{
        if $crate::level_enabled($crate::Level::Warn) {
            $crate::__log_impl!($crate::Level::Warn, $($arg)*);
        }
    }};
}

/// Log an info message.
#[macro_export]
macro_rules! info {
    ($($arg:tt)*) => {{
        if $crate::level_enabled($crate::Level::Info) {
            $crate::__log_impl!($crate::Level::Info, $($arg)*);
        }
    }};
}

/// Log a debug message.
#[macro_export]
macro_rules! debug {
    ($($arg:tt)*) => {{
        if $crate::level_enabled($crate::Level::Debug) {
            $crate::__log_impl!($crate::Level::Debug, $($arg)*);
        }
    }};
}

/// Log a trace message.
#[macro_export]
macro_rules! trace {
    ($($arg:tt)*) => {{
        if $crate::level_enabled($crate::Level::Trace) {
            $crate::__log_impl!($crate::Level::Trace, $($arg)*);
        }
    }};
}
