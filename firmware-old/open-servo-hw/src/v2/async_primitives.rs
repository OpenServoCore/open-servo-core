//! Async primitive traits for executor-agnostic services.
//!
//! These traits abstract over runtime-specific primitives (`embassy_sync`, `rtic_sync`, etc.),
//! allowing the `services` and `hw-utils` crates to remain portable.
//!
//! ## Implementation Pattern
//!
//! Firmware provides concrete implementations via newtype wrappers:
//!
//! ```rust,ignore
//! // Simple notification signal (no payload)
//! struct EmbassySignal(&'static Signal<CriticalSectionRawMutex, ()>);
//!
//! impl SignalReader for EmbassySignal {
//!     async fn wait(&self) { self.0.wait().await; }
//! }
//!
//! impl SignalWriter for EmbassySignal {
//!     fn signal(&self, _: ()) { self.0.signal(()); }
//! }
//!
//! // Value-carrying signal
//! struct ResetSignal(&'static Signal<CriticalSectionRawMutex, ResetLevel>);
//!
//! impl SignalReader<ResetLevel> for ResetSignal {
//!     async fn wait(&self) -> ResetLevel { self.0.wait().await }
//! }
//!
//! impl SignalWriter<ResetLevel> for ResetSignal {
//!     fn signal(&self, level: ResetLevel) { self.0.signal(level); }
//! }
//! ```
//!
//! ## Trait Summary
//!
//! | Trait | Side | Purpose |
//! |-------|------|---------|
//! | [`SignalWriter<T>`] | Sync (ISR) | Notify async tasks with value |
//! | [`SignalReader<T>`] | Async | Wait for notifications, receive value |
//! | [`AsyncTimer`] | Async | Monotonic time and delays |

use core::future::Future;

use open_servo_units::{MicroSecond, TimeStampUs};

/// Factory reset level (Dynamixel Protocol 2.0 compatible).
///
/// Determines which EEPROM fields to preserve during factory reset.
/// Used by persist service to selectively delete keys from flash.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum ResetLevel {
    /// Reset all EEPROM fields to factory defaults (0xFF).
    All = 0xFF,
    /// Reset all except ID (0x01).
    ExceptId = 0x01,
    /// Reset all except ID and baud rate (0x02).
    ExceptIdBaud = 0x02,
}

impl ResetLevel {
    /// Parse from DXL protocol value.
    ///
    /// Returns `None` for invalid values.
    pub fn from_dxl(value: u8) -> Option<Self> {
        match value {
            0xFF => Some(ResetLevel::All),
            0x01 => Some(ResetLevel::ExceptId),
            0x02 => Some(ResetLevel::ExceptIdBaud),
            _ => None,
        }
    }
}

/// Async signal writer (sync side, callable from ISR).
///
/// Used to notify async tasks from interrupt context.
/// Generic over payload type `T` (defaults to `()` for simple notifications).
pub trait SignalWriter<T = ()> {
    /// Signal the reader with a value. Non-blocking, ISR-safe.
    fn signal(&self, value: T);
}

/// Async signal reader (async side).
///
/// Used to wait for signals from ISR or other sync contexts.
/// Generic over payload type `T` (defaults to `()` for simple notifications).
pub trait SignalReader<T = ()> {
    /// Wait for a signal. Returns the signaled value when woken.
    fn wait(&self) -> impl Future<Output = T>;
}

/// Async timer for delays and timeouts.
///
/// Provides executor-agnostic timing using our own `MicroSecond` and `TimeStampUs` types.
/// Firmware implements this by wrapping `embassy_time::Timer` or similar.
pub trait AsyncTimer {
    /// Get current monotonic time.
    fn now(&self) -> TimeStampUs;

    /// Delay for the specified duration.
    fn delay(&self, duration: MicroSecond) -> impl Future<Output = ()>;
}

// =============================================================================
// Semantic Signal Trait Aliases
// =============================================================================
//
// Short trait names for signal roles. Blanket impls auto-implement.
//
// | Trait | Underlying | Usage |
// |-------|------------|-------|
// | PersistR | SignalReader<()> | Persist service waits for save requests |
// | PersistW | SignalWriter<()> | Shadow storage signals on EEPROM write |
// | ResetR | SignalReader<ResetLevel> | Persist service waits for reset commands |
// | ResetW | SignalWriter<ResetLevel> | RPC/DXL handler triggers factory reset |

/// Persist signal reader. Persist service waits on this for save requests.
pub trait PersistR: SignalReader<()> {}
impl<T: SignalReader<()>> PersistR for T {}

/// Persist signal writer. Shadow storage signals this on EEPROM write.
pub trait PersistW: SignalWriter<()> {}
impl<T: SignalWriter<()>> PersistW for T {}

/// Factory reset signal reader. Persist service waits on this for reset commands.
pub trait ResetR: SignalReader<ResetLevel> {}
impl<T: SignalReader<ResetLevel>> ResetR for T {}

/// Factory reset signal writer. RPC/DXL handlers signal this to trigger reset.
pub trait ResetW: SignalWriter<ResetLevel> {}
impl<T: SignalWriter<ResetLevel>> ResetW for T {}
