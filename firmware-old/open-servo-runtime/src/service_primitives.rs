//! Service primitives trait for async service orchestration.
//!
//! This trait defines what async primitives the runtime needs for services.
//! Firmware implements this trait to provide the concrete signal/channel types.
//!
//! # Design
//!
//! - Runtime defines WHAT primitives it needs (this trait)
//! - Firmware provides HOW (impl with static storage)
//! - Services use trait bounds from `open_servo_hw::v2::async_primitives`

use open_servo_hw::v2::{PersistR, PersistW, ResetR, ResetW};

pub use open_servo_hw::v2::ResetLevel;

/// Defines the async primitives needed by the runtime's service orchestration.
///
/// Firmware implements this trait to provide concrete signal types
/// backed by static storage (e.g., `embassy_sync::Signal`).
///
/// # Example
///
/// ```rust,ignore
/// pub struct Stm32Primitives;
///
/// impl ServicePrimitives for Stm32Primitives {
///     type PersistRx = EmbassySignal;
///     type PersistTx = EmbassySignal;
///     type ResetRx = ResetSignalWrapper;
///     type ResetTx = ResetSignalWrapper;
///
///     fn persist_signal(&self) -> (&Self::PersistTx, &Self::PersistRx) { ... }
///     fn reset_signal(&self) -> (&Self::ResetTx, &Self::ResetRx) { ... }
/// }
/// ```
pub trait ServicePrimitives {
    /// Persist signal reader (async side).
    type PersistRx: PersistR;
    /// Persist signal writer (sync side).
    type PersistTx: PersistW;
    /// Factory reset signal reader (async side).
    type ResetRx: ResetR;
    /// Factory reset signal writer (sync side).
    type ResetTx: ResetW;

    /// Get persist signal (tx, rx).
    fn persist_signal(&self) -> (&Self::PersistTx, &Self::PersistRx);

    /// Get factory reset signal (tx, rx).
    fn reset_signal(&self) -> (&Self::ResetTx, &Self::ResetRx);
}
