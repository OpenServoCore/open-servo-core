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

use open_servo_hw::v2::{SignalReader, SignalWriter};

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
///     type PersistSignal = Signal<CriticalSectionRawMutex, ()>;
///     type PersistSignalTx = Signal<CriticalSectionRawMutex, ()>;
///
///     fn persist_signal(&self) -> (&Self::PersistSignalTx, &Self::PersistSignal) {
///         static S: Signal<CriticalSectionRawMutex, ()> = Signal::new();
///         (&S, &S)
///     }
/// }
/// ```
pub trait ServicePrimitives {
    /// Signal reader type for persist notifications.
    type PersistSignal: SignalReader;

    /// Signal writer type for persist notifications (may be same as reader).
    type PersistSignalTx: SignalWriter;

    /// Get the persist signal (tx, rx).
    ///
    /// Called to wire up:
    /// - TX side: shadow storage callback on EEPROM write
    /// - RX side: persist service waiting for work
    fn persist_signal(&self) -> (&Self::PersistSignalTx, &Self::PersistSignal);

    // Future: add more signals/channels as services need them
    // fn rpc_tick_signal(&self) -> ...;
    // fn dxl_op_channel(&self) -> ...;
}
