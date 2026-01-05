//! Service orchestration bundle.
//!
//! The `Services` struct bundles all async services and provides a single
//! `run_once()` method that firmware wraps in a loop.
//!
//! # Architecture
//!
//! ```text
//! firmware                → loop { services.run_once().await }
//! runtime (Services)      → select! over signals, dispatch to services
//! services crate          → individual async handlers
//! ```
//!
//! # Design
//!
//! - Services does NOT loop forever - firmware controls the loop
//! - Services does NOT know about embassy - uses trait bounds
//! - Firmware provides `impl ServicePrimitives` with static signal storage

use embedded_storage_async::nor_flash::NorFlash;
use open_servo_hw::v2::SignalReader;

use crate::service_primitives::ServicePrimitives;
use crate::shadow_storage::ShadowStorage;

/// Error type for service operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ServiceError {
    /// Persist operation failed.
    Persist,
    // Future: add more error variants as services are added
}

/// Service orchestration bundle.
///
/// Owns service instances and provides `run_once()` for firmware to call in a loop.
///
/// # Type Parameters
///
/// - `P`: ServicePrimitives impl (provides signals)
/// - `F`: Flash type for persist service
/// - `N`: Shadow table size
pub struct Services<'a, P, F, const N: usize>
where
    P: ServicePrimitives,
    F: NorFlash,
{
    /// Async primitives provider (signals, channels).
    primitives: &'a P,

    /// Flash storage for EEPROM persistence.
    #[allow(dead_code)]
    flash: F,

    /// Shadow table reference.
    #[allow(dead_code)]
    shadow: &'a ShadowStorage<N>,
}

impl<'a, P, F, const N: usize> Services<'a, P, F, N>
where
    P: ServicePrimitives,
    F: NorFlash,
{
    /// Create a new service bundle.
    pub fn new(primitives: &'a P, flash: F, shadow: &'a ShadowStorage<N>) -> Self {
        Self {
            primitives,
            flash,
            shadow,
        }
    }

    /// Run one service cycle.
    ///
    /// Waits for a signal, handles it, then returns. Firmware wraps this in a loop:
    ///
    /// ```rust,ignore
    /// loop {
    ///     if let Err(e) = services.run_once().await {
    ///         defmt::error!("Service error: {:?}", e);
    ///     }
    /// }
    /// ```
    ///
    /// # Returns
    ///
    /// - `Ok(())` after handling one event
    /// - `Err(ServiceError)` if a service operation failed
    pub async fn run_once(&mut self) -> Result<(), ServiceError> {
        // Get signal reader
        let (_, persist_rx) = self.primitives.persist_signal();

        // For now, just wait for persist signal.
        // Future: use futures::select! to multiplex multiple services.
        persist_rx.wait().await;

        // TODO: Call actual persist service when migrated
        // self.persist.handle(self.shadow).await.map_err(|_| ServiceError::Persist)?;

        Ok(())
    }
}
