//! Service orchestration bundle.
//!
//! The [`Services`] struct bundles async services and provides `run_once()`
//! for firmware to call in a loop.
//!
//! ## Architecture
//!
//! ```text
//! firmware    → loop { services.run_once().await }
//! Services    → select! over signals, dispatch to handlers
//! handlers    → persist, rpc, dxl (individual async logic)
//! ```

use core::ops::Range;

use embedded_storage_async::nor_flash::NorFlash;
use open_servo_hw::v2::SignalReader;
use open_servo_runtime::service_primitives::ServicePrimitives;
use open_servo_runtime::shadow_storage::ShadowStorage;

use crate::persist::{PersistResult, PersistTask};

/// Error type for service operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ServiceError {
    /// Persist init failed.
    PersistInit,
    /// Persist write failed.
    PersistWrite,
}

/// Service orchestration bundle.
///
/// Owns service instances and provides `run_once()` for firmware to call in a loop.
///
/// # Type Parameters
///
/// - `P`: [`ServicePrimitives`] impl (provides signals)
/// - `F`: Flash type for persist service
/// - `N`: Shadow table size
pub struct Services<'a, P, F, const N: usize>
where
    P: ServicePrimitives,
    F: NorFlash,
{
    primitives: &'a P,
    persist: PersistTask<F>,
    shadow: &'a ShadowStorage<N>,
    initialized: bool,
}

impl<'a, P, F, const N: usize> Services<'a, P, F, N>
where
    P: ServicePrimitives,
    F: NorFlash,
{
    /// Create a new service bundle.
    ///
    /// Call [`init()`](Self::init) before [`run_once()`](Self::run_once).
    pub fn new(
        primitives: &'a P,
        flash: F,
        flash_range: Range<u32>,
        shadow: &'a ShadowStorage<N>,
    ) -> Self {
        Self {
            primitives,
            persist: PersistTask::new(flash, flash_range),
            shadow,
            initialized: false,
        }
    }

    /// Initialize services (restore EEPROM from flash).
    ///
    /// Call once at startup before entering the run loop.
    pub async fn init(&mut self) -> Result<(), ServiceError> {
        self.persist
            .init(self.shadow)
            .await
            .map_err(|_| ServiceError::PersistInit)?;
        self.initialized = true;
        Ok(())
    }

    /// Run one service cycle.
    ///
    /// Waits for a signal, handles it, then returns. Firmware wraps in a loop:
    ///
    /// ```rust,ignore
    /// services.init().await?;
    /// loop {
    ///     if let Err(e) = services.run_once().await {
    ///         defmt::error!("Service error: {:?}", e);
    ///     }
    /// }
    /// ```
    pub async fn run_once(&mut self) -> Result<(), ServiceError> {
        let (_, persist_rx) = self.primitives.persist_signal();

        // Wait for persist signal
        persist_rx.wait().await;

        // Handle persist request
        match self.persist.persist(self.shadow).await {
            PersistResult::Ok | PersistResult::NoChange | PersistResult::Busy => Ok(()),
            PersistResult::WriteFailed | PersistResult::FirstBoot => {
                Err(ServiceError::PersistWrite)
            }
        }
    }

    /// Check if services have been initialized.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Get the last persist result.
    pub fn last_persist_result(&self) -> Option<PersistResult> {
        self.persist.last_result()
    }
}
