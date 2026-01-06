//! Embassy-specific runtime wrapper.
//!
//! [`EmbassyRuntime`] wraps the core [`Runtime`] and provides
//! service lifecycle management with embassy-specific types.

use core::ops::Range;

use embedded_storage_async::nor_flash::MultiwriteNorFlash;

use open_servo_hw::config::BoardConfig;
use open_servo_hw::v2::Board;
use open_servo_hw::Timebase;
use open_servo_runtime::Runtime;
use open_servo_services::{PersistSignals, PersistTask, ServiceCtx, ServiceTask};

use crate::primitives::{EmbassySignal, EmbassyTimer, ResetSignalWrapper};
use crate::signals::{reset_signal, save_signal};
use crate::{get_shadow, SHADOW_TABLE_SIZE};

#[cfg(feature = "osctl")]
use crate::signals::rpc_tick;
#[cfg(feature = "osctl")]
use open_servo_hw_utils::rtt_async::RttChannels;
#[cfg(feature = "osctl")]
use open_servo_services::RpcService;
#[cfg(feature = "osctl")]
use static_cell::StaticCell;

/// Embassy-specific runtime wrapper that owns service lifecycle.
///
/// This wrapper provides:
/// - `persist_forever()`: Runs the persist service forever
/// - `rpc_forever()`: Runs the RPC service forever (with osctl feature)
///
/// # Usage
///
/// ```rust,ignore
/// let runtime = init_runtime(board, kernel, EmbassySignal(save_signal()));
/// let embassy_rt = EmbassyRuntime::new(runtime);
///
/// // In embassy task:
/// embassy_rt.persist_forever().await;
/// ```
pub struct EmbassyRuntime<B>
where
    B: Board + Timebase + BoardConfig + 'static,
{
    runtime: &'static Runtime<B, EmbassySignal>,
}

impl<B> EmbassyRuntime<B>
where
    B: Board + Timebase + BoardConfig + 'static,
{
    /// Create a new embassy runtime wrapper.
    pub fn new(runtime: &'static Runtime<B, EmbassySignal>) -> Self {
        Self { runtime }
    }

    /// Get the underlying runtime reference.
    ///
    /// Used for ISR access to `run_fast_tick()`.
    #[inline]
    pub fn runtime(&self) -> &'static Runtime<B, EmbassySignal> {
        self.runtime
    }

    /// Get the shadow storage reference.
    #[inline]
    pub fn shadow(&self) -> &'static crate::ShadowStorage<SHADOW_TABLE_SIZE, EmbassySignal> {
        self.runtime.shadow()
    }
}

impl<B> EmbassyRuntime<B>
where
    B: Board + Timebase + BoardConfig + 'static,
    B::Flash: MultiwriteNorFlash,
{
    /// Run persist service forever.
    ///
    /// Takes flash from runtime and runs the persist service loop.
    /// This method never returns.
    ///
    /// # Panics
    ///
    /// Panics if flash was already taken (persist_forever called twice).
    pub async fn persist_forever(&self) {
        // Take flash from runtime
        let (flash, range) = unsafe { self.runtime.take_flash_for_persist() }
            .expect("flash already taken - persist_forever can only be called once");

        self.run_persist_with_flash(flash, range).await
    }

    /// Run persist service with provided flash.
    ///
    /// Lower-level method for cases where flash is obtained separately.
    pub async fn run_persist_with_flash(&self, flash: B::Flash, range: Range<u32>) {
        let signals = PersistSignals::new(
            EmbassySignal(save_signal()),
            ResetSignalWrapper(reset_signal()),
        );

        let mut task = PersistTask::new(flash, range, signals);

        static TIMER: EmbassyTimer = EmbassyTimer;
        let ctx = ServiceCtx::new(get_shadow(), &TIMER);

        task.init(&ctx).await;
        loop {
            task.handle_once(&ctx).await;
        }
    }
}

#[cfg(feature = "osctl")]
impl<B> EmbassyRuntime<B>
where
    B: Board + Timebase + BoardConfig + 'static,
{
    /// Run RPC service forever.
    ///
    /// Initializes RTT channels and runs the RPC service loop.
    /// This method never returns.
    pub async fn rpc_forever(&self) {
        // Initialize RTT and steal channels
        let rtt = RttChannels::init(EmbassySignal(rpc_tick()));

        // Allocate RPC buffer
        static RPC_BUF: StaticCell<[u8; 192]> = StaticCell::new();
        let buf = RPC_BUF.init([0u8; 192]);

        static TIMER: EmbassyTimer = EmbassyTimer;
        let mut service = RpcService::new(rtt.rpc, buf, get_shadow(), &TIMER);

        service.init(&()).await;
        loop {
            service.handle_once(&()).await;
        }
    }
}
