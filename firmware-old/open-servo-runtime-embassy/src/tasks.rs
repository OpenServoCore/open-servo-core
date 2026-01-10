//! Service tasks and loop helpers.
//!
//! This module provides:
//! - `#[embassy_executor::task]` definitions for services with concrete types
//! - Async loop helpers for services with board-specific types (e.g., flash)
//!
//! ## Tasks vs Helpers
//!
//! | Function | Type | Use Case |
//! |----------|------|----------|
//! | [`rtt_rpc_task`] | `#[task]` | RTT RPC - no board-specific types |
//! | [`run_persist_loop`] | helper | Persist - needs board-specific flash |
//!
//! ## Usage
//!
//! ```rust,ignore
//! use open_servo_runtime_embassy::{run, set_shadow, rtt_rpc_task, run_persist_loop};
//!
//! // Firmware only defines tasks that need board-specific types:
//! #[embassy_executor::task]
//! async fn persist_task(flash: MyFlash, range: Range<u32>, shadow: &'static MyShadow) {
//!     run_persist_loop(flash, range, shadow).await
//! }
//!
//! fn main() -> ! {
//!     set_shadow(runtime.shadow());
//!     run(|spawner| {
//!         spawner.must_spawn(rtt_rpc_task());  // from runtime-embassy
//!         spawner.must_spawn(persist_task(flash, range, shadow));
//!     })
//! }
//! ```

use core::ops::Range;

use embedded_io_async::{Read, Write};
use embedded_storage_async::nor_flash::MultiwriteNorFlash;

use open_servo_services::{
    HostOps, PersistSignals, PersistTask, RpcService, ServiceCtx, ServiceTask,
};

use crate::primitives::{EmbassySignal, EmbassyTimer, ResetSignalWrapper};
use crate::signals::{reset_signal, save_signal};

#[cfg(feature = "osctl")]
use crate::get_shadow;
#[cfg(feature = "osctl")]
use crate::signals::rpc_tick;
#[cfg(feature = "osctl")]
use open_servo_hw_utils::rtt_async::RttChannels;
#[cfg(feature = "osctl")]
use static_cell::StaticCell;

/// Run the persist service loop.
///
/// Call this from a firmware `#[embassy_executor::task]` with concrete types.
///
/// # Arguments
///
/// - `flash`: Flash driver implementing `MultiwriteNorFlash`
/// - `flash_range`: Address range to use for storage
/// - `shadow`: Shadow table reference (any `HostOps` implementor)
pub async fn run_persist_loop<F: MultiwriteNorFlash, S: HostOps>(
    flash: F,
    flash_range: Range<u32>,
    shadow: &'static S,
) {
    static TIMER: EmbassyTimer = EmbassyTimer;

    let signals = PersistSignals::new(
        EmbassySignal(save_signal()),
        ResetSignalWrapper(reset_signal()),
    );

    let mut task = PersistTask::new(flash, flash_range, signals);

    // Create service context
    let ctx = ServiceCtx::new(shadow, &TIMER);

    // Initialize (restore from flash)
    task.init(&ctx).await;

    // Service loop
    loop {
        task.handle_once(&ctx).await;
    }
}

/// Run the RPC service loop.
///
/// Call this from a firmware `#[embassy_executor::task]` with concrete types.
///
/// # Arguments
///
/// - `io`: IO transport implementing `Read + Write`
/// - `buf`: Buffer for COBS framing (recommend 192 bytes)
/// - `shadow`: Shadow table reference (any `HostOps` implementor)
pub async fn run_rpc_loop<IO: Read + Write, S: HostOps>(
    io: IO,
    buf: &'static mut [u8],
    shadow: &'static S,
) {
    static TIMER: EmbassyTimer = EmbassyTimer;

    let mut service = RpcService::new(io, buf, shadow, &TIMER);

    // Initialize (no-op for RPC)
    service.init(&()).await;

    // Service loop
    loop {
        service.handle_once(&()).await;
    }
}

/// RTT RPC service task.
///
/// This task handles:
/// - RTT channel initialization
/// - RPC buffer allocation
/// - RPC service loop
///
/// Requires `set_shadow()` to be called before spawning.
#[cfg(feature = "osctl")]
#[embassy_executor::task]
pub async fn rtt_rpc_task() {
    run_rtt_rpc_loop().await
}

/// Run the RTT-based RPC service loop (internal implementation).
#[cfg(feature = "osctl")]
async fn run_rtt_rpc_loop() {
    // Initialize RTT channels
    let rtt = RttChannels::init(EmbassySignal(rpc_tick()));

    // Allocate RPC buffer (192 bytes for COBS framing)
    static RPC_BUF: StaticCell<[u8; 192]> = StaticCell::new();
    let buf = RPC_BUF.init([0u8; 192]);

    // Run RPC service loop with shadow from global reference
    run_rpc_loop(rtt.rpc, buf, get_shadow()).await
}
