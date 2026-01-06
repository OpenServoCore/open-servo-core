#![no_std]
//! # open-servo-runtime-embassy
//!
//! Embassy-specific runtime implementation for open-servo.
//!
//! This crate provides:
//! - Embassy executor with [`run()`] function (requires arch feature)
//! - Async primitive wrappers ([`EmbassySignal`], [`EmbassyTimer`])
//! - Service tasks ([`rtt_rpc_task`]) and loop helpers
//! - Signal storage for inter-task communication
//!
//! ## Features
//!
//! Enable one architecture feature to get the `run()` function:
//! - `arch-cortex-m` - ARM Cortex-M targets
//! - `arch-riscv32` - RISC-V 32-bit targets
//!
//! ## Usage
//!
//! ```rust,ignore
//! use open_servo_runtime_embassy::{run, save_signal, set_shadow, EmbassySignal, rtt_rpc_task};
//!
//! fn main() -> ! {
//!     // ... hardware init ...
//!
//!     // Create runtime with save signal for persist notifications
//!     let runtime = Runtime::new(board, kernel, EmbassySignal(save_signal()));
//!     runtime.init();
//!
//!     // Wire up shadow reference for global access
//!     set_shadow(runtime.shadow());
//!
//!     // Run executor with tasks from runtime-embassy
//!     run(|spawner| {
//!         spawner.must_spawn(rtt_rpc_task());
//!         spawner.must_spawn(persist_task(flash));  // flash is board-specific
//!     })
//! }
//! ```

mod embassy_runtime;
#[macro_use]
mod macros;
mod primitives;
mod signals;
pub mod tasks;

pub use embassy_runtime::EmbassyRuntime;

// Private module for macro trait bounds (not part of public API)
#[doc(hidden)]
pub mod __private {
    use embedded_storage_async::nor_flash::MultiwriteNorFlash;
    use open_servo_hw::config::BoardConfig;
    use open_servo_hw::v2::Board;
    use open_servo_hw::Timebase;

    /// Trait bound alias for boards that support persist service.
    pub trait PersistBoard: Board + Timebase + BoardConfig + 'static
    where
        Self::Flash: MultiwriteNorFlash,
    {
    }
    impl<B> PersistBoard for B
    where
        B: Board + Timebase + BoardConfig + 'static,
        B::Flash: MultiwriteNorFlash,
    {
    }

    /// Trait bound alias for boards that support RPC service.
    pub trait RpcBoard: Board + Timebase + BoardConfig + 'static {}
    impl<B> RpcBoard for B where B: Board + Timebase + BoardConfig + 'static {}
}
pub use primitives::{EmbassySignal, EmbassyTimer, ResetSignalWrapper};
pub use signals::{on_eeprom_write, reset_signal, rpc_tick, save_signal};
pub use tasks::{run_persist_loop, run_rpc_loop};

#[cfg(feature = "osctl")]
pub use tasks::rtt_rpc_task;

// Re-export for convenience
pub use open_servo_hw::v2::ResetLevel;
pub use open_servo_runtime::{Runtime, ShadowStorage, SHADOW_TABLE_SIZE};
pub use open_servo_services::{PersistSignals, PersistTask, RpcService, ServiceCtx, ServiceTask};

// Re-export panic handler for firmware use
#[cfg(feature = "panic-rtt")]
pub use panic_rtt_target as panic_handler;

// =============================================================================
// Shadow storage reference (set by firmware during init)
// =============================================================================

use core::cell::UnsafeCell;

/// Type alias for shadow storage with embassy signal type.
pub type EmbassyShadow = ShadowStorage<SHADOW_TABLE_SIZE, EmbassySignal>;

/// Wrapper for shadow reference that implements Sync.
///
/// SAFETY: Access is controlled by init ordering - set_shadow() is called
/// during init before any tasks run, and get_shadow() is only called from
/// tasks after init completes.
struct ShadowRef(UnsafeCell<Option<&'static EmbassyShadow>>);

// SAFETY: Single-threaded embedded context with controlled init ordering.
// set_shadow() is called during init before interrupts/tasks are enabled.
unsafe impl Sync for ShadowRef {}

/// Global shadow reference storage.
static SHADOW_REF: ShadowRef = ShadowRef(UnsafeCell::new(None));

/// Set the shadow storage reference.
///
/// Must be called exactly once before spawning tasks that need shadow access.
pub fn set_shadow(shadow: &'static EmbassyShadow) {
    // SAFETY: Called once during init before tasks run
    unsafe {
        *SHADOW_REF.0.get() = Some(shadow);
    }
}

/// Get the shadow storage reference.
///
/// # Panics
///
/// Panics if called before `set_shadow()`.
#[inline]
pub fn get_shadow() -> &'static EmbassyShadow {
    // SAFETY: Only called after set_shadow() during init
    unsafe { (*SHADOW_REF.0.get()).expect("shadow not initialized - call set_shadow() first") }
}

// =============================================================================
// Executor (requires arch feature)
// =============================================================================

#[cfg(not(any(feature = "arch-cortex-m", feature = "arch-riscv32")))]
compile_error!(
    "open-servo-runtime-embassy requires an architecture feature. \
     Enable one of: `arch-cortex-m`, `arch-riscv32`"
);

#[cfg(any(feature = "arch-cortex-m", feature = "arch-riscv32"))]
mod executor {
    use embassy_executor::{Executor, Spawner};
    use static_cell::StaticCell;

    /// Static executor storage.
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();

    /// Run the embassy executor with services.
    ///
    /// This function never returns. Firmware provides a closure that spawns
    /// all required service tasks.
    ///
    /// # Example
    ///
    /// ```rust,ignore
    /// run(|spawner| {
    ///     spawner.must_spawn(persist_task(flash, range, shadow));
    ///     spawner.must_spawn(rpc_task(io, buf, shadow));
    /// })
    /// ```
    pub fn run<F: FnOnce(Spawner)>(spawn_services: F) -> ! {
        let executor = EXECUTOR.init(Executor::new());
        executor.run(spawn_services)
    }
}

#[cfg(any(feature = "arch-cortex-m", feature = "arch-riscv32"))]
pub use executor::run;
