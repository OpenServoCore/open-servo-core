//! Static signal storage for service communication.
//!
//! Provides global signals used for inter-task communication:
//! - `RPC_TICK`: SysTick ISR → RPC service polling
//! - `SAVE_SIGNAL`: Shadow EEPROM write → persist service
//! - `RESET_SIGNAL`: RPC/DXL factory reset → persist service

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use open_servo_hw::v2::ResetLevel;

/// Signal for RPC service polling (signaled by SysTick ISR).
static RPC_TICK: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal for persist Save operations (EEPROM write → persist task).
static SAVE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Signal for persist FactoryReset operations (RPC/DXL → persist task).
static RESET_SIGNAL: Signal<CriticalSectionRawMutex, ResetLevel> = Signal::new();

/// Get reference to RPC tick signal.
#[inline]
pub fn rpc_tick() -> &'static Signal<CriticalSectionRawMutex, ()> {
    &RPC_TICK
}

/// Get reference to save signal.
#[inline]
pub fn save_signal() -> &'static Signal<CriticalSectionRawMutex, ()> {
    &SAVE_SIGNAL
}

/// Get reference to reset signal.
#[inline]
pub fn reset_signal() -> &'static Signal<CriticalSectionRawMutex, ResetLevel> {
    &RESET_SIGNAL
}

/// Callback for persist signaling (called from ShadowStorage on EEPROM write).
///
/// Safe to call from critical section (Signal::signal is designed for this).
pub fn on_eeprom_write() {
    SAVE_SIGNAL.signal(());
}
