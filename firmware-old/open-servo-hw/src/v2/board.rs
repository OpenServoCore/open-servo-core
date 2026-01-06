//! Unified hardware abstraction trait.
//!
//! The `Board` trait is the single interface between kernel and hardware.
//! It replaces the previous 24+ fragmented traits with a minimal, cohesive API.

use crate::v2::capability::{MotorType, SensorCapabilities};
use crate::v2::io::{MotorCommand, SensorFrame};
use crate::v2::ServoPosKind;
use embedded_io_async::{Read, Write};
use embedded_storage_async::nor_flash::NorFlash;

/// Unified hardware abstraction for servo boards.
///
/// This is intentionally minimal. Timing (`now`, `dt_us`) is passed as parameters
/// to kernel tick functions by the board/HAL caller, not via this trait.
///
/// # Design notes
///
/// - Board owns ADC DMA buffer internally (sidesteps const generic in trait)
/// - All sensor readings are returned via [`SensorFrame`]
/// - Optional sensors use `None` when not present, `Some(Reading::Valid/Invalid)` otherwise
/// - Motor commands go through [`MotorCommand`]
/// - Peripheral I/O (UART, flash) accessed via associated types using standard traits
///
/// # Peripheral I/O
///
/// The Board trait provides access to peripheral I/O through associated types:
/// - `Uart`: Async UART using standard `embedded-io-async` traits
/// - `Flash`: Async NorFlash for EEPROM persistence
///
/// DMA is an implementation detail - firmware provides `impl Read + Write`,
/// runtime doesn't care if it uses DMA, interrupts, or polling.
///
/// ## RTT (Debug I/O)
///
/// RTT is NOT part of the Board trait because:
/// 1. RTT channels can be accessed globally via `rtt-target` macros
/// 2. RTT is debug-only, not production hardware
/// 3. defmt uses RTT implicitly via macros
/// 4. Keeps Board focused on production hardware
///
/// Debug I/O consumers (osctl/RPC, telemetry dump) access RTT directly
/// via `rtt-target` when the `osctl` feature is enabled.
pub trait Board {
    /// Async UART for Dynamixel protocol.
    ///
    /// Uses standard `embedded-io-async` traits. DMA is an implementation detail.
    /// Half-duplex timing and TX enable are handled by runtime/protocol layer.
    type Uart: Read + Write;

    /// Async NorFlash for EEPROM persistence.
    ///
    /// Used by sequential-storage for wear-leveled key-value storage.
    type Flash: NorFlash;

    /// Servo position semantics for this board.
    fn servo_pos_kind(&self) -> ServoPosKind;

    /// Motor topology for this board (BDC or BLDC).
    fn motor_type(&self) -> MotorType;

    /// Runtime capability flags for optional sensors.
    fn sensor_capabilities(&self) -> SensorCapabilities;

    /// Read all sensor samples from internal ADC buffer.
    ///
    /// Board owns `AdcDmaBuf<N>` internally (N determined by board features).
    /// This method converts raw ADC values to typed samples using calibration.
    fn read_sensors(&mut self) -> SensorFrame;

    /// Apply motor command to hardware.
    fn write_motor(&mut self, cmd: MotorCommand);

    /// Get mutable reference to the UART.
    fn uart(&mut self) -> &mut Self::Uart;

    /// Get mutable reference to the flash storage (if not taken).
    ///
    /// Returns `None` if flash was already taken via [`take_flash`].
    fn flash(&mut self) -> Option<&mut Self::Flash>;

    /// Take ownership of the flash storage.
    ///
    /// Returns the flash driver, transferring ownership to the caller.
    /// Subsequent calls to `flash()` or `take_flash()` will return `None`.
    ///
    /// Used by Runtime to give flash to PersistTask.
    fn take_flash(&mut self) -> Option<Self::Flash>;

    /// Flash address range for EEPROM persistence (relative offsets).
    ///
    /// Returns the address range within the flash region allocated for
    /// EEPROM storage. Typically `0..capacity` for flash drivers that
    /// use offset-based addressing.
    fn eeprom_flash_range(&self) -> core::ops::Range<u32>;

    /// Trigger a software reset. Never returns.
    ///
    /// Used by:
    /// - DXL REBOOT instruction (0x08)
    /// - DXL FACTORY_RESET instruction (0x06) after EEPROM erase
    /// - RPC factory reset after EEPROM erase
    fn reboot(&self) -> !;
}
