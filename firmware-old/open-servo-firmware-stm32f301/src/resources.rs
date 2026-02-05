//! Static resources for firmware.
//!
//! Contains:
//! - Runtime storage and accessor
//! - EmbassyRuntime wrapper storage (osctl only)
//! - DMA buffers (ADC, UART)

use open_servo_kernel::ServoKernel;
use open_servo_runtime::Runtime;
use open_servo_runtime_embassy::EmbassySignal;
use static_cell::StaticCell;

#[cfg(feature = "osctl")]
use open_servo_runtime_embassy::EmbassyRuntime;

use crate::adc_config::{AdcBuffer, ADC_CHANNEL_COUNT};
use crate::board::Stm32f301Board;

// =============================================================================
// Runtime storage
// =============================================================================

/// Runtime type alias for this board with embassy signal type.
pub type BoardRuntime = Runtime<Stm32f301Board, EmbassySignal>;

/// EmbassyRuntime type alias for this board.
#[cfg(feature = "osctl")]
pub type BoardEmbassyRuntime = EmbassyRuntime<Stm32f301Board>;

/// Global runtime instance storage.
static RUNTIME: StaticCell<BoardRuntime> = StaticCell::new();

/// Global embassy runtime wrapper storage.
#[cfg(feature = "osctl")]
static EMBASSY_RT: StaticCell<BoardEmbassyRuntime> = StaticCell::new();

/// Global runtime reference (set after init).
static mut RUNTIME_REF: Option<&'static BoardRuntime> = None;

/// Global embassy runtime reference (set after init).
#[cfg(feature = "osctl")]
static mut EMBASSY_RT_REF: Option<&'static BoardEmbassyRuntime> = None;

/// Initialize the runtime with the given save signal.
///
/// The save signal is used to notify the persist service when EEPROM is modified.
///
/// # Safety
/// Must be called exactly once, before interrupts are enabled.
pub unsafe fn init_runtime(
    board: Stm32f301Board,
    kernel: ServoKernel,
    save_signal: EmbassySignal,
) -> &'static BoardRuntime {
    let runtime = RUNTIME.init(Runtime::new(board, kernel, save_signal));
    RUNTIME_REF = Some(runtime);
    runtime
}

/// Initialize the embassy runtime wrapper.
///
/// Must be called after init_runtime() and before spawning tasks.
///
/// # Safety
/// Must be called exactly once, after init_runtime().
#[cfg(feature = "osctl")]
pub unsafe fn init_embassy_runtime(runtime: &'static BoardRuntime) -> &'static BoardEmbassyRuntime {
    let embassy_rt = EMBASSY_RT.init(EmbassyRuntime::new(runtime));
    EMBASSY_RT_REF = Some(embassy_rt);
    embassy_rt
}

/// Get reference to the runtime (for ISR and async tasks).
///
/// # Safety
/// Must only be called after runtime is initialized via init_runtime().
#[inline]
pub fn get_runtime() -> &'static BoardRuntime {
    unsafe { RUNTIME_REF.unwrap_unchecked() }
}

/// Get reference to the embassy runtime (for async tasks).
///
/// # Safety
/// Must only be called after init_embassy_runtime().
#[cfg(feature = "osctl")]
#[inline]
pub fn get_embassy_runtime() -> &'static BoardEmbassyRuntime {
    unsafe { EMBASSY_RT_REF.unwrap_unchecked() }
}

// =============================================================================
// DMA buffers
// =============================================================================

/// UART RX DMA buffer size.
pub const UART_RX_BUF_SIZE: usize = 256;

/// ADC DMA target buffer.
///
/// Written by DMA, read by Board::read_sensors() in ISR.
#[no_mangle]
pub static mut ADC_DMA_BUF: AdcBuffer = [0u16; ADC_CHANNEL_COUNT];

/// UART RX DMA circular buffer.
///
/// Written by DMA, read by main loop.
#[no_mangle]
pub static mut UART_RX_DMA_BUF: [u8; UART_RX_BUF_SIZE] = [0u8; UART_RX_BUF_SIZE];
