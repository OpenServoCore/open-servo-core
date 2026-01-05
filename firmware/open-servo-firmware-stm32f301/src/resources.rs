//! Static resource storage for ISR and main loop.
//!
//! Architecture:
//! - Data-plane: DMA buffers (no per-byte critical section)
//! - Control-plane: SPSC queues with critical-section guards
//! - ISR-owned: ControlExecutor and sinks (single-writer from ADC ISR)

use core::ptr::addr_of_mut;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use heapless::spsc::{Consumer, Producer, Queue};
use open_servo_runtime::executor::ControlExecutor;
use open_servo_runtime::shadow_storage::ShadowStorage;
use open_servo_kernel::ServoKernel;
use open_servo_kernel_api::ops::{KernelOp, KernelResult};

use crate::adc_config::{AdcBuffer, ADC_CHANNEL_COUNT};
use crate::sinks::{StubFaultSink, StubTelemetrySink};

/// UART RX DMA buffer size.
pub const UART_RX_BUF_SIZE: usize = 256;

/// Control-plane queue capacity.
pub const QUEUE_CAPACITY: usize = 8;

// =============================================================================
// Data-plane: DMA buffers (no critical section per-byte)
// =============================================================================

/// ADC DMA target buffer.
///
/// Written by DMA, read by ADC ISR.
#[no_mangle]
pub static mut ADC_DMA_BUF: AdcBuffer = [0u16; ADC_CHANNEL_COUNT];

/// UART RX DMA circular buffer.
///
/// Written by DMA, read by main loop.
#[no_mangle]
pub static mut UART_RX_DMA_BUF: [u8; UART_RX_BUF_SIZE] = [0u8; UART_RX_BUF_SIZE];

// =============================================================================
// Control-plane: SPSC queues
// =============================================================================

/// KernelOp queue (main/async → ADC ISR).
static mut OP_QUEUE: Queue<KernelOp, QUEUE_CAPACITY> = Queue::new();

/// KernelResult queue (ADC ISR → main/async).
static mut RESULT_QUEUE: Queue<KernelResult, QUEUE_CAPACITY> = Queue::new();

/// Queue halves storage (set once at init).
static mut OP_PROD: Option<Producer<'static, KernelOp, QUEUE_CAPACITY>> = None;
static mut OP_CONS: Option<Consumer<'static, KernelOp, QUEUE_CAPACITY>> = None;
static mut RESULT_PROD: Option<Producer<'static, KernelResult, QUEUE_CAPACITY>> = None;
static mut RESULT_CONS: Option<Consumer<'static, KernelResult, QUEUE_CAPACITY>> = None;

// =============================================================================
// ISR-owned resources
// =============================================================================

/// ControlExecutor wrapper (single-writer, owned by ADC ISR).
static mut EXECUTOR: Option<ControlExecutor<ServoKernel>> = None;

/// Fault sink (ISR-only).
static mut FAULT_SINK: StubFaultSink = StubFaultSink;

/// Telemetry sink (ISR-only).
static mut TELEM_SINK: StubTelemetrySink = StubTelemetrySink;

/// Shadow table size (1024 bytes, Dynamixel Protocol 2.0 compatible).
pub const SHADOW_TABLE_SIZE: usize = 1024;

/// Shadow storage (shared between main and ISR with discipline).
static SHADOW_STORAGE: ShadowStorage<SHADOW_TABLE_SIZE> = ShadowStorage::new();

// =============================================================================
// Async task signal (SysTick → RPC task)
// =============================================================================

/// Signal for RPC service polling (signaled by SysTick ISR).
static RPC_TICK: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Get reference to RPC tick signal.
#[inline]
pub fn rpc_tick() -> &'static Signal<CriticalSectionRawMutex, ()> {
    &RPC_TICK
}

/// Signal for persist service (dxl_req → persist task).
static PERSIST_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Get reference to persist signal.
#[inline]
pub fn persist_signal() -> &'static Signal<CriticalSectionRawMutex, ()> {
    &PERSIST_SIGNAL
}

/// Callback for persist signaling (called from ShadowStorage on EEPROM write).
///
/// Safe to call from critical section (Signal::signal is designed for this).
pub fn on_eeprom_write() {
    PERSIST_SIGNAL.signal(());
}

// =============================================================================
// Initialization functions
// =============================================================================

/// Split queues and return main-side handles.
///
/// # Safety
/// Must be called exactly once before interrupts are enabled.
pub unsafe fn init_queues() -> (
    &'static mut Producer<'static, KernelOp, QUEUE_CAPACITY>,
    &'static mut Consumer<'static, KernelResult, QUEUE_CAPACITY>,
) {
    let (op_prod, op_cons) = (*addr_of_mut!(OP_QUEUE)).split();
    let (result_prod, result_cons) = (*addr_of_mut!(RESULT_QUEUE)).split();

    *addr_of_mut!(OP_PROD) = Some(op_prod);
    *addr_of_mut!(OP_CONS) = Some(op_cons);
    *addr_of_mut!(RESULT_PROD) = Some(result_prod);
    *addr_of_mut!(RESULT_CONS) = Some(result_cons);

    (
        (*addr_of_mut!(OP_PROD)).as_mut().unwrap(),
        (*addr_of_mut!(RESULT_CONS)).as_mut().unwrap(),
    )
}

/// Set ISR resources.
///
/// # Safety
/// Must be called exactly once before interrupts are enabled.
pub unsafe fn set_isr_resources(executor: ControlExecutor<ServoKernel>) {
    *addr_of_mut!(EXECUTOR) = Some(executor);
}

/// Get mutable reference to executor (ISR-only).
///
/// # Safety
/// Must only be called from ADC DMA ISR.
#[inline]
pub unsafe fn get_executor() -> &'static mut ControlExecutor<ServoKernel> {
    (*addr_of_mut!(EXECUTOR)).as_mut().unwrap()
}

/// Get ISR-side queue handles.
///
/// # Safety
/// Must only be called from ADC DMA ISR.
#[inline]
pub unsafe fn get_isr_queues() -> (
    &'static mut Consumer<'static, KernelOp, QUEUE_CAPACITY>,
    &'static mut Producer<'static, KernelResult, QUEUE_CAPACITY>,
) {
    (
        (*addr_of_mut!(OP_CONS)).as_mut().unwrap(),
        (*addr_of_mut!(RESULT_PROD)).as_mut().unwrap(),
    )
}

/// Get mutable reference to fault sink (ISR-only).
///
/// # Safety
/// Must only be called from ADC DMA ISR.
#[inline]
pub unsafe fn get_fault_sink() -> &'static mut StubFaultSink {
    &mut *addr_of_mut!(FAULT_SINK)
}

/// Get mutable reference to telemetry sink (ISR-only).
///
/// # Safety
/// Must only be called from ADC DMA ISR.
#[inline]
pub unsafe fn get_telem_sink() -> &'static mut StubTelemetrySink {
    &mut *addr_of_mut!(TELEM_SINK)
}

/// Get reference to shadow storage (ISR and main loop).
///
/// Access discipline:
/// - Main loop uses `host_*` methods (with critical_section)
/// - ISR uses `kernel_*` methods (single-writer, no CS needed)
#[inline]
pub fn get_shadow_storage() -> &'static ShadowStorage<SHADOW_TABLE_SIZE> {
    &SHADOW_STORAGE
}
