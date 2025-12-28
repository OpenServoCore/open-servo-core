//! Static resource storage for ISR and main loop.
//!
//! Architecture:
//! - Data-plane: DMA buffers (no per-byte critical section)
//! - Control-plane: SPSC queues with critical-section guards
//! - ISR-owned: Executor and sinks (single-writer from ADC ISR)

use core::ptr::addr_of_mut;

use heapless::spsc::{Consumer, Producer, Queue};
use open_servo_device::executor::Executor;
use open_servo_kernel::ServoKernel;
use open_servo_kernel_api::host_op::{HostOp, HostResult};

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

/// HostOp queue (main → ADC ISR).
static mut OP_QUEUE: Queue<HostOp, QUEUE_CAPACITY> = Queue::new();

/// HostResult queue (ADC ISR → main).
static mut RESULT_QUEUE: Queue<HostResult, QUEUE_CAPACITY> = Queue::new();

/// Queue halves storage (set once at init).
static mut OP_PROD: Option<Producer<'static, HostOp, QUEUE_CAPACITY>> = None;
static mut OP_CONS: Option<Consumer<'static, HostOp, QUEUE_CAPACITY>> = None;
static mut RESULT_PROD: Option<Producer<'static, HostResult, QUEUE_CAPACITY>> = None;
static mut RESULT_CONS: Option<Consumer<'static, HostResult, QUEUE_CAPACITY>> = None;

// =============================================================================
// ISR-owned resources
// =============================================================================

/// Executor wrapper (single-writer, owned by ADC ISR).
static mut EXECUTOR: Option<Executor<ServoKernel>> = None;

/// Fault sink (ISR-only).
static mut FAULT_SINK: StubFaultSink = StubFaultSink;

/// Telemetry sink (ISR-only).
static mut TELEM_SINK: StubTelemetrySink = StubTelemetrySink;

// =============================================================================
// Initialization functions
// =============================================================================

/// Split queues and return main-side handles.
///
/// # Safety
/// Must be called exactly once before interrupts are enabled.
pub unsafe fn init_queues() -> (
    &'static mut Producer<'static, HostOp, QUEUE_CAPACITY>,
    &'static mut Consumer<'static, HostResult, QUEUE_CAPACITY>,
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
pub unsafe fn set_isr_resources(executor: Executor<ServoKernel>) {
    *addr_of_mut!(EXECUTOR) = Some(executor);
}

/// Get mutable reference to executor (ISR-only).
///
/// # Safety
/// Must only be called from ADC DMA ISR.
#[inline]
pub unsafe fn get_executor() -> &'static mut Executor<ServoKernel> {
    (*addr_of_mut!(EXECUTOR)).as_mut().unwrap()
}

/// Get ISR-side queue handles.
///
/// # Safety
/// Must only be called from ADC DMA ISR.
#[inline]
pub unsafe fn get_isr_queues() -> (
    &'static mut Consumer<'static, HostOp, QUEUE_CAPACITY>,
    &'static mut Producer<'static, HostResult, QUEUE_CAPACITY>,
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
