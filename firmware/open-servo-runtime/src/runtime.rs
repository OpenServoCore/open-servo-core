//! Runtime: coordinator between board, kernel, and services.
//!
//! The [`Runtime`] struct is the application framework that owns:
//! - The board (hardware abstraction)
//! - The kernel executor (control loop)
//! - The shadow storage (register table)
//! - SPSC queues (host ↔ kernel communication)
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────┐
//! │ Runtime<B, K, N>                                        │
//! │                                                         │
//! │  ┌─────────┐  ┌─────────────────┐  ┌─────────────────┐  │
//! │  │  Board  │  │ ControlExecutor │  │  ShadowStorage  │  │
//! │  │  (B)    │  │      (K)        │  │      (N)        │  │
//! │  └─────────┘  └─────────────────┘  └─────────────────┘  │
//! │                                                         │
//! │  ┌───────────────────┐  ┌───────────────────┐           │
//! │  │   op_queue        │  │   result_queue    │           │
//! │  │ (main → ISR)      │  │ (ISR → main)      │           │
//! │  └───────────────────┘  └───────────────────┘           │
//! └─────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage Pattern
//!
//! The firmware creates a static Runtime and calls into it from ISR:
//!
//! ```rust,ignore
//! static RUNTIME: StaticCell<Runtime<Board, Kernel, 1024>> = StaticCell::new();
//!
//! #[entry]
//! fn main() -> ! {
//!     let board = Board::new(...);
//!     let kernel = Kernel::new(config);
//!     let runtime = RUNTIME.init(Runtime::new(board, kernel));
//!     runtime.run_main_loop()  // or loop { wfi() } if no services
//! }
//!
//! #[interrupt]
//! fn DMA1_CH1() {
//!     unsafe { RUNTIME.get().run_fast_tick(&mut faults, &mut telem) };
//! }
//! ```

use core::cell::UnsafeCell;

use heapless::spsc::{Consumer, Producer, Queue};

use open_servo_hw::v2::board::Board;
use open_servo_hw::v2::io::{MotorCommand, SensorFrame};
use open_servo_hw::Timebase;
use open_servo_kernel_api::ops::{KernelOp, KernelResult};
use open_servo_kernel_api::{
    kernel::{Kernel, KernelHost},
    shadow::ShadowKernel,
    FaultSink, TelemetrySink,
};
use open_servo_units::MicroSecond;

use crate::executor::ControlExecutor;
use crate::shadow_storage::ShadowStorage;

/// Queue capacity for host operations.
pub const QUEUE_CAPACITY: usize = 8;

/// Runtime coordinator that owns board, kernel, shadow, and queues.
///
/// This is the central struct that the firmware creates once at startup.
/// It coordinates the kernel execution domain (ISR) with async services (main).
///
/// # Generic Parameters
///
/// - `B`: Board type implementing [`Board`] and [`Timebase`]
/// - `K`: Kernel type implementing [`Kernel`], [`KernelHost`], and [`ShadowKernel`]
/// - `N`: Shadow table size in bytes (typically 1024)
pub struct Runtime<B, K, const N: usize> {
    /// The board (hardware abstraction).
    board: UnsafeCell<B>,

    /// The kernel executor (owns kernel + decimation).
    executor: UnsafeCell<ControlExecutor<K>>,

    /// The shadow storage (register table).
    shadow: ShadowStorage<N>,

    /// Op queue: main/async → ADC ISR.
    op_queue: UnsafeCell<Queue<KernelOp, QUEUE_CAPACITY>>,

    /// Result queue: ADC ISR → main/async.
    result_queue: UnsafeCell<Queue<KernelResult, QUEUE_CAPACITY>>,

    /// Op queue producer (main side) - set after split.
    op_prod: UnsafeCell<Option<Producer<'static, KernelOp, QUEUE_CAPACITY>>>,

    /// Op queue consumer (ISR side) - set after split.
    op_cons: UnsafeCell<Option<Consumer<'static, KernelOp, QUEUE_CAPACITY>>>,

    /// Result queue producer (ISR side) - set after split.
    result_prod: UnsafeCell<Option<Producer<'static, KernelResult, QUEUE_CAPACITY>>>,

    /// Result queue consumer (main side) - set after split.
    result_cons: UnsafeCell<Option<Consumer<'static, KernelResult, QUEUE_CAPACITY>>>,
}

// SAFETY: Runtime is designed for single-writer access patterns:
// - ISR accesses: board (write), executor (write), op_cons, result_prod
// - Main accesses: shadow (host_* methods), op_prod, result_cons
// The UnsafeCell wrappers enforce this discipline at the API level.
unsafe impl<B, K, const N: usize> Sync for Runtime<B, K, N>
where
    B: Send,
    K: Send,
{
}

impl<B, K, const N: usize> Runtime<B, K, N>
where
    B: Board + Timebase,
    K: Kernel<Frame = SensorFrame, Command = MotorCommand> + KernelHost + ShadowKernel,
{
    /// Create a new runtime with the given board and kernel.
    ///
    /// Call `init()` after storing in a static to split the queues.
    pub fn new(board: B, kernel: K) -> Self {
        Self {
            board: UnsafeCell::new(board),
            executor: UnsafeCell::new(ControlExecutor::new(kernel)),
            shadow: ShadowStorage::new(),
            op_queue: UnsafeCell::new(Queue::new()),
            result_queue: UnsafeCell::new(Queue::new()),
            op_prod: UnsafeCell::new(None),
            op_cons: UnsafeCell::new(None),
            result_prod: UnsafeCell::new(None),
            result_cons: UnsafeCell::new(None),
        }
    }

    /// Initialize the runtime after storing in a static.
    ///
    /// This splits the SPSC queues and sets up the persist callback.
    /// Must be called exactly once before interrupts are enabled.
    ///
    /// # Safety
    ///
    /// Must be called exactly once before interrupts are enabled.
    /// The runtime must be stored in a static location before calling.
    pub unsafe fn init(&'static self) {
        // Split the op queue.
        let op_queue = &mut *self.op_queue.get();
        let (op_prod, op_cons) = op_queue.split();
        *self.op_prod.get() = Some(op_prod);
        *self.op_cons.get() = Some(op_cons);

        // Split the result queue.
        let result_queue = &mut *self.result_queue.get();
        let (result_prod, result_cons) = result_queue.split();
        *self.result_prod.get() = Some(result_prod);
        *self.result_cons.get() = Some(result_cons);
    }

    /// ADC ISR entrypoint: read sensors, run kernel, write motor.
    ///
    /// This is the **only** function that should call into the kernel.
    /// It runs at the ADC completion rate (typically 10kHz).
    ///
    /// # Safety
    ///
    /// Must only be called from the ADC DMA completion ISR.
    /// Must not be called concurrently with itself.
    #[inline]
    pub unsafe fn run_fast_tick<F, T>(&self, dt_us: MicroSecond, faults: &mut F, telem: &mut T)
    where
        F: FaultSink,
        T: TelemetrySink,
    {
        let board = &mut *self.board.get();
        let executor = &mut *self.executor.get();
        let op_cons = (*self.op_cons.get()).as_mut().unwrap();
        let result_prod = (*self.result_prod.get()).as_mut().unwrap();

        // Read sensors from board.
        let frame = board.read_sensors();
        let now = board.now_us();

        // Run kernel via executor.
        let cmd = executor.on_adc_complete(
            frame,
            dt_us,
            now,
            op_cons,
            result_prod,
            faults,
            telem,
            &self.shadow,
        );

        // Write motor command to board.
        board.write_motor(cmd);
    }

    /// Get reference to shadow storage for host operations.
    ///
    /// This is safe to call from main context. Use `host_*` methods.
    #[inline]
    pub fn shadow(&self) -> &ShadowStorage<N> {
        &self.shadow
    }

    /// Get reference to op producer for enqueuing host operations.
    ///
    /// # Safety
    ///
    /// Must only be called from main/async context, not ISR.
    #[inline]
    pub unsafe fn op_producer(&self) -> &mut Producer<'static, KernelOp, QUEUE_CAPACITY> {
        (*self.op_prod.get()).as_mut().unwrap()
    }

    /// Get reference to result consumer for reading kernel responses.
    ///
    /// # Safety
    ///
    /// Must only be called from main/async context, not ISR.
    #[inline]
    pub unsafe fn result_consumer(&self) -> &mut Consumer<'static, KernelResult, QUEUE_CAPACITY> {
        (*self.result_cons.get()).as_mut().unwrap()
    }

    /// Set the persist callback on shadow storage.
    ///
    /// This should be called during init to wire up EEPROM persistence.
    pub fn set_persist_callback(&self, callback: fn()) {
        self.shadow.set_persist_callback(callback);
    }
}
