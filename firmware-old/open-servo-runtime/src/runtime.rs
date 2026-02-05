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
//! │ Runtime<B>                                              │
//! │                                                         │
//! │  ┌─────────┐  ┌─────────────────┐  ┌─────────────────┐  │
//! │  │  Board  │  │ ControlExecutor │  │  ShadowStorage  │  │
//! │  │  (B)    │  │  (ServoKernel)  │  │     (1024)      │  │
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
//! static RUNTIME: StaticCell<Runtime<MyBoard>> = StaticCell::new();
//!
//! #[entry]
//! fn main() -> ! {
//!     let board = MyBoard::new(...);
//!     let kernel = ServoKernel::new(config);
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

use open_servo_hw::config::BoardConfig;
use open_servo_hw::v2::board::Board;
use open_servo_hw::v2::PersistW;
use open_servo_hw::Timebase;
use open_servo_kernel::ServoKernel;
use open_servo_kernel_api::ops::{KernelOp, KernelResult};
use open_servo_kernel_api::{FaultSink, TelemetrySink};
use open_servo_units::MicroSecond;

use crate::executor::ControlExecutor;
use crate::shadow::{create_storage, ServoShadowStorage};

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
/// - `PW`: Signal type implementing [`PersistW`] for persist notifications
///
/// The kernel type is concrete ([`ServoKernel`]) and shadow size is fixed at 1024 bytes.
pub struct Runtime<B, PW: PersistW> {
    /// The board (hardware abstraction).
    board: UnsafeCell<B>,

    /// The kernel executor (owns kernel + decimation).
    executor: UnsafeCell<ControlExecutor>,

    /// The shadow storage (register table, 1024 bytes).
    shadow: ServoShadowStorage<PW>,

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
unsafe impl<B, PW: PersistW> Sync for Runtime<B, PW> where B: Send {}

impl<B, PW: PersistW> Runtime<B, PW>
where
    B: Board + Timebase + BoardConfig,
{
    /// Create a new runtime with the given board, kernel, and save signal.
    ///
    /// The save signal is used to notify the persist service when EEPROM
    /// registers are modified.
    ///
    /// Call `init()` after storing in a static to split the queues.
    pub fn new(board: B, kernel: ServoKernel, save_signal: PW) -> Self {
        Self {
            board: UnsafeCell::new(board),
            executor: UnsafeCell::new(ControlExecutor::new(kernel)),
            shadow: create_storage(save_signal),
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
    /// This performs all runtime initialization:
    /// 1. Initializes registry hash maps
    /// 2. Applies board defaults to shadow table (DXL identity, etc.)
    /// 3. Splits the SPSC queues
    ///
    /// Must be called exactly once before interrupts are enabled.
    ///
    /// # Safety
    ///
    /// Must be called exactly once before interrupts are enabled.
    /// The runtime must be stored in a static location before calling.
    pub unsafe fn init(&'static self) {
        // Initialize registry hash maps (for EEPROM field lookup).
        open_servo_registry::init();

        // Apply board defaults to shadow (model_number, id, baud_rate, etc.).
        // This must happen BEFORE flash restore so virgin flash gets defaults.
        self.apply_board_defaults();

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

    /// Apply board defaults to shadow table.
    ///
    /// Writes DXL identity (RO) and EEPROM defaults (RW) to shadow.
    /// Called during init, before flash restore, so virgin flash keeps defaults.
    ///
    /// Uses `load_defaults` to avoid marking dirty bits (no persist trigger).
    fn apply_board_defaults(&self) {
        // SAFETY: Called during init before interrupts enabled
        let board = unsafe { &*self.board.get() };
        let identity = board.dxl_identity();
        let defaults = board.eeprom_defaults();

        use open_servo_registry::dxl::addr;

        // Use load_defaults to write without marking dirty bits
        let _ = self.shadow.load_defaults(|write| {
            // Write read-only fields (board-provided, immutable)
            write(addr::MODEL_NUMBER, &identity.model_number.to_le_bytes())?;
            write(
                addr::MODEL_INFORMATION,
                &identity.model_information.to_le_bytes(),
            )?;
            write(addr::FIRMWARE_VERSION, &[identity.firmware_version])?;
            write(addr::PROTOCOL_TYPE, &[identity.protocol_type])?;

            // Write writable EEPROM fields (factory defaults)
            write(addr::ID, &[defaults.id])?;
            write(addr::BAUD_RATE, &[defaults.baud_rate])?;
            write(addr::SECONDARY_ID, &[defaults.secondary_id])?;

            Ok(())
        });
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
    /// This is safe to call from main context. Use `host_shadow().with_view(...)`.
    #[inline]
    pub fn shadow(&self) -> &ServoShadowStorage<PW> {
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

    /// Take flash storage from the board.
    ///
    /// Returns the flash driver, transferring ownership. Subsequent calls return `None`.
    /// Used to give flash to the persist service.
    ///
    /// # Safety
    ///
    /// Must only be called from main context during init, before ISR is enabled.
    pub unsafe fn take_flash(&self) -> Option<B::Flash> {
        let board = &mut *self.board.get();
        board.take_flash()
    }

    /// Take flash storage and range for persist service.
    ///
    /// Returns the flash driver and EEPROM address range as a tuple.
    /// Subsequent calls return `None` (flash is owned by persist service).
    ///
    /// This is the preferred method for persist service initialization as it
    /// bundles both the flash driver and its configured range.
    ///
    /// # Safety
    ///
    /// Must only be called from main context during init, before ISR is enabled.
    pub unsafe fn take_flash_for_persist(&self) -> Option<(B::Flash, core::ops::Range<u32>)> {
        let board = &mut *self.board.get();
        let range = board.eeprom_flash_range();
        board.take_flash().map(|flash| (flash, range))
    }
}
