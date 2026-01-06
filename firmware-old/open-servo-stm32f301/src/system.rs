//! Global system state management.

use core::cell::RefCell;
use cortex_m::interrupt::{free, Mutex};
use heapless::spsc::Queue;
use open_servo_control::PidController;
use open_servo_core::{App, Event, EventConsumer, EventProducer};

use crate::board::Board;

#[cfg(feature = "debug-shell")]
use open_servo_core::DebugShell;
#[cfg(feature = "debug-shell")]
use open_servo_hw_utils::rtt_debug::RttDebugIo;

/// Event queue size for this board
pub const EVENT_QUEUE_SIZE: usize = 32;

/// Global system state container
pub struct SystemState;

// Global static storage for App, Hardware, and Event system
static APP: Mutex<RefCell<Option<App<PidController>>>> = Mutex::new(RefCell::new(None));
static BOARD: Mutex<RefCell<Option<Board>>> = Mutex::new(RefCell::new(None));
static EVENT_PRODUCER: Mutex<RefCell<Option<EventProducer<EVENT_QUEUE_SIZE>>>> =
    Mutex::new(RefCell::new(None));
static EVENT_CONSUMER: Mutex<RefCell<Option<EventConsumer<EVENT_QUEUE_SIZE>>>> =
    Mutex::new(RefCell::new(None));

#[cfg(feature = "debug-shell")]
static DEBUG_SHELL: Mutex<RefCell<Option<DebugShell<RttDebugIo>>>> = Mutex::new(RefCell::new(None));

impl SystemState {
    /// Initialize system state with app and board
    pub fn init_app_and_board(app: App<PidController>, board: Board) {
        free(|cs| {
            APP.borrow(cs).replace(Some(app));
            BOARD.borrow(cs).replace(Some(board));
        });
    }

    /// Initialize event system
    pub fn init_event_system(
        producer: EventProducer<EVENT_QUEUE_SIZE>,
        consumer: EventConsumer<EVENT_QUEUE_SIZE>,
    ) {
        free(|cs| {
            EVENT_PRODUCER.borrow(cs).replace(Some(producer));
            EVENT_CONSUMER.borrow(cs).replace(Some(consumer));
        });
    }

    /// Initialize debug shell
    #[cfg(feature = "debug-shell")]
    pub fn init_debug_shell(shell: DebugShell<RttDebugIo>) {
        free(|cs| {
            DEBUG_SHELL.borrow(cs).replace(Some(shell));
        });
    }

    /// Access app and board together
    pub fn with_app_and_board<F>(f: F)
    where
        F: FnOnce(&mut App<PidController>, &mut Board),
    {
        free(|cs| {
            let mut app_ref = APP.borrow(cs).borrow_mut();
            let mut board_ref = BOARD.borrow(cs).borrow_mut();

            if let (Some(app), Some(board)) = (app_ref.as_mut(), board_ref.as_mut()) {
                f(app, board);
            }
        });
    }

    /// Access event producer
    pub fn with_event_producer<F>(f: F)
    where
        F: FnOnce(&mut EventProducer<EVENT_QUEUE_SIZE>),
    {
        free(|cs| {
            if let Some(producer) = EVENT_PRODUCER.borrow(cs).borrow_mut().as_mut() {
                f(producer);
            }
        });
    }

    /// Try to dequeue an event
    pub fn dequeue_event() -> Option<Event> {
        free(|cs| {
            let mut consumer_ref = EVENT_CONSUMER.borrow(cs).borrow_mut();
            consumer_ref.as_mut().and_then(|c| c.dequeue())
        })
    }

    /// Process all pending events
    pub fn process_events() {
        while let Some(event) = Self::dequeue_event() {
            Self::with_app_and_board(|app, board| {
                app.handle_event(board, event);
            });
        }
    }

    /// Poll debug shell
    #[cfg(feature = "debug-shell")]
    pub fn poll_debug_shell() {
        free(|cs| {
            let mut shell_ref = DEBUG_SHELL.borrow(cs).borrow_mut();
            let mut app_ref = APP.borrow(cs).borrow_mut();
            let mut board_ref = BOARD.borrow(cs).borrow_mut();

            if let (Some(shell), Some(app), Some(board)) =
                (shell_ref.as_mut(), app_ref.as_mut(), board_ref.as_mut())
            {
                shell.poll(app, board);
            }
        });
    }
}
