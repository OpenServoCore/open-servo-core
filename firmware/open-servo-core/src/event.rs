use heapless::spsc::{Consumer, Producer, Queue};
use open_servo_hw::UartPort;
use super::fault::FaultKind;

/// Events that flow through the system
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Event {
    /// UART byte received
    UartRx { port: UartPort, byte: u8 },
    
    /// Slow tick for periodic tasks (e.g., telemetry, status)
    SlowTick,
    
    /// Fault occurred (for monitoring/logging, not safety-critical path)
    Fault(FaultKind),
}

/// Event queue size - small since we process events quickly
const QUEUE_SIZE: usize = 32;

/// Global event queue storage
static mut EVENT_QUEUE_STORAGE: Queue<Event, QUEUE_SIZE> = Queue::new();

/// Event queue handle
pub struct EventQueue {
    producer: Producer<'static, Event, QUEUE_SIZE>,
    consumer: Consumer<'static, Event, QUEUE_SIZE>,
}

impl EventQueue {
    /// Initialize the event queue (call once at startup)
    /// 
    /// # Safety
    /// Must only be called once, before any interrupts are enabled
    pub unsafe fn init() -> Self {
        // Use a pointer to avoid direct mutable reference
        let queue_ptr = core::ptr::addr_of_mut!(EVENT_QUEUE_STORAGE);
        let (producer, consumer) = (*queue_ptr).split();
        EventQueue { producer, consumer }
    }

    /// Split into producer and consumer
    pub fn split(self) -> (EventProducer, EventConsumer) {
        (
            EventProducer { producer: self.producer },
            EventConsumer { consumer: self.consumer },
        )
    }
}

/// Producer side of event queue (used by ISRs)
pub struct EventProducer {
    producer: Producer<'static, Event, QUEUE_SIZE>,
}

impl EventProducer {
    /// Enqueue an event from ISR context
    /// Returns false if queue is full
    pub fn enqueue(&mut self, event: Event) -> bool {
        self.producer.enqueue(event).is_ok()
    }
}

/// Consumer side of event queue (used by main loop)
pub struct EventConsumer {
    consumer: Consumer<'static, Event, QUEUE_SIZE>,
}

impl EventConsumer {
    /// Dequeue next event (non-blocking)
    pub fn dequeue(&mut self) -> Option<Event> {
        self.consumer.dequeue()
    }
}