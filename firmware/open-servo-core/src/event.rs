use heapless::spsc::{Consumer, Producer, Queue};
use open_servo_hw::UartPort;
use super::fault::FaultKind;
use core::sync::atomic::{AtomicBool, Ordering};

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

/// Queue overflow flag - set when events are dropped
static QUEUE_OVERFLOW: AtomicBool = AtomicBool::new(false);

/// Event queue handle
pub struct EventQueue<const N: usize> {
    producer: Producer<'static, Event, N>,
    consumer: Consumer<'static, Event, N>,
}

impl<const N: usize> EventQueue<N> {
    /// Initialize the event queue from a static mutable queue
    /// 
    /// The queue must have 'static lifetime and this should only be called once
    /// before any interrupts are enabled.
    pub fn from_queue(queue: &'static mut Queue<Event, N>) -> Self {
        let (producer, consumer) = queue.split();
        EventQueue { producer, consumer }
    }

    /// Split into producer and consumer
    pub fn split(self) -> (EventProducer<N>, EventConsumer<N>) {
        (
            EventProducer { producer: self.producer },
            EventConsumer { consumer: self.consumer },
        )
    }
}

/// Producer side of event queue (used by ISRs)
pub struct EventProducer<const N: usize> {
    producer: Producer<'static, Event, N>,
}

impl<const N: usize> EventProducer<N> {
    /// Enqueue an event from ISR context
    /// Returns false if queue is full
    pub fn enqueue(&mut self, event: Event) -> bool {
        if self.producer.enqueue(event).is_err() {
            // Set overflow flag for main loop to detect
            QUEUE_OVERFLOW.store(true, Ordering::Relaxed);
            false
        } else {
            true
        }
    }
    
    /// Check and clear queue overflow flag
    /// Returns true if overflow occurred since last check
    pub fn check_overflow() -> bool {
        QUEUE_OVERFLOW.swap(false, Ordering::Relaxed)
    }
}

/// Consumer side of event queue (used by main loop)
pub struct EventConsumer<const N: usize> {
    consumer: Consumer<'static, Event, N>,
}

impl<const N: usize> EventConsumer<N> {
    /// Dequeue next event (non-blocking)
    pub fn dequeue(&mut self) -> Option<Event> {
        self.consumer.dequeue()
    }
}