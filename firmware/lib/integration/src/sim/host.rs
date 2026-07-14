//! Host-in-the-loop: the production `osc_host` engine over sim providers,
//! closing both columns of the grid inside the DES -- production host
//! scheduling against the production servo stack on one modeled wire.
//!
//! The provider shapes mirror `providers.rs` (shared `Rc` state the `Sim`
//! also holds): the host ring is fed by the same wire deliveries the servo
//! rings get (minus the host's own TX -- the no-echo contract), the
//! deadline schedules `HostCompare` events, and the TX wire schedules the
//! same `WireBreak`/`WireData` events a scripted `host_send` would.

use std::cell::Cell;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;

use osc_host::engine::{HostBus, Terminal};
use osc_host::traits::{Deadline, EdgeCapture, Providers, RxRing, TxWire, UsartBaud};
use osc_protocol::wire::BaudRate;
use osc_servo_drivers::bus::RESCUE_LOW_US;

use super::core::{Core, Event, TICKS_PER_US, Talker, break_ticks, byte_ticks};
use super::providers::{BaudState, DeadlineState, RingState};

/// One engine event, copied out of the ring for the harness.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum HostEvent {
    Status {
        slot: u8,
        id: u8,
        inst: u8,
        payload: Vec<u8>,
    },
    Done(Terminal),
    /// An instrument wire op closed (raw send / burst / pulse).
    WireDone {
        tick: u32,
    },
}

pub struct HostRing(pub Rc<RingState>);

impl RxRing for HostRing {
    fn bytes(&self) -> &[u8] {
        self.0.bytes()
    }

    fn cursor(&self) -> u16 {
        self.0.cursor()
    }
}

/// Crystal-clocked host deadline: the sim clock IS the host clock (no
/// skew -- the host is the syntonization root).
pub struct HostDeadline {
    core: Rc<RefCell<Core>>,
    state: Rc<DeadlineState>,
}

impl Deadline for HostDeadline {
    const TICKS_PER_US: u32 = TICKS_PER_US as u32;

    fn now(&self) -> u32 {
        self.core.borrow().now() as u32
    }

    fn set(&mut self, at: u32) {
        let generation = self.state.bump();
        let sim_now = self.core.borrow().now();
        let delta = at.wrapping_sub(sim_now as u32);
        // A past `at` fires immediately (pend-on-past, the chip contract).
        let delta = if delta > i32::MAX as u32 {
            0
        } else {
            delta as u64
        };
        self.core
            .borrow_mut()
            .schedule(Event::HostCompare { generation }, sim_now + delta);
    }

    fn cancel(&mut self) {
        self.state.bump();
    }
}

/// The host's TX onto the shared wire. Break/byte events mirror the
/// scripted `host_send` path; the frame record finalizes off scheduled
/// `HostFrameEnd` events so bare CAL-train breaks and full frames both
/// record like their scripted twins.
pub struct HostWire {
    core: Rc<RefCell<Core>>,
    baud: Rc<BaudState>,
    /// End tick of the last-scheduled unit; the next unit starts at
    /// `max(cursor, now)`.
    tx_cursor: Cell<u64>,
    /// A sent break not yet claimed by a frame body: flushed as an empty
    /// recorded frame by the next unit (CAL train marks).
    bare_break: Cell<Option<u64>>,
    /// Rescue pulse start while the line is held dominant.
    low_since: Cell<Option<u64>>,
}

impl TxWire for HostWire {
    fn claim(&mut self) {
        let now = self.core.borrow().now();
        self.tx_cursor.set(self.tx_cursor.get().max(now));
    }

    fn send_break(&mut self) {
        let baud = self.baud.current();
        let mut c = self.core.borrow_mut();
        let now = c.now();
        // Flush a preceding bare break's record before its successor begins.
        if let Some(end) = self.bare_break.take() {
            c.schedule(Event::HostFrameEnd, end.max(now));
        }
        let start = self.tx_cursor.get().max(now);
        let end = start + break_ticks(baud);
        c.claim(Talker::Host, start, end);
        c.schedule(
            Event::WireBreak {
                talker: Talker::Host,
                baud,
                break_start: start,
            },
            end,
        );
        self.tx_cursor.set(end);
        self.bare_break.set(Some(end));
    }

    fn send(&mut self, span: &[u8]) {
        // The break heads this frame -- one record, no bare flush.
        self.bare_break.take();
        let baud = self.baud.current();
        let bt = byte_ticks(baud);
        let mut c = self.core.borrow_mut();
        let start = self.tx_cursor.get().max(c.now());
        for (k, &b) in span.iter().enumerate() {
            c.schedule(
                Event::WireData {
                    talker: Talker::Host,
                    byte: b,
                    baud,
                },
                start + (k as u64 + 1) * bt,
            );
        }
        let end = start + span.len() as u64 * bt;
        c.claim(Talker::Host, start, end);
        c.schedule(Event::HostFrameEnd, end);
        c.schedule(Event::HostTxDone, end);
        self.tx_cursor.set(end);
    }

    fn hold_low(&mut self) {
        let baud = self.baud.current();
        let mut c = self.core.borrow_mut();
        let now = c.now();
        self.low_since.set(Some(now));
        // The engine holds well past the sampler threshold by contract; the
        // fleet-wide declaration lands mid-pulse (sec 9.1).
        c.schedule(
            Event::RescueDeclare,
            now + byte_ticks(baud) + RESCUE_LOW_US as u64 * TICKS_PER_US,
        );
    }

    fn release(&mut self) {
        let mut c = self.core.borrow_mut();
        let now = c.now();
        if let Some(end) = self.bare_break.take() {
            // Last CAL-train break: its record closes after delivery.
            c.schedule(Event::HostFrameEnd, end.max(now));
        }
        if let Some(start) = self.low_since.take() {
            // Pulse end: the rising edge is where break detectors latch.
            c.claim(Talker::Host, start, now);
            let baud = self.baud.current();
            c.schedule(Event::StrayBreak { baud }, now);
        }
    }
}

pub struct HostUsart(pub Rc<BaudState>);

impl UsartBaud for HostUsart {
    fn apply(&mut self, baud: BaudRate) {
        self.0.apply(baud);
    }
}

/// Edge capture has no sim model (hardware-stamped pin transitions are
/// silicon-only by nature): drains answer empty, honestly.
pub struct HostEdges;

impl EdgeCapture for HostEdges {
    fn drain_falls(&mut self, _buf: &mut [u16]) -> usize {
        0
    }
    fn drain_rises(&mut self, _buf: &mut [u16]) -> usize {
        0
    }
    fn overflow(&self) -> bool {
        false
    }
    fn reset(&mut self) {}
}

/// Provider bundle for the sim-hosted engine.
pub struct SimHostProviders;

impl Providers for SimHostProviders {
    type Ring = HostRing;
    type Deadline = HostDeadline;
    type Tx = HostWire;
    type Baud = HostUsart;
    type Edges = HostEdges;
}

/// The attached host: the production engine plus the shared handles the
/// `Sim` feeds during wire delivery.
pub struct SimHost {
    pub bus: HostBus<SimHostProviders>,
    pub ring: Rc<RingState>,
    pub deadline: Rc<DeadlineState>,
    pub baud: Rc<BaudState>,
    pub events: Vec<HostEvent>,
}

impl SimHost {
    pub fn build(core: &Rc<RefCell<Core>>, rate: BaudRate) -> Self {
        let ring = RingState::new();
        let deadline = DeadlineState::new();
        let baud = BaudState::new(rate);
        let bus = HostBus::new(
            HostRing(ring.clone()),
            HostDeadline {
                core: core.clone(),
                state: deadline.clone(),
            },
            HostWire {
                core: core.clone(),
                baud: baud.clone(),
                tx_cursor: Cell::new(0),
                bare_break: Cell::new(None),
                low_since: Cell::new(None),
            },
            HostUsart(baud.clone()),
            HostEdges,
            rate,
        );
        Self {
            bus,
            ring,
            deadline,
            baud,
            events: Vec::new(),
        }
    }
}
