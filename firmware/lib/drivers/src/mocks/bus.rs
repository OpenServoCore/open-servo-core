//! Hand-rolled fake providers for the `ServoBus` composite. Not mockall:
//! `RxRing::bytes(&self) -> &[u8]` and the stateful cursor fight mockall's
//! lifetime model, so these follow the state-companion spirit — cloneable
//! `Rc` handles the test configures and inspects, one of which is moved into
//! the driver.

use core::cell::{Cell, UnsafeCell};
use std::boxed::Box;
use std::cell::RefCell;
use std::rc::Rc;
use std::vec::Vec;

use osc_core::BaudRate;
use osc_core::traits::Dispatch;
use osc_protocol::crc::osc_crc_continue;

use crate::bus::{DispatchConsumer, Handoff, ServoBus};
use crate::traits::bus::{
    CrcEngine, Deadline, DispatchWake, Lane, LineSense, Providers, RxRing, SequenceWake, TxWire,
    UsartBaud,
};

/// Ring length — even and larger than `FRAME_MAX` (matches the V006 512 B ring).
pub const RING_LEN: usize = 512;

#[repr(align(2))]
struct RingBuf([u8; RING_LEN]);

struct RingState {
    buf: UnsafeCell<RingBuf>,
    cursor: Cell<u16>,
}

/// Counted RX ring backed by a halfword-aligned buffer.
#[derive(Clone)]
pub struct FakeRing(Rc<RingState>);

impl FakeRing {
    pub fn new() -> Self {
        FakeRing(Rc::new(RingState {
            buf: UnsafeCell::new(RingBuf([0; RING_LEN])),
            cursor: Cell::new(0),
        }))
    }

    /// Lay bytes into the ring at `at` (wrapping), before any event fires.
    pub fn place(&self, at: usize, bytes: &[u8]) {
        // SAFETY: test-only, single-threaded; scenarios never call `place`
        // while a `bytes()` slice is live.
        let buf = unsafe { &mut (*self.0.buf.get()).0 };
        for (i, &b) in bytes.iter().enumerate() {
            buf[(at + i) % RING_LEN] = b;
        }
    }

    pub fn set_cursor(&self, c: u16) {
        self.0.cursor.set(c);
    }
}

impl Default for FakeRing {
    fn default() -> Self {
        Self::new()
    }
}

impl RxRing for FakeRing {
    fn bytes(&self) -> &[u8] {
        // SAFETY: test-only aliasing; the buffer is never mutated while a
        // returned slice is live (see `place`).
        let arr: &[u8; RING_LEN] = unsafe { &(*self.0.buf.get()).0 };
        &arr[..]
    }

    fn cursor(&self) -> u16 {
        self.0.cursor.get()
    }
}

struct DeadlineState {
    now: Cell<u32>,
    armed: Cell<Option<u32>>,
    cancels: Cell<u32>,
}

/// One-shot compare with a settable `now` and a record of the armed tick.
#[derive(Clone)]
pub struct FakeDeadline(Rc<DeadlineState>);

impl FakeDeadline {
    pub fn new() -> Self {
        FakeDeadline(Rc::new(DeadlineState {
            now: Cell::new(0),
            armed: Cell::new(None),
            cancels: Cell::new(0),
        }))
    }

    pub fn set_now(&self, n: u32) {
        self.0.now.set(n);
    }

    pub fn armed(&self) -> Option<u32> {
        self.0.armed.get()
    }

    pub fn cancels(&self) -> u32 {
        self.0.cancels.get()
    }
}

impl Default for FakeDeadline {
    fn default() -> Self {
        Self::new()
    }
}

impl Deadline for FakeDeadline {
    const TICKS_PER_US: u32 = 1;

    fn now(&self) -> u32 {
        self.0.now.get()
    }

    fn set(&mut self, at: u32) {
        self.0.armed.set(Some(at));
    }

    fn cancel(&mut self) {
        self.0.armed.set(None);
        self.0.cancels.set(self.0.cancels.get() + 1);
    }
}

/// Software osc-CRC accumulator with an immediate result — asserts even feeds
/// (F12) exactly like the `tx` engine tests.
pub struct FakeCrc {
    state: u16,
    snap: Vec<u8>,
}

impl FakeCrc {
    pub fn new() -> Self {
        FakeCrc {
            state: 0,
            snap: Vec::new(),
        }
    }
}

impl Default for FakeCrc {
    fn default() -> Self {
        Self::new()
    }
}

impl CrcEngine for FakeCrc {
    fn reset(&mut self) {
        self.state = 0;
    }

    fn feed(&mut self, span: &[u8]) {
        assert_eq!(span.len() % 2, 0, "CRC feed must be even-length (F12)");
        self.state = osc_crc_continue(self.state, span);
    }

    fn snapshot(&mut self, off: u16, src: &[u8]) -> *const u8 {
        let off = off as usize;
        if self.snap.len() < off + src.len() {
            self.snap.resize(off + src.len(), 0);
        }
        self.snap[off..off + src.len()].copy_from_slice(src);
        // SAFETY-adjacent note: tests never grow the Vec between a snapshot
        // and its consumption, so the pointer stays stable like the chip's
        // static buffer.
        unsafe { self.snap.as_ptr().add(off) }
    }

    fn result(&mut self) -> Option<u16> {
        Some(self.state)
    }
}

/// One recorded TX-wire operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireEvent {
    Start,
    Send(Vec<u8>),
    Release,
}

/// Half-duplex TX wire recording break/arm/release into a shared log.
#[derive(Clone, Default)]
pub struct FakeWire(Rc<RefCell<Vec<WireEvent>>>);

impl FakeWire {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn log(&self) -> Vec<WireEvent> {
        self.0.borrow().clone()
    }

    pub fn started(&self) -> bool {
        self.0.borrow().contains(&WireEvent::Start)
    }

    /// Concatenated `Send` bytes — the reply frame minus its `0x00` prefix.
    pub fn sent(&self) -> Vec<u8> {
        let mut out = Vec::new();
        for e in self.0.borrow().iter() {
            if let WireEvent::Send(v) = e {
                out.extend_from_slice(v);
            }
        }
        out
    }
}

impl TxWire for FakeWire {
    fn start_frame(&mut self) {
        self.0.borrow_mut().push(WireEvent::Start);
    }

    fn send(&mut self, span: &[u8]) {
        self.0.borrow_mut().push(WireEvent::Send(span.to_vec()));
    }

    fn release(&mut self) {
        self.0.borrow_mut().push(WireEvent::Release);
    }
}

/// USART baud control recording each applied rate.
#[derive(Clone, Default)]
pub struct FakeBaud(Rc<RefCell<Vec<BaudRate>>>);

impl FakeBaud {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn applied(&self) -> Vec<BaudRate> {
        self.0.borrow().clone()
    }
}

impl UsartBaud for FakeBaud {
    fn apply(&mut self, baud: BaudRate) {
        self.0.borrow_mut().push(baud);
    }
}

/// Settable bus-line level for rescue-break confirmation.
#[derive(Clone, Default)]
pub struct FakeLine(Rc<Cell<bool>>);

impl FakeLine {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_low(&self, low: bool) {
        self.0.set(low);
    }
}

impl LineSense for FakeLine {
    fn is_low(&self) -> bool {
        self.0.get()
    }
}

/// Records each consumer wake's lane — the interleave-policy observable.
#[derive(Clone, Default)]
pub struct FakeDispatchWake(Rc<RefCell<Vec<Lane>>>);

impl FakeDispatchWake {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn lanes(&self) -> Vec<Lane> {
        self.0.borrow().clone()
    }
}

impl DispatchWake for FakeDispatchWake {
    fn job_ready(&mut self, lane: Lane) {
        self.0.borrow_mut().push(lane);
    }
}

/// Consumer completion wake — the harness pumps adoption synchronously, so
/// this only records that a wake was requested.
#[derive(Clone, Default)]
pub struct FakeSequenceWake(Rc<Cell<bool>>);

impl SequenceWake for FakeSequenceWake {
    fn reply_ready(&mut self) {
        self.0.set(true);
    }
}

/// ZST binding each role to its fake (driver-pattern §5.4).
pub struct TestProviders;

impl Providers for TestProviders {
    type Ring = FakeRing;
    type Deadline = FakeDeadline;
    type Crc = FakeCrc;
    type Tx = FakeWire;
    type Baud = FakeBaud;
    type Line = FakeLine;
    type Wake = FakeDispatchWake;
}

/// Owns the shared fake state and builds a `ServoBus` over it.
pub struct Harness {
    pub ring: FakeRing,
    pub deadline: FakeDeadline,
    pub wire: FakeWire,
    pub baud: FakeBaud,
    pub line: FakeLine,
    pub wake: FakeDispatchWake,
    pub handoff: &'static Handoff,
}

impl Harness {
    pub fn new() -> Self {
        Harness {
            ring: FakeRing::new(),
            deadline: FakeDeadline::new(),
            wire: FakeWire::new(),
            baud: FakeBaud::new(),
            line: FakeLine::new(),
            wake: FakeDispatchWake::new(),
            // Leaked per harness: the cell is 'static on the chip; tests get
            // one fresh slot each.
            handoff: Box::leak(Box::new(Handoff::new())),
        }
    }

    pub fn build(
        &self,
        id: u8,
        rate: BaudRate,
        response_deadline_us: u16,
    ) -> ServoBus<TestProviders> {
        ServoBus::new(
            self.ring.clone(),
            self.deadline.clone(),
            FakeCrc::new(),
            self.wire.clone(),
            self.baud.clone(),
            self.line.clone(),
            self.wake.clone(),
            self.handoff,
            id,
            rate,
            response_deadline_us,
        )
    }

    /// Run the LOW consumer over the outstanding job (if any), then deliver
    /// the adoption wake — the synchronous equivalent of the chip's LOW
    /// vector followed by the pended HIGH re-entry.
    pub fn pump<D: Dispatch>(&self, bus: &mut ServoBus<TestProviders>, d: &mut D) {
        let mut consumer =
            DispatchConsumer::new(self.handoff, self.ring.clone(), FakeSequenceWake::default());
        if consumer.process(d) {
            bus.on_deadline(d);
        }
    }
}

impl Default for Harness {
    fn default() -> Self {
        Self::new()
    }
}
