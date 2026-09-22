//! Recording fake providers shared by the engine and link test suites
//! (the drivers-crate mock spirit: cloneable `Rc` state companions, one
//! clone moved into the engine).

use std::cell::{Cell, RefCell, UnsafeCell};
use std::rc::Rc;
use std::vec::Vec;

use osc_protocol::reply::FrameBuf;
use osc_protocol::wire::{BaudRate, Id, Inst, ResultCode};

use crate::traits;

pub const RING_LEN: usize = 512;

struct RingState {
    buf: UnsafeCell<[u8; RING_LEN]>,
    cursor: Cell<u16>,
}

/// Counted RX ring the test feeds like the wire would.
#[derive(Clone)]
pub struct FakeRing(Rc<RingState>);

impl FakeRing {
    pub fn new() -> Self {
        FakeRing(Rc::new(RingState {
            buf: UnsafeCell::new([0xFF; RING_LEN]),
            cursor: Cell::new(0),
        }))
    }

    /// Ring bytes in at the cursor, advancing it -- one wire arrival.
    pub fn feed(&self, bytes: &[u8]) {
        // SAFETY: test-only, single-threaded; never called while a
        // `bytes()` slice is live.
        let buf = unsafe { &mut *self.0.buf.get() };
        let mut c = self.0.cursor.get() as usize;
        for &b in bytes {
            buf[c] = b;
            c = (c + 1) % RING_LEN;
        }
        self.0.cursor.set(c as u16);
    }
}

impl traits::RxRing for FakeRing {
    fn bytes(&self) -> &[u8] {
        // SAFETY: test-only aliasing; the buffer is never fed while a
        // returned slice is live (see `feed`).
        let arr: &[u8; RING_LEN] = unsafe { &*self.0.buf.get() };
        &arr[..]
    }

    fn cursor(&self) -> u16 {
        self.0.cursor.get()
    }
}

struct ClockState {
    now: Cell<u32>,
    armed: Cell<Option<u32>>,
}

/// Settable clock + a record of the armed compare.
#[derive(Clone)]
pub struct FakeDeadline(Rc<ClockState>);

impl FakeDeadline {
    pub fn new() -> Self {
        FakeDeadline(Rc::new(ClockState {
            now: Cell::new(0),
            armed: Cell::new(None),
        }))
    }

    pub fn advance(&self, ticks: u32) {
        self.0.now.set(self.0.now.get().wrapping_add(ticks));
    }

    pub fn armed(&self) -> Option<u32> {
        self.0.armed.get()
    }
}

impl traits::Deadline for FakeDeadline {
    const TICKS_PER_US: u32 = 1;

    fn now(&self) -> u32 {
        self.0.now.get()
    }

    fn set(&mut self, at: u32) {
        self.0.armed.set(Some(at));
    }

    fn cancel(&mut self) {
        self.0.armed.set(None);
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireOp {
    Claim,
    Break,
    Send(Vec<u8>),
    HoldLow,
    Release,
}

#[derive(Clone, Default)]
pub struct FakeWire(Rc<RefCell<Vec<WireOp>>>);

impl FakeWire {
    pub fn log(&self) -> Vec<WireOp> {
        self.0.borrow().clone()
    }

    pub fn clear(&self) {
        self.0.borrow_mut().clear();
    }
}

impl traits::TxWire for FakeWire {
    fn claim(&mut self) {
        self.0.borrow_mut().push(WireOp::Claim);
    }
    fn send_break(&mut self) {
        self.0.borrow_mut().push(WireOp::Break);
    }
    fn send(&mut self, span: &[u8]) {
        self.0.borrow_mut().push(WireOp::Send(span.to_vec()));
    }
    fn hold_low(&mut self) {
        self.0.borrow_mut().push(WireOp::HoldLow);
    }
    fn release(&mut self) {
        self.0.borrow_mut().push(WireOp::Release);
    }
}

#[derive(Clone, Default)]
pub struct FakeBaud {
    applied: Rc<RefCell<Vec<BaudRate>>>,
    #[cfg(feature = "bench")]
    raw: Rc<RefCell<Vec<u32>>>,
}

impl FakeBaud {
    pub fn applied(&self) -> Vec<BaudRate> {
        self.applied.borrow().clone()
    }
}

impl traits::UsartBaud for FakeBaud {
    fn apply(&mut self, baud: BaudRate) {
        self.applied.borrow_mut().push(baud);
    }

    #[cfg(feature = "bench")]
    fn apply_raw(&mut self, bps: u32) {
        self.raw.borrow_mut().push(bps);
    }
}

pub struct TestProviders;
impl traits::Providers for TestProviders {
    type Ring = FakeRing;
    type Deadline = FakeDeadline;
    type Tx = FakeWire;
    type Baud = FakeBaud;
    #[cfg(feature = "bench")]
    type Edges = bench::FakeEdges;
}

pub fn sealed_status(id: u8, result: ResultCode, payload: &[u8]) -> Vec<u8> {
    let mut b = FrameBuf::<264>::new();
    b.start(Id::new(id), Inst::status(result, false));
    b.payload_mut()[..payload.len()].copy_from_slice(payload);
    b.finish(payload.len() as u8);
    b.seal().to_vec()
}

/// The instrument side of the fakes.
#[cfg(feature = "bench")]
pub mod bench {
    use std::vec::Vec;

    use super::{FakeBaud, traits};

    impl FakeBaud {
        pub fn applied_raw(&self) -> Vec<u32> {
            self.raw.borrow().clone()
        }
    }

    /// Preloadable edge-capture fake, reached through `HostBus::edges`:
    /// tests stage ticks, drains pop in order.
    #[derive(Default)]
    pub struct FakeEdges {
        falls: Vec<u16>,
        rises: Vec<u16>,
        overflow: bool,
        resets: u32,
    }

    impl FakeEdges {
        pub fn stage(&mut self, falls: &[u16], rises: &[u16]) {
            self.falls.extend_from_slice(falls);
            self.rises.extend_from_slice(rises);
        }

        pub fn set_overflow(&mut self) {
            self.overflow = true;
        }

        pub fn resets(&self) -> u32 {
            self.resets
        }
    }

    impl traits::EdgeCapture for FakeEdges {
        fn drain_falls(&mut self, buf: &mut [u16]) -> usize {
            let n = buf.len().min(self.falls.len());
            buf[..n].copy_from_slice(&self.falls[..n]);
            self.falls.drain(..n);
            n
        }

        fn drain_rises(&mut self, buf: &mut [u16]) -> usize {
            let n = buf.len().min(self.rises.len());
            buf[..n].copy_from_slice(&self.rises[..n]);
            self.rises.drain(..n);
            n
        }

        fn overflow(&self) -> bool {
            self.overflow
        }

        fn reset(&mut self) {
            self.falls.clear();
            self.rises.clear();
            self.overflow = false;
            self.resets += 1;
        }
    }
}

/// Link-server rig: the production server over the engine on the fakes.
pub mod link {
    use std::vec;
    use std::vec::Vec;

    use osc_protocol::wire::{BaudRate, Inst, Opcode};

    use crate::engine::HostBus;
    use crate::link::record::{REC_SUBMIT, VERB_EXCHANGE};
    use crate::link::{LinkServer, RecordSink};

    use super::{FakeBaud, FakeDeadline, FakeRing, FakeWire, TestProviders};

    #[derive(Default)]
    pub struct Sink(pub Vec<Vec<u8>>);

    impl RecordSink for Sink {
        fn record(&mut self, record: &[u8]) {
            self.0.push(record.to_vec());
        }
    }

    pub struct Rig {
        pub server: LinkServer,
        pub bus: HostBus<TestProviders>,
        pub ring: FakeRing,
        pub clock: FakeDeadline,
        pub wire: FakeWire,
        #[cfg(feature = "bench")]
        pub baud: FakeBaud,
        pub sink: Sink,
    }

    pub fn rig() -> Rig {
        let ring = FakeRing::new();
        let clock = FakeDeadline::new();
        let wire = FakeWire::default();
        let baud = FakeBaud::default();
        let bus = HostBus::new(
            ring.clone(),
            clock.clone(),
            wire.clone(),
            baud.clone(),
            BaudRate::B1000000,
        );
        Rig {
            server: LinkServer::new(),
            bus,
            ring,
            clock,
            wire,
            #[cfg(feature = "bench")]
            baud,
            sink: Sink::default(),
        }
    }

    /// Length-prefix a type+body into pipe bytes.
    pub fn rec(body: &[u8]) -> Vec<u8> {
        let mut v = vec![body.len() as u8, (body.len() >> 8) as u8];
        v.extend_from_slice(body);
        v
    }

    pub fn submit_ping(seq: u16, id: u8) -> Vec<u8> {
        let inst = Inst::instruction(Opcode::Ping, 0);
        rec(&[
            REC_SUBMIT,
            seq as u8,
            (seq >> 8) as u8,
            VERB_EXCHANGE,
            id,
            inst.0,
        ])
    }
}
