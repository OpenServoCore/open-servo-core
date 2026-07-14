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
pub struct FakeBaud(Rc<RefCell<Vec<BaudRate>>>);

impl FakeBaud {
    pub fn applied(&self) -> Vec<BaudRate> {
        self.0.borrow().clone()
    }
}

impl traits::UsartBaud for FakeBaud {
    fn apply(&mut self, baud: BaudRate) {
        self.0.borrow_mut().push(baud);
    }
}

struct EdgeState {
    falls: Vec<u16>,
    rises: Vec<u16>,
    overflow: bool,
    resets: u32,
}

/// Preloadable edge-capture fake: tests stage ticks, drains pop in order.
#[derive(Clone, Default)]
pub struct FakeEdges(Rc<RefCell<Option<EdgeState>>>);

impl FakeEdges {
    fn state(&self) -> core::cell::RefMut<'_, EdgeState> {
        core::cell::RefMut::map(self.0.borrow_mut(), |s| {
            s.get_or_insert_with(|| EdgeState {
                falls: Vec::new(),
                rises: Vec::new(),
                overflow: false,
                resets: 0,
            })
        })
    }

    pub fn stage(&self, falls: &[u16], rises: &[u16]) {
        let mut s = self.state();
        s.falls.extend_from_slice(falls);
        s.rises.extend_from_slice(rises);
    }

    pub fn set_overflow(&self) {
        self.state().overflow = true;
    }

    pub fn resets(&self) -> u32 {
        self.state().resets
    }
}

impl traits::EdgeCapture for FakeEdges {
    fn drain_falls(&mut self, buf: &mut [u16]) -> usize {
        let mut s = self.state();
        let n = buf.len().min(s.falls.len());
        buf[..n].copy_from_slice(&s.falls[..n]);
        s.falls.drain(..n);
        n
    }

    fn drain_rises(&mut self, buf: &mut [u16]) -> usize {
        let mut s = self.state();
        let n = buf.len().min(s.rises.len());
        buf[..n].copy_from_slice(&s.rises[..n]);
        s.rises.drain(..n);
        n
    }

    fn overflow(&self) -> bool {
        self.state().overflow
    }

    fn reset(&mut self) {
        let mut s = self.state();
        s.falls.clear();
        s.rises.clear();
        s.overflow = false;
        s.resets += 1;
    }
}

pub struct TestProviders;
impl traits::Providers for TestProviders {
    type Ring = FakeRing;
    type Deadline = FakeDeadline;
    type Tx = FakeWire;
    type Baud = FakeBaud;
    type Edges = FakeEdges;
}

pub fn sealed_status(id: u8, result: ResultCode, payload: &[u8]) -> Vec<u8> {
    let mut b = FrameBuf::<264>::new();
    b.start(Id::new(id), Inst::status(result, false));
    b.payload_mut()[..payload.len()].copy_from_slice(payload);
    b.finish(payload.len() as u8);
    b.seal().to_vec()
}
