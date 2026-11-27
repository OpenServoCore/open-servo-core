//! Hand-rolled fake providers for the `ServoBus` composite. Not mockall:
//! `RxRing::bytes(&self) -> &[u8]` and the stateful cursor fight mockall's
//! lifetime model, so these follow the state-companion spirit -- cloneable
//! `Rc` handles the test configures and inspects, one of which is moved into
//! the driver.

use core::cell::{Cell, UnsafeCell};
use std::boxed::Box;
use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;
use std::vec::Vec;

use osc_protocol::crc::osc_crc_continue;
use osc_servo_core::BaudRate;

use crate::bus::ServoBus;
use crate::bus::snapshot::{Beat, Buf, gather};
use crate::traits::bus::{BreakStamps, CrcEngine, Deadline, Providers, RxRing, TxWire, UsartBaud};

/// Ring length -- even and larger than `FRAME_MAX` (matches the V006 512 B ring).
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
    const CLOCK_TRIM_STEP_PPM: u32 = 2500;

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

/// Software osc-CRC accumulator with an immediate result -- asserts even feeds
/// (F12) exactly like the `tx` engine tests.
pub struct FakeCrc {
    state: u16,
    snap: Box<Buf>,
    sick: bool,
    copies: usize,
    probe: Option<Box<dyn FnMut(*mut u8, *const u8)>>,
}

impl FakeCrc {
    pub fn new() -> Self {
        FakeCrc {
            state: 0,
            snap: Box::new(Buf::new()),
            sick: false,
            copies: 0,
            probe: None,
        }
    }

    /// `copy` calls so far.
    pub fn copies(&self) -> usize {
        self.copies
    }

    /// An engine whose result never comes.
    pub fn sick() -> Self {
        FakeCrc {
            sick: true,
            ..Self::new()
        }
    }

    /// An engine that hands `probe` each copy's ends at its arm, before
    /// the copy lands.
    pub fn probed(probe: impl FnMut(*mut u8, *const u8) + 'static) -> Self {
        FakeCrc {
            probe: Some(Box::new(probe)),
            ..Self::new()
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

    unsafe fn snapshot(&mut self, spans: &[&[u8]]) -> *mut u8 {
        // SAFETY: the caller's total.
        unsafe {
            gather(&mut *self.snap, spans, |s, d, beat, beats| {
                aligned_copy(s, d, beat, beats)
            })
        }
    }

    unsafe fn copy(&mut self, dst: *mut u8, src: *const u8, units: u16) {
        assert_eq!((dst as usize | src as usize) & 1, 0, "halfword ends");
        self.copies += 1;
        if let Some(p) = self.probe.as_mut() {
            p(dst, src);
        }
        // SAFETY: the caller's contract.
        unsafe { core::ptr::copy_nonoverlapping(src, dst, 2 * units as usize) };
    }

    fn result(&mut self) -> Option<u16> {
        (!self.sick).then_some(self.state)
    }
}

/// The copy channel's arm, landed at once, refusing the unaligned beat the
/// silicon would silently misplace (RM sec 8.3.5).
///
/// # Safety
/// `src` readable and `dst` writable for `beats` beats of `beat`.
pub unsafe fn aligned_copy(src: *const u8, dst: *mut u8, beat: Beat, beats: u16) {
    let w = beat as usize;
    assert_eq!((src.addr() | dst.addr()) % w, 0, "unaligned {w}-byte beat");
    // SAFETY: the caller's.
    unsafe { core::ptr::copy_nonoverlapping(src, dst, w * beats as usize) };
}

/// One recorded TX-wire operation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum WireEvent {
    Start,
    Send(Vec<u8>),
    Release,
    /// A TEL lane break, logged when the scenario lets it finish.
    Break,
    /// A TEL lane image, read whole when the scenario lets it finish.
    Stream(Vec<u8>),
}

#[derive(Default)]
struct WireState {
    log: RefCell<Vec<WireEvent>>,
    /// A lane break or image the scenario has not let finish.
    busy: Cell<Option<(*const u8, usize)>>,
}

/// Half-duplex TX wire recording break/arm/release into a shared log. The
/// TEL lane's break and image stay on the wire until [`Self::finish`].
#[derive(Clone, Default)]
pub struct FakeWire(Rc<WireState>);

impl FakeWire {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn log(&self) -> Vec<WireEvent> {
        self.0.log.borrow().clone()
    }

    pub fn started(&self) -> bool {
        self.0.log.borrow().contains(&WireEvent::Start)
    }

    /// Concatenated `Send` bytes -- the reply frame minus its `0x00` prefix.
    pub fn sent(&self) -> Vec<u8> {
        let mut out = Vec::new();
        for e in self.0.log.borrow().iter() {
            if let WireEvent::Send(v) = e {
                out.extend_from_slice(v);
            }
        }
        out
    }

    /// The lane's break or image leaves the wire now; an image is read at
    /// this instant, as late as the DMA could read its last byte.
    pub fn finish(&self) {
        let Some((ptr, len)) = self.0.busy.take() else {
            return;
        };
        let e = if len == 0 {
            WireEvent::Break
        } else {
            // SAFETY: test-only: the image is a leaked channel's, live for
            // the program.
            WireEvent::Stream(unsafe { core::slice::from_raw_parts(ptr, len) }.to_vec())
        };
        self.0.log.borrow_mut().push(e);
    }

    /// The lane's break is on the wire.
    pub fn breaking(&self) -> bool {
        matches!(self.0.busy.get(), Some((_, 0)))
    }

    /// The length of the lane image on the wire.
    pub fn streaming(&self) -> Option<usize> {
        self.0
            .busy
            .get()
            .map(|(_, len)| len)
            .filter(|&len| len != 0)
    }

    /// Every lane image finished so far, in order.
    pub fn streamed(&self) -> Vec<Vec<u8>> {
        self.0
            .log
            .borrow()
            .iter()
            .filter_map(|e| match e {
                WireEvent::Stream(v) => Some(v.clone()),
                _ => None,
            })
            .collect()
    }
}

impl TxWire for FakeWire {
    fn start_frame(&mut self) {
        self.0.log.borrow_mut().push(WireEvent::Start);
    }

    fn send(&mut self, span: &[u8]) {
        self.0.log.borrow_mut().push(WireEvent::Send(span.to_vec()));
    }

    fn release(&mut self) {
        self.0.log.borrow_mut().push(WireEvent::Release);
    }

    fn idle(&self) -> bool {
        self.0.busy.get().is_none()
    }

    fn break_start(&mut self) {
        self.0.busy.set(Some((core::ptr::null(), 0)));
    }

    fn stream(&mut self, image: &[u8]) {
        self.0.busy.set(Some((image.as_ptr(), image.len())));
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

/// Break stamps a scenario latches ahead of the wakes that read them: times
/// while the bus has the latch on the clock, ring positions otherwise.
#[derive(Clone, Default)]
pub struct FakeStamps(Rc<RefCell<StampLatch>>);

#[derive(Default)]
pub struct StampLatch {
    times: VecDeque<u16>,
    newest: Option<u16>,
    timing: bool,
}

impl FakeStamps {
    /// One break: `time` on the time ruler, `position` on the ring's.
    pub fn latch(&self, time: u16, position: u16) {
        let mut l = self.0.borrow_mut();
        if l.timing {
            l.times.push_back(time);
        } else {
            l.newest = Some(position);
        }
    }
}

impl BreakStamps for FakeStamps {
    fn newest(&mut self) -> Option<u16> {
        self.0.borrow().newest
    }

    fn take(&mut self) -> Option<u16> {
        self.0.borrow_mut().times.pop_front()
    }

    fn time(&mut self) {
        let mut l = self.0.borrow_mut();
        l.timing = true;
        l.times.clear();
    }

    fn position(&mut self) {
        let mut l = self.0.borrow_mut();
        l.timing = false;
        l.newest = None;
    }
}

/// ZST binding each role to its fake (driver-pattern sec 5.4).
pub struct TestProviders;

impl Providers for TestProviders {
    type Ring = FakeRing;
    type Deadline = FakeDeadline;
    type Crc = FakeCrc;
    type Tx = FakeWire;
    type Baud = FakeBaud;
    type Stamps = FakeStamps;
}

/// Owns the shared fake state and builds a `ServoBus` over it.
pub struct Harness {
    pub ring: FakeRing,
    pub deadline: FakeDeadline,
    pub wire: FakeWire,
    pub baud: FakeBaud,
    pub stamps: FakeStamps,
}

impl Harness {
    pub fn new() -> Self {
        Harness {
            ring: FakeRing::new(),
            deadline: FakeDeadline::new(),
            wire: FakeWire::new(),
            baud: FakeBaud::new(),
            stamps: FakeStamps::default(),
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
            self.stamps.clone(),
            id,
            rate,
            response_deadline_us,
        )
    }
}

impl Default for Harness {
    fn default() -> Self {
        Self::new()
    }
}
