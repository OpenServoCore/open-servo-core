//! The six `osc_servo_drivers::traits::bus` providers over per-servo shared state
//! (`docs/osc-native-protocol.md` sec 4, 10). Each provider is a thin view onto
//! `Rc`-shared cells the [`super::Sim`] also holds a handle to, so wire
//! deliveries and the driver see one ring, one clock, one baud.

use std::cell::{Cell, RefCell, UnsafeCell};
use std::rc::Rc;

use osc_protocol::crc::osc_crc_continue;
use osc_servo_core::BaudRate;
use osc_servo_drivers::traits::bus::{
    CrcEngine, Deadline, Providers, RxRing, TxWire, UsartBaud, tick_reached,
};

use super::core::{Core, Event, Talker, break_ticks, byte_ticks};

/// Ring length (sec 11): even and larger than the 258 B max frame, matching V006.
pub const RING_LEN: usize = 512;

// --- shared per-servo state -------------------------------------------------

#[repr(align(2))]
struct RingBuf([u8; RING_LEN]);

pub struct RingState {
    buf: UnsafeCell<RingBuf>,
    cursor: Cell<u16>,
}

impl RingState {
    pub fn new() -> Rc<Self> {
        Rc::new(Self {
            buf: UnsafeCell::new(RingBuf([0; RING_LEN])),
            cursor: Cell::new(0),
        })
    }

    /// Land one received byte at the cursor and advance it -- the DMA write the
    /// wire model performs before any `on_break` (the pinned provider contract:
    /// the ring byte precedes the ISR).
    pub fn push(&self, b: u8) {
        let i = self.cursor.get() as usize;
        // SAFETY: single-threaded harness; the Sim only pushes between driver
        // calls, never while a `bytes()` slice is live (see `RxRing::bytes`).
        unsafe { (*self.buf.get()).0[i] = b };
        self.cursor.set(((i + 1) % RING_LEN) as u16);
    }

    pub fn bytes(&self) -> &[u8] {
        // SAFETY: test-only aliasing (see `push`); the buffer is never
        // mutated while a returned slice is live.
        let arr: &[u8; RING_LEN] = unsafe { &(*self.buf.get()).0 };
        &arr[..]
    }

    pub fn cursor(&self) -> u16 {
        self.cursor.get()
    }
}

pub struct DeadlineState {
    /// Bumped on every set/cancel; a `Compare` event fires only if its `generation`
    /// still matches -- stale (superseded) compares are dropped.
    generation: Cell<u64>,
    armed: Cell<Option<u32>>,
    /// Piecewise-linear skewed clock (sec 9.3): `local = anchor_local +
    /// (sim - anchor_sim) * (1 + skew)`. Re-anchoring at a skew change keeps
    /// the local clock continuous -- real oscillators drift, they never step.
    skew_ppm: Cell<i32>,
    anchor_sim: Cell<u64>,
    anchor_local: Cell<u64>,
}

impl DeadlineState {
    pub fn new() -> Rc<Self> {
        Rc::new(Self {
            generation: Cell::new(0),
            armed: Cell::new(None),
            skew_ppm: Cell::new(0),
            anchor_sim: Cell::new(0),
            anchor_local: Cell::new(0),
        })
    }

    pub fn generation(&self) -> u64 {
        self.generation.get()
    }

    /// Supersede any armed compare; returns the new generation.
    pub fn bump(&self) -> u64 {
        let g = self.generation.get() + 1;
        self.generation.set(g);
        g
    }

    /// The servo's local clock at `sim_now`, unwrapped.
    fn local_u64(&self, sim_now: u64) -> u64 {
        let d = (sim_now - self.anchor_sim.get()) as i128;
        let scaled = d * (1_000_000 + self.skew_ppm.get() as i128) / 1_000_000;
        self.anchor_local.get() + scaled as u64
    }

    /// Injected drift: the rate changes at `sim_now`, the reading does not.
    pub fn set_skew(&self, sim_now: u64, ppm: i32) {
        self.anchor_local.set(self.local_u64(sim_now));
        self.anchor_sim.set(sim_now);
        self.skew_ppm.set(ppm);
    }
}

pub struct BaudState {
    applied: RefCell<Vec<BaudRate>>,
    current: Cell<BaudRate>,
}

impl BaudState {
    pub fn new(initial: BaudRate) -> Rc<Self> {
        Rc::new(Self {
            applied: RefCell::new(Vec::new()),
            current: Cell::new(initial),
        })
    }

    /// The servo's live operational baud -- the wire matches reception against
    /// it and the TX side times bytes from it.
    pub fn current(&self) -> BaudRate {
        self.current.get()
    }

    /// Applied-baud log -- a harness inspection hook for the rescue suite (sec 9.1
    /// verifies the volatile switch to the 0.5 M rescue rate).
    #[allow(dead_code)]
    pub fn applied(&self) -> Vec<BaudRate> {
        self.applied.borrow().clone()
    }

    pub fn apply(&self, baud: BaudRate) {
        self.applied.borrow_mut().push(baud);
        self.current.set(baud);
    }
}

/// Handles the Sim keeps to reach into one servo's state during delivery.
pub struct Handles {
    pub ring: Rc<RingState>,
    pub deadline: Rc<DeadlineState>,
    pub baud: Rc<BaudState>,
}

// --- providers --------------------------------------------------------------

pub struct SimRing(Rc<RingState>);

impl SimRing {
    pub fn new(state: Rc<RingState>) -> Self {
        Self(state)
    }
}

impl RxRing for SimRing {
    fn bytes(&self) -> &[u8] {
        // SAFETY: test-only aliasing (mirrors mocks::bus::FakeRing); the buffer
        // is never mutated while a returned slice is live -- the Sim pushes only
        // between driver calls.
        let arr: &[u8; RING_LEN] = unsafe { &(*self.0.buf.get()).0 };
        &arr[..]
    }

    fn cursor(&self) -> u16 {
        self.0.cursor.get()
    }
}

pub struct SimDeadline {
    core: Rc<RefCell<Core>>,
    state: Rc<DeadlineState>,
    idx: usize,
}

impl SimDeadline {
    pub fn new(
        core: Rc<RefCell<Core>>,
        state: Rc<DeadlineState>,
        idx: usize,
        skew_ppm: i32,
    ) -> Self {
        state.skew_ppm.set(skew_ppm);
        Self { core, state, idx }
    }
}

/// Invert the skew to find how much *sim* time a `delta` of skewed ticks spans.
fn unskew(delta: u64, skew_ppm: i32) -> u64 {
    if skew_ppm == 0 {
        return delta;
    }
    (delta as i128 * 1_000_000 / (1_000_000 + skew_ppm as i128)) as u64
}

impl Deadline for SimDeadline {
    const TICKS_PER_US: u32 = super::core::TICKS_PER_US as u32;
    const CLOCK_TRIM_STEP_PPM: u32 = 2500;

    fn now(&self) -> u32 {
        self.state.local_u64(self.core.borrow().now()) as u32
    }

    fn set(&mut self, at: u32) {
        let generation = self.state.generation.get() + 1;
        self.state.generation.set(generation);
        self.state.armed.set(Some(at));

        let sim_now = self.core.borrow().now();
        let now_sk = self.state.local_u64(sim_now) as u32;
        // A past `at` fires immediately -- the chip provider pends reached
        // deadlines rather than waiting out the u32 wrap (deadline-mux
        // contract; ISR bodies legitimately overrun pending deadlines).
        let delta_sk = at.wrapping_sub(now_sk);
        let delta_sk = if delta_sk > i32::MAX as u32 {
            0
        } else {
            delta_sk as u64
        };
        let mut fire = sim_now + unskew(delta_sk, self.state.skew_ppm.get());
        // Rounding must never land the wake *before* `at` (the driver's `due`
        // check is a >= test) -- nudge forward until the skewed clock reaches it.
        let mut guard = 0;
        while !tick_reached(self.state.local_u64(fire) as u32, at) && guard < 64 {
            fire += 1;
            guard += 1;
        }
        self.core.borrow_mut().schedule(
            Event::Compare {
                servo: self.idx,
                generation,
            },
            fire,
        );
    }

    fn cancel(&mut self) {
        self.state.generation.set(self.state.generation.get() + 1);
        self.state.armed.set(None);
    }
}

/// Software osc-CRC accumulator with an immediate result (sec 3.2, F6 modelled as
/// instantaneous). Even-length feeds are asserted (F12); the even-*address*
/// half of F12 is not -- heap-backed test buffers give no absolute-parity
/// guarantee even where the driver's offsets are correct.
#[derive(Default)]
pub struct SimCrc {
    state: u16,
    snap: Vec<u8>,
}

impl SimCrc {
    pub fn new() -> Self {
        Self::default()
    }
}

impl CrcEngine for SimCrc {
    fn reset(&mut self) {
        self.state = 0;
    }

    fn feed(&mut self, span: &[u8]) {
        assert_eq!(span.len() % 2, 0, "osc-CRC feed must be even-length (F12)");
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

pub struct SimWire {
    core: Rc<RefCell<Core>>,
    baud: Rc<BaudState>,
    idx: usize,
    /// End tick of the last-scheduled byte -- the next arm streams from here so
    /// arms sit back-to-back on the wire (sec 4.2 tolerates the re-arm gap).
    tx_cursor: Cell<u64>,
}

impl SimWire {
    pub fn new(core: Rc<RefCell<Core>>, baud: Rc<BaudState>, idx: usize) -> Self {
        Self {
            core,
            baud,
            idx,
            tx_cursor: Cell::new(0),
        }
    }
}

impl TxWire for SimWire {
    fn start_frame(&mut self) {
        let baud = self.baud.current();
        let brk = break_ticks(baud);
        let mut c = self.core.borrow_mut();
        let start = c.now();
        let break_end = start + brk;
        c.claim(Talker::Servo(self.idx), start, break_end);
        c.schedule(
            Event::WireBreak {
                talker: Talker::Servo(self.idx),
                baud,
                break_start: start,
            },
            break_end,
        );
        self.tx_cursor.set(break_end);
    }

    fn send(&mut self, span: &[u8]) {
        let baud = self.baud.current();
        let bt = byte_ticks(baud);
        let mut c = self.core.borrow_mut();
        // A late TC delivery (busy CPU) arms the next span after the previous
        // one drained: on silicon that is an inter-byte gap on the wire
        // (legal -- nothing times on idle, sec 4.2). Clamp so the DES clock never
        // rewinds.
        let start = self.tx_cursor.get().max(c.now());
        for (k, &b) in span.iter().enumerate() {
            let t = start + (k as u64 + 1) * bt;
            c.schedule(
                Event::WireData {
                    talker: Talker::Servo(self.idx),
                    byte: b,
                    baud,
                },
                t,
            );
        }
        let end = start + span.len() as u64 * bt;
        c.claim(Talker::Servo(self.idx), start, end);
        c.schedule(Event::TxArmDone { servo: self.idx }, end);
        self.tx_cursor.set(end);
    }

    fn release(&mut self) {
        self.core.borrow_mut().finalize_frame();
    }
}

pub struct SimBaud {
    state: Rc<BaudState>,
}

impl SimBaud {
    pub fn new(state: Rc<BaudState>) -> Self {
        Self { state }
    }
}

impl UsartBaud for SimBaud {
    fn apply(&mut self, baud: BaudRate) {
        self.state.apply(baud);
    }
}

/// ZST binding each provider role for the `ServoBus` composite.
pub struct SimProviders;

impl Providers for SimProviders {
    type Ring = SimRing;
    type Deadline = SimDeadline;
    type Crc = SimCrc;
    type Tx = SimWire;
    type Baud = SimBaud;
}
