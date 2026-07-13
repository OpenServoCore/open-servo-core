//! Discrete-event simulator for the osc-native bus. A single `Sim` owns the
//! event queue, the shared half-duplex wire, and a set of boxed `SimServo`s
//! (real `ServoBus` + real `osc_core` dispatch over sim providers). The wire
//! model is derived from the measured silicon facts (sec 12): break framing, one
//! FE per break, DMA-ringed bytes, drive discipline. Handler invocations
//! route through a per-servo PFIC occupancy model (`cpu`): with nonzero
//! [`HandlerCost`], events landing mid-body pend and coalesce as on silicon.
//! See the module docs of `core`, `cpu`, `providers`, and `servo` for the
//! moving parts.

mod core;
mod cpu;
mod providers;
mod servo;
mod store;
mod support;

#[cfg(test)]
mod tests;

use std::cell::RefCell;
use std::rc::Rc;

use osc_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_core::{BaudRate, BootMode, ControlTable};
use osc_drivers::bus::LinkDiag;
use osc_drivers::bus::RESCUE_LOW_US;

use self::core::{Core, Event, TICKS_PER_US, Talker, break_ticks, byte_ticks};
use self::cpu::{Cpu, Vector};
use self::providers::Handles;
use self::servo::SimServo;

pub use self::cpu::HandlerCost;
pub use self::store::RamStore;

/// XOR mask applied to bytes ringed at a baud-mismatched receiver: arbitrary
/// but nonzero, so mismatched data never survives as valid content.
const MISMATCH_GARBLE: u8 = 0xA5;

pub use self::support::{assert_valid, instruction, status};

/// Who put a frame on the wire, as recorded.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Source {
    Host,
    Servo(u8),
}

/// One frame observed on the wire, as the ring would image it.
#[derive(Clone, Debug)]
pub struct WireFrame {
    /// Break start, ticks.
    pub at: u64,
    /// Last byte's end, ticks.
    pub end: u64,
    pub from: Source,
    /// The ring image: 0x00 break byte, then ID..CRC.
    pub bytes: Vec<u8>,
    /// Another servo transmitted into this frame (sec 9.2 ENUM collision):
    /// `bytes` interleaves both talkers' spans -- garbage, as on the wire.
    pub collided: bool,
}

pub struct Sim {
    core: Rc<RefCell<Core>>,
    // Box is load-bearing: a `SimServo`'s address must be stable while the TX
    // engine streams a read reply zero-copy (raw pointers into its control
    // table, sec 4.2). `Vec<SimServo>` would move elements on realloc.
    #[allow(clippy::vec_box)]
    servos: Vec<Box<SimServo>>,
    handles: Vec<Handles>,
    cpus: Vec<Cpu>,
    rate: BaudRate,
    /// The scheduling host queues each frame after its own prior traffic.
    host_free_at: u64,
}

impl Sim {
    pub fn new(rate: BaudRate) -> Self {
        Self {
            core: Rc::new(RefCell::new(Core::new())),
            servos: Vec::new(),
            handles: Vec::new(),
            cpus: Vec::new(),
            rate,
            host_free_at: 0,
        }
    }

    /// Add a servo with the default table seed at this id; returns its index.
    pub fn add_servo(&mut self, id: u8) -> usize {
        self.add_servo_full(id, 0, DEFAULT_RESPONSE_DEADLINE_US, None)
    }

    pub fn add_servo_with(&mut self, id: u8, skew_ppm: i32, response_deadline_us: u16) -> usize {
        self.add_servo_full(id, skew_ppm, response_deadline_us, None)
    }

    /// Add a servo with a persistence store: its boot overlay runs against
    /// the store's slots (sec 9.4), so a saved image's comms block wins over
    /// `id` -- sharing one leaked store across `Sim` instances models a
    /// reboot with flash intact.
    pub fn add_servo_with_store(&mut self, id: u8, store: &'static RamStore) -> usize {
        self.add_servo_full(id, 0, DEFAULT_RESPONSE_DEADLINE_US, Some(store))
    }

    fn add_servo_full(
        &mut self,
        id: u8,
        skew_ppm: i32,
        response_deadline_us: u16,
        store: Option<&'static RamStore>,
    ) -> usize {
        let idx = self.servos.len();
        let (servo, handles) = SimServo::build(
            &self.core,
            idx,
            id,
            self.rate,
            skew_ppm,
            response_deadline_us,
            store,
        );
        self.servos.push(servo);
        self.handles.push(handles);
        self.cpus.push(Cpu::default());
        idx
    }

    /// Give servo `i`'s handler bodies sim-time cost (`cpu` module): events
    /// landing while a body runs pend and coalesce as on silicon. Zero-cost
    /// (the default) is the ideal-CPU model.
    pub fn set_handler_cost(&mut self, i: usize, cost: HandlerCost) {
        self.cpus[i].cost = cost;
    }

    /// `on_break` invocations delivered to servo `i` -- wire break events
    /// minus this counts pends that coalesced.
    pub fn delivered_breaks(&self, i: usize) -> u64 {
        self.cpus[i].delivered_breaks()
    }

    /// Inspect a servo's live control table.
    pub fn servo_table<R>(&self, i: usize, f: impl FnOnce(&ControlTable) -> R) -> R {
        self.servos[i].with_table(f)
    }

    /// Chip-side mutation of a servo's table (fault flags, telemetry) -- the
    /// sim's stand-in for the control/fault ISRs the chip band will own.
    pub fn servo_table_mut<R>(&self, i: usize, f: impl FnOnce(&mut ControlTable) -> R) -> R {
        self.servos[i].with_table_mut(f)
    }

    pub fn servo_diag(&self, i: usize) -> LinkDiag {
        self.servos[i].diag()
    }

    /// The chip main loop's trim poll (sec 9.3), between exchanges.
    pub fn poll_clock_trim(&mut self, i: usize) -> Option<i8> {
        self.servos[i].poll_clock_trim()
    }

    pub fn take_reboot(&mut self, i: usize) -> Option<BootMode> {
        self.servos[i].take_reboot()
    }

    /// Replace servo `i`'s factory UID (the chip band seeds it from ESIG at
    /// bringup; tests seed it before traffic for controlled ENUM prefixes).
    pub fn seed_servo_uid(&self, i: usize, uid: [u8; 16]) {
        self.servos[i].seed_uid(uid);
    }

    pub fn servo_uid(&self, i: usize) -> [u8; 16] {
        self.servos[i].uid()
    }

    /// Queue a host frame starting when the wire is free of host traffic.
    pub fn host_send(&mut self, frame: &[u8]) {
        let start = self.host_free_at.max(self.core.borrow().now());
        self.queue_host_frame(start, frame);
    }

    /// Sim time is monotonic: an `at_us` the drained queue has already passed
    /// starts as soon as prior activity quiesced instead of rewinding the
    /// clock (the `cpu` occupancy model depends on pops never running
    /// backwards; zero-cost handlers merely never noticed the rewind).
    fn clamp_at(&self, at_us: u64) -> u64 {
        (at_us * TICKS_PER_US).max(self.core.borrow().now())
    }

    /// Serialized against the host's own prior traffic: one transmitter
    /// cannot overlap itself, so a clamped `at_us` queues after
    /// `host_free_at` instead of colliding on the wire.
    pub fn host_send_at(&mut self, at_us: u64, frame: &[u8]) {
        let start = self.clamp_at(at_us).max(self.host_free_at);
        self.queue_host_frame(start, frame);
    }

    /// A bare break at `at_us` -- the MGMT CAL train's ruler mark (sec 9.3):
    /// one FE at every listener, no data bytes. Successive calls with exact
    /// `at_us` spacing model a host whose timer paces the train.
    pub fn host_send_break_at(&mut self, at_us: u64) {
        let start = self.clamp_at(at_us).max(self.host_free_at);
        let baud = self.rate;
        let break_end = start + break_ticks(baud);
        let mut c = self.core.borrow_mut();
        c.claim(Talker::Host, start, break_end);
        c.schedule(
            Event::WireBreak {
                talker: Talker::Host,
                baud,
                break_start: start,
            },
            break_end,
        );
        c.schedule(Event::HostFrameEnd, break_end);
        drop(c);
        self.host_free_at = break_end;
    }

    /// Servo `i`'s oscillator rate becomes `ppm` at `at_us` -- thermal drift
    /// as the drift tracker sees it: the rate changes, the clock never steps.
    pub fn set_servo_skew_at(&mut self, at_us: u64, i: usize, ppm: i32) {
        let at = self.clamp_at(at_us);
        self.core
            .borrow_mut()
            .schedule(Event::SkewChange { servo: i, ppm }, at);
    }

    /// Inject a spurious break wake at servo `i`: the break vector
    /// re-enters with NO new wire byte -- a coalesced or lagged service
    /// (sec 3.4: breaks are not countable events; wakes carry no position and
    /// no time, and any code deriving either from them kills live frames --
    /// bench-caught twice: the fence, then the tracker starvation).
    pub fn inject_wake_refire_at(&mut self, at_us: u64, i: usize) {
        let at = self.clamp_at(at_us);
        self.core
            .borrow_mut()
            .schedule(Event::WakeRefire { servo: i }, at);
    }

    /// Queue a host frame whose transmitter stalls mid-frame: bytes
    /// `..split` stream normally, then the wire idles high for `stall_us`,
    /// then the rest streams. Models the pirate's TXE-poll bubbles (bench:
    /// 58-94-bit pauses INSIDE frames on failing plain-burst cycles) -- a
    /// legal wire per sec 4.1 (nothing times on idle), and the
    /// stress that parks the frontier at the starvation horizon.
    pub fn host_send_stalled(&mut self, frame: &[u8], split: usize, stall_us: u64) {
        let start = self.host_free_at.max(self.core.borrow().now());
        let baud = self.rate;
        let bt = byte_ticks(baud);
        let break_end = start + break_ticks(baud);
        let n = frame.len() as u64 - 1;
        let split = split.clamp(1, frame.len() - 1) as u64;
        let stall = stall_us * TICKS_PER_US;
        let end = break_end + n * bt + stall;

        let mut c = self.core.borrow_mut();
        c.claim(Talker::Host, start, end);
        c.schedule(
            Event::WireBreak {
                talker: Talker::Host,
                baud,
                break_start: start,
            },
            break_end,
        );
        for (k, &b) in frame[1..].iter().enumerate() {
            let mut t = break_end + (k as u64 + 1) * bt;
            if (k as u64) >= split {
                t += stall;
            }
            c.schedule(
                Event::WireData {
                    talker: Talker::Host,
                    byte: b,
                    baud,
                },
                t,
            );
        }
        c.schedule(Event::HostFrameEnd, end);
        drop(c);
        self.host_free_at = end;
    }

    /// One lone noise byte on the wire (line noise, F4): rings at every
    /// servo, wakes nothing (sec 3.4 -- errors never interrupt).
    pub fn inject_garble_at(&mut self, at_us: u64, b: u8) {
        let at = self.clamp_at(at_us);
        self.core
            .borrow_mut()
            .schedule(Event::WireGarble { byte: b }, at);
    }

    /// A foreign break dropped onto the wire at `at_us`, bypassing host
    /// serialization (a break can land inside another talker's window --
    /// collision, glitch): rings a 0x00 and wakes qualified receivers.
    pub fn inject_break_at(&mut self, at_us: u64) {
        let at = self.clamp_at(at_us);
        let baud = self.rate;
        self.core
            .borrow_mut()
            .schedule(Event::StrayBreak { baud }, at);
    }

    /// Rescue pulse: line dominant for `us` (sec 9.1). Two modeled effects:
    /// each servo's main-loop sampler declares once >= RESCUE_LOW_US of
    /// frozen-ring low has elapsed (measured from the pulse's ringed 0x00,
    /// ~a byte-time in), and the pulse's END fires an ordinary break wake --
    /// the detector latches only at the rising edge (silicon).
    /// A pulse too short to cross the threshold delivers only the end wake.
    pub fn hold_line_low_at(&mut self, at_us: u64, us: u64) {
        let start = self.clamp_at(at_us);
        let dur = us * TICKS_PER_US;
        let baud = self.rate;
        let mut c = self.core.borrow_mut();
        let declare = start + byte_ticks(baud) + RESCUE_LOW_US as u64 * TICKS_PER_US;
        if declare < start + dur {
            c.schedule(Event::RescueDeclare, declare);
        }
        c.schedule(Event::StrayBreak { baud }, start + dur);
    }

    pub fn set_host_baud(&mut self, rate: BaudRate) {
        self.rate = rate;
    }

    /// Drain the event queue; return every frame observed since the last call.
    pub fn run(&mut self) -> Vec<WireFrame> {
        loop {
            let ev = self.core.borrow_mut().pop();
            let Some(ev) = ev else { break };
            self.dispatch(ev);
        }
        self.core.borrow_mut().take_recorded()
    }

    pub fn now_us(&self) -> u64 {
        self.core.borrow().now() / TICKS_PER_US
    }

    // --- internals --------------------------------------------------------

    fn queue_host_frame(&mut self, start: u64, frame: &[u8]) {
        let baud = self.rate;
        let bt = byte_ticks(baud);
        let break_end = start + break_ticks(baud);
        let n = frame.len() as u64 - 1; // data bytes (0x00 prefix stays a break)
        let end = break_end + n * bt;

        let mut c = self.core.borrow_mut();
        c.claim(Talker::Host, start, end);
        c.schedule(
            Event::WireBreak {
                talker: Talker::Host,
                baud,
                break_start: start,
            },
            break_end,
        );
        for (k, &b) in frame[1..].iter().enumerate() {
            let t = break_end + (k as u64 + 1) * bt;
            c.schedule(
                Event::WireData {
                    talker: Talker::Host,
                    byte: b,
                    baud,
                },
                t,
            );
        }
        c.schedule(Event::HostFrameEnd, end);
        drop(c);
        self.host_free_at = end;
    }

    fn dispatch(&mut self, ev: Event) {
        match ev {
            Event::WireBreak {
                talker,
                baud,
                break_start,
            } => self.deliver_break(talker, baud, break_start),
            Event::WireData { talker, byte, baud } => self.deliver_data(talker, byte, baud),
            Event::WireGarble { byte } => self.deliver_garble(byte),
            Event::StrayBreak { baud } => self.deliver_stray_break(baud),
            Event::RescueDeclare => self.deliver_rescue_declare(),
            Event::SkewChange { servo, ppm } => {
                let now = self.core.borrow().now();
                self.handles[servo].deadline.set_skew(now, ppm);
            }
            Event::HostFrameEnd => self.core.borrow_mut().finalize_frame(),
            Event::Compare { servo, generation } => {
                // The generation gate is checked at the match instant only: a
                // deadline re-aimed before its match never fires, but once
                // matched the pend survives any re-aim (PFIC semantics) and
                // the handler sorts out staleness itself.
                if self.handles[servo].deadline.generation() == generation {
                    self.deliver(servo, Vector::Compare);
                }
            }
            Event::TxArmDone { servo } => self.deliver(servo, Vector::TxDone),
            Event::CpuFree { servo } => self.cpu_free(servo),
            Event::WakeRefire { servo } => self.deliver(servo, Vector::Break),
        }
    }

    /// Run `v`'s handler on servo `j` now, or pend it if a body is running.
    fn deliver(&mut self, j: usize, v: Vector) {
        let now = self.core.borrow().now();
        if self.cpus[j].busy(now) {
            self.cpus[j].pend(v);
            self.schedule_free(j);
        } else {
            self.run_vector(j, v);
        }
    }

    fn run_vector(&mut self, j: usize, v: Vector) {
        let now = self.core.borrow().now();
        self.cpus[j].charge(now, v);
        match v {
            Vector::Compare => self.servos[j].on_deadline(),
            Vector::Break => self.servos[j].on_break(),
            Vector::TxDone => self.servos[j].on_tx_complete(),
        }
    }

    fn schedule_free(&mut self, j: usize) {
        if !self.cpus[j].free_scheduled {
            self.cpus[j].free_scheduled = true;
            let at = self.cpus[j].busy_until();
            self.core
                .borrow_mut()
                .schedule(Event::CpuFree { servo: j }, at);
        }
    }

    /// A handler body ended: deliver ONE pended vector (highest arbitration
    /// first), then re-arm for the rest -- each delivery is its own event so
    /// every handler reads the clock at its true entry tick.
    fn cpu_free(&mut self, j: usize) {
        self.cpus[j].free_scheduled = false;
        let now = self.core.borrow().now();
        if self.cpus[j].busy(now) {
            // A same-tick wire event beat this wake and re-occupied the CPU.
            if self.cpus[j].any_pend() {
                self.schedule_free(j);
            }
            return;
        }
        if let Some(v) = self.cpus[j].take_pend() {
            self.run_vector(j, v);
            if self.cpus[j].any_pend() {
                self.schedule_free(j);
            }
        }
    }

    fn deliver_break(&mut self, talker: Talker, baud: BaudRate, break_start: u64) {
        self.core.borrow_mut().begin_frame(break_start, talker);
        for j in 0..self.servos.len() {
            if talker == Talker::Servo(j) {
                continue; // no own-TX echo (F9)
            }
            self.deliver_break_to(j, baud);
        }
    }

    /// Length-qualified break delivery (sec 3.4): the 10-bit span covers >=10 of
    /// the receiver's bit-times only at receivers at or above the talker's
    /// rate -- those decode the all-zeros character and wake. A slower
    /// receiver hears a sub-character low: compressed junk, no wake.
    fn deliver_break_to(&mut self, j: usize, baud: BaudRate) {
        if self.handles[j].baud.current().as_hz() >= baud.as_hz() {
            self.handles[j].ring.push(0x00);
            self.deliver(j, Vector::Break);
        } else {
            self.handles[j].ring.push(MISMATCH_GARBLE);
        }
    }

    fn deliver_data(&mut self, talker: Talker, byte: u8, baud: BaudRate) {
        let now = self.core.borrow().now();
        self.core.borrow_mut().append_byte(byte, now);
        for j in 0..self.servos.len() {
            if talker == Talker::Servo(j) {
                continue;
            }
            let matched = self.handles[j].baud.current() == baud;
            if matched {
                self.handles[j].ring.push(byte);
            } else {
                // A baud mismatch garbles both value and framing (sec 2
                // approximation): the sampled bits are wrong-rate noise, so
                // the ringed byte must not survive as valid data at either a
                // faster or slower receiver. No wake (sec 3.4): a data byte's
                // dominant runs stay under the detector's 10-bit bar in this
                // model (silicon: rare slower-baud bytes do qualify -- leg D
                // of the lbd_wake spike -- but the wake-into-junk exposure is
                // already carried by the slower talker's breaks).
                self.handles[j].ring.push(byte ^ MISMATCH_GARBLE);
            }
        }
    }

    fn deliver_garble(&mut self, byte: u8) {
        for j in 0..self.servos.len() {
            self.handles[j].ring.push(byte);
        }
    }

    fn deliver_stray_break(&mut self, baud: BaudRate) {
        for j in 0..self.servos.len() {
            self.deliver_break_to(j, baud);
        }
    }

    fn deliver_rescue_declare(&mut self) {
        // Every servo's sampler crosses the threshold together (the low is
        // baud-agnostic); the declaration is thread-level -- no vector, no
        // CPU pend, mirroring the chip's main-loop + critical-section path.
        // The pulse's own ringed 0x00 lands here (it rang a byte-time in;
        // the model folds it into the declaration instant).
        for j in 0..self.servos.len() {
            self.handles[j].ring.push(0x00);
            self.servos[j].on_rescue();
        }
    }
}
