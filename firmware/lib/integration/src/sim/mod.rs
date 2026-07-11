//! Discrete-event simulator for the osc-native bus. A single `Sim` owns the
//! event queue, the shared half-duplex wire, and a set of boxed `SimServo`s
//! (real `ServoBus` + real `osc_core` dispatch over sim providers). The wire
//! model is derived from the measured silicon facts (§12): break framing, one
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
}

pub struct Sim {
    core: Rc<RefCell<Core>>,
    // Box is load-bearing: a `SimServo`'s address must be stable while the TX
    // engine streams a read reply zero-copy (raw pointers into its control
    // table, §4.2). `Vec<SimServo>` would move elements on realloc.
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
            core: Rc::new(RefCell::new(Core::new(rate))),
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
    /// the store's slots (§9.4), so a saved image's comms block wins over
    /// `id` — sharing one leaked store across `Sim` instances models a
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

    /// `on_break` invocations delivered to servo `i` — wire FE events minus
    /// this counts pends that coalesced.
    pub fn delivered_breaks(&self, i: usize) -> u64 {
        self.cpus[i].delivered_breaks()
    }

    /// Inspect a servo's live control table.
    pub fn servo_table<R>(&self, i: usize, f: impl FnOnce(&ControlTable) -> R) -> R {
        self.servos[i].with_table(f)
    }

    /// Chip-side mutation of a servo's table (fault flags, telemetry) — the
    /// sim's stand-in for the control/fault ISRs the chip band will own.
    pub fn servo_table_mut<R>(&self, i: usize, f: impl FnOnce(&mut ControlTable) -> R) -> R {
        self.servos[i].with_table_mut(f)
    }

    pub fn servo_diag(&self, i: usize) -> LinkDiag {
        self.servos[i].diag()
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

    pub fn host_send_at(&mut self, at_us: u64, frame: &[u8]) {
        let start = self.clamp_at(at_us);
        self.queue_host_frame(start, frame);
    }

    /// Inject a latched-flag re-fire at servo `i`: the fault vector
    /// re-enters with NO new wire byte. Level-pend hardware does this
    /// whenever a flag's SR-DR retire pair hasn't formed (routinely, after
    /// NOREPLY frames — nothing transmits, so nothing retires the flag);
    /// the fault contract exists because these wakes carry no position and
    /// no time, and any code deriving either from them kills live frames
    /// (bench 2026-07-11).
    pub fn inject_fault_refire_at(&mut self, at_us: u64, i: usize) {
        let at = self.clamp_at(at_us);
        self.core
            .borrow_mut()
            .schedule(Event::FaultRefire { servo: i }, at);
    }

    /// Queue a host frame whose transmitter stalls mid-frame: bytes
    /// `..split` stream normally, then the wire idles high for `stall_us`,
    /// then the rest streams. Models the pirate's TXE-poll bubbles (bench
    /// 2026-07-08: 58–94-bit pauses INSIDE frames on failing plain-burst
    /// cycles) — a legal wire per §4.1 (nothing times on idle), and the
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
        c.hold_low(start, break_end + 1);
        c.schedule(
            Event::WireBreak {
                talker: Talker::Host,
                break_start: start,
            },
            break_end,
        );
        for (k, &b) in frame[1..].iter().enumerate() {
            let mut t = break_end + (k as u64 + 1) * bt;
            if (k as u64) >= split {
                t += stall;
            }
            if b == 0 {
                c.hold_low(t - bt, t - bt / 10);
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

    /// One lone FE byte on the wire, no break framing (line noise, F4).
    pub fn inject_garble_at(&mut self, at_us: u64, b: u8) {
        let at = self.clamp_at(at_us);
        self.core
            .borrow_mut()
            .schedule(Event::WireGarble { byte: b }, at);
    }

    /// Rescue pulse: line dominant for `us`, raising one FE at every servo
    /// while the line is still low (§9.1).
    pub fn hold_line_low_at(&mut self, at_us: u64, us: u64) {
        let start = self.clamp_at(at_us);
        let dur = us * TICKS_PER_US;
        let mut c = self.core.borrow_mut();
        c.hold_low(start, start + dur);
        // FE fires ~one break-time in, while the pulse is still dominant.
        let fe = start + break_ticks(c.host_baud());
        c.schedule(Event::RescuePulse, fe);
    }

    pub fn set_host_baud(&mut self, rate: BaudRate) {
        self.rate = rate;
        self.core.borrow_mut().set_host_baud(rate);
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
        // Dominant through the FE delivery instant: on real silicon the break
        // is still low at FE-ISR entry (F5's caveat — the flag leads the rise
        // by a couple of bits), which is what arms the §9.1 rescue confirm on
        // ordinary frames. One tick past `break_end` is also physically true:
        // the first data byte's start bit is dominant.
        c.hold_low(start, break_end + 1);
        c.schedule(
            Event::WireBreak {
                talker: Talker::Host,
                break_start: start,
            },
            break_end,
        );
        for (k, &b) in frame[1..].iter().enumerate() {
            let t = break_end + (k as u64 + 1) * bt;
            // A 0x00 data byte holds the line dominant for start + 8 data
            // bits (9 of its 10 bit-times) — `LineSense` must see it, or the
            // sim can't reproduce phantom-rescue confirms sampling mid-frame.
            if b == 0 {
                c.hold_low(t - bt, t - bt / 10);
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

    fn dispatch(&mut self, ev: Event) {
        match ev {
            Event::WireBreak {
                talker,
                break_start,
            } => self.deliver_break(talker, break_start),
            Event::WireData { talker, byte, baud } => self.deliver_data(talker, byte, baud),
            Event::WireGarble { byte } => self.deliver_garble(byte),
            Event::RescuePulse => self.deliver_rescue(),
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
            Event::FaultPend { servo } => {
                let fw = &self.handles[servo].fault_wake;
                if fw.wake.get() && fw.latched.replace(false) {
                    self.deliver(servo, Vector::Break);
                }
            }
            Event::FaultRefire { servo } => self.deliver_fault(servo),
        }
    }

    /// Wire-fault wake gate (§6 A4, level-pend model): deliver the FE's
    /// `on_break` only while servo `j`'s wake is on — otherwise it latches,
    /// and `set_fault_wake(true)` schedules the late delivery.
    fn deliver_fault(&mut self, j: usize) {
        if self.handles[j].fault_wake.wake.get() {
            self.deliver(j, Vector::Break);
        } else {
            self.handles[j].fault_wake.latched.set(true);
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
    /// first), then re-arm for the rest — each delivery is its own event so
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

    fn deliver_break(&mut self, talker: Talker, break_start: u64) {
        self.core.borrow_mut().begin_frame(break_start, talker);
        for j in 0..self.servos.len() {
            if talker == Talker::Servo(j) {
                continue; // no own-TX echo (F9)
            }
            self.handles[j].ring.push(0x00);
            self.deliver_fault(j);
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
                // A baud mismatch garbles both value and framing (§2
                // approximation): the sampled bits are wrong-rate noise, so
                // the ringed byte must not survive as valid data at either a
                // faster or slower receiver.
                self.handles[j].ring.push(byte ^ MISMATCH_GARBLE);
                self.deliver_fault(j);
            }
        }
    }

    fn deliver_garble(&mut self, byte: u8) {
        for j in 0..self.servos.len() {
            self.handles[j].ring.push(byte);
            self.deliver_fault(j);
        }
    }

    fn deliver_rescue(&mut self) {
        // Baud-agnostic dominant low: every servo takes one FE with the line
        // still low, so each schedules its rescue recheck (§9.1). No recorded
        // frame — a rescue pulse is not data.
        for j in 0..self.servos.len() {
            self.handles[j].ring.push(0x00);
            self.deliver_fault(j);
        }
    }
}
