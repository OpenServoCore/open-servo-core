//! Discrete-event simulator for the osc-native bus. A single `Sim` owns the
//! event queue, the shared half-duplex wire, and a set of boxed `SimServo`s
//! (real `ServoBus` + real `osc_core` dispatch over sim providers). The wire
//! model is derived from the measured silicon facts (§12): break framing, one
//! FE per break, DMA-ringed bytes, drive discipline. See the module docs of
//! `core`, `providers`, and `servo` for the moving parts.

mod core;
mod providers;
mod servo;
mod support;

#[cfg(test)]
mod tests;

use std::cell::RefCell;
use std::rc::Rc;

use osc_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_core::{BaudRate, BootMode, ControlTable};
use osc_drivers::bus::LinkDiag;

use self::core::{Core, Event, TICKS_PER_US, Talker, break_ticks, byte_ticks};
use self::providers::Handles;
use self::servo::SimServo;

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
            rate,
            host_free_at: 0,
        }
    }

    /// Add a servo with the default table seed at this id; returns its index.
    pub fn add_servo(&mut self, id: u8) -> usize {
        self.add_servo_with(id, 0, DEFAULT_RESPONSE_DEADLINE_US)
    }

    pub fn add_servo_with(&mut self, id: u8, skew_ppm: i32, response_deadline_us: u16) -> usize {
        let idx = self.servos.len();
        let (servo, handles) = SimServo::build(
            &self.core,
            idx,
            id,
            self.rate,
            skew_ppm,
            response_deadline_us,
        );
        self.servos.push(servo);
        self.handles.push(handles);
        idx
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

    /// Queue a host frame starting when the wire is free of host traffic.
    pub fn host_send(&mut self, frame: &[u8]) {
        let start = self.host_free_at.max(self.core.borrow().now());
        self.queue_host_frame(start, frame);
    }

    pub fn host_send_at(&mut self, at_us: u64, frame: &[u8]) {
        self.queue_host_frame(at_us * TICKS_PER_US, frame);
    }

    /// One lone FE byte on the wire, no break framing (line noise, F4).
    pub fn inject_garble_at(&mut self, at_us: u64, b: u8) {
        self.core
            .borrow_mut()
            .schedule(Event::WireGarble { byte: b }, at_us * TICKS_PER_US);
    }

    /// Rescue pulse: line dominant for `us`, raising one FE at every servo
    /// while the line is still low (§9.1).
    pub fn hold_line_low_at(&mut self, at_us: u64, us: u64) {
        let start = at_us * TICKS_PER_US;
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
                if self.handles[servo].deadline.generation() == generation {
                    self.servos[servo].on_deadline();
                }
            }
            Event::TxArmDone { servo } => self.servos[servo].on_tx_complete(),
        }
    }

    fn deliver_break(&mut self, talker: Talker, break_start: u64) {
        self.core.borrow_mut().begin_frame(break_start, talker);
        for j in 0..self.servos.len() {
            if talker == Talker::Servo(j) {
                continue; // no own-TX echo (F9)
            }
            self.handles[j].ring.push(0x00);
            self.servos[j].on_break();
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
                self.servos[j].on_break();
            }
        }
    }

    fn deliver_garble(&mut self, byte: u8) {
        for j in 0..self.servos.len() {
            self.handles[j].ring.push(byte);
            self.servos[j].on_break();
        }
    }

    fn deliver_rescue(&mut self) {
        // Baud-agnostic dominant low: every servo takes one FE with the line
        // still low, so each schedules its rescue recheck (§9.1). No recorded
        // frame — a rescue pulse is not data.
        for j in 0..self.servos.len() {
            self.handles[j].ring.push(0x00);
            self.servos[j].on_break();
        }
    }
}
