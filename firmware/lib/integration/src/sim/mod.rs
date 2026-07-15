//! Discrete-event simulator for the osc-native bus. A single `Sim` owns the
//! event queue, the shared half-duplex wire, and a set of boxed `SimServo`s
//! (real `ServoBus` + real `osc_servo_core` dispatch over sim providers). The wire
//! model is derived from the measured silicon facts (sec 12): break framing, one
//! FE per break, DMA-ringed bytes, drive discipline. Handler invocations
//! route through a per-servo PFIC occupancy model (`cpu`): with nonzero
//! [`HandlerCost`], events landing mid-body pend and coalesce as on silicon.
//! See the module docs of `core`, `cpu`, `providers`, and `servo` for the
//! moving parts.

mod core;
mod cpu;
mod host;
mod providers;
mod resample;
mod servo;
mod store;
mod support;

#[cfg(test)]
mod tests;

use std::cell::RefCell;
use std::rc::Rc;

use osc_servo_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_servo_core::{BaudRate, BootMode, ControlTable};
use osc_servo_drivers::bus::LinkDiag;
use osc_servo_drivers::bus::RESCUE_LOW_US;

use self::core::{Core, Event, TICKS_PER_US, Talker, break_ticks, byte_ticks};
use self::cpu::{Cpu, Vector};
use self::providers::Handles;
use self::resample::{CrossRx, RxOut};
use self::servo::SimServo;

pub use self::cpu::HandlerCost;
pub use self::host::HostEvent;
pub use self::store::RamStore;

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
    /// Per-servo cross-baud reception machines (see `resample`): fed
    /// whenever a wire event's rate differs from the receiver's.
    cross: Vec<CrossRx>,
    host_cross: Option<CrossRx>,
    rate: BaudRate,
    /// The scheduling host queues each frame after its own prior traffic.
    host_free_at: u64,
    /// The production engine, when attached (host-in-the-loop scenarios);
    /// scripted `host_send*` and the engine can coexist but must not overlap
    /// on the wire -- the claim assert catches a scenario that mixes them.
    host: Option<host::SimHost>,
    /// The production link server in front of the attached engine, when
    /// attached (client-in-the-loop scenarios): pipe bytes in through
    /// [`Self::link_send`], records out through [`Self::link_recv`]. Engine
    /// events leave as records; `host_events` stays empty in this mode.
    link: Option<LinkRig>,
}

struct LinkRig {
    server: osc_host::link::LinkServer,
    out: RecordLog,
}

/// Records append verbatim (length-prefixed already): the sink IS the pipe's
/// outbound byte stream.
struct RecordLog(Vec<u8>);

impl osc_host::link::RecordSink for RecordLog {
    fn record(&mut self, record: &[u8]) {
        self.0.extend_from_slice(record);
    }
}

impl Sim {
    pub fn new(rate: BaudRate) -> Self {
        Self {
            core: Rc::new(RefCell::new(Core::new())),
            servos: Vec::new(),
            handles: Vec::new(),
            cpus: Vec::new(),
            cross: Vec::new(),
            host_cross: None,
            rate,
            host_free_at: 0,
            host: None,
            link: None,
        }
    }

    /// Attach the production host engine at the sim's wire rate. Submit
    /// through [`Self::host_submit`], drain events after [`Self::run`]
    /// through [`Self::host_events`].
    pub fn attach_host(&mut self) {
        assert!(self.host.is_none(), "host already attached");
        self.host = Some(host::SimHost::build(&self.core, self.rate));
        self.host_cross = Some(CrossRx::new(self.rate));
    }

    /// Submit one command to the attached engine (panics if none).
    pub fn host_submit(
        &mut self,
        cmd: osc_host::engine::Command<'_>,
    ) -> Result<(), osc_host::engine::SubmitError> {
        let r = self.host.as_mut().expect("host attached").bus.submit(cmd);
        self.host_pump();
        r
    }

    /// Drain everything the engine yielded since the last call.
    pub fn host_events(&mut self) -> Vec<HostEvent> {
        std::mem::take(&mut self.host.as_mut().expect("host attached").events)
    }

    /// Put the production link server in front of the attached engine
    /// (panics if no host). From here the pipe surface replaces
    /// `host_submit`/`host_events`.
    pub fn attach_link(&mut self) {
        assert!(self.host.is_some(), "attach_link needs an attached host");
        assert!(self.link.is_none(), "link already attached");
        self.link = Some(LinkRig {
            server: osc_host::link::LinkServer::new(),
            out: RecordLog(Vec::new()),
        });
    }

    /// Feed pipe bytes to the attached link server (panics if none). The
    /// caller advances the wire with [`Self::run`]; records accumulate for
    /// [`Self::link_recv`].
    pub fn link_send(&mut self, bytes: &[u8]) {
        let h = self.host.as_mut().expect("host attached");
        let rig = self.link.as_mut().expect("link attached");
        rig.server.on_pipe(bytes, &mut h.bus, &mut rig.out);
        // Wire-silent commands (HostBaud, SetResponseDeadline) terminal
        // inside on_pipe and schedule no wire event, so [`Self::run`] never
        // reaches a pump -- drain the engine here or their records strand.
        rig.server.pump(&mut h.bus, &mut rig.out);
    }

    /// Let `us` of quiet sim time pass (link mode's pipe pause): schedules
    /// a no-op that far out and runs to it, so servo-side horizons that need
    /// wire silence (starve resync, sec 3.4) actually elapse.
    pub fn idle(&mut self, us: u64) {
        let t = self.core.borrow().now() + us * TICKS_PER_US;
        self.core.borrow_mut().schedule(Event::Idle, t);
        self.run();
    }

    /// Drain the outbound record byte stream (panics if no link).
    pub fn link_recv(&mut self) -> Vec<u8> {
        std::mem::take(&mut self.link.as_mut().expect("link attached").out.0)
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
        self.cross.push(CrossRx::new(self.rate));
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
    /// then the rest streams. Models a soft-timed host's TXE-poll bubbles
    /// (bench-measured: 58-94-bit pauses INSIDE frames on failing
    /// plain-burst cycles) -- a legal wire per sec 4.1 (nothing times on
    /// idle), and the stress that parks the frontier at the starvation
    /// horizon.
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
            Event::HostFrameEnd => {
                self.core.borrow_mut().finalize_frame();
                self.flush_cross();
            }
            Event::Idle => {}
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
            Event::HostCompare { generation } => {
                if let Some(h) = self.host.as_mut()
                    && h.deadline.generation() == generation
                {
                    h.bus.on_deadline();
                }
            }
            Event::HostTxDone => {
                if let Some(h) = self.host.as_mut() {
                    h.bus.on_tx_complete();
                }
            }
        }
        // The adapter's main loop is a tight poll: drain the engine after
        // every event so its clocks and framer track the wire promptly.
        self.host_pump();
    }

    /// Poll the attached engine to exhaustion. Link mode routes through the
    /// production server (events leave as records); otherwise events copy
    /// out of the ring for the harness.
    fn host_pump(&mut self) {
        let Some(h) = self.host.as_mut() else { return };
        if let Some(rig) = self.link.as_mut() {
            rig.server.pump(&mut h.bus, &mut rig.out);
            return;
        }
        loop {
            match h.bus.poll() {
                Some(osc_host::engine::Event::Status {
                    slot,
                    id,
                    inst,
                    payload,
                }) => {
                    let (a, b) = payload.segments();
                    let mut bytes = a.to_vec();
                    bytes.extend_from_slice(b);
                    h.events.push(HostEvent::Status {
                        slot,
                        id: id.as_byte(),
                        inst: inst.0,
                        payload: bytes,
                    });
                }
                Some(osc_host::engine::Event::Done(t)) => h.events.push(HostEvent::Done(t)),
                Some(osc_host::engine::Event::WireDone { tick }) => {
                    h.events.push(HostEvent::WireDone { tick })
                }
                None => return,
            }
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
            self.deliver_break_to(j, baud, break_start);
        }
        // The host hears every foreign break (its ring contract excludes
        // only its own TX); no wake exists to qualify -- just the ring image.
        if talker != Talker::Host {
            let rx = self.host.as_ref().map(|h| h.baud.current());
            if let Some(rx) = rx {
                if rx == baud {
                    self.host.as_ref().expect("host attached").ring.push(0x00);
                } else {
                    let mut out = Vec::new();
                    let hc = self.host_cross.as_mut().expect("host attached");
                    hc.retune(rx);
                    hc.on_break(break_start, break_ticks(baud), &mut out);
                    self.cross_deliver_host(out);
                }
            }
        }
    }

    /// Break delivery: a matched receiver decodes the all-zeros character
    /// and wakes; a mismatched one hears the span through its cross-baud
    /// machine (sec 3.4 length qualification falls out of the resample --
    /// faster receivers see a qualified break, slower ones sub-bar junk).
    fn deliver_break_to(&mut self, j: usize, baud: BaudRate, break_start: u64) {
        let rx = self.handles[j].baud.current();
        if rx == baud {
            self.handles[j].ring.push(0x00);
            self.deliver(j, Vector::Break);
        } else {
            let mut out = Vec::new();
            self.cross[j].retune(rx);
            self.cross[j].on_break(break_start, break_ticks(baud), &mut out);
            self.cross_deliver(j, out);
        }
    }

    fn deliver_data(&mut self, talker: Talker, byte: u8, baud: BaudRate) {
        let now = self.core.borrow().now();
        self.core.borrow_mut().append_byte(byte, now);
        if talker != Talker::Host {
            let rx = self.host.as_ref().map(|h| h.baud.current());
            if let Some(rx) = rx {
                if rx == baud {
                    self.host.as_ref().expect("host attached").ring.push(byte);
                } else {
                    let mut out = Vec::new();
                    let hc = self.host_cross.as_mut().expect("host attached");
                    hc.retune(rx);
                    hc.on_byte(now, byte, baud, &mut out);
                    self.cross_deliver_host(out);
                }
            }
        }
        for j in 0..self.servos.len() {
            if talker == Talker::Servo(j) {
                continue;
            }
            let rx = self.handles[j].baud.current();
            if rx == baud {
                self.handles[j].ring.push(byte);
            } else {
                // Wrong-rate reception goes through the waveform model: a
                // slower talker's characters multiply and its low runs can
                // wake as spurious breaks (the baud-migration garble the
                // fleet measures); a faster talker's spans mostly vanish.
                let mut out = Vec::new();
                self.cross[j].retune(rx);
                self.cross[j].on_byte(now, byte, baud, &mut out);
                self.cross_deliver(j, out);
            }
        }
    }

    /// Ring cross-baud artifacts at servo `j`: characters ring as data, a
    /// qualified break rings one 0x00 and wakes (F2/F3).
    fn cross_deliver(&mut self, j: usize, out: Vec<RxOut>) {
        for o in out {
            match o {
                RxOut::Byte(b) => self.handles[j].ring.push(b),
                RxOut::Break => {
                    self.handles[j].ring.push(0x00);
                    self.deliver(j, Vector::Break);
                }
            }
        }
    }

    fn cross_deliver_host(&mut self, out: Vec<RxOut>) {
        if let Some(h) = &self.host {
            for o in out {
                match o {
                    RxOut::Byte(b) => h.ring.push(b),
                    RxOut::Break => h.ring.push(0x00),
                }
            }
        }
    }

    /// A frame ended and the line is idle: complete every mismatched
    /// receiver's in-flight sampling against the high line.
    fn flush_cross(&mut self) {
        for j in 0..self.cross.len() {
            let mut out = Vec::new();
            self.cross[j].flush(&mut out);
            if !out.is_empty() {
                self.cross_deliver(j, out);
            }
        }
        if self.host_cross.is_some() {
            let mut out = Vec::new();
            self.host_cross.as_mut().expect("checked").flush(&mut out);
            self.cross_deliver_host(out);
        }
    }

    fn deliver_garble(&mut self, byte: u8) {
        for j in 0..self.servos.len() {
            self.handles[j].ring.push(byte);
        }
        if let Some(h) = &self.host {
            h.ring.push(byte);
        }
    }

    fn deliver_stray_break(&mut self, baud: BaudRate) {
        let start = {
            let now = self.core.borrow().now();
            now.saturating_sub(break_ticks(baud))
        };
        for j in 0..self.servos.len() {
            self.deliver_break_to(j, baud, start);
        }
        if let Some(h) = &self.host {
            if h.baud.current().as_hz() >= baud.as_hz() {
                h.ring.push(0x00);
            } else {
                h.ring.push(0xA5); // injected junk, value arbitrary
            }
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
