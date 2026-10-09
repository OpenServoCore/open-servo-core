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

use osc_servo_core::data_state::DataJob;
use osc_servo_core::pos_lut::POINTS;
use osc_servo_core::regions::config::DEFAULT_RESPONSE_DEADLINE_US;
use osc_servo_core::{BaudRate, BootMode, ControlTable};
use osc_servo_drivers::bus::LinkDiag;

use self::core::{Core, Event, TICKS_PER_US, Talker, bit_ticks, break_ticks, byte_ticks};
use self::cpu::{Cpu, KERNEL_PERIOD, Vector};
use self::providers::Handles;
use self::resample::{CrossRx, RxOut};
use self::servo::SimServo;

pub use self::cpu::{
    Entries, HandlerCost, KERNEL_HOLD, KERNEL_QUIET, KernelLane, KernelLevel, KernelStats,
    READ32_3M_COST,
};
pub use self::host::HostEvent;
pub use self::servo::{DEV_V006_SENSE, DEV_V006_SENSE_EXT};
pub use self::store::{Kind as ImageKind, RamStore, Tear};

pub use self::support::{
    assert_valid, expect_tel_payload, expect_tel_payload_rows, frame_crc_ok, instruction, status,
    status_frame, tel_sample,
};
pub use osc_servo_core::tel::TelSample;
pub use osc_servo_core::{CalibSense, CalibSenseExt};

/// TEL fast-tick period: the kernel's 20 kHz control tick.
const TEL_TICK: u64 = KERNEL_PERIOD;

/// When a break's wake reaches a servo, relative to its ringed 0x00.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum BreakWake {
    /// The wake leads the byte by 0.75 bit-times: the chip's TIM2 detector
    /// overflows at 9.25 bit-times of low, ahead of the stop-bit sample
    /// that rings the 0x00.
    BeforeByte,
    /// The 0x00 has rung when the wake is serviced: a detector that latches
    /// at the span's end, or a TIM2 wake serviced late.
    AfterByte,
    /// Each wake flips between the two: service latency straddling the
    /// byte's landing.
    Alternating,
}

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
    /// Per-servo TEL fast-tick pumps (see `tel_pump`).
    tels: Vec<TelPump>,
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
    /// Run the main loop's reboot poll (see [`Self::set_self_reboot`]).
    self_reboot: bool,
    /// Run the main loop's data-job poll (see [`Self::set_data_jobs`]).
    data_jobs: bool,
    /// Wake order of every break (see [`Self::set_break_wake`]).
    break_wake: BreakWake,
    /// The next [`BreakWake::Alternating`] wake trails its byte.
    alternate_after: bool,
}

/// One servo's TEL fast-tick pump: the sim's stand-in for the kernel's 50 us
/// ADC tick, running only while a burst is armed (the kernel's `tel.active()`
/// gate). Sample values are synthesized from the per-burst tick index
/// ([`tel_sample`]), so tests pin payload bytes against the same function -
/// or, with a [`Track`], played back from recorded rows.
struct TelPump {
    running: bool,
    /// Stale scheduled ticks die by epoch (the Compare generation idiom).
    epoch: u64,
    /// Per-burst tick index, reset at every pump start.
    n: u32,
    /// Synthesized samples carry fault=true over ticks [from, to).
    fault: Option<(u32, u32)>,
    track: Option<Track>,
}

/// Recorded fast-tick rows a servo plays back in place of the synthesized
/// samples. The cursor is sim time itself: row = fast ticks elapsed since
/// `base`, mod length - so the burst pump (ticking on the 50 us grid) and
/// the between-events table mirror read the same row at the same instant.
struct Track {
    rows: Vec<TelSample>,
    base: u64,
    /// Row last written into the live table; skips the rewrite while the
    /// same row is current (a dozen wire events per tick at 3 M).
    mirrored: Option<usize>,
}

impl Track {
    fn row(&self, now: u64) -> usize {
        ((now - self.base) / TEL_TICK) as usize % self.rows.len()
    }
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
            tels: Vec::new(),
            host_cross: None,
            rate,
            host_free_at: 0,
            host: None,
            link: None,
            self_reboot: false,
            data_jobs: true,
            break_wake: BreakWake::BeforeByte,
            alternate_after: false,
        }
    }

    /// Order every later break wake against its ringed 0x00
    /// ([`BreakWake::BeforeByte`] by default).
    pub fn set_break_wake(&mut self, w: BreakWake) {
        self.break_wake = w;
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

    /// A new host session on the pipe (the adapter's USB bus reset or
    /// SET_CONFIGURATION): undelivered records drop and the server resets
    /// its session, as the chip's main loop does (panics if no link).
    pub fn link_reopen(&mut self) {
        let rig = self.link.as_mut().expect("link attached");
        rig.out.0.clear();
        rig.server.reset_session();
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
        self.tels.push(TelPump {
            running: false,
            epoch: 0,
            n: 0,
            fault: None,
            track: None,
        });
        idx
    }

    /// Servo `i` plays `track` back at the fast-tick rate from now: bursts
    /// serve its rows in place of [`tel_sample`] (looping), and the live
    /// telemetry table mirrors the current row ahead of every event. Playback
    /// only - no control or fault behaviour follows from the rows. An empty
    /// track restores the synthesized samples.
    pub fn set_track(&mut self, i: usize, track: Vec<TelSample>) {
        let now = self.core.borrow().now();
        self.tels[i].track = (!track.is_empty()).then_some(Track {
            rows: track,
            base: now,
            mirrored: None,
        });
        self.mirror_track(i, now);
    }

    /// The track row servo `i` is at now (`None` without a track).
    pub fn track_row(&self, i: usize) -> Option<usize> {
        let now = self.core.borrow().now();
        self.tels[i].track.as_ref().map(|t| t.row(now))
    }

    /// Servo `i`'s synthesized samples carry fault=true over per-burst ticks
    /// [from, to) -- the sim's stand-in for a kernel fault window mid-burst.
    pub fn set_tel_fault_ticks(&mut self, i: usize, from: u32, to: u32) {
        self.tels[i].fault = Some((from, to));
    }

    /// Give servo `i`'s handler bodies sim-time cost (`cpu` module): events
    /// landing while a body runs pend and coalesce as on silicon. Zero-cost
    /// (the default) is the ideal-CPU model.
    pub fn set_handler_cost(&mut self, i: usize, cost: HandlerCost) {
        self.cpus[i].cost = cost;
    }

    /// Run servo `i`'s kernel lane from the next scan until `until_us`.
    pub fn set_kernel_lane(&mut self, i: usize, lane: KernelLane, until_us: u64) {
        let now = self.core.borrow().now();
        self.cpus[i].start_kernel(lane, until_us * TICKS_PER_US);
        self.core.borrow_mut().schedule(
            Event::KernelScan { servo: i },
            (now / KERNEL_PERIOD + 1) * KERNEL_PERIOD,
        );
    }

    pub fn kernel_stats(&self, i: usize) -> KernelStats {
        self.cpus[i].kernel_stats()
    }

    /// Servo `i`'s next handler body enters on time and is preempted for
    /// `us` before its first ring read: clock reads ahead of that read come
    /// back stale by the preemption.
    pub fn preempt_before_ring_read(&mut self, i: usize, us: u64) {
        self.cpus[i].preempt = Some(us * TICKS_PER_US);
    }

    /// `on_break` invocations delivered to servo `i` -- wire break events
    /// minus this counts pends that coalesced.
    /// Handler bodies servo `i` has run, per vector (PFIC HIGH entries).
    pub fn entries(&self, i: usize) -> Entries {
        self.cpus[i].entries()
    }

    pub fn delivered_breaks(&self, i: usize) -> u64 {
        self.cpus[i].delivered_breaks()
    }

    /// The most own frames one handler body of servo `i` dispatched: how far
    /// the ladder ran behind the wire.
    pub fn frames_per_body_max(&self, i: usize) -> u64 {
        self.cpus[i].frames_max()
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

    /// Inspect a servo's position table array behind the CONTROL window.
    pub fn servo_pos_lut<R>(&self, i: usize, f: impl FnOnce(&[i16; POINTS]) -> R) -> R {
        self.servos[i].with_pos_lut(f)
    }

    /// Give servo `i` another board's sense chain: install re-stamps these
    /// RO facts at every bringup, so a FACTORY wipe leaves them standing.
    /// Power-cycles the servo onto them - pre-traffic only.
    pub fn set_servo_sense(&mut self, i: usize, sense: CalibSense, sense_ext: CalibSenseExt) {
        self.servos[i].set_sense(sense, sense_ext);
    }

    /// Persist servo `i`'s live CONFIG, PROFILE and CALIB regions into its
    /// store, as MGMT SAVE would (sec 9.4) but without the wire: what a
    /// servo that left the bench calibrated carries in flash.
    pub fn persist_servo(&self, i: usize) {
        self.servos[i].persist();
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

    /// Model the chip main loop's reboot poll: a servo that staged a reboot
    /// (MGMT REBOOT, or FACTORY's self-reset) re-runs bringup mid-run once
    /// its ack has drained, so the store's verdict shows up on the wire
    /// without rebuilding the `Sim`. Off by default - scenarios that assert
    /// the staged mode consume it through [`Self::take_reboot`] instead.
    pub fn set_self_reboot(&mut self, on: bool) {
        self.self_reboot = on;
    }

    /// Model the chip main loop's data-job poll (`data_state` module): the
    /// checkpoint a covered or stamp write posts and the verdict a LUT
    /// COMMIT posts land after the reply, once the handler body returns. On
    /// by default; off, a scenario services servo `i` by hand through
    /// [`Self::poll_data_job`], or splits the run from its publish with
    /// [`Self::data_job_run`] and [`Self::data_job_publish`] to land a
    /// write mid-job.
    pub fn set_data_jobs(&mut self, on: bool) {
        self.data_jobs = on;
    }

    pub fn poll_data_job(&self, i: usize) -> bool {
        self.servos[i].poll_data_job()
    }

    pub fn data_job_run(&self, i: usize) -> Option<DataJob> {
        self.servos[i].data_job_run()
    }

    pub fn data_job_publish(&self, i: usize, job: DataJob) -> bool {
        self.servos[i].data_job_publish(job)
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

    /// Servo `i`'s oscillator rate becomes `ppm` at `at_us` - thermal drift:
    /// the rate changes, the clock never steps.
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
    /// bench-caught twice).
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
    /// each servo's main-loop sampler reads it low on the sample cadence
    /// (declaring once >= RESCUE_LOW_US of frozen-ring low has elapsed), and
    /// one ordinary break wake. Under [`BreakWake::AfterByte`] it latches at
    /// the pulse's END; otherwise it fires a break-length into the pulse and
    /// the rising edge only re-arms the detector.
    /// A pulse too short to cross the threshold delivers only the wake.
    pub fn hold_line_low_at(&mut self, at_us: u64, us: u64) {
        let start = self.clamp_at(at_us);
        let dur = us * TICKS_PER_US;
        let baud = self.rate;
        let mut c = self.core.borrow_mut();
        c.hold_low(start, start + dur);
        if self.break_wake == BreakWake::AfterByte {
            c.schedule(Event::StrayBreak { baud }, start + dur);
        } else if dur >= break_ticks(baud) {
            c.schedule(Event::PulseWake, start + break_ticks(baud));
        }
    }

    /// One main-loop line sample at every servo at `at_us` (sec 9.1).
    pub fn sample_line_at(&mut self, at_us: u64) {
        let at = self.clamp_at(at_us);
        self.core
            .borrow_mut()
            .schedule(Event::LineSample { pump: false }, at);
    }

    pub fn set_host_baud(&mut self, rate: BaudRate) {
        self.rate = rate;
    }

    /// Drain the event queue; return every frame observed since the last call.
    pub fn run(&mut self) -> Vec<WireFrame> {
        self.run_until_ticks(u64::MAX)
    }

    /// Drain events up to and including `at_us` - the main loop's turn
    /// between frames at a chosen instant (a trim poll and apply), with the
    /// traffic after it already queued at its true cadence. A frame queued
    /// only after `run` returns is clamped to `now`, which sits behind the
    /// servo's trailing wakes: the next frames serialize back-to-back and
    /// the compressed seams read as drift.
    pub fn run_until(&mut self, at_us: u64) -> Vec<WireFrame> {
        self.run_until_ticks(at_us * TICKS_PER_US)
    }

    fn run_until_ticks(&mut self, limit: u64) -> Vec<WireFrame> {
        loop {
            let ev = self.core.borrow_mut().pop_until(limit);
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
        let now = self.core.borrow().now();
        for j in 0..self.servos.len() {
            self.mirror_track(j, now);
        }
        match ev {
            Event::WireBreak {
                talker,
                baud,
                break_start,
            } => self.deliver_break(talker, baud, break_start),
            Event::WireData { talker, byte, baud } => self.deliver_data(talker, byte, baud),
            Event::WireGarble { byte } => self.deliver_garble(byte),
            Event::StrayBreak { baud } => self.deliver_stray_break(baud),
            Event::LineSample { pump } => self.sample_line(pump),
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
            Event::TelTick { servo, epoch } => self.tel_tick(servo, epoch),
            Event::CpuFree { servo } => self.cpu_free(servo),
            Event::KernelScan { servo } => self.kernel_scan(servo),
            Event::KernelRetry { servo } => {
                let now = self.core.borrow().now();
                if self.cpus[servo].take_kernel_retry(now) {
                    self.kernel_enter(servo);
                }
            }
            Event::WakeRefire { servo } => self.deliver(servo, Vector::Break),
            Event::BreakByte { servo } => self.handles[servo].ring.push(0x00),
            Event::PulseWake => self.deliver_pulse_wake(),
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
        self.tel_pump();
        self.job_pump();
        self.reboot_pump();
    }

    /// The servos' data-job poll after every event, the CPU permitting: on
    /// the chip the handler's return wakes the loop, so the job runs
    /// before the next frame can land.
    fn job_pump(&mut self) {
        if !self.data_jobs {
            return;
        }
        let now = self.core.borrow().now();
        for j in 0..self.servos.len() {
            if !self.cpus[j].busy(now) {
                self.servos[j].poll_data_job();
            }
        }
    }

    /// The servos' other main-loop residue: honor any staged reboot. A body
    /// mid-run owns the CPU, so the poll waits for it like the real loop.
    fn reboot_pump(&mut self) {
        if !self.self_reboot {
            return;
        }
        let now = self.core.borrow().now();
        for j in 0..self.servos.len() {
            if self.cpus[j].busy(now) || self.servos[j].take_reboot().is_none() {
                continue;
            }
            self.servos[j].reboot();
            // The burst died with the old driver: park the pump and retire
            // its scheduled ticks by epoch. A track outlives the reboot (it
            // is the rig's stimulus, not servo state) but must re-image,
            // since the rebuilt table lost the mirrored row.
            let t = &mut self.tels[j];
            t.running = false;
            t.epoch += 1;
            t.n = 0;
            if let Some(track) = t.track.as_mut() {
                track.mirrored = None;
            }
            self.cross[j] = CrossRx::new(self.rate);
        }
    }

    /// Start the fast-tick pump for any burst the event just armed.
    fn tel_pump(&mut self) {
        let now = self.core.borrow().now();
        for j in 0..self.servos.len() {
            if self.servos[j].tel_active() {
                if !self.tels[j].running {
                    let t = &mut self.tels[j];
                    t.running = true;
                    t.epoch += 1;
                    t.n = 0;
                    let at = (now / TEL_TICK + 1) * TEL_TICK;
                    self.core.borrow_mut().schedule(
                        Event::TelTick {
                            servo: j,
                            epoch: t.epoch,
                        },
                        at,
                    );
                }
            } else {
                self.tels[j].running = false;
            }
        }
    }

    /// One 50 us fast tick at servo `j`: synthesize (or play back) the next
    /// sample, feed the kernel-side encoder, re-arm. Ticks from a dead pump
    /// (burst ended or aborted since scheduling) drop by the running/epoch
    /// gates.
    fn tel_tick(&mut self, j: usize, epoch: u64) {
        let now = self.core.borrow().now();
        let t = &mut self.tels[j];
        if !t.running || t.epoch != epoch || !self.servos[j].tel_active() {
            return;
        }
        let mut s = match &t.track {
            Some(track) => track.rows[track.row(now)],
            None => tel_sample(t.n),
        };
        if let Some((from, to)) = t.fault {
            s.fault = t.n >= from && t.n < to;
        }
        t.n += 1;
        self.servos[j].tel_tick(&s);
        // the chip's tick tail polls; one landing inside a HIGH body leaves
        // the stage to the next tick
        if !self.cpus[j].busy(now) {
            self.servos[j].poll_tel();
        }
        self.core
            .borrow_mut()
            .schedule(Event::TelTick { servo: j, epoch }, now + TEL_TICK);
    }

    /// Image servo `j`'s current track row in its live table: the raw ADC
    /// frame into the sensors block, the kernel conclusions the row carries
    /// into the estimates block. Runs ahead of every event, so a reply
    /// streamed at this instant reads the row the burst pump would serve.
    fn mirror_track(&mut self, j: usize, now: u64) {
        let Some(t) = self.tels[j].track.as_mut() else {
            return;
        };
        let row = t.row(now);
        if t.mirrored == Some(row) {
            return;
        }
        t.mirrored = Some(row);
        let s = t.rows[row];
        self.servos[j].with_table_mut(|tb| {
            let sn = &mut tb.telemetry.sensors;
            sn.pos = s.pos;
            sn.current = s.current_raw;
            sn.current_trough = s.current_trough;
            sn.vmotor_a = s.vmotor_a;
            sn.vmotor_b = s.vmotor_b;
            sn.vbus_raw = s.vbus_raw;
            sn.ntc_raw = s.ntc_raw;
            let es = &mut tb.telemetry.estimates;
            es.vbus_counts = s.vbus;
            es.duty_applied_q15 = s.duty_q15;
            es.i_hat_counts = s.current;
        });
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
        if let Some(ticks) = self.cpus[j].preempt.take() {
            self.cpus[j].defer(now, v, ticks);
            self.schedule_free(j);
            return;
        }
        self.run_body(j, v);
    }

    fn run_body(&mut self, j: usize, v: Vector) {
        let now = self.core.borrow().now();
        self.cpus[j].charge(now, v);
        let before = self.servos[j].dispatched();
        match v {
            Vector::Compare => self.servos[j].on_deadline(),
            Vector::Break => self.servos[j].on_break(),
            Vector::TxDone => self.servos[j].on_tx_complete(),
        }
        self.cpus[j].charge_frames(self.servos[j].dispatched() - before);
        self.handles[j].clock_lag.set(0);
    }

    fn kernel_scan(&mut self, j: usize) {
        let now = self.core.borrow().now();
        if let Some(next) = self.cpus[j].kernel_scan(now) {
            self.core
                .borrow_mut()
                .schedule(Event::KernelScan { servo: j }, next);
        }
        self.kernel_enter(j);
    }

    fn kernel_enter(&mut self, j: usize) {
        let now = self.core.borrow().now();
        if let Some(at) = self.cpus[j].enter_kernel(now) {
            self.core
                .borrow_mut()
                .schedule(Event::KernelRetry { servo: j }, at);
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
        // A kernel above the bus takes the CPU ahead of any bus pend.
        self.kernel_enter(j);
        let now = self.core.borrow().now();
        if self.cpus[j].busy(now) {
            // A same-tick wire event beat this wake and re-occupied the CPU.
            if self.cpus[j].any_pend() {
                self.schedule_free(j);
            }
            return;
        }
        if let Some((v, entry)) = self.cpus[j].deferred.take() {
            self.handles[j].clock_lag.set(now - entry);
            self.run_body(j, v);
        } else if let Some(v) = self.cpus[j].take_pend() {
            self.run_vector(j, v);
        }
        if self.cpus[j].any_pend() {
            self.schedule_free(j);
        }
    }

    fn deliver_break(&mut self, talker: Talker, baud: BaudRate, break_start: u64) {
        self.core.borrow_mut().begin_frame(break_start, talker);
        for j in 0..self.servos.len() {
            if talker == Talker::Servo(j) {
                continue; // no own-TX echo (F9), the break wake muted for it
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
            self.wake_on_break(j, bit_ticks(rx) * 3 / 4);
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
                // A resampled batch rings in order, so its break byte lands
                // at the wake's own tick, still after the wake.
                RxOut::Break => self.wake_on_break(j, 0),
            }
        }
    }

    /// A qualified break at servo `j`: ring its 0x00 and wake, in the
    /// configured [`BreakWake`] order, the byte `lead` ticks behind the wake
    /// under [`BreakWake::BeforeByte`].
    fn wake_on_break(&mut self, j: usize, lead: u64) {
        let after = match self.break_wake {
            BreakWake::BeforeByte => false,
            BreakWake::AfterByte => true,
            BreakWake::Alternating => {
                self.alternate_after = !self.alternate_after;
                !self.alternate_after
            }
        };
        if after {
            self.handles[j].ring.push(0x00);
            self.deliver(j, Vector::Break);
            return;
        }
        self.deliver(j, Vector::Break);
        if lead == 0 {
            self.handles[j].ring.push(0x00);
        } else {
            let mut c = self.core.borrow_mut();
            let at = c.now() + lead;
            c.schedule(Event::BreakByte { servo: j }, at);
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
        // Mid-frame noise corrupts the recorded image too -- every listener
        // rings the byte, so the record (a host collector's view of an
        // unsolicited frame) must carry it.
        {
            let mut c = self.core.borrow_mut();
            let now = c.now();
            c.append_byte(byte, now);
        }
        for h in &self.handles {
            h.ring.push(byte);
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

    /// A dominant low held a break's length: a break at every receiver's own
    /// rate (the low is baud-agnostic, nothing to resample).
    fn deliver_pulse_wake(&mut self) {
        for j in 0..self.servos.len() {
            let rx = self.handles[j].baud.current();
            self.wake_on_break(j, bit_ticks(rx) * 3 / 4);
        }
        if let Some(h) = &self.host {
            h.ring.push(0x00);
        }
    }

    /// The samplers are thread-level - no vector, no CPU pend - mirroring the
    /// chip's main-loop critical section.
    fn sample_line(&mut self, pump: bool) {
        for j in 0..self.servos.len() {
            let low = self.core.borrow().line_low(j);
            self.servos[j].sample_rescue(low);
        }
        let mut c = self.core.borrow_mut();
        if pump && c.held_now() {
            let at = c.now() + TEL_TICK;
            c.schedule(Event::LineSample { pump }, at);
        }
    }
}
