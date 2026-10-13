//! The driver loop from osc-ident's exp module doc: Write -> wire write,
//! Read -> telemetry gread + parse, the latest watched window attached
//! ([`Windows`]), Pause -> a watch: TEL bursts of its whole windows, each
//! folded and handed to the experiment as it lands, the pause ended at the
//! first guard trip (a pause too short for a window sleeps), Stream -> one
//! TEL burst on the main bus (HOLD+COMMIT when it carries a goal), Done ->
//! break. A stall permit the experiment holds is rewritten between
//! commands and between watch bursts while the lease runs.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::pipe::Pipe;
use osc_client::{BaudRate, Id, Inst, Opcode, Outcome, ResultCode};
use osc_ident::burst::{self, ArmSeen, BurstIo, Capture, CaptureCfg, Pre};
use osc_ident::exp::{Cmd, Experiment};
use osc_ident::frame::{StreamAssembler, TelBurst, TelFrame, TelemetrySnapshot, chunks_text};
use osc_ident::limits::PermitLease;
use osc_ident::regs::{Reg, calib, config, control, telemetry};
use osc_ident::units::{self, SenseParams};
use osc_ident::window::{WATCH_BURST_MS, Watch, Windows, watch_rows};
use osc_protocol::build;
use osc_protocol::tel::{self, Chunk, Chunks};

use super::csvio::SnapshotLog;
use super::servo::{SAFE, Servo, Wire};

pub(crate) static STOP: AtomicBool = AtomicBool::new(false);

pub(crate) fn install_ctrlc() {
    let _ = ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst));
}

/// Write one register, value LE-truncated to the field width (negative
/// i32 -> correct two's complement for 2/4-byte fields).
pub(crate) fn write_reg<P: Pipe>(c: &mut Client<P>, id: Id, reg: Reg, value: i32) -> Result<()> {
    let bytes = value.to_le_bytes();
    c.write(id, reg.addr, &bytes[..reg.width as usize])
        .with_context(|| format!("write addr {:#06x}", reg.addr))?;
    Ok(())
}

pub(crate) fn read_i32<P: Pipe>(c: &mut Client<P>, id: Id, reg: Reg) -> Result<i32> {
    let raw = c.read(id, reg.addr, 4).context("field read")?;
    Ok(i32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
}

const TEL_BASE: u16 = telemetry::FAULT_FLAGS.addr;
const TEL_LEN: u16 = telemetry::WINDOW_FLOOR_Q15.addr + 2 - TEL_BASE;

/// One telemetry snapshot, no window.
pub(crate) fn read_snapshot<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<TelemetrySnapshot> {
    read_stamped(c, id, Instant::now())
}

/// [`read_snapshot`] stamped with the wall clock since `t0`, ms, at the
/// middle of the region read: the time an experiment's fits take, since
/// every read stalls the servo's own tick counter.
pub(crate) fn read_stamped<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    t0: Instant,
) -> Result<TelemetrySnapshot> {
    let ms = || t0.elapsed().as_secs_f64() * 1000.0;
    let before = ms();
    let raw = c.read(id, TEL_BASE, TEL_LEN).context("telemetry read")?;
    let mut snap =
        TelemetrySnapshot::parse(TEL_BASE, &raw).context("telemetry parse (short read?)")?;
    snap.host_ms = (before + ms()) / 2.0;
    Ok(snap)
}

/// Run the closure, then force the servo safe (duty/goals zero, torque,
/// stall permit and TEL off) whether it succeeded, failed, or was
/// ctrl-c'd.
/// A hard kill skips this - a leased permit then runs out within a second
/// (a plain-level one stays until a reboot), and the servo's own
/// protections are the backstop.
pub(crate) fn with_guard<P: Pipe, T>(
    c: &mut Client<P>,
    id: Id,
    f: impl FnOnce(&mut Client<P>) -> Result<T>,
) -> Result<T> {
    let r = f(c);
    for (reg, v) in SAFE {
        let _ = write_reg(c, id, reg, v);
    }
    r
}

/// The stall permit lease on the servo's clock ([`PermitLease`]): writes go
/// out through it so it sees torque and permit, and `keep` rewrites a held
/// permit when a refresh is due. With `hold` set, every torque enable is
/// followed by the permit.
pub(crate) struct Lease {
    state: PermitLease,
    hold: bool,
}

impl Lease {
    pub(crate) fn new(hold: bool) -> Self {
        Self {
            state: PermitLease::default(),
            hold,
        }
    }

    pub(crate) fn write<S: Servo>(&mut self, s: &mut S, reg: Reg, value: i32) -> Result<()> {
        s.write(reg, value)?;
        self.state.wrote(reg, value, s.now_ms());
        Ok(())
    }

    /// Torque on, then the permit when this run holds one.
    pub(crate) fn torque_on<S: Servo>(&mut self, s: &mut S) -> Result<()> {
        self.write(s, control::TORQUE_ENABLE, 1)?;
        if self.hold {
            self.write(s, control::STALL_PERMIT, 1)?;
        }
        Ok(())
    }

    /// The longest stretch of a pause before the permit is due again.
    pub(crate) fn slice(&self, ms: u32) -> u32 {
        self.state.slice(ms)
    }

    pub(crate) fn keep<S: Servo>(&mut self, s: &mut S) -> Result<()> {
        if self.state.due(s.now_ms()) {
            self.write(s, control::STALL_PERMIT, 1)?;
        }
        Ok(())
    }

    /// Refuse a TEL burst the held permit would lapse inside.
    pub(crate) fn check_stream(&self, samples: u16) -> Result<()> {
        self.state
            .check_stream(stream_span(samples).as_millis() as u32)?;
        Ok(())
    }
}

/// The sampled span of a TEL burst: one sample per 50 us fast tick.
fn stream_span(samples: u16) -> Duration {
    Duration::from_micros(samples as u64 * 50)
}

/// Whole-burst window: the sampled span plus wire/turnaround margin. The
/// client pipe guard must sit above it (see exchange_stream).
fn stream_window(samples: u16) -> Duration {
    stream_span(samples) * 5 / 4 + Duration::from_millis(250)
}

/// Per-burst evidence for the diag line.
pub(crate) struct BurstStats {
    pub(crate) frames: usize,
    pub(crate) samples: usize,
    pub(crate) holes: u64,
    pub(crate) garble: u16,
    /// Whole adapter RX rings lost unread; their frames are among `holes`.
    pub(crate) laps: u16,
    /// TEL rows the servo dropped from the burst: [`Servo::stream`] reads
    /// them off the table, a bare burst leaves 0.
    pub(crate) rows_dropped: u16,
}

/// Chunks a burst can stream, or why not in words: the servo's own arm
/// rules ([`Chunks::valid`]). Whether the row fits the line is the servo's
/// call ([`admit_tel_chunks`]).
pub(crate) fn check_tel_chunks(chunks: Chunks) -> Result<Chunks> {
    if !chunks.valid() {
        bail!(
            "TEL chunks {} are not a row: each chunk starts and ends on an even address \
             below {}, the first one is not empty, and the row is at most {} B",
            chunks_text(&chunks),
            tel::TABLE_BYTES,
            tel::ROW_BYTES_MAX
        );
    }
    Ok(chunks)
}

/// Chunks as a user writes them, `addr+len` joined by `,` (`0x238+2`,
/// `0x232+6,0x23c+6`; addresses hex or decimal), checked against the arm
/// rules.
pub(crate) fn parse_tel_chunks(text: &str) -> Result<Chunks> {
    let mut chunks = Chunks::NONE;
    for (n, part) in text.split(',').enumerate() {
        let (addr, len) = part
            .trim()
            .split_once('+')
            .with_context(|| format!("TEL chunk {part:?} is not addr+len"))?;
        let c = chunks
            .0
            .get_mut(n)
            .with_context(|| format!("{text:?}: a row has at most {} chunks", tel::CHUNKS_MAX))?;
        *c = tel::Chunk::new(crate::parse_u16(addr)?, crate::parse_u16(len)?);
    }
    check_tel_chunks(chunks)
}

/// The servo's TEL admission, read off its identity block: the line's rate
/// and the longest row a burst arms with (protocol sec 5.6).
pub(crate) fn tel_row_limit<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<(BaudRate, usize)> {
    use osc_protocol::table::{BAUD_RATE_IDX, CAP_TEL_ROWS, CAPABILITY_FLAGS, TEL_ROW_BYTES_MAX};
    let n = BAUD_RATE_IDX + 1 - CAPABILITY_FLAGS;
    let raw = c.read(id, CAPABILITY_FLAGS, n).context("tel row limit")?;
    let at = |addr: u16| raw.get((addr - CAPABILITY_FLAGS) as usize).copied();
    let caps = u32::from_le_bytes(
        raw.get(..4)
            .and_then(|b| b.try_into().ok())
            .unwrap_or([0; 4]),
    );
    if caps & CAP_TEL_ROWS == 0 {
        bail!("the servo publishes no TEL row limits: its firmware predates packed TEL rows");
    }
    let idx = at(BAUD_RATE_IDX).context("short identity read")?;
    let baud = BaudRate::from_idx(idx).with_context(|| format!("baud_rate_idx {idx}"))?;
    let max = at(TEL_ROW_BYTES_MAX).context("short identity read")?;
    Ok((baud, max as usize))
}

/// What a window folds, duty to the differential with the trough riding
/// along: the one place a drive's watch selects its fields.
const WINDOW_CHUNKS: Chunks = Chunks::one(
    telemetry::DUTY_TICKS.addr,
    telemetry::VDIFF_COUNTS.addr + 2 - telemetry::DUTY_TICKS.addr,
);

/// The watch with the pot: the guard judges every row's position and the
/// ladder pairs every window with its own.
pub(crate) const WATCH_CHUNKS: Chunks =
    Chunks::two(Chunk::new(telemetry::POS.addr, 2), WINDOW_CHUNKS.0[0]);

/// The chunks a watch streams on a line whose rows hold `row_bytes_max`:
/// the pot too when it fits, else the window's alone, the pot then read at
/// the region reads; None when not even those fit.
fn watch_selection(row_bytes_max: usize) -> Option<Chunks> {
    [WATCH_CHUNKS, WINDOW_CHUNKS]
        .into_iter()
        .find(|c| c.row_bytes() <= row_bytes_max)
}

/// The chunks a drive's watches stream on this line ([`watch_selection`]),
/// or, on a line that streams no TEL or too short a row for the window's
/// own fields, a refusal before anything drives.
pub(crate) fn admit_watch<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Chunks> {
    let (baud, max) = tel_row_limit(c, id)?;
    if baud != tel::BAUD {
        bail!(
            "identification watches its drives on a TEL stream, which the servo streams at {} \
             Mbaud only; the bus runs at {} kbaud. Move the bus to {}M first (osc baud {}), on \
             a short cable with good contacts.",
            tel::BAUD.as_hz() / 1_000_000,
            baud.as_hz() / 1000,
            tel::BAUD.as_hz() / 1_000_000,
            tel::BAUD.as_hz()
        );
    }
    watch_selection(max).with_context(|| {
        format!(
            "identification watches its drives on TEL rows of duty, current and vdiff, {} B; \
             the servo streams rows of at most {max} B (protocol sec 5.6).",
            WINDOW_CHUNKS.row_bytes()
        )
    })
}

/// Chunks the servo streams on a line at `baud` with rows of at most `max`
/// B, or why not in words. The servo acks an arm at any other rate and
/// never streams.
pub(crate) fn admit_tel_chunks(chunks: Chunks, baud: BaudRate, max: usize) -> Result<Chunks> {
    if baud != tel::BAUD {
        bail!(
            "TEL streams at {} Mbaud only and the bus runs at {} kbaud: raise the bus rate to \
             {}M, and use a short cable with good contacts",
            tel::BAUD.as_hz() / 1_000_000,
            baud.as_hz() / 1000,
            tel::BAUD.as_hz() / 1_000_000
        );
    }
    let row = check_tel_chunks(chunks)?.row_bytes();
    if row > max {
        bail!(
            "TEL chunks {} make {row} B rows; the servo streams rows of at most {max} B (its \
             tel_row_bytes_max, protocol sec 5.6)",
            chunks_text(&chunks)
        );
    }
    Ok(chunks)
}

/// The one WRITE that sets the row and arms the burst: the four chunk
/// registers, then `tel_count`.
fn tel_arm(chunks: &Chunks, samples: u16) -> [u8; 4 * tel::CHUNKS_MAX + 2] {
    let mut arm = [0; 4 * tel::CHUNKS_MAX + 2];
    let (regs, count) = arm.split_at_mut(4 * tel::CHUNKS_MAX);
    regs.copy_from_slice(&chunks.to_le_bytes());
    count.copy_from_slice(&samples.to_le_bytes());
    arm
}

const _: () =
    assert!(control::CHUNK0_ADDR.addr + 4 * tel::CHUNKS_MAX as u16 == control::TEL_COUNT.addr);

/// One TEL burst on the main bus, refused before anything is sent unless
/// the servo would stream `chunks` at the line's rate
/// ([`stream_tel_burst`]).
pub(crate) fn exchange_tel_burst<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    samples: u16,
    goal: Option<(Reg, i32)>,
    chunks: Chunks,
) -> Result<(Vec<TelFrame>, BurstStats)> {
    let (baud, max) = tel_row_limit(c, id)?;
    admit_tel_chunks(chunks, baud, max)?;
    let floor = super::snapshot::read_u16(c, id, calib::I_WINDOW_MIN_TICKS)?;
    stream_tel_burst(c, id, samples, goal, chunks, floor)
}

/// One TEL burst on the main bus of `chunks` rows. With `goal` Some the
/// goal write and the arm are staged under HOLD and a broadcast COMMIT (the
/// stream carrier) applies both in the same instant; with None the acked
/// arm write itself carries the stream. Frames decode through
/// [`StreamAssembler`], the window judged against `floor`, the servo's
/// `i_window_min_ticks`; corrupt frames never appear (the adapter drops
/// them into garble + a seq hole).
pub(crate) fn stream_tel_burst<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    samples: u16,
    goal: Option<(Reg, i32)>,
    chunks: Chunks,
    floor: u16,
) -> Result<(Vec<TelFrame>, BurstStats)> {
    let (frames, stats, whole, _) = stream_frames(c, id, samples, goal, chunks, floor)?;
    if !whole {
        bail!(
            "tel burst timed out mid-stream ({} of {} samples)",
            stats.samples,
            samples
        );
    }
    Ok((frames, stats))
}

/// One watch burst of `rows` of `chunks`: its rows and whether a frame
/// flagged a fault. A frame the line lost leaves the rows short, which the
/// fold counts: lost rows, never the run. The fold judges no window, so no
/// floor is read.
pub(crate) fn watch_rows_on<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    rows: u16,
    chunks: Chunks,
) -> Result<(Vec<TelFrame>, bool)> {
    let (frames, _, _, alert) = stream_frames(c, id, rows, None, chunks, 0)?;
    Ok((frames, alert))
}

/// [`stream_tel_burst`]'s exchange: the rows, the burst's evidence, false
/// when the LAST frame never came, and whether a frame flagged a fault.
fn stream_frames<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    samples: u16,
    goal: Option<(Reg, i32)>,
    chunks: Chunks,
    floor: u16,
) -> Result<(Vec<TelFrame>, BurstStats, bool, bool)> {
    let mut asm = StreamAssembler::new(chunks, floor).context("tel chunks invalid for an arm")?;
    let arm = tel_arm(&chunks, samples);
    let window = stream_window(samples);
    c.set_guard(window + Duration::from_secs(1));
    let reply = match goal {
        Some((reg, value)) => {
            let bytes = value.to_le_bytes();
            c.write_hold(id, reg.addr, &bytes[..reg.width as usize])
                .context("hold goal")?;
            c.write_hold(id, control::CHUNK0_ADDR.addr, &arm)
                .context("hold tel arm")?;
            let inst = Inst::instruction(Opcode::Commit, 0);
            c.exchange_stream(Id::BROADCAST, inst, &[], window)
        }
        None => {
            let mut p = [0u8; 2 + 4 * tel::CHUNKS_MAX + 2];
            let n =
                build::write(&mut p, control::CHUNK0_ADDR.addr, &arm).context("tel arm payload")?;
            let inst = Inst::instruction(Opcode::Write, 0);
            c.exchange_stream(id, inst, &p[..n], window)
        }
    };
    c.set_guard(Duration::from_secs(2));
    let reply = reply?;
    if let Some(ack) = &reply.ack
        && ack.result != Some(ResultCode::Ok)
    {
        bail!("stream arm answered {:?}", ack.result);
    }
    let mut frames = Vec::new();
    for f in &reply.frames {
        asm.push(
            f.result == Some(ResultCode::Stream),
            f.alert,
            &f.payload,
            &mut frames,
        );
    }
    let stats = BurstStats {
        frames: reply.frames.len(),
        samples: frames.len(),
        holes: asm.holes() + asm.skipped(),
        garble: reply.garble,
        laps: reply.laps,
        rows_dropped: 0,
    };
    let whole = !matches!(reply.outcome, Outcome::Timeout { .. });
    Ok((frames, stats, whole, asm.alert()))
}

/// The burst handshake's wire moves. The arm has to be atomic - a servo
/// that sees arm=1 before the new duty or chans captures the old one - so
/// duty, chans and arm go out under HOLD and one broadcast COMMIT applies
/// all three. The chans byte is its own one-byte write: the byte after it
/// is a reserved alignment byte that refuses writes. Before them
/// `tel_count` goes to 0: the servo refuses an arm while a TEL count is
/// armed, which it is only while a stream runs (the servo zeroes it when
/// the stream ends).
struct WireBurstIo<'a, P: Pipe> {
    c: &'a mut Client<P>,
    id: Id,
}

impl<P: Pipe> BurstIo for WireBurstIo<'_, P> {
    type Error = anyhow::Error;

    fn arm(&mut self, duty_q15: i16, chans: u8) -> Result<()> {
        write_reg(self.c, self.id, control::TEL_COUNT, 0).context("disarm tel")?;
        self.c
            .write_hold(self.id, burst::wire::DUTY_Q15.addr, &duty_q15.to_le_bytes())
            .context("hold burst duty")?;
        self.c
            .write_hold(self.id, burst::wire::CHANS.addr, &[chans])
            .context("hold burst chans")?;
        self.c
            .write_hold(self.id, burst::wire::ARM.addr, &[1])
            .context("hold burst arm")?;
        self.c.commit().context("commit burst arm")?;
        Ok(())
    }

    fn select_page(&mut self, page: u8) -> Result<()> {
        write_reg(self.c, self.id, burst::wire::PAGE, page as i32)
    }

    fn release(&mut self) -> Result<()> {
        write_reg(self.c, self.id, burst::wire::ARM, 0)
    }

    fn read_burst(&mut self, addr: u16, len: u16) -> Result<Vec<u8>> {
        self.c
            .read(self.id, addr, len)
            .with_context(|| format!("burst read {addr:#06x}"))
    }

    fn pause_ms(&mut self, ms: u32) {
        std::thread::sleep(Duration::from_millis(ms as u64));
    }
}

/// One high-rate capture. The rail, the current-sense zero, the terminal
/// divider bias and the position are read BEFORE the arm: the burst
/// suspends the scan.
pub(crate) fn capture_burst<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    duty_q15: i16,
    pre_q15: i16,
    chans: u8,
    seated: bool,
) -> Result<Capture> {
    let pre = Pre {
        pre_q15,
        vbus_raw: super::snapshot::read_u16(c, id, telemetry::VBUS_RAW)?,
        bias: super::snapshot::read_u16(c, id, telemetry::CURRENT_BIAS_COUNTS)?,
        vmotor_bias: super::snapshot::read_u16(c, id, telemetry::VMOTOR_BIAS_COUNTS)?,
        pos: super::snapshot::read_u16(c, id, telemetry::POS)?,
        seated,
        drive_polarity: super::snapshot::read_bool(c, id, config::DRIVE_POLARITY)?,
    };
    let mut io = WireBurstIo { c, id };
    match burst::capture(&mut io, duty_q15, chans, pre, &CaptureCfg::default()) {
        Ok(cap) => Ok(cap),
        Err(burst::Error::Rejected) => bail!("{}", burst::rejected(duty_q15, &arm_seen(c, id)?)),
        Err(e) => bail!("burst capture: {e}"),
    }
}

/// What the servo judged a refused arm by: the rail, the position against
/// the soft limits, the permit, a fault.
fn arm_seen<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<ArmSeen> {
    let tel = read_snapshot(c, id)?;
    let sense = SenseParams {
        shunt_r_mohm: 0,
        gain_milli: 0,
        vmotor_div_top: super::snapshot::read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: super::snapshot::read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        vdd_mv: super::snapshot::read_u16(c, id, calib::VDD_MV)?,
        tick_hz: 0,
    };
    Ok(ArmSeen {
        vbus_counts: tel.vbus_counts,
        mv_per_count: units::volts_per_count(&sense) * 1000.0,
        pos: tel.pos,
        soft: (
            read_i32(c, id, config::POS_MIN_SOFT_COUNTS)?,
            read_i32(c, id, config::POS_MAX_SOFT_COUNTS)?,
        ),
        limit_flags: tel.limit_flags,
        fault_flags: tel.fault_flags,
    })
}

/// What a run's watches stream and how they fold, read off the servo once.
struct Watching {
    chunks: Chunks,
    tick_hz: f64,
    pwm: u16,
    windows: Windows,
}

impl Watching {
    fn get<'w, S: Servo>(slot: &'w mut Option<Self>, s: &mut S) -> Result<&'w mut Self> {
        if slot.is_none() {
            let pwm = s.pwm_period_ticks()?;
            *slot = Some(Self {
                chunks: s.watch_chunks()?,
                tick_hz: s.tick_hz()?,
                pwm,
                windows: Windows::new(pwm),
            });
        }
        Ok(slot.as_mut().expect("set above"))
    }
}

pub(crate) struct Pump<'a, S: Servo = Wire<&'a mut Client<NusbPipe>>> {
    servo: S,
    log: Option<&'a mut SnapshotLog>,
    /// The watch, once the run's first read or pause admitted the line.
    watch: Option<Watching>,
    /// Rows the watches lacked.
    lost: u32,
    /// Every decoded frame across the run's bursts, in order - the CSV log
    /// source (the experiment gets the same frames via push_tel).
    pub(crate) tel: Vec<TelFrame>,
    /// The experiment writes its own permit; this only keeps it alive.
    lease: Lease,
    /// The servo's PWM period, read at the first burst.
    pwm_period_ticks: Option<u16>,
}

impl<'a> Pump<'a> {
    pub(crate) fn new(
        client: &'a mut Client<NusbPipe>,
        id: Id,
        log: Option<&'a mut SnapshotLog>,
    ) -> Self {
        Pump::on(Wire::new(client, id), log)
    }
}

impl<'a, S: Servo> Pump<'a, S> {
    pub(crate) fn on(servo: S, log: Option<&'a mut SnapshotLog>) -> Self {
        Self {
            servo,
            log,
            watch: None,
            lost: 0,
            tel: Vec::new(),
            lease: Lease::new(false),
            pwm_period_ticks: None,
        }
    }

    /// Run one experiment to completion.
    pub(crate) fn run(&mut self, exp: &mut dyn Experiment) -> Result<()> {
        let mut pending: Option<TelemetrySnapshot> = None;
        let s = &mut self.servo;
        loop {
            if STOP.load(Ordering::SeqCst) {
                bail!("interrupted");
            }
            self.lease.keep(s)?;
            match exp.step(pending.take().as_ref()) {
                Cmd::Write { reg, value } => self.lease.write(s, reg, value)?,
                Cmd::Read => {
                    let w = Watching::get(&mut self.watch, s)?;
                    let mut snap = s.snapshot()?;
                    w.windows.attach(&mut snap);
                    if let Some(log) = self.log.as_mut() {
                        log.push(snap.host_ms, &snap, w.pwm)?;
                    }
                    pending = Some(snap);
                }
                Cmd::Pause { ms } => {
                    let w = Watching::get(&mut self.watch, s)?;
                    let mut left = ms;
                    while left > 0 && exp.halted().is_none() {
                        if STOP.load(Ordering::SeqCst) {
                            bail!("interrupted");
                        }
                        let slice = self.lease.slice(left).min(WATCH_BURST_MS);
                        let end = s.now_ms() + slice as f64;
                        let rows = watch_rows(slice as f64, w.tick_hz);
                        if rows > 0 {
                            let (frames, alert) = s.watch(rows, w.chunks)?;
                            let watch =
                                Watch::fold(&frames, rows, alert, s.now_ms(), w.tick_hz, w.pwm);
                            self.lost += watch.lost as u32;
                            w.windows.watched(&watch);
                            exp.push_watch(&watch);
                        }
                        // 5 ms sleeps keep ctrl-c prompt while nothing streams
                        while s.now_ms() < end && exp.halted().is_none() {
                            s.sleep(((end - s.now_ms()).ceil() as u32).clamp(1, 5));
                        }
                        left -= slice;
                        self.lease.keep(s)?;
                    }
                }
                Cmd::Stream {
                    samples,
                    goal,
                    chunks,
                } => {
                    self.lease.check_stream(samples)?;
                    let (frames, st) = s.stream(samples, goal, chunks)?;
                    eprintln!(
                        "tel: {} frames, {} samples, {} seq holes, {} garble bytes, {} ring laps",
                        st.frames, st.samples, st.holes, st.garble, st.laps
                    );
                    let pwm_period_ticks = match self.pwm_period_ticks {
                        Some(p) => p,
                        None => *self.pwm_period_ticks.insert(s.pwm_period_ticks()?),
                    };
                    let tel = TelBurst {
                        frames,
                        rows_dropped: st.rows_dropped,
                        pwm_period_ticks,
                    };
                    exp.push_tel(&tel);
                    self.tel.extend_from_slice(&tel.frames);
                }
                Cmd::Burst {
                    duty_q15,
                    pre_q15,
                    chans,
                    seated,
                } => {
                    let id = s.id();
                    let cap = capture_burst(s.client(), id, duty_q15, pre_q15, chans, seated)?;
                    eprintln!(
                        "burst: {} samples, step at {}, duty {} (pre {}), chans {} frame_len {}, \
                         pos {}{}",
                        cap.samples.len(),
                        cap.meta.step_index,
                        duty_q15,
                        pre_q15,
                        cap.meta.chans,
                        cap.meta.frame_len,
                        cap.meta.pos,
                        if seated { " seated" } else { "" }
                    );
                    exp.push_burst(&cap);
                }
                Cmd::Done => {
                    if self.lost > 0 {
                        eprintln!(
                            "warning: the watches lacked {} rows (lost frames or dropped rows)",
                            self.lost
                        );
                    }
                    return Ok(());
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use osc_ident::frame::{TEL_LADDER, TEL_POS, TEL_RAW, TEL_RL};

    /// Admission refuses a line under 3 M before anything else, then the
    /// arm rules, then the servo's row limit.
    #[test]
    fn chunks_the_servo_would_not_stream_say_why() {
        let err = |c, b, max| admit_tel_chunks(c, b, max).unwrap_err().to_string();
        assert_eq!(
            err(TEL_RAW, BaudRate::B2000000, 12),
            "TEL streams at 3 Mbaud only and the bus runs at 2000 kbaud: raise the bus rate to \
             3M, and use a short cable with good contacts"
        );
        assert_eq!(
            admit_tel_chunks(TEL_RAW, BaudRate::B3000000, 12).unwrap(),
            TEL_RAW
        );
        assert_eq!(
            err(Chunks::one(0x225, 2), BaudRate::B3000000, 12),
            "TEL chunks 0x225+2 are not a row: each chunk starts and ends on an even address \
             below 1024, the first one is not empty, and the row is at most 12 B"
        );
        assert_eq!(
            err(TEL_RAW, BaudRate::B3000000, 8),
            "TEL chunks 0x232+8,0x240+4 make 12 B rows; the servo streams rows of at most 8 B (its \
             tel_row_bytes_max, protocol sec 5.6)"
        );
        assert!(err(Chunks::NONE, BaudRate::B3000000, 12).contains("not a row"));
    }

    /// The limit comes off the servo's own identity block, and a line
    /// under 3 M is refused before the arm goes out: the chunk registers
    /// and `tel_count` still read zero.
    #[test]
    fn a_burst_under_three_megabaud_is_refused_before_the_arm() {
        use osc_client::fake::FakePipe;
        let pipe = FakePipe::new(BaudRate::B1000000, &[1]);
        let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
        assert_eq!(tel_row_limit(&mut c, id).unwrap(), (BaudRate::B1000000, 12));
        let Err(refused) = exchange_tel_burst(&mut c, id, 16, None, TEL_LADDER) else {
            panic!("a burst armed at 1 M");
        };
        assert!(
            refused
                .to_string()
                .starts_with("TEL streams at 3 Mbaud only"),
            "{refused}"
        );
        let regs = c.read(id, control::CHUNK0_ADDR.addr, 10).unwrap();
        assert_eq!(regs, [0; 10]);
    }

    /// A burst through the in-process servo stack decodes, field for field,
    /// the table bytes its rows copied from the sim's samples: the presets,
    /// the rail estimate in a second chunk, and a row with `tick_flags`.
    /// The window is the servo's `i_window_min_ticks` on the duty, or the
    /// flags themselves.
    #[test]
    fn bursts_decode_what_the_servo_copied() {
        use super::super::snapshot::read_u16;
        use osc_client::fake::{FakePipe, tel_sample};
        use osc_ident::frame::window_valid;
        use osc_ident::regs::telemetry as t;
        use osc_protocol::tel::Chunk;
        let one = |r: Reg| Chunk::new(r.addr, r.width as u16);
        for chunks in [
            TEL_RAW,
            TEL_RL,
            TEL_LADDER,
            TEL_POS,
            Chunks::two(Chunk::new(0x240, 8), one(t::VBUS_COUNTS)),
            Chunks::two(Chunk::new(0x238, 4), one(t::TICK_FLAGS)),
        ] {
            let pipe = FakePipe::new(BaudRate::B3000000, &[1]);
            let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
            let floor = read_u16(&mut c, id, calib::I_WINDOW_MIN_TICKS).unwrap();
            let (frames, st) = exchange_tel_burst(&mut c, id, 40, None, chunks).unwrap();
            assert_eq!((frames.len(), st.holes), (40, 0), "{chunks:?}");
            let sel = |r: Reg| chunks.offset(r.addr, r.width as u16).is_some();
            for f in &frames {
                let (s, vbus) = {
                    let s = tel_sample(f.tick as u32);
                    (s.tick, s.vbus_counts)
                };
                let duty = sel(t::DUTY_TICKS).then_some(s.duty_ticks);
                let want = TelFrame {
                    tick: f.tick,
                    window_valid: if sel(t::TICK_FLAGS) {
                        s.tick_flags & osc_ident::frame::tick_flag::I_VALID != 0
                    } else {
                        duty.is_some_and(|d| window_valid(d, floor))
                    },
                    pos: sel(t::POS).then_some(s.pos),
                    current: sel(t::I_TICK_COUNTS).then_some(s.i_tick_counts),
                    current_trough: sel(t::CURRENT_TROUGH).then_some(s.current_trough),
                    duty_counts: duty,
                    vdiff: sel(t::VDIFF_COUNTS).then_some(s.vdiff_counts),
                    vbus: sel(t::VBUS_COUNTS).then_some(vbus),
                    current_raw: sel(t::CURRENT_RAW).then_some(s.current_raw),
                    vmotor_a: sel(t::VMOTOR_A).then_some(s.vmotor_a),
                    vmotor_b: sel(t::VMOTOR_B).then_some(s.vmotor_b),
                    vbus_raw: sel(t::VBUS_RAW).then_some(s.vbus_raw),
                    ntc_raw: sel(t::NTC_RAW).then_some(s.ntc_raw),
                };
                assert_eq!(*f, want, "{chunks:?}");
            }
            let armed = c.read(id, control::CHUNK0_ADDR.addr, 8).unwrap();
            assert_eq!(armed, chunks.to_le_bytes(), "one write set the row");
        }
    }

    #[test]
    fn chunks_parse_as_the_user_writes_them() {
        assert_eq!(parse_tel_chunks("0x232+8,0x240+4").unwrap(), TEL_RAW);
        assert_eq!(parse_tel_chunks("0x232+6, 0x23c+6").unwrap(), TEL_RL);
        assert_eq!(parse_tel_chunks("568+2").unwrap(), TEL_POS);
        assert_eq!(parse_tel_chunks(&chunks_text(&TEL_RL)).unwrap(), TEL_RL);
        assert!(parse_tel_chunks("0x228").is_err());
        assert!(parse_tel_chunks("0x224+2,0x228+2,0x22c+2").is_err());
        assert!(parse_tel_chunks("0x229+2").is_err());
    }

    /// The arm is the chunks then the count, LE: one 10 B WRITE at
    /// `chunk0_addr`.
    #[test]
    fn the_arm_is_the_chunks_then_the_count() {
        assert_eq!(
            tel_arm(&TEL_RL, 4000),
            [0x32, 0x02, 6, 0, 0x3C, 0x02, 6, 0, 0xA0, 0x0F]
        );
    }

    /// A watch through the in-process servo stack: one burst of three
    /// whole windows of the chunks the line admits, folded to the floor
    /// means of what the servo copied, the pot's extremes kept.
    #[test]
    fn a_watch_streams_whole_through_the_servo_stack() {
        use osc_client::fake::{FakePipe, TelemetryTick, tel_sample};
        let pipe = FakePipe::new(BaudRate::B3000000, &[1]);
        let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
        let chunks = admit_watch(&mut c, id).unwrap();
        assert_eq!(chunks, WATCH_CHUNKS);
        let (frames, alert) = watch_rows_on(&mut c, id, 48, chunks).unwrap();
        let w = Watch::fold(&frames, 48, alert, 100.0, 20_000.0, 1200);
        assert_eq!((w.windows.len(), w.lost, w.alert), (3, 0, false));
        let s: Vec<TelemetryTick> = frames
            .iter()
            .map(|f| tel_sample(f.tick as u32).tick)
            .collect();
        for (k, x) in w.windows.iter().enumerate() {
            let rows = &s[16 * k..16 * (k + 1)];
            let mean = |f: fn(&TelemetryTick) -> i16| {
                (rows.iter().map(|r| f(r) as i32).sum::<i32>() >> 4) as i16
            };
            assert_eq!(x.window.i_mean, mean(|r| r.i_tick_counts));
            assert_eq!(x.window.vdiff_mean, mean(|r| r.vdiff_counts));
            assert_eq!(x.window.duty_mean, mean(|r| r.duty_ticks));
        }
        let lo = s.iter().map(|r| r.pos).min().unwrap();
        let hi = s.iter().map(|r| r.pos).max().unwrap();
        assert_eq!(w.pos, Some((lo, hi)));
    }

    /// The servo zeroes `tel_count` when a watch's LAST frame leaves, so
    /// the burst arm after a watch commits; the arm's own disarm write
    /// finds it at 0 already.
    #[test]
    fn a_burst_arm_after_a_watch_finds_tel_disarmed() {
        use crate::rig::snapshot::read_u16;
        use osc_client::fake::FakePipe;
        let pipe = FakePipe::new(BaudRate::B3000000, &[1]);
        let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
        let chunks = admit_watch(&mut c, id).unwrap();
        let (frames, _) = watch_rows_on(&mut c, id, 48, chunks).unwrap();
        assert_eq!(frames.len(), 48);
        assert_eq!(read_u16(&mut c, id, control::TEL_COUNT).unwrap(), 0);
        WireBurstIo { c: &mut c, id }
            .arm(8192, burst::CHANS_DIFF)
            .unwrap();
        assert_eq!(read_u16(&mut c, id, control::TEL_COUNT).unwrap(), 0);
        assert_eq!(read_u16(&mut c, id, burst::wire::DUTY_Q15).unwrap(), 8192);
        assert_eq!(c.read(id, burst::wire::ARM.addr, 1).unwrap(), [1]);
    }

    /// A watch arms its own chunks; the experiment's stream after it runs
    /// under the stream's own, and the read after the watch carries its
    /// window.
    #[test]
    fn a_stream_after_a_watch_runs_under_its_own_chunks() {
        use super::super::servo::bench::Bench;
        use crate::capture::Supply;

        #[derive(Default)]
        struct Exp {
            cmds: Vec<Cmd>,
            tel: Vec<TelFrame>,
            seen: Vec<TelemetrySnapshot>,
        }
        impl Experiment for Exp {
            fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
                self.seen.extend(obs.copied());
                if self.cmds.is_empty() {
                    Cmd::Done
                } else {
                    self.cmds.remove(0)
                }
            }
            fn push_tel(&mut self, tel: &TelBurst) {
                self.tel.extend_from_slice(&tel.frames);
            }
        }
        let chunks = Chunks::two(
            Chunk::new(telemetry::POS.addr, 2),
            Chunk::new(telemetry::I_TICK_COUNTS.addr, 2),
        );
        let mut exp = Exp {
            cmds: vec![
                Cmd::Read,
                Cmd::Pause { ms: 5 },
                Cmd::Stream {
                    samples: 32,
                    goal: None,
                    chunks,
                },
                Cmd::Read,
            ],
            ..Default::default()
        };
        let mut b = Bench::mg90(Supply::TwoS);
        Pump::on(&mut b, None).run(&mut exp).unwrap();
        assert_eq!(exp.tel.len(), 32);
        assert!(
            exp.tel
                .iter()
                .all(|f| f.pos.is_some() && f.current.is_some() && f.vdiff.is_none())
        );
        let [a, b] = exp.seen[..] else {
            panic!("two reads, {} seen", exp.seen.len())
        };
        assert!(a.pwm_period_ticks != 0 && b.agg_seq > a.agg_seq);
    }

    #[test]
    fn write_arm_truncates_by_width() {
        // the exact bytes the Write arm sends per width
        let cases: [(i32, u8, &[u8]); 4] = [
            (1, 1, &[0x01]),
            (-9000, 2, &(-9000i16).to_le_bytes()),
            (0x1B, 2, &[0x1B, 0x00]),
            (-1500, 4, &(-1500i32).to_le_bytes()),
        ];
        for (value, width, want) in cases {
            let bytes = value.to_le_bytes();
            assert_eq!(
                &bytes[..width as usize],
                want,
                "value {value} width {width}"
            );
        }
    }

    #[test]
    fn telemetry_span_covers_the_region_through_the_floor() {
        assert_eq!(TEL_BASE, 0x200);
        assert_eq!(TEL_LEN, 0x80);
    }

    /// The watch takes what the servo's rows hold, the pot included when it
    /// fits; a line under 3 M, which streams no TEL, is refused in words
    /// before anything drives.
    #[test]
    fn identification_refuses_a_line_that_streams_no_watch() {
        use osc_client::fake::FakePipe;
        let (full, bare) = (WATCH_CHUNKS.row_bytes(), WINDOW_CHUNKS.row_bytes());
        assert_eq!(watch_selection(full), Some(WATCH_CHUNKS));
        assert_eq!(watch_selection(full - 1), Some(WINDOW_CHUNKS));
        assert_eq!(watch_selection(bare - 1), None);
        for baud in [BaudRate::B1000000, BaudRate::B2000000] {
            let pipe = FakePipe::new(baud, &[1]);
            let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
            let e = admit_watch(&mut c, id).unwrap_err().to_string();
            assert_eq!(
                e,
                format!(
                    "identification watches its drives on a TEL stream, which the servo streams \
                     at 3 Mbaud only; the bus runs at {} kbaud. Move the bus to 3M first (osc \
                     baud 3000000), on a short cable with good contacts.",
                    baud.as_hz() / 1000
                )
            );
        }
        let pipe = FakePipe::new(BaudRate::B3000000, &[1]);
        let (mut c, id) = (Client::connect(pipe).unwrap(), Id::new(1));
        assert_eq!(admit_watch(&mut c, id).unwrap(), WATCH_CHUNKS);
    }

    #[test]
    fn a_held_permit_refuses_a_stream_it_would_lapse_inside() {
        let mut l = Lease::new(false);
        l.state.wrote(control::TORQUE_ENABLE, 1, 0.0);
        l.state.wrote(control::STALL_PERMIT, 1, 0.0);
        assert!(l.check_stream(15_000).is_ok(), "750 ms");
        assert!(l.check_stream(15_020).is_err());
        l.state.wrote(control::TORQUE_ENABLE, 0, 0.0);
        assert!(l.check_stream(u16::MAX).is_ok(), "torque off holds nothing");
    }

    #[test]
    fn stream_window_covers_the_burst_with_margin() {
        // 3000 samples = 150 ms of ticks: window must exceed that span
        assert!(stream_window(3000) > Duration::from_millis(150));
        // the biggest arm still fits under the raised pipe guard
        assert!(stream_window(u16::MAX) < Duration::from_secs(5));
    }
}
