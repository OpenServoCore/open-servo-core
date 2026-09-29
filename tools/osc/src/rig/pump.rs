//! The driver loop from osc-ident's exp module doc: Write -> wire write,
//! Read -> telemetry gread + parse, Pause -> sleep in slices that honor
//! ctrl-c, Stream -> one TEL burst on the main bus (HOLD+COMMIT when it
//! carries a goal), Done -> break. A stall permit the experiment holds is
//! rewritten between commands and pause slices while the lease runs.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::pipe::Pipe;
use osc_client::{Id, Inst, Opcode, Outcome, ResultCode};
use osc_ident::burst::{self, ArmSeen, BurstIo, Capture, CaptureCfg, Pre};
use osc_ident::exp::{Cmd, Experiment};
use osc_ident::frame::{StreamAssembler, TelFrame, TelemetrySnapshot};
use osc_ident::limits::PermitLease;
use osc_ident::regs::{Reg, calib, config, control, telemetry};
use osc_ident::units::{self, SenseParams};
use osc_protocol::build;

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
const IDENT_BASE: u16 = telemetry::I_MEAN_COUNTS.addr;
const IDENT_LEN: u16 = telemetry::AGG_SEQ.addr + 2 - IDENT_BASE;

/// One telemetry snapshot with the torn-ident-window guard: the full read
/// is paired with a 12 B re-read of the ident block, and only a pair whose
/// agg_seq agrees is returned (the block is written mid-tick; agg_seq lands
/// last). Bounded retries - a stubbornly torn read returns the last full
/// snapshot, which the engine's WindowStream then dedups by seq anyway.
pub(crate) fn read_snapshot<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<TelemetrySnapshot> {
    read_stamped(c, id, Instant::now())
}

/// [`read_snapshot`] stamped with the wall clock since `t0`, ms, at the
/// middle of the region read it returns: the time an experiment's fits
/// take, since every read stalls the servo's own tick counter.
pub(crate) fn read_stamped<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    t0: Instant,
) -> Result<TelemetrySnapshot> {
    let ms = || t0.elapsed().as_secs_f64() * 1000.0;
    let mut last = None;
    for _ in 0..3 {
        let before = ms();
        let raw = c.read(id, TEL_BASE, TEL_LEN).context("telemetry read")?;
        let mut snap =
            TelemetrySnapshot::parse(TEL_BASE, &raw).context("telemetry parse (short read?)")?;
        snap.host_ms = (before + ms()) / 2.0;
        let ib = c.read(id, IDENT_BASE, IDENT_LEN).context("ident re-read")?;
        let re_seq = u16::from_le_bytes([ib[10], ib[11]]);
        if re_seq == snap.agg_seq {
            return Ok(snap);
        }
        last = Some(snap);
    }
    Ok(last.expect("loop ran"))
}

/// Run the closure, then force the servo safe (duty/goals zero, torque,
/// stall permit and TEL off) whether it succeeded, failed, or was ctrl-c'd.
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
}

/// One TEL burst on the main bus. With `goal` Some the goal write and the
/// TEL_COUNT arm are staged under HOLD and a broadcast COMMIT (the stream
/// carrier) applies both in the same instant; with None the acked
/// TEL_COUNT write itself carries the stream. Frames decode through
/// [`StreamAssembler`] under `mask`; corrupt frames never appear (the
/// adapter drops them into garble + a seq hole).
pub(crate) fn exchange_tel_burst<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    samples: u16,
    goal: Option<(Reg, i32)>,
    mask: u16,
) -> Result<(Vec<TelFrame>, BurstStats)> {
    let window = stream_window(samples);
    c.set_guard(window + Duration::from_secs(1));
    let reply = match goal {
        Some((reg, value)) => {
            let bytes = value.to_le_bytes();
            c.write_hold(id, reg.addr, &bytes[..reg.width as usize])
                .context("hold goal")?;
            c.write_hold(id, control::TEL_COUNT.addr, &samples.to_le_bytes())
                .context("hold tel_count")?;
            let inst = Inst::instruction(Opcode::Commit, 0);
            c.exchange_stream(Id::BROADCAST, inst, &[], window)
        }
        None => {
            let mut p = [0u8; 8];
            let n = build::write(&mut p, control::TEL_COUNT.addr, &samples.to_le_bytes())
                .context("tel_count payload")?;
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
    let mut asm = StreamAssembler::new(mask).context("tel mask invalid for a stream arm")?;
    let mut frames = Vec::new();
    for f in &reply.frames {
        asm.push(
            f.result == Some(ResultCode::Stream),
            &f.payload,
            &mut frames,
        );
    }
    let stats = BurstStats {
        frames: reply.frames.len(),
        samples: frames.len(),
        holes: asm.holes() + asm.skipped(),
        garble: reply.garble,
    };
    if matches!(reply.outcome, Outcome::Timeout { .. }) {
        bail!(
            "tel burst timed out mid-stream ({} of {} samples)",
            stats.samples,
            samples
        );
    }
    Ok((frames, stats))
}

/// The burst handshake's wire moves. The arm has to be atomic - a servo
/// that sees arm=1 before the new duty or mask captures the old one - so
/// duty, mask and arm go out under HOLD and one broadcast COMMIT applies
/// all three. The mask is its own one-byte write: the byte after it is a
/// reserved alignment byte that refuses writes.
struct WireBurstIo<'a, P: Pipe> {
    c: &'a mut Client<P>,
    id: Id,
}

impl<P: Pipe> BurstIo for WireBurstIo<'_, P> {
    type Error = anyhow::Error;

    fn arm(&mut self, duty_q15: i16, chans: u8) -> Result<()> {
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

pub(crate) struct Pump<'a, S: Servo = Wire<&'a mut Client<NusbPipe>>> {
    servo: S,
    log: Option<&'a mut SnapshotLog>,
    /// Mirror of the sticky TEL_MASK register, tracked off the experiment's
    /// own writes; the stream decoder keys on it.
    mask: u16,
    /// Every decoded frame across the run's bursts, in order - the CSV log
    /// source (the experiment gets the same frames via push_tel).
    pub(crate) tel: Vec<TelFrame>,
    /// The experiment writes its own permit; this only keeps it alive.
    lease: Lease,
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
            mask: 0,
            tel: Vec::new(),
            lease: Lease::new(false),
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
                Cmd::Write { reg, value } => {
                    if reg == control::TEL_MASK {
                        self.mask = value as u16;
                    }
                    self.lease.write(s, reg, value)?;
                }
                Cmd::Read => {
                    let snap = s.snapshot()?;
                    if let Some(log) = self.log.as_mut() {
                        log.push(snap.host_ms, &snap)?;
                    }
                    pending = Some(snap);
                }
                Cmd::Pause { ms } => {
                    // 5 ms slices keep ctrl-c prompt and the lease fresh
                    let mut left = ms;
                    while left > 0 {
                        if STOP.load(Ordering::SeqCst) {
                            bail!("interrupted");
                        }
                        let slice = left.min(5);
                        s.sleep(slice);
                        left -= slice;
                        self.lease.keep(s)?;
                    }
                }
                Cmd::Stream { samples, goal } => {
                    self.lease.check_stream(samples)?;
                    let (frames, st) = s.stream(samples, goal, self.mask)?;
                    eprintln!(
                        "tel: {} frames, {} samples, {} seq holes, {} garble bytes",
                        st.frames, st.samples, st.holes, st.garble
                    );
                    exp.push_tel(&frames);
                    self.tel.extend_from_slice(&frames);
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
                Cmd::Done => return Ok(()),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
    fn telemetry_span_covers_the_ident_block() {
        assert_eq!(TEL_BASE, 0x200);
        assert_eq!(TEL_LEN, 0x6a);
        assert_eq!(IDENT_BASE, 0x25a);
        assert_eq!(IDENT_LEN, 12);
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
