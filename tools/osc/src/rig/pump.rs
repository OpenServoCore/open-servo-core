//! The driver loop from osc-ident's exp module doc: Write -> wire write,
//! Read -> telemetry gread + parse, Pause -> sleep in slices that honor
//! ctrl-c, Stream -> one TEL burst on the main bus (HOLD+COMMIT when it
//! carries a goal), Done -> break.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_client::{Id, Inst, Opcode, Outcome, ResultCode};
use osc_ident::exp::{Cmd, Experiment};
use osc_ident::frame::{StreamAssembler, TelFrame, TelemetrySnapshot};
use osc_ident::regs::{Reg, config, control, telemetry};
use osc_protocol::build;

use super::csvio::SnapshotLog;

pub(crate) static STOP: AtomicBool = AtomicBool::new(false);

pub(crate) fn install_ctrlc() {
    let _ = ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst));
}

/// Write one register, value LE-truncated to the field width (negative
/// i32 -> correct two's complement for 2/4-byte fields).
pub(crate) fn write_reg(c: &mut Client<NusbPipe>, id: Id, reg: Reg, value: i32) -> Result<()> {
    let bytes = value.to_le_bytes();
    c.write(id, reg.addr, &bytes[..reg.width as usize])
        .with_context(|| format!("write addr {:#06x}", reg.addr))?;
    Ok(())
}

pub(crate) fn read_i32(c: &mut Client<NusbPipe>, id: Id, reg: Reg) -> Result<i32> {
    let raw = c.read(id, reg.addr, 4).context("field read")?;
    Ok(i32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]))
}

const TEL_BASE: u16 = telemetry::FAULT_FLAGS.addr;
const TEL_LEN: u16 = telemetry::AGG_SEQ.addr + 2 - TEL_BASE;
const IDENT_BASE: u16 = telemetry::I_MEAN_COUNTS.addr;
const IDENT_LEN: u16 = telemetry::AGG_SEQ.addr + 2 - IDENT_BASE;

/// One telemetry snapshot with the torn-ident-window guard: the full read
/// is paired with a 12 B re-read of the ident block, and only a pair whose
/// agg_seq agrees is returned (the block is written mid-tick; agg_seq lands
/// last). Bounded retries - a stubbornly torn read returns the last full
/// snapshot, which the engine's WindowStream then dedups by seq anyway.
pub(crate) fn read_snapshot(c: &mut Client<NusbPipe>, id: Id) -> Result<TelemetrySnapshot> {
    let mut last = None;
    for _ in 0..3 {
        let raw = c.read(id, TEL_BASE, TEL_LEN).context("telemetry read")?;
        let snap =
            TelemetrySnapshot::parse(TEL_BASE, &raw).context("telemetry parse (short read?)")?;
        let ib = c.read(id, IDENT_BASE, IDENT_LEN).context("ident re-read")?;
        let re_seq = u16::from_le_bytes([ib[10], ib[11]]);
        if re_seq == snap.agg_seq {
            return Ok(snap);
        }
        last = Some(snap);
    }
    Ok(last.expect("loop ran"))
}

/// Run the closure, then force the servo safe (duty/goals zero, torque and
/// TEL off) whether it succeeded, failed, or was ctrl-c'd. A hard kill
/// skips this - the servo's own protections are the backstop.
pub(crate) fn with_guard<T>(
    c: &mut Client<NusbPipe>,
    id: Id,
    f: impl FnOnce(&mut Client<NusbPipe>) -> Result<T>,
) -> Result<T> {
    let r = f(c);
    for (reg, v) in [
        (control::GOAL_DUTY, 0),
        (control::GOAL_CURRENT, 0),
        (control::GOAL_VELOCITY, 0),
        (control::TORQUE_ENABLE, 0),
        (control::TEL_COUNT, 0),
        (control::TEL_MASK, 0),
    ] {
        let _ = write_reg(c, id, reg, v);
    }
    r
}

/// Park the soft limits at the phys limits so an experiment can stall at
/// the mechanical rails (the firmware clamps OpenLoop duty to zero at
/// pos_min/max_soft_counts); returns the originals for the restore.
pub(crate) fn widen_soft_limits(c: &mut Client<NusbPipe>, id: Id) -> Result<(i32, i32)> {
    let saved = (
        read_i32(c, id, config::POS_MIN_SOFT_COUNTS)?,
        read_i32(c, id, config::POS_MAX_SOFT_COUNTS)?,
    );
    let phys_lo = read_i32(c, id, config::POS_MIN_PHYS_COUNTS)?;
    let phys_hi = read_i32(c, id, config::POS_MAX_PHYS_COUNTS)?;
    write_reg(c, id, config::POS_MIN_SOFT_COUNTS, phys_lo)?;
    write_reg(c, id, config::POS_MAX_SOFT_COUNTS, phys_hi)?;
    Ok(saved)
}

pub(crate) fn restore_soft_limits(
    c: &mut Client<NusbPipe>,
    id: Id,
    (lo, hi): (i32, i32),
) -> Result<()> {
    write_reg(c, id, config::POS_MIN_SOFT_COUNTS, lo)?;
    write_reg(c, id, config::POS_MAX_SOFT_COUNTS, hi)?;
    Ok(())
}

/// Whole-burst window: the sampled span plus wire/turnaround margin. The
/// client pipe guard must sit above it (see exchange_stream).
fn stream_window(samples: u16) -> Duration {
    Duration::from_micros(samples as u64 * 50 * 5 / 4 + 250_000)
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
pub(crate) fn exchange_tel_burst(
    c: &mut Client<NusbPipe>,
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

pub(crate) struct Pump<'a> {
    client: &'a mut Client<NusbPipe>,
    id: Id,
    log: Option<&'a mut SnapshotLog>,
    /// Mirror of the sticky TEL_MASK register, tracked off the experiment's
    /// own writes; the stream decoder keys on it.
    mask: u16,
    /// Every decoded frame across the run's bursts, in order - the CSV log
    /// source (the experiment gets the same frames via push_tel).
    pub(crate) tel: Vec<TelFrame>,
}

impl<'a> Pump<'a> {
    pub(crate) fn new(
        client: &'a mut Client<NusbPipe>,
        id: Id,
        log: Option<&'a mut SnapshotLog>,
    ) -> Self {
        Self {
            client,
            id,
            log,
            mask: 0,
            tel: Vec::new(),
        }
    }

    /// Run one experiment to completion.
    pub(crate) fn run(&mut self, exp: &mut dyn Experiment) -> Result<()> {
        let mut pending: Option<TelemetrySnapshot> = None;
        let t0 = Instant::now();
        loop {
            if STOP.load(Ordering::SeqCst) {
                bail!("interrupted");
            }
            match exp.step(pending.take().as_ref()) {
                Cmd::Write { reg, value } => {
                    if reg == control::TEL_MASK {
                        self.mask = value as u16;
                    }
                    write_reg(self.client, self.id, reg, value)?;
                }
                Cmd::Read => {
                    let snap = read_snapshot(self.client, self.id)?;
                    if let Some(log) = self.log.as_mut() {
                        log.push(t0.elapsed().as_secs_f64() * 1000.0, &snap)?;
                    }
                    pending = Some(snap);
                }
                Cmd::Pause { ms } => {
                    // 5 ms slices keep ctrl-c prompt
                    let mut left = ms;
                    while left > 0 {
                        if STOP.load(Ordering::SeqCst) {
                            bail!("interrupted");
                        }
                        let slice = left.min(5);
                        std::thread::sleep(Duration::from_millis(slice as u64));
                        left -= slice;
                    }
                }
                Cmd::Stream { samples, goal } => {
                    let (frames, st) =
                        exchange_tel_burst(self.client, self.id, samples, goal, self.mask)?;
                    eprintln!(
                        "tel: {} frames, {} samples, {} seq holes, {} garble bytes",
                        st.frames, st.samples, st.holes, st.garble
                    );
                    exp.push_tel(&frames);
                    self.tel.extend_from_slice(&frames);
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
        assert_eq!(TEL_LEN, 0x60);
        assert_eq!(IDENT_BASE, 0x254);
        assert_eq!(IDENT_LEN, 12);
    }

    #[test]
    fn stream_window_covers_the_burst_with_margin() {
        // 3000 samples = 150 ms of ticks: window must exceed that span
        assert!(stream_window(3000) > Duration::from_millis(150));
        // the biggest arm still fits under the raised pipe guard
        assert!(stream_window(u16::MAX) < Duration::from_secs(5));
    }
}
