//! TEL bursts (protocol sec 5.6): the line's admission, one burst armed
//! and read to its LAST frame over the client, and its frames assembled
//! into ticked rows. A burst that lost a row is said so: it is no
//! evidence.

use std::time::Duration;

use osc_client::descriptor::Descriptor;
use osc_client::{BaudRate, Client, Error, Id, Inst, Opcode, Outcome, ResultCode, pipe::Pipe};
use osc_protocol::build;
use osc_protocol::table::{
    BAUD_RATE_IDX, CAP_EXPERIMENT, CAP_TEL_ROWS, CAPABILITY_FLAGS, TEL_ROW_BYTES_MAX,
};
use osc_protocol::tel::{self, Chunks};

use super::rows::{Layout, TelFrame, chunks_text, decode_row};
use crate::install::Refused;
use crate::state::field;

/// Assembles one burst's statuses, in arrival order, into ticked rows.
/// Each frame's rows tick on from its stamp, unwrapped against the
/// previous frame's (a u16 wraps every 3.3 s at 20 kHz, past any gap in a
/// burst), so a lost frame or dropped rows leave a tick hole, never a time
/// skew. stream_seq restarts at 0 per arm, so one assembler serves one
/// burst.
pub struct StreamAssembler {
    chunks: Chunks,
    i_window_min_ticks: u16,
    last_seq: Option<u8>,
    frame_idx: u64,
    stamp: Option<(u16, u64)>,
    first: Option<u16>,
    holes: u64,
    skipped: u64,
    alert: bool,
}

impl StreamAssembler {
    /// None unless the servo could arm `chunks`.
    pub fn new(chunks: Chunks, i_window_min_ticks: u16) -> Option<Self> {
        chunks.valid().then_some(Self {
            chunks,
            i_window_min_ticks,
            last_seq: None,
            frame_idx: 0,
            stamp: None,
            first: None,
            holes: 0,
            skipped: 0,
            alert: false,
        })
    }

    /// One status: `stream` for a Stream-result frame (anything else is
    /// skipped and counted), `alert` its fault bit.
    pub fn push(&mut self, stream: bool, alert: bool, payload: &[u8], out: &mut Vec<TelFrame>) {
        let Some(f) = stream
            .then(|| tel::StreamFrame::parse(self.chunks.row_bytes(), payload))
            .flatten()
        else {
            self.skipped += 1;
            return;
        };
        self.alert |= alert;
        let (idx, expected) = match self.last_seq {
            // bursts arm at seq 0: a nonzero first seq means missed frames
            None => (f.seq as u64, 0),
            Some(last) => (
                self.frame_idx + f.seq.wrapping_sub(last) as u64,
                self.frame_idx + 1,
            ),
        };
        self.holes += idx.saturating_sub(expected);
        self.last_seq = Some(f.seq);
        self.frame_idx = idx;
        let tick = self
            .stamp
            .map_or(0, |(s, t)| t + f.tick.wrapping_sub(s) as u64);
        self.stamp = Some((f.tick, tick));
        self.first.get_or_insert(f.tick);
        out.extend(f.rows().enumerate().map(|(i, row)| {
            decode_row(&self.chunks, self.i_window_min_ticks, row, tick + i as u64)
        }));
    }

    /// The servo's `sample_tick` of the first row, low 16 bits.
    pub fn first_stamp(&self) -> Option<u16> {
        self.first
    }

    /// Frames missing from stream_seq continuity.
    pub fn holes(&self) -> u64 {
        self.holes
    }

    /// Statuses that were no decodable stream frame.
    pub fn skipped(&self) -> u64 {
        self.skipped
    }

    /// A frame said a fault was latched.
    pub fn alert(&self) -> bool {
        self.alert
    }
}

/// Which image the servo runs, from `capability_flags`.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Image {
    Production,
    Experiment,
}

impl Image {
    pub fn name(self) -> &'static str {
        match self {
            Image::Production => "production",
            Image::Experiment => "experiment",
        }
    }
}

/// The servo's line as the identity block says it (protocol sec 5.4).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Line {
    pub caps: u32,
    pub baud: BaudRate,
    pub row_bytes_max: u8,
}

impl Line {
    pub fn image(&self) -> Image {
        if self.caps & CAP_EXPERIMENT != 0 {
            Image::Experiment
        } else {
            Image::Production
        }
    }

    /// The image in words, its flags beside it.
    pub fn said(&self) -> String {
        format!(
            "{} image (capability_flags {:#010x})",
            self.image().name(),
            self.caps
        )
    }

    /// The line streams `layout`, or why not, before anything is written.
    pub fn admit(&self, layout: Layout) -> Result<(), Refused> {
        if self.baud != tel::BAUD {
            return Err(Refused(format!(
                "identification captures on TEL bursts, which a servo streams at {} Mbaud only, and the bus runs at {} kbaud: move the bus to 3M (`osc baud 3000000`), on a short cable with good contacts; that move is volatile, and a run reboots the servo, after which this tool moves it back to 3M",
                tel::BAUD.as_hz() / 1_000_000,
                self.baud.as_hz() / 1000
            )));
        }
        if self.caps & CAP_TEL_ROWS == 0 {
            return Err(Refused(
                "the servo publishes no TEL row limits: its firmware predates packed TEL rows"
                    .into(),
            ));
        }
        let row = layout.chunks().row_bytes();
        if row > self.row_bytes_max as usize {
            return Err(Refused(format!(
                "the {} row ({}) is {row} B; the servo streams rows of at most {} B",
                layout.name(),
                chunks_text(&layout.chunks()),
                self.row_bytes_max
            )));
        }
        Ok(())
    }
}

/// The identity block, read only.
pub async fn line<P: Pipe>(c: &mut Client<P>, id: Id) -> Result<Line, Error> {
    let n = BAUD_RATE_IDX + 1 - CAPABILITY_FLAGS;
    let raw = c.read(id, CAPABILITY_FLAGS, n).await?;
    let short = || Error::Link(osc_client::LinkError::Desync("short identity read".into()));
    let at = |addr: u16| raw.get((addr - CAPABILITY_FLAGS) as usize).copied();
    let caps = u32::from_le_bytes(
        raw.get(..4)
            .and_then(|b| b.try_into().ok())
            .ok_or_else(short)?,
    );
    let idx = at(BAUD_RATE_IDX).ok_or_else(short)?;
    let baud = BaudRate::from_idx(idx).ok_or_else(|| {
        Error::Link(osc_client::LinkError::Desync(format!(
            "baud_rate_idx {idx}"
        )))
    })?;
    Ok(Line {
        caps,
        baud,
        row_bytes_max: at(TEL_ROW_BYTES_MAX).ok_or_else(short)?,
    })
}

/// One burst as received, and its evidence.
#[derive(Clone, Debug)]
pub struct Burst {
    pub layout: Layout,
    pub armed: u16,
    pub rows: Vec<TelFrame>,
    pub frames: usize,
    pub holes: u64,
    pub skipped: u64,
    pub garble: u16,
    pub laps: u16,
    /// The LAST frame never came inside the window.
    pub timed_out: bool,
    /// A frame flagged a latched fault.
    pub alert: bool,
    /// `tel_drop_count` rose across the burst: rows the servo dropped.
    pub dropped: u16,
    pub first_stamp: Option<u16>,
}

impl Burst {
    /// Rows whose tick is not the one after the row before.
    pub fn tick_gaps(&self) -> u64 {
        let mut gaps = self.rows.first().map_or(0, |f| f.tick);
        for w in self.rows.windows(2) {
            gaps += w[1].tick.saturating_sub(w[0].tick + 1);
        }
        gaps
    }

    /// Why the burst is not whole, a phrase a reason; none when it is.
    pub fn lost(&self) -> Vec<String> {
        let mut why = Vec::new();
        if self.timed_out {
            why.push("its LAST frame never came".to_string());
        }
        if self.rows.len() != self.armed as usize {
            why.push(format!("{} rows of {} armed", self.rows.len(), self.armed));
        }
        if self.holes > 0 {
            why.push(format!("{} frames missing from stream_seq", self.holes));
        }
        if self.tick_gaps() > 0 {
            why.push(format!("{} ticks missing between rows", self.tick_gaps()));
        }
        if self.dropped > 0 {
            why.push(format!(
                "the servo dropped {} rows (tel_drop_count)",
                self.dropped
            ));
        }
        if self.skipped > 0 {
            why.push(format!("{} statuses no stream frame", self.skipped));
        }
        if self.garble > 0 || self.laps > 0 {
            why.push(format!(
                "{} B of garble, {} adapter rings lost",
                self.garble, self.laps
            ));
        }
        why
    }
}

/// The sampled span of `rows` rows at `tick_hz`.
pub fn span(rows: u16, tick_hz: u16) -> Duration {
    Duration::from_micros(rows as u64 * 1_000_000 / tick_hz.max(1) as u64)
}

/// The whole-burst window the adapter waits: the span, a quarter more,
/// and turnaround.
fn window(rows: u16, tick_hz: u16) -> Duration {
    span(rows, tick_hz) * 5 / 4 + Duration::from_millis(250)
}

async fn read_u16<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    name: &str,
) -> Result<u16, Error> {
    let f = field(d, name)?;
    let b = c.read(id, f.addr, 2).await?;
    Ok(u16::from_le_bytes([b[0], b[1]]))
}

/// What every burst of a step is read by, read once a step so a burst
/// inside a drive costs no round trip for it: the servo's own
/// `i_window_min_ticks`, which judges a row's window, and its tick rate.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct StreamCfg {
    pub i_window_min_ticks: u16,
    pub tick_hz: u16,
}

impl StreamCfg {
    pub async fn read<P: Pipe>(c: &mut Client<P>, id: Id, d: &Descriptor) -> Result<Self, Error> {
        Ok(StreamCfg {
            i_window_min_ticks: read_u16(c, id, d, "i_window_min_ticks").await?,
            tick_hz: read_u16(c, id, d, "tick_hz").await?,
        })
    }
}

/// One burst of `rows` rows of `layout`, armed by the acked WRITE of the
/// chunks and the count (one 10 B write, protocol sec 5.6), read to its
/// LAST frame with the host silent. The servo's `tel_drop_count` before
/// and after counts the rows it dropped.
pub async fn stream<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    cfg: &StreamCfg,
    layout: Layout,
    rows: u16,
) -> Result<Burst, Error> {
    let drops_before = read_u16(c, id, d, "tel_drop_count").await?;
    let (chunk0, arm) = arm(d, layout, rows)?;
    let mut p = [0u8; 4 + 4 * tel::CHUNKS_MAX + 2];
    let n = build::write(&mut p, chunk0, &arm).ok_or(Error::Servo(ResultCode::Limit))?;
    let inst = Inst::instruction(Opcode::Write, 0);
    let mut b = carry(c, id, inst, &p[..n], true, cfg, layout, rows).await?;
    let drops_after = read_u16(c, id, d, "tel_drop_count").await?;
    b.dropped = drops_after.wrapping_sub(drops_before);
    Ok(b)
}

/// One burst of `rows` rows of `layout` whose arm is staged under HOLD
/// after `held`, each an address and its bytes, so one broadcast COMMIT
/// applies them all on one tick: the capture starts at the commit and a
/// goal among them lands inside it, at the first medium tick after
/// (protocol sec 5.6). Read to its LAST frame with the host silent.
/// `dropped` is left 0: the caller reads `tel_drop_count` around its
/// drive, since a read here would hold the drive's duty a round trip
/// longer.
pub async fn stream_committed<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    cfg: &StreamCfg,
    layout: Layout,
    rows: u16,
    held: &[(u16, &[u8])],
) -> Result<Burst, Error> {
    let (chunk0, arm) = arm(d, layout, rows)?;
    for (addr, bytes) in held {
        c.write_hold(id, *addr, bytes).await?;
    }
    c.write_hold(id, chunk0, &arm).await?;
    let inst = Inst::instruction(Opcode::Commit, 0);
    carry(c, Id::BROADCAST, inst, &[], false, cfg, layout, rows).await
}

/// [`stream`] with `held`, a write at its address, staged under HOLD with
/// the arm and applied by one broadcast COMMIT that carries the burst: a
/// goal and its capture start in the same instant (protocol sec 5.6).
pub async fn stream_with<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    d: &Descriptor,
    cfg: &StreamCfg,
    layout: Layout,
    rows: u16,
    held: Option<(u16, &[u8])>,
) -> Result<Burst, Error> {
    let Some(held) = held else {
        return stream(c, id, d, cfg, layout, rows).await;
    };
    let drops_before = read_u16(c, id, d, "tel_drop_count").await?;
    let mut b = stream_committed(c, id, d, cfg, layout, rows, &[held]).await?;
    let drops_after = read_u16(c, id, d, "tel_drop_count").await?;
    b.dropped = drops_after.wrapping_sub(drops_before);
    Ok(b)
}

/// The arm's address and bytes: the chunks of `layout` and the count, one
/// 10 B run from `chunk0_addr`.
fn arm(d: &Descriptor, layout: Layout, rows: u16) -> Result<(u16, Vec<u8>), Error> {
    let chunk0 = field(d, "chunk0_addr")?.addr;
    if chunk0 + 4 * tel::CHUNKS_MAX as u16 != field(d, "tel_count")?.addr {
        return Err(Error::Descriptor(
            "tel_count does not follow the chunks".into(),
        ));
    }
    let mut arm = layout.chunks().to_le_bytes().to_vec();
    arm.extend(rows.to_le_bytes());
    Ok((chunk0, arm))
}

/// `inst` sent to `id` as a burst's carrier, the burst read to LAST and
/// assembled, `dropped` 0; `ack` whether a status is owed ahead of the
/// frames (a broadcast COMMIT owes none).
#[allow(clippy::too_many_arguments)]
async fn carry<P: Pipe>(
    c: &mut Client<P>,
    id: Id,
    inst: Inst,
    payload: &[u8],
    ack: bool,
    cfg: &StreamCfg,
    layout: Layout,
    rows: u16,
) -> Result<Burst, Error> {
    let chunks = layout.chunks();
    let StreamCfg {
        i_window_min_ticks: floor,
        tick_hz,
    } = *cfg;
    let window = window(rows, tick_hz);
    let guard = c.guard();
    c.set_guard(window + Duration::from_secs(1));
    let reply = c.exchange_stream(id, inst, payload, window).await;
    c.set_guard(guard);
    let reply = reply?;
    match &reply.ack {
        Some(a) if a.result == Some(ResultCode::Ok) => {}
        Some(a) => {
            return Err(a
                .result
                .map_or(Error::Link(osc_client::LinkError::Malformed), Error::Servo));
        }
        None if ack => return Err(Error::Timeout { slot: 0 }),
        None => {}
    }
    let mut asm =
        StreamAssembler::new(chunks, floor).ok_or_else(|| Error::Descriptor("no row".into()))?;
    let mut out = Vec::with_capacity(rows as usize);
    for f in &reply.frames {
        asm.push(
            f.result == Some(ResultCode::Stream),
            f.alert,
            &f.payload,
            &mut out,
        );
    }
    Ok(Burst {
        layout,
        armed: rows,
        rows: out,
        frames: reply.frames.len(),
        holes: asm.holes(),
        skipped: asm.skipped(),
        garble: reply.garble,
        laps: reply.laps,
        timed_out: matches!(reply.outcome, Outcome::Timeout { .. }),
        alert: asm.alert(),
        dropped: 0,
        first_stamp: asm.first_stamp(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::capture::rows::{RowFields, csv_header, csv_rows};
    use osc_protocol::tel::encode_stream_head;
    use osc_servo_core::tel::STREAM_HDR;

    /// A row of `layout` holding `f`'s fields at their table bytes.
    fn encode(layout: Layout, f: &TelFrame) -> Vec<u8> {
        let mut table = vec![0u8; tel::TABLE_BYTES as usize];
        let mut put = |addr: u16, v: Option<i32>, w: usize| {
            if let Some(v) = v {
                table[addr as usize..addr as usize + w].copy_from_slice(&v.to_le_bytes()[..w]);
            }
        };
        use crate::capture::rows::telemetry as t;
        let u = |v: Option<u16>| v.map(i32::from);
        let i = |v: Option<i16>| v.map(i32::from);
        put(t::POS.addr, u(f.pos), 2);
        put(t::VREF_RAW.addr, u(f.vref_raw), 2);
        put(t::VBUS_RAW.addr, u(f.vbus_raw), 2);
        put(t::NTC_RAW.addr, u(f.ntc_raw), 2);
        put(t::CURRENT_RAW.addr, u(f.current_raw), 2);
        put(t::VMOTOR_A.addr, u(f.vmotor_a), 2);
        put(t::VMOTOR_B.addr, u(f.vmotor_b), 2);
        put(t::DUTY_TICKS.addr, i(f.duty_counts), 2);
        put(t::CURRENT_TROUGH.addr, u(f.current_trough), 2);
        put(t::I_TICK_COUNTS.addr, i(f.current), 2);
        put(t::VDIFF_COUNTS.addr, i(f.vdiff), 2);
        let mut row = Vec::new();
        for c in layout.chunks().0 {
            row.extend(&table[c.addr as usize..(c.addr + c.len) as usize]);
        }
        row
    }

    /// One parsed CSV line: seg, commanded duty, and the row.
    fn parse(line: &str) -> (u32, i16, TelFrame) {
        let c: Vec<&str> = line.split(',').collect();
        assert_eq!(c.len(), 18, "{line}");
        let opt = |k: usize| (!c[k].is_empty()).then(|| c[k].parse::<i32>().expect("int"));
        let u = |k: usize| opt(k).map(|v| v as u16);
        let i = |k: usize| opt(k).map(|v| v as i16);
        (
            c[0].parse().expect("seg"),
            c[1].parse().expect("duty"),
            TelFrame {
                tick: c[3].parse().expect("tick"),
                window_valid: c[4] == "1",
                pos: u(5),
                current: i(6),
                current_trough: u(7),
                duty_counts: i(8),
                vdiff: i(9),
                vbus: u(10),
                current_raw: u(11),
                vmotor_a: u(12),
                vmotor_b: u(13),
                vbus_raw: u(14),
                ntc_raw: u(15),
                vref_raw: u(16),
                ..TelFrame::default()
            },
        )
    }

    /// A committed capture's rows, put back on the wire as the servo
    /// streams them (16 rows a frame, each frame stamped with its first
    /// row's tick, seq from 0 per segment), assembled and decoded, and
    /// written as CSV, are the committed file byte for byte.
    fn round_trip(file: &str, layout: Layout, floor: u16) {
        let path = format!("{}/testdata/captures/{file}", env!("CARGO_MANIFEST_DIR"));
        let text = std::fs::read_to_string(&path).expect("fixture");
        let mut lines = text.lines();
        assert_eq!(lines.next(), Some(csv_header(layout).as_str()));
        let mut segs: Vec<(u32, i16, Vec<TelFrame>)> = Vec::new();
        for line in lines {
            let (seg, duty, f) = parse(line);
            match segs.last_mut() {
                Some((s, _, rows)) if *s == seg => rows.push(f),
                _ => segs.push((seg, duty, vec![f])),
            }
        }
        assert!(!segs.is_empty());
        let mut out = csv_header(layout);
        out.push('\n');
        for (seg, duty, rows) in &segs {
            let mut asm = StreamAssembler::new(layout.chunks(), floor).expect("arms");
            let mut got = Vec::new();
            for (k, batch) in rows.chunks(16).enumerate() {
                let mut p = vec![0; STREAM_HDR];
                encode_stream_head(
                    k as u8,
                    k == rows.len().div_ceil(16) - 1,
                    batch[0].tick as u16,
                    &mut p,
                );
                for f in batch {
                    p.extend(encode(layout, f));
                }
                asm.push(true, false, &p, &mut got);
            }
            assert_eq!((asm.holes(), asm.skipped()), (0, 0));
            assert_eq!(got.len(), rows.len());
            for (a, b) in got.iter().zip(rows) {
                assert_eq!(RowFields(a).to_string(), RowFields(b).to_string());
            }
            csv_rows(&mut out, layout, *seg, *duty, &got);
        }
        assert!(out == text, "{file} drifted");
    }

    /// Trimmed from committed notebook captures (testdata/captures/sources.json):
    /// the raw row of an open-loop ladder, and the rest row. The rest row is
    /// re-laid from a capture of the older row with `vref_raw`: that column
    /// emptied, `vmotor_b` a copy of `vmotor_a`; a capture of the new row
    /// replaces it at the bench gate.
    #[test]
    fn golden_rows_match_a_committed_capture() {
        round_trip("raw-row.sweep.csv", Layout::Raw, 64);
        round_trip("rest-row.sweep.csv", Layout::Rest, 64);
    }

    fn payload(seq: u8, last: bool, tick: u16, rows: u16) -> Vec<u8> {
        let mut p = vec![
            seq,
            if last {
                osc_protocol::wire::STREAM_FLAG_LAST
            } else {
                0
            },
        ];
        p.extend(tick.to_le_bytes());
        for i in 0..rows {
            p.extend((tick + i).to_le_bytes());
        }
        p
    }

    #[test]
    fn a_missing_frame_leaves_a_seq_hole_and_a_tick_jump() {
        let pos = Chunks::one(0x236, 2);
        let mut a = StreamAssembler::new(pos, 0).expect("arms");
        let mut out = Vec::new();
        for seq in [0u8, 1, 3] {
            a.push(
                true,
                false,
                &payload(seq, seq == 3, 16 * seq as u16, 16),
                &mut out,
            );
        }
        assert_eq!((out[31].tick, out[32].tick), (31, 48));
        assert_eq!(a.holes(), 1);
    }

    /// Rows the servo dropped against full buffers leave seq whole: the
    /// stamp tells.
    #[test]
    fn dropped_rows_jump_the_ticks_not_the_seq() {
        let pos = Chunks::one(0x236, 2);
        let mut a = StreamAssembler::new(pos, 0).expect("arms");
        let mut out = Vec::new();
        a.push(true, false, &payload(0, false, 7, 16), &mut out);
        a.push(true, false, &payload(1, true, 7 + 20, 16), &mut out);
        assert_eq!((out[15].tick, out[16].tick), (15, 20));
        assert_eq!(a.holes(), 0);
        let b = Burst {
            layout: Layout::Rest,
            armed: 32,
            rows: out,
            frames: 2,
            holes: 0,
            skipped: 0,
            garble: 0,
            laps: 0,
            timed_out: false,
            alert: false,
            dropped: 0,
            first_stamp: Some(7),
        };
        assert_eq!(b.tick_gaps(), 4);
        assert_eq!(b.lost(), ["4 ticks missing between rows"]);
    }

    #[test]
    fn the_stamps_unwrap_across_the_u16() {
        let pos = Chunks::one(0x236, 2);
        let mut a = StreamAssembler::new(pos, 0).expect("arms");
        let mut out = Vec::new();
        for k in 0..258u64 {
            let tick = 0xF000u16.wrapping_add(16 * k as u16);
            let mut p = vec![k as u8, 0];
            p.extend(tick.to_le_bytes());
            p.extend(0u16.to_le_bytes());
            a.push(true, false, &p, &mut out);
        }
        assert_eq!((out[255].tick, out[257].tick), (255 * 16, 257 * 16));
        assert_eq!((a.holes(), a.skipped()), (0, 0));
    }
}
