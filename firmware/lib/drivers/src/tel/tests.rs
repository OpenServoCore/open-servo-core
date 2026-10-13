use super::*;
use crate::mocks::bus::{FakeCrc, FakeDeadline, FakeRing, FakeWire, TestProviders};
use osc_protocol::crc::osc_crc;
use osc_protocol::tel::StreamFrame;
use osc_servo_core::regions::telemetry::addr::{estimates, tick};
use osc_servo_core::tel::Chunk;
use osc_servo_core::{RegionStorage, TelemetryTick};

/// The six default fields, current_raw to pos and duty to current_trough:
/// 12 B over two chunks.
const SIX: Chunks = Chunks::two(
    Chunk::new(tick::CURRENT_RAW, 8),
    Chunk::new(tick::DUTY_TICKS, 4),
);
/// Duty to vdiff and the rail estimate: 10 B over two chunks.
const IDENT_RAIL: Chunks = Chunks::two(
    Chunk::new(tick::DUTY_TICKS, 8),
    Chunk::new(estimates::VBUS_COUNTS, 2),
);
const POS: Chunks = Chunks::one(tick::POS, 2);
const ID: u8 = 9;

struct Rig {
    feed: TelFeed<TestProviders>,
    drain: TelDrain,
    wire: FakeWire,
    ring: FakeRing,
    ch: &'static TelChannel,
    table: &'static ControlTableCell,
    /// The kernel's `hold_start` on every tick.
    hold_start: bool,
    /// Each tick's row step and, while the lane is the kernel's, the lane's
    /// step in place of `on_tick_end`; kept when set.
    steps: Option<std::vec::Vec<(Option<TelStep>, Option<TelStep>)>>,
}

fn rig() -> Rig {
    rig_with(FakeCrc::new())
}

fn rig_with(crc: FakeCrc) -> Rig {
    let ch: &'static TelChannel = std::boxed::Box::leak(std::boxed::Box::new(TelChannel::new()));
    let table: &'static ControlTableCell =
        std::boxed::Box::leak(std::boxed::Box::new(ControlTableCell::new()));
    let wire = FakeWire::new();
    let ring = FakeRing::new();
    let (feed, drain) =
        ch.split::<TestProviders>(wire.clone(), crc, ring.clone(), FakeDeadline::new(), table);
    Rig {
        feed,
        drain,
        wire,
        ring,
        ch,
        table,
        hold_start: false,
        steps: None,
    }
}

/// Tick `i` as the table holds it at the row: the tick block, and the rail
/// estimate as last published.
fn sample(i: u16) -> (TelemetryTick, u16) {
    let t = TelemetryTick {
        i_tick_counts: -(i as i16),
        vdiff_counts: 300 + i as i16,
        duty_ticks: i as i16 - 100,
        pos: 0x400 + i,
        current_trough: 0xB00 + i,
        current_raw: 0x100 + i,
        vmotor_a: 0xA00 + i,
        vmotor_b: 0x600 + i,
        tick_flags: if i == 3 { tick_flag::FAULT } else { 0 },
        sample_tick: 0xFFF0u32.wrapping_add(i as u32),
        ..Default::default()
    };
    (t, 1800 + i)
}

/// The row `c` copies on tick `i`.
fn row(c: Chunks, i: u16) -> std::vec::Vec<u8> {
    let table = ControlTableCell::new();
    let (t, vbus) = sample(i);
    table.with_mut(|tb| {
        tb.telemetry.tick = t;
        tb.telemetry.estimates.vbus_counts = vbus;
    });
    let base = table.region_ptr().cast::<u8>();
    c.0.iter()
        .flat_map(|c| c.addr..c.addr + c.len)
        // SAFETY: test read of a local table; the chunks lie inside it.
        .map(|a| unsafe { base.add(a as usize).read() })
        .collect()
}

impl Rig {
    /// The kernel's tick: the tick block lands beside the rail as last
    /// published, the row, then the medium step's rail and the next scan,
    /// which land before the lane at the tick's end. A break left on the
    /// wire ends first (11 bit-times, under a tick).
    fn tick(&mut self, i: u16) {
        if self.wire.breaking() {
            self.wire.finish();
        }
        let (t, vbus) = sample(i);
        self.table.with_mut(|tb| {
            tb.telemetry.tick = t;
            tb.telemetry.estimates.vbus_counts = vbus;
        });
        let hold = self.hold_start;
        let row = (self.feed.remaining != 0).then(|| self.feed.row_step());
        self.feed.on_tick(|| hold);
        let (next, rail) = sample(i.wrapping_add(0x80));
        self.table.with_mut(|tb| {
            tb.telemetry.tick = next;
            tb.telemetry.estimates.vbus_counts = rail;
        });
        match self.steps.as_mut() {
            Some(steps) => {
                let lane = self.feed.active().then(|| self.feed.lane());
                steps.push((row, lane));
            }
            None => self.feed.on_tick_end(),
        }
    }

    /// Ticks rows `from..to`, the wire carrying each image for its wire
    /// time at 3M (15 bytes a tick) and reading it whole at the end.
    fn run(&mut self, from: u16, to: u16) {
        const BYTES_PER_TICK: usize = 15;
        let mut left: Option<usize> = None;
        for i in from..to {
            self.tick(i);
            if left.is_none() {
                left = self
                    .wire
                    .streaming()
                    .map(|len| len.div_ceil(BYTES_PER_TICK));
            }
            if let Some(l) = left.as_mut() {
                if *l == 0 {
                    self.wire.finish();
                    left = None;
                } else {
                    *l -= 1;
                }
            }
        }
    }

    /// The bus's arm: post the chunks, then lend the lane.
    fn arm(&mut self, c: Chunks, count: u16) -> u8 {
        self.drain.post(c);
        self.drain.start(count, ID)
    }

    /// Ticks until the wire carries an image, at most `bound`.
    fn tick_to_stream(&mut self, i: &mut u16, bound: u16) {
        for _ in 0..bound {
            self.tick(*i);
            *i += 1;
            if !self.wire.idle() && !self.wire.breaking() {
                return;
            }
        }
        panic!("no image armed in {bound} ticks");
    }
}

/// The streamed image's payload, its CRC checked over ALIGN..payload.
fn payload(frame: &[u8]) -> &[u8] {
    let mut covered = std::vec![0u8];
    covered.extend_from_slice(&frame[..frame.len() - CRC_LEN]);
    let tail = &frame[frame.len() - CRC_LEN..];
    assert_eq!(
        osc_crc(&covered),
        u16::from_le_bytes([tail[0], tail[1]]),
        "frame CRC"
    );
    &frame[Header::SIZE - 1..frame.len() - CRC_LEN]
}

/// The head and the rows the table held tick by tick.
fn golden(c: Chunks, seq: u8, last: bool, rows: core::ops::Range<u16>) -> std::vec::Vec<u8> {
    let mut buf = std::vec![0u8; STREAM_HDR];
    encode_stream_head(seq, last, sample(rows.start).0.sample_tick as u16, &mut buf);
    for i in rows {
        buf.extend(row(c, i));
    }
    buf
}

/// Two frames, the second the LAST: each starts on its lead row, the wire
/// reads it at its own pace, and it decodes byte for byte as the table's
/// chunks tick by tick behind the frame header.
#[test]
fn frames_stream_cut_through_and_carry_the_table() {
    let mut r = rig();
    r.arm(SIX, 20);
    let mut i = 0;
    r.tick_to_stream(&mut i, 16);
    // 12 B rows at 3M: cov 200, the CRC 198 byte-times (13.2 ticks) behind
    // the arm, so the break follows row 7 and the arm row 8.
    assert_eq!(i, 9, "armed on the tick after row 7's break");
    r.run(i, 64);
    let frames = r.wire.streamed();
    assert_eq!(frames.len(), 2);
    let inst = |alert: bool| Inst::status(ResultCode::Stream, alert).0;
    assert_eq!(
        frames[0][..3],
        [ID, wire::len_for(4 + 16 * 12), inst(false)]
    );
    assert_eq!(payload(&frames[0]), golden(SIX, 0, false, 0..16));
    assert_eq!(frames[1][..3], [ID, wire::len_for(4 + 4 * 12), inst(false)]);
    assert_eq!(payload(&frames[1]), golden(SIX, 1, true, 16..20));
    assert!(r.feed.idle());
}

/// A row takes both chunks when the tick block is published, ahead of the
/// next scan and the medium step: the shorter by the CPU and the longer
/// by one copy, whichever leads the row. Each carries its tick's values,
/// the rail as last published.
#[test]
fn two_chunk_rows_copy_both_chunks_before_the_next_scan_lands() {
    let [duty, rail] = IDENT_RAIL.0;
    for c in [IDENT_RAIL, Chunks::two(rail, duty), SIX] {
        let mut r = rig();
        r.arm(c, 16);
        r.run(0, 40);
        let f = &r.wire.streamed()[0];
        assert_eq!(payload(f), golden(c, 0, true, 0..16));
        assert_eq!(r.feed.crc.copies(), 16, "one copy a row");
    }
}

/// INST ALERT is the fault flag of the frame's first row.
#[test]
fn alert_is_the_first_rows_fault() {
    let mut r = rig();
    r.arm(POS, 8);
    r.run(3, 30);
    let f = &r.wire.streamed()[0];
    assert_eq!(f[2], Inst::status(ResultCode::Stream, true).0);
    let mut r = rig();
    r.arm(POS, 8);
    r.run(4, 30);
    let f = &r.wire.streamed()[0];
    assert_eq!(
        f[2],
        Inst::status(ResultCode::Stream, false).0,
        "row 1's fault"
    );
}

/// An image the wire reads before its rows land ships stale bytes against
/// a CRC the engine took over the fresh ones: the frame fails its CRC.
#[test]
fn rows_landing_behind_the_read_fail_the_crc() {
    let mut r = rig();
    r.arm(SIX, 16);
    let mut i = 0;
    r.tick_to_stream(&mut i, 16);
    r.wire.finish();
    while i < 20 {
        r.tick(i);
        i += 1;
    }
    let f = &r.wire.streamed()[0];
    let mut covered = std::vec![0u8];
    covered.extend_from_slice(&f[..f.len() - CRC_LEN]);
    let tail = u16::from_le_bytes([f[f.len() - 2], f[f.len() - 1]]);
    assert_ne!(osc_crc(&covered), tail);
    assert!(r.feed.idle(), "the lane still finished");
}

/// A CRC engine whose result never comes costs its frames their CRC,
/// never the lane: every frame still streams and the burst ends.
#[test]
fn sick_crc_engine_costs_the_crc_never_the_lane() {
    let mut r = rig_with(FakeCrc::sick());
    r.arm(SIX, 40);
    r.run(0, 120);
    let frames = r.wire.streamed();
    assert_eq!(frames.len(), 3);
    for f in &frames {
        let mut covered = std::vec![0u8];
        covered.extend_from_slice(&f[..f.len() - CRC_LEN]);
        let tail = u16::from_le_bytes([f[f.len() - 2], f[f.len() - 1]]);
        assert_ne!(osc_crc(&covered), tail, "never patched");
    }
    assert!(r.feed.idle());
}

/// The start rule by row width: the rows a full frame lands before its
/// break.
#[test]
fn lead_follows_the_start_rule() {
    for (c, lead) in [
        (SIX, 7),
        (IDENT_RAIL, 9),
        (Chunks::one(tick::I_TICK_COUNTS, 6), 14),
    ] {
        let mut r = rig();
        r.arm(c, 32);
        let mut n = 0;
        while r.wire.idle() {
            r.tick(n);
            n += 1;
        }
        assert_eq!(n - 1, lead, "{} B: break after row {lead}", c.row_bytes());
    }
}

/// Both images hold unfinished frames: the next frame's rows drop, are
/// counted, and the next frame's stamp jumps over them.
#[test]
fn full_images_drop_and_count() {
    let mut r = rig();
    r.arm(POS, 64);
    let mut i = 0;
    // The wire never finishes an image: frame 0 streams, frame 1 banks at
    // row 31, and frame 2's row 0 drops from tick 32 on.
    for _ in 0..37 {
        r.tick(i);
        i += 1;
    }
    assert_eq!(r.ch.drops(), 5);
    // Frame 0 leaves the wire; tick 37's row lands before its tail polls
    // that, so it drops too, and frame 2 opens on row 38.
    r.wire.finish();
    for _ in 0..40 {
        r.tick(i);
        r.wire.finish();
        i += 1;
    }
    assert_eq!(r.ch.drops(), 6);
    let frames = r.wire.streamed();
    let f2 = StreamFrame::parse(2, payload(&frames[2])).expect("frame 2");
    assert_eq!((f2.seq, f2.tick), (2, sample(38).0.sample_tick as u16));
}

/// The LAST frame's arm publishes the RX cursor the bus resumes at, and
/// nothing before it does; an arm that is no row hands the lane straight
/// back.
#[test]
fn last_frame_publishes_the_resume_cursor() {
    let mut r = rig();
    let epoch = r.arm(POS, 2);
    r.ring.set_cursor(77);
    let mut i = 0;
    while r.wire.streaming().is_none() {
        assert_eq!(r.drain.last_out(epoch), None, "tick {i}");
        r.tick(i);
        i += 1;
        assert!(i < 16, "the LAST frame never armed");
    }
    assert_eq!(r.drain.last_out(epoch), Some(77));

    let epoch = r.arm(Chunks::one(tick::POS, 14), 5);
    r.ring.set_cursor(91);
    r.tick(0);
    assert_eq!(r.drain.last_out(epoch), Some(91));
    assert!(r.feed.idle());
}

/// The lane runs only while lent: no tick touches the wire before the
/// arm or after the bus takes it back.
#[test]
fn lane_runs_only_while_lent() {
    let mut r = rig();
    for i in 0..20 {
        r.tick(i);
    }
    assert!(r.wire.log().is_empty());
    r.arm(POS, 64);
    r.drain.stop();
    for i in 0..20 {
        r.tick(i);
    }
    assert!(r.wire.idle() && r.wire.log().is_empty());
}

/// The bus posts the chunks where it dispatches the arm and lends at the
/// ack's release: a post alone moves nothing, and the lend streams the
/// latest chunks posted.
#[test]
fn posted_chunks_wait_for_their_lend() {
    let mut r = rig();
    r.drain.post(POS);
    r.drain.post(SIX);
    for i in 0..20 {
        r.tick(i);
    }
    assert!(r.wire.log().is_empty());
    r.drain.start(16, ID);
    r.run(0, 40);
    let f = &r.wire.streamed()[0];
    assert_eq!(payload(f), golden(SIX, 0, true, 0..16));
    assert!(r.feed.idle());
}

/// A held start leaves the arm in the mailbox and the lane idle: the burst
/// starts on the first tick not held, with that tick's row, and a hold
/// once it streams holds nothing.
#[test]
fn a_held_start_waits_for_the_first_tick_not_held() {
    let mut r = rig();
    r.arm(SIX, 16);
    r.hold_start = true;
    for i in 0..5 {
        r.tick(i);
    }
    assert!(r.wire.log().is_empty(), "the lane moved on a held start");
    assert!(!r.feed.idle(), "the arm was dropped");
    r.hold_start = false;
    r.tick(5);
    r.hold_start = true;
    r.run(6, 50);
    let f = &r.wire.streamed()[0];
    assert_eq!(payload(f), golden(SIX, 0, true, 5..21));
    assert!(r.feed.idle());
}

/// A held tick takes no lane step: the break the start rule puts behind
/// row 7 waits a tick, and the frame still streams whole.
#[test]
fn a_held_tick_takes_no_lane_step() {
    let mut r = rig();
    r.arm(SIX, 16);
    for i in 0..8 {
        if i == 7 {
            r.feed.hold();
        }
        r.tick(i);
    }
    assert!(
        r.wire.idle() && !r.wire.breaking(),
        "a break on the held tick"
    );
    r.tick(8);
    assert!(r.wire.breaking(), "no break the tick after");
    r.run(9, 40);
    let f = &r.wire.streamed()[0];
    assert_eq!(payload(f), golden(SIX, 0, true, 0..16));
}

/// A hold outlives the row its tick drops: the frame end and the next
/// start that tick owed wait a tick.
#[test]
fn a_hold_outlives_a_dropped_row() {
    let mut r = rig();
    r.arm(POS, 64);
    for i in 0..37 {
        r.tick(i);
    }
    r.wire.finish();
    r.feed.hold();
    r.tick(37);
    assert_eq!(r.ch.drops(), 6);
    assert!(r.wire.idle() && !r.wire.breaking(), "frame 1 started");
    r.tick(38);
    assert!(r.wire.breaking(), "frame 1 never started");
}

/// The bus half's end of an arm zeroes `tel_count`.
#[test]
fn an_arm_end_zeroes_tel_count() {
    let mut r = rig();
    r.table.with_mut(|t| t.control.tel.tel_count = 4000);
    r.drain.end();
    assert_eq!(r.table.with(|t| t.control.tel.tel_count), 0);
}

/// The steps the TEL cost probe books: a row step on each row after the
/// pickup's (which lands frame 0's head), heads and banks by the frame's
/// rows landed, and one lane step on every tick the lane is the kernel's.
#[test]
fn tel_steps_follow_the_rows_and_the_lane() {
    let mut r = rig();
    r.steps = Some(std::vec::Vec::new());
    r.arm(SIX, 20);
    r.run(0, 64);
    assert_eq!(r.wire.streamed().len(), 2);
    let steps = r.steps.take().unwrap_or_default();
    let rows: std::vec::Vec<TelStep> = steps.iter().filter_map(|s| s.0).collect();
    let mut want = std::vec![TelStep::Row; 14];
    want.extend([TelStep::Bank, TelStep::Head, TelStep::Row, TelStep::Row]);
    want.push(TelStep::Bank);
    assert_eq!(rows, want, "row 0 lands with the pickup");
    let lanes: std::vec::Vec<TelStep> = steps.iter().filter_map(|s| s.1).collect();
    let active = steps.iter().take_while(|s| s.1.is_some()).count();
    assert_eq!(lanes.len(), active, "a lane step on every active tick");
    for (t, (row, lane)) in steps.iter().enumerate() {
        let landed = t == 0 || matches!(row, Some(TelStep::Head | TelStep::Bank));
        if landed {
            assert_eq!(
                *lane,
                Some(TelStep::Poll),
                "tick {t}: a landed tick holds the lane"
            );
        }
    }
    // Frame 0 starts cut-through: its arm, the feed past its bank, the
    // patch. Frame 1 starts banked, fed whole: the patch, then its arm.
    let work: std::vec::Vec<TelStep> = lanes
        .iter()
        .copied()
        .filter(|&s| s != TelStep::Poll)
        .collect();
    use TelStep::{Service, Start};
    assert_eq!(
        work,
        [Start, Service, Service, Service, Start, Service, Service]
    );
}
