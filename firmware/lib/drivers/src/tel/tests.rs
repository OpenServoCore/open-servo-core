use super::*;
use crate::mocks::bus::{FakeCrc, FakeRing, FakeWire, TestProviders};
use osc_protocol::crc::osc_crc;
use osc_protocol::tel::StreamFrame;
use osc_servo_core::tel::{
    BIT_CURRENT, BIT_CURRENT_RAW, BIT_POS, BIT_POS_LIN, BIT_VBUS, BIT_VDIFF, encode_stream,
};

/// pos, current, trough, duty, vdiff, vbus: an 11 B row.
const MASK_SIX: u16 = 0x3F;
const ID: u8 = 9;
const RATE: BaudRate = BaudRate::B3000000;

struct Rig {
    feed: TelFeed<TestProviders>,
    drain: TelDrain,
    wire: FakeWire,
    ring: FakeRing,
    ch: &'static TelChannel,
}

fn rig() -> Rig {
    rig_with(FakeCrc::new())
}

fn rig_with(crc: FakeCrc) -> Rig {
    let ch: &'static TelChannel = std::boxed::Box::leak(std::boxed::Box::new(TelChannel::new()));
    let wire = FakeWire::new();
    let ring = FakeRing::new();
    let (feed, drain) = ch.split::<TestProviders>(wire.clone(), crc, ring.clone());
    Rig {
        feed,
        drain,
        wire,
        ring,
        ch,
    }
}

impl Rig {
    /// The kernel's tick: ask for the row, hand it over, then the tail.
    /// A break left on the wire ends first (11 bit-times, under a tick).
    fn tick(&mut self, s: &TelSample) {
        if self.wire.breaking() {
            self.wire.finish();
        }
        if self.feed.row_mask() != 0 {
            self.feed.on_tick(s);
        }
        self.feed.on_tick_end();
    }

    /// Ticks rows `from..to`, the wire carrying each image for its wire
    /// time at 3M (15 bytes a tick) and reading it whole at the end.
    fn run(&mut self, from: u16, to: u16) {
        const BYTES_PER_TICK: usize = 15;
        let mut left: Option<usize> = None;
        for i in from..to {
            self.tick(&sample(i));
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

    /// The bus's arm: post the plan, then lend the lane.
    fn arm(&mut self, mask: u16, count: u16, id: u8, rate: BaudRate) -> u8 {
        self.drain.post(mask);
        self.drain.start(count, id, rate)
    }

    /// Ticks until the wire carries an image, at most `bound`.
    fn tick_to_stream(&mut self, i: &mut u16, bound: u16) {
        for _ in 0..bound {
            self.tick(&sample(*i));
            *i += 1;
            if !self.wire.idle() && !self.wire.breaking() {
                return;
            }
        }
        panic!("no image armed in {bound} ticks");
    }
}

fn sample(i: u16) -> TelSample {
    TelSample {
        pos: 0x400 + i,
        current: -(i as i16),
        tick: 0xFFF0u16.wrapping_add(i),
        window_valid: i.is_multiple_of(2),
        fault: i == 3,
        ..Default::default()
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

fn golden(mask: u16, seq: u8, last: bool, rows: core::ops::Range<u16>) -> std::vec::Vec<u8> {
    let s: std::vec::Vec<TelSample> = rows.map(sample).collect();
    let mut buf = [0u8; STREAM_PAYLOAD_MAX];
    let n = encode_stream(mask, seq, last, &s, &mut buf);
    buf[..n].to_vec()
}

/// Two frames, the second the LAST: each starts on its lead row, the wire
/// reads it at its own pace, and it decodes byte for byte as the reference
/// encoder's payload behind the frame header.
#[test]
fn frames_stream_cut_through_and_match_encode_stream() {
    let mut r = rig();
    r.arm(MASK_SIX, 20, ID, RATE);
    let mut i = 0;
    r.tick_to_stream(&mut i, 16);
    // 11 B rows at 3M: cov 190, the CRC 188 byte-times (12.5 ticks) behind
    // the arm, so the break follows row 8 and the arm row 9.
    assert_eq!(i, 10, "armed on the tick after row 8's break");
    r.run(i, 64);
    let frames = r.wire.streamed();
    assert_eq!(frames.len(), 2);
    let inst = |alert: bool| Inst::status(ResultCode::Stream, alert).0;
    assert_eq!(
        frames[0][..3],
        [ID, wire::len_for(4 + 16 * 11 + 6), inst(false)]
    );
    assert_eq!(payload(&frames[0]), golden(MASK_SIX, 0, false, 0..16));
    assert_eq!(
        frames[1][..3],
        [ID, wire::len_for(4 + 4 * 11 + 6), inst(false)]
    );
    assert_eq!(payload(&frames[1]), golden(MASK_SIX, 1, true, 16..20));
    assert!(r.feed.idle());
}

/// An image the wire reads before its rows land ships stale bytes against
/// a CRC the engine took over the fresh ones: the frame fails its CRC.
#[test]
fn rows_landing_behind_the_read_fail_the_crc() {
    let mut r = rig();
    r.arm(MASK_SIX, 16, ID, RATE);
    let mut i = 0;
    r.tick_to_stream(&mut i, 16);
    r.wire.finish();
    while i < 20 {
        r.tick(&sample(i));
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
    r.arm(MASK_SIX, 40, ID, RATE);
    r.run(0, 120);
    let frames = r.wire.streamed();
    assert_eq!(frames.len(), 3);
    for f in &frames {
        assert_eq!(f[f.len() - CRC_LEN..], [0, 0], "never patched");
    }
    assert!(r.feed.idle());
}

/// The start rule at each rate's widest admitted row: the rows a full
/// frame lands before its break.
#[test]
fn lead_follows_the_start_rule() {
    use osc_servo_core::tel::row_bytes;
    for (rate, mask, lead) in [
        (BaudRate::B3000000, 0x1CD, 10),
        (BaudRate::B3000000, MASK_SIX, 8),
        (
            BaudRate::B2000000,
            BIT_CURRENT | BIT_VDIFF | BIT_VBUS | BIT_POS_LIN,
            7,
        ),
        (BaudRate::B1000000, BIT_POS | BIT_CURRENT_RAW, 9),
    ] {
        let mut r = rig();
        r.arm(mask, 32, ID, rate);
        let mut n = 0;
        while r.wire.idle() {
            r.tick(&sample(n));
            n += 1;
        }
        assert_eq!(
            n - 1,
            lead,
            "{rate:?} {} B: break after row {lead}",
            row_bytes(mask)
        );
    }
}

/// An odd covered span folds its last byte into the engine's result.
#[test]
fn odd_covered_span_folds_its_tail_byte() {
    let mut r = rig();
    // current_raw and vmotor_a: one 12-bit pair, a 3 B row; three rows
    // leave the covered span odd.
    let mask = 0x0C0;
    assert_eq!(osc_servo_core::tel::row_bytes(mask), 3);
    r.arm(mask, 3, ID, RATE);
    for i in 0..8 {
        r.tick(&sample(i));
        r.wire.finish();
    }
    let f = &r.wire.streamed()[0];
    assert_eq!(payload(f), golden(mask, 0, true, 0..3));
}

/// Both images hold unfinished frames: the next frame's rows drop, are
/// counted, and the next frame's stamp jumps over them.
#[test]
fn full_images_drop_and_count() {
    let mut r = rig();
    r.arm(BIT_POS, 64, ID, RATE);
    let mut i = 0;
    // The wire never finishes an image: frame 0 streams, frame 1 banks at
    // row 31, and frame 2's row 0 drops from tick 32 on.
    for _ in 0..37 {
        r.tick(&sample(i));
        i += 1;
    }
    assert_eq!(r.ch.drops(), 5);
    // Frame 0 leaves the wire; tick 37's row lands before its tail polls
    // that, so it drops too, and frame 2 opens on row 38.
    r.wire.finish();
    for _ in 0..40 {
        r.tick(&sample(i));
        r.wire.finish();
        i += 1;
    }
    assert_eq!(r.ch.drops(), 6);
    let frames = r.wire.streamed();
    let f2 = StreamFrame::parse(BIT_POS, payload(&frames[2])).expect("frame 2");
    assert_eq!((f2.seq, f2.tick), (2, sample(38).tick));
}

/// The LAST frame's arm publishes the RX cursor the bus resumes at, and
/// nothing before it does; an empty arm hands the lane straight back.
#[test]
fn last_frame_publishes_the_resume_cursor() {
    let mut r = rig();
    let epoch = r.arm(BIT_POS, 2, ID, RATE);
    r.ring.set_cursor(77);
    let mut i = 0;
    while r.wire.streaming().is_none() {
        assert_eq!(r.drain.last_out(epoch), None, "tick {i}");
        r.tick(&sample(i));
        i += 1;
        assert!(i < 16, "the LAST frame never armed");
    }
    assert_eq!(r.drain.last_out(epoch), Some(77));

    let epoch = r.arm(0, 5, ID, RATE);
    r.ring.set_cursor(91);
    r.tick(&sample(0));
    assert_eq!(r.drain.last_out(epoch), Some(91));
    assert!(r.feed.idle());
}

/// The lane runs only while lent: no tick touches the wire before the
/// arm or after the bus takes it back.
#[test]
fn lane_runs_only_while_lent() {
    let mut r = rig();
    for i in 0..20 {
        r.tick(&sample(i));
    }
    assert!(r.wire.log().is_empty());
    r.arm(BIT_POS, 64, ID, RATE);
    r.drain.stop();
    for i in 0..20 {
        r.tick(&sample(i));
    }
    assert!(r.wire.idle() && r.wire.log().is_empty());
}

/// The bus posts the plan where it dispatches the arm and lends at the
/// ack's release: a post alone moves nothing, and the lend streams the
/// latest plan posted.
#[test]
fn a_posted_plan_waits_for_its_lend() {
    let mut r = rig();
    r.drain.post(BIT_POS);
    r.drain.post(MASK_SIX);
    for i in 0..20 {
        r.tick(&sample(i));
    }
    assert!(r.wire.log().is_empty());
    r.drain.start(16, ID, RATE);
    r.run(0, 40);
    let f = &r.wire.streamed()[0];
    assert_eq!(payload(f), golden(MASK_SIX, 0, true, 0..16));
    assert!(r.feed.idle());
}
