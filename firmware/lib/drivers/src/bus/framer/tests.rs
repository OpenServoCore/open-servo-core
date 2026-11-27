//! The break stamp's rule at the framer: it only ends a frontier a newer
//! break lies strictly inside, in either order of latch and drain.

use super::*;

const TPB: u32 = 10;

/// A ring of `0xFF` with `bytes` laid in from index 0.
fn ring(bytes: &[u8]) -> [u8; 512] {
    let mut r = [0xFF; 512];
    r[..bytes.len()].copy_from_slice(bytes);
    r
}

fn resolve(f: &mut Framer, r: &[u8], cursor: u16) -> FramerOut {
    f.resolve(r, cursor, 0, TPB, |_| FrameNeeds::Covered)
}

/// Junk `00 00 07` with a ping's break behind it.
const JUNK_PING: [u8; 9] = [0x00, 0x00, 0x07, 0x00, 0x05, 0x03, 0x10, 0x11, 0x22];

#[test]
fn a_newer_break_inside_a_frontier_ends_it() {
    let r = ring(&JUNK_PING);
    let mut f = Framer::new();
    f.stamped(Some(3));
    assert!(matches!(resolve(&mut f, &r, 5), FramerOut::Cut));
    assert_eq!((f.anchor, f.drops()), (1, 1));
    // The hunt lands on the ping's break, which its own stamp leaves alone.
    assert!(matches!(resolve(&mut f, &r, 5), FramerOut::Wait(_)));
    assert_eq!((f.anchor, f.drops()), (3, 1));
}

#[test]
fn a_stamp_past_its_own_drained_break_keeps_the_frame() {
    let r = ring(&JUNK_PING[3..]);
    let mut f = Framer::new();
    f.stamped(Some(1));
    assert!(matches!(resolve(&mut f, &r, 2), FramerOut::Wait(_)));
    assert_eq!((f.anchor, f.drops()), (0, 0));
}

#[test]
fn a_drained_break_past_the_junk_ends_it_once_its_id_rings() {
    let r = ring(&JUNK_PING);
    let mut f = Framer::new();
    f.stamped(Some(4));
    assert!(matches!(resolve(&mut f, &r, 4), FramerOut::Wait(_)));
    assert_eq!(f.relook(&r, 4), Some(1));
    assert!(matches!(resolve(&mut f, &r, 5), FramerOut::Cut));
}

#[test]
fn a_stamp_at_the_cursor_waits_for_its_byte() {
    let r = ring(&JUNK_PING);
    let mut f = Framer::new();
    f.stamped(Some(3));
    assert!(matches!(resolve(&mut f, &r, 3), FramerOut::Wait(_)));
    assert_eq!((f.anchor, f.drops()), (0, 0));
    assert_eq!(f.relook(&r, 3), Some(1));
    assert!(matches!(resolve(&mut f, &r, 4), FramerOut::Cut));
}

#[test]
fn a_whole_frame_is_the_crcs_alone() {
    let r = ring(&JUNK_PING[3..]);
    let mut f = Framer::new();
    f.stamped(Some(3));
    assert!(matches!(resolve(&mut f, &r, 6), FramerOut::Frame(_)));
    assert_eq!(f.drops(), 0);
}

#[test]
fn a_stamp_behind_the_ladder_is_spent() {
    let r = ring(&JUNK_PING);
    let mut f = Framer::new();
    f.resync(3);
    f.stamped(Some(1));
    assert!(matches!(resolve(&mut f, &r, 5), FramerOut::Wait(_)));
    assert_eq!(f.stamp, None);
}
