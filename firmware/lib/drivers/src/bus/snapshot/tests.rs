//! The copy plan over every alignment: the units tile the span, every beat
//! is aligned on both ends, every CPU unit is one natural source load, and
//! every field a row promises lands in exactly one unit (the read-side
//! mirror of control-table's `copy_stores_each_aligned_field_whole`).

use super::*;
use std::boxed::Box;
use std::vec::Vec;

/// Where the plan tests put the snapshot buffer: any 4-aligned address.
const BUF: usize = 0x2000_0100;

fn lens() -> impl Iterator<Item = usize> {
    (0..=40).chain([240, 252])
}

#[derive(Debug, Clone, Copy)]
struct Unit {
    at: usize,
    width: usize,
    dma: bool,
}

/// The units one span's copy makes, in order.
fn units_of(src: usize, dst: usize, len: usize) -> (Plan, Vec<Unit>) {
    let p = plan(src, dst, len);
    assert_eq!(p.mid, p.head + p.beat as usize * p.beats, "{p:?}");
    let mut v = Vec::new();
    units(src, &p, len, |at, w| {
        v.push(Unit {
            at,
            width: w as usize,
            dma: false,
        })
    });
    for k in 0..p.beats {
        v.push(Unit {
            at: p.head + k * p.beat as usize,
            width: p.beat as usize,
            dma: true,
        });
    }
    v.sort_by_key(|u| u.at);
    (p, v)
}

/// Assert the rules every copy keeps; `whole` is the widest field the copy
/// promises whole.
fn check(src: usize, dst: usize, len: usize, whole: usize) -> (Plan, Vec<Unit>) {
    let (p, us) = units_of(src, dst, len);
    let mut next = 0;
    for u in &us {
        assert_eq!(
            u.at, next,
            "gap or overlap: src {src:#x} dst {dst:#x} len {len} {p:?}"
        );
        next += u.width;
        assert_eq!(
            (src + u.at) % u.width,
            0,
            "unaligned load: src {src:#x} dst {dst:#x} len {len} {p:?}"
        );
        if u.dma || (src ^ dst) & 3 == 0 {
            assert_eq!(
                (dst + u.at) % u.width,
                0,
                "unaligned store: src {src:#x} dst {dst:#x} len {len} {p:?}"
            );
        }
    }
    assert_eq!(
        next, len,
        "short or long: src {src:#x} dst {dst:#x} len {len} {p:?}"
    );
    let first = us.iter().position(|u| u.dma).unwrap_or(0);
    assert!(
        us.iter().skip(first).take(p.beats).all(|u| u.dma),
        "the beats are not one arm: src {src:#x} dst {dst:#x} len {len} {p:?}"
    );
    for w in [2usize, 4].into_iter().filter(|&w| w <= whole) {
        for f in (src..src + len).filter(|a| a % w == 0 && a + w <= src + len) {
            let n = us
                .iter()
                .filter(|u| src + u.at <= f && f + w <= src + u.at + u.width)
                .count();
            assert_eq!(
                n, 1,
                "field {f:#x}+{w}: src {src:#x} dst {dst:#x} len {len} {p:?}"
            );
        }
    }
    (p, us)
}

#[test]
fn a_plain_read_copies_every_field_whole() {
    for src in 0x200..0x208 {
        for len in lens() {
            let base = BUF + frame_shift(src);
            let dst = base + Header::SIZE;
            assert_eq!(dst % 4, src % 4, "src {src:#x}");
            assert!(
                base - BUF + Header::SIZE + len <= LEN,
                "src {src:#x} len {len}"
            );
            assert!(base % 2 == 0 || base > BUF, "no pad byte: src {src:#x}");
            let (p, us) = check(src, dst, len, 4);
            assert!(
                p.beats == 0 || p.beat == Beat::Word,
                "src {src:#x} len {len} {p:?}"
            );
            let cpu: usize = us.iter().filter(|u| !u.dma).map(|u| u.width).sum();
            assert!(
                cpu <= 6 || len <= CPU_SPAN_MAX,
                "src {src:#x} len {len} {p:?}"
            );
        }
    }
}

#[test]
fn a_gathered_span_copies_per_its_row() {
    for s in 0..4 {
        for d in 0..4 {
            for len in lens() {
                let (src, dst) = (0x200 + s, BUF + d);
                let skew = (s ^ d) & 3;
                let whole = match skew {
                    _ if skew == 0 || len <= CPU_SPAN_MAX => 4,
                    _ if skew & 1 == 0 => 2,
                    _ => 1,
                };
                let (p, us) = check(src, dst, len, whole);
                let cpu: Vec<_> = us.iter().filter(|u| !u.dma).collect();
                let case = (s, d, len, p);
                match whole {
                    _ if len <= CPU_SPAN_MAX => assert_eq!(p.beats, 0, "{case:?}"),
                    _ if skew == 0 => assert_eq!(p.beat, Beat::Word, "{case:?}"),
                    2 => {
                        assert_eq!(p.beat, Beat::Half, "{case:?}");
                        assert!(
                            cpu.len() <= 2 && cpu.iter().all(|u| u.width == 1),
                            "{case:?}"
                        );
                    }
                    _ => {
                        assert_eq!((p.beat, p.beats), (Beat::Byte, len), "{case:?}");
                        assert!(cpu.is_empty(), "{case:?}");
                    }
                }
            }
        }
    }
}

#[repr(C, align(4))]
struct Src([u8; 512]);

/// Run `gather` against a channel that refuses an unaligned beat, and check
/// the payload lands whole, shifted to the first span's alignment, with no
/// byte written outside it.
fn gather_exact(spans: &[&[u8]]) {
    let mut buf = Box::new(Buf([0xA5; LEN]));
    let bp: *mut Buf = &mut *buf;
    // SAFETY: the tests' spans total at most MAX_PAYLOAD; `gather` hands
    // the channel in-bounds ends.
    let base = unsafe {
        gather(bp, spans, |s, d, beat, beats| {
            let w = beat as usize;
            assert_eq!((s.addr() | d.addr()) % w, 0, "unaligned beat");
            core::ptr::copy_nonoverlapping(s, d, w * beats as usize);
        })
    };
    let shift = base.addr() - bp.addr();
    assert_eq!(shift, frame_shift(spans[0].as_ptr().addr()));
    let concat = spans.concat();
    let at = shift + Header::SIZE;
    assert_eq!(buf.0[at..at + concat.len()], concat[..], "{spans:02x?}");
    assert!(
        buf.0[..at]
            .iter()
            .chain(&buf.0[at + concat.len()..])
            .all(|&b| b == 0xA5),
        "a byte outside the payload: {spans:02x?}"
    );
}

#[test]
fn gather_lands_the_payload_at_the_source_alignment() {
    let src = Src(core::array::from_fn(|i| (i as u8) ^ 0x5A));
    let s = &src.0;
    for a in 0..8 {
        for len in lens() {
            gather_exact(&[&s[a..a + len]]);
        }
    }
    for a in 0..4 {
        for l0 in 1..=9 {
            for b in 0..4 {
                for l1 in [1, 2, 3, 4, 5, 8, 9, 40] {
                    let c = 200 + (a + b) % 4;
                    gather_exact(&[&s[a..a + l0], &s[64 + b..64 + b + l1], &s[c..c + 7]]);
                }
            }
        }
    }
}
