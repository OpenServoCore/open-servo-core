//! Edge-to-stamp decoder: the adapter's captured wire edges reconstructed
//! into the byte+tick stamp stream [`crate::osc::parse_exchange`] consumes.
//!
//! The capture contract, measured on the production capture path at all
//! four bauds (own and servo traffic): every edge is tick-exact, breaks
//! included -- a break captures as its real fall plus a rise one law span
//! (10 bits) later. The gap between a break's rise and its frame's byte-0
//! fall is arbitrary idle (measured: 2-3 bits of TX-chain seam on the
//! adapter's own frames, ~5 bits of enable-when-ready seam on a servo
//! reply), so nothing anchors on law arithmetic: a dominant span of at
//! least 9.5 bits is a break (the servo's own qualification bar, sitting
//! between a 9-bit 0x00 character and the 10-bit law break at any legal
//! clock skew), and each frame anchors on its own byte-0 start fall.
//!
//! Characters sample at `start + bit*(3 + 2k)/2` (the integration
//! resampler's math) and re-anchor per character on their captured start
//! falls, so clock skew never accumulates. Unanchorable garble decodes as
//! garbage characters rather than vanishing -- superimposed-reply energy
//! stays visible to `parse_exchange` as trailing stamps.

use osc_client::wire::{Level, WireEdge};

/// One decoded stamp: byte + tick, wire order. Break stamps carry the
/// captured break-fall tick (BOUNDARY); data stamps carry their own
/// captured start-fall tick -- every tick is a real capture, no stride
/// synthesis anywhere.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct BStamp {
    pub tick: u32,
    pub byte: u8,
    pub flags: u8,
}

impl BStamp {
    /// Break stamp: tick is the captured break fall.
    pub const BOUNDARY: u8 = 1 << 1;
}

/// Decode a time-ordered edge capture into wire-order stamps.
/// `bit_ticks` = capture ticks per bit at the operational baud.
pub fn stamps_from_edges(edges: &[WireEdge], bit_ticks: u32) -> Vec<BStamp> {
    let b = bit_ticks as i64;
    let t = |i: usize| edges[i].tick as i64;
    let mut out = Vec::new();
    let mut i = 0;
    while i < edges.len() {
        if edges[i].level != Level::Low {
            i += 1;
            continue;
        }
        let Some(next) = edges.get(i + 1) else {
            // Dangling fall (a pulse or frame still in flight at the
            // drain): nothing trustworthy to decode.
            break;
        };
        // The break bar: a 0x00 character is 9 low bits, the law break 10.
        if (next.tick as i64 - t(i)) * 2 >= 19 * b {
            out.push(BStamp {
                tick: edges[i].tick,
                byte: 0,
                flags: BStamp::BOUNDARY,
            });
            i += 1;
            continue;
        }
        // A character start: decode the frame from here, one char per
        // captured start fall. Back-to-back streaming puts the next char
        // one char time out (+/- clock skew); anything later ends the
        // frame.
        let mut start = t(i);
        loop {
            out.push(BStamp {
                tick: start as u32,
                byte: sample_char(edges, start, b),
                flags: 0,
            });
            match next_fall_in(edges, start + 19 * b / 2, start + 11 * b) {
                Some(j) => start = t(j),
                None => break,
            }
        }
        i = edges.partition_point(|e| (e.tick as i64) < start + 10 * b);
    }
    out
}

/// Sample one 8N1 character anchored at its captured start fall.
fn sample_char(edges: &[WireEdge], start: i64, b: i64) -> u8 {
    let mut byte = 0u8;
    for k in 0..8u8 {
        let at = start + b * (3 + 2 * k as i64) / 2;
        if level_at(edges, at) == Level::High {
            byte |= 1 << k;
        }
    }
    byte
}

/// Line level at `at`: the latest edge's level, idle-high before any.
fn level_at(edges: &[WireEdge], at: i64) -> Level {
    match edges.partition_point(|e| (e.tick as i64) <= at) {
        0 => Level::High,
        n => edges[n - 1].level,
    }
}

/// Index of the first fall edge with tick in `[lo, hi]`.
fn next_fall_in(edges: &[WireEdge], lo: i64, hi: i64) -> Option<usize> {
    let from = edges.partition_point(|e| (e.tick as i64) < lo);
    edges[from..]
        .iter()
        .position(|e| e.level == Level::Low && (e.tick as i64) <= hi)
        .map(|p| from + p)
        .filter(|&j| (edges[j].tick as i64) <= hi)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::osc::{build_ping, parse_exchange};
    use osc_protocol::crc::osc_crc;

    /// Emit one 8N1 character's real edges (transitions of
    /// start + LSB-first data + stop) at `start`.
    fn char_edges(start: i64, b: i64, byte: u8, out: &mut Vec<WireEdge>) {
        let mut level = true;
        for (k, bit) in std::iter::once(false)
            .chain((0..8).map(|k| byte >> k & 1 == 1))
            .chain(std::iter::once(true))
            .enumerate()
        {
            if bit != level {
                out.push(WireEdge {
                    tick: (start + k as i64 * b) as u32,
                    level: if bit { Level::High } else { Level::Low },
                });
                level = bit;
            }
        }
    }

    /// Emit a break-framed frame the way the capture records it: the break
    /// fall, its rise 10 bits later, `seam` bits of idle, then exact data
    /// edges. `pitch` is the char spacing (10*b nominal; skew models the
    /// talker's HSI). `rise` = false drops the break rise (the capture
    /// contract keeps it, this pins the decoder's tolerance).
    fn frame_edges(
        break_fall: i64,
        b: i64,
        seam: i64,
        pitch: i64,
        bytes: &[u8],
        rise: bool,
        out: &mut Vec<WireEdge>,
    ) {
        out.push(WireEdge {
            tick: break_fall as u32,
            level: Level::Low,
        });
        if rise {
            out.push(WireEdge {
                tick: (break_fall + 10 * b) as u32,
                level: Level::High,
            });
        }
        let anchor = break_fall + 10 * b + seam;
        for (k, &byte) in bytes.iter().enumerate() {
            char_edges(anchor + k as i64 * pitch, b, byte, out);
        }
        out.sort_by_key(|e| e.tick);
    }

    /// A valid ping status frame for servo `id` (LEN 6: model + fw + CRC).
    fn ping_status(id: u8) -> Vec<u8> {
        let mut f = vec![id, 0x06, 0x80, 0x34, 0x12, 0x07];
        let crc = osc_crc(&f);
        f.extend_from_slice(&crc.to_le_bytes());
        f
    }

    #[test]
    fn decodes_one_frame_with_captured_break() {
        let b = 18; // 1M in the 18 MHz tick domain
        let sent = build_ping(1);
        let mut edges = Vec::new();
        // 2-bit own-TX seam between break rise and byte 0 (measured).
        frame_edges(1000, b, 2 * b, 10 * b, &sent, true, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 1 + sent.len());
        assert_eq!(stamps[0].byte, 0);
        assert_eq!(stamps[0].flags, BStamp::BOUNDARY);
        assert_eq!(stamps[0].tick, 1000, "break stamp = the captured fall");
        let anchor = 1000 + 12 * b;
        for (k, s) in stamps[1..].iter().enumerate() {
            assert_eq!(s.byte, sent[k], "byte {k}");
            assert_eq!(s.tick, (anchor + k as i64 * 10 * b) as u32, "tick {k}");
        }
    }

    #[test]
    fn bare_break_train_stamps_breaks_only() {
        // A CAL train: announce-less bare breaks must never decode as data.
        let b = 18;
        let mut edges = Vec::new();
        for k in 0..4i64 {
            let fall = 5_000 + k * 400 * b;
            edges.push(WireEdge {
                tick: fall as u32,
                level: Level::Low,
            });
            edges.push(WireEdge {
                tick: (fall + 10 * b) as u32,
                level: Level::High,
            });
        }
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 4);
        assert!(stamps.iter().all(|s| s.flags == BStamp::BOUNDARY));
        assert_eq!(stamps[3].tick, (5_000 + 3 * 400 * 18) as u32);
    }

    #[test]
    fn full_exchange_parses_with_exact_turnaround() {
        let b = 6; // 3M
        let sent = build_ping(1);
        let reply = ping_status(1);
        let mut edges = Vec::new();
        frame_edges(2000, b, 3 * b, 10 * b, &sent, true, &mut edges);
        // Instruction wire end = last echo char start + 10 bits.
        let echo_anchor = 2000 + 13 * b;
        let echo_end = echo_anchor + (sent.len() as i64 - 1) * 10 * b + 10 * b;
        let turnaround = 700; // ~39 us at 3M
        // Reply rides the servo's HSI (chars 1 tick wide of nominal) with
        // the ~5-bit enable-when-ready seam behind its break.
        frame_edges(
            echo_end + turnaround,
            b,
            5 * b,
            10 * b + 1,
            &reply,
            true,
            &mut edges,
        );
        let stamps = stamps_from_edges(&edges, b as u32);
        let ex = parse_exchange(&stamps, &sent, b as u32).expect("parses");
        assert_eq!(ex.turnaround_ticks, turnaround as u32);
        assert_eq!(ex.status.id, 1);
        assert_eq!(ex.status.payload, [0x34, 0x12, 0x07]);
        assert_eq!(ex.stamps_end, stamps.len());
    }

    #[test]
    fn burst_frames_anchor_independently() {
        let b = 18;
        let f1 = build_ping(1);
        let f2 = build_ping(2);
        let mut edges = Vec::new();
        frame_edges(1000, b, 2 * b, 10 * b, &f1, true, &mut edges);
        // Next break fall lands half a char after frame 1's wire end
        // (burst chaining gap is under a character time).
        let f1_end = 1000 + 12 * b + f1.len() as i64 * 10 * b;
        let brk2 = f1_end + 5 * b;
        frame_edges(brk2, b, 2 * b, 10 * b, &f2, true, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 2 * (1 + f1.len()));
        let second = &stamps[1 + f1.len()..];
        assert_eq!(second[0].flags, BStamp::BOUNDARY);
        assert_eq!(second[0].tick, brk2 as u32);
        assert_eq!(second[1].byte, f2[0]);
    }

    #[test]
    fn payload_zero_bytes_are_data_not_breaks() {
        let b = 18;
        // WRITE id 2 addr 0x0100 data AA (protocol sec 3.2 vector): the
        // 0x00 addr byte is a 9-bit low span -- under the break bar.
        let frame = [0x02, 0x06, 0x30, 0x00, 0x01, 0xAA, 0x07, 0x0D];
        let mut edges = Vec::new();
        frame_edges(9000, b, 2 * b, 10 * b, &frame, true, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 1 + frame.len());
        let bytes: Vec<u8> = stamps[1..].iter().map(|s| s.byte).collect();
        assert_eq!(bytes, frame);
        assert!(stamps[1..].iter().all(|s| s.flags == 0));
    }

    #[test]
    fn missing_break_rise_still_detects_the_break() {
        let b = 6;
        let sent = build_ping(3);
        let mut edges = Vec::new();
        frame_edges(4000, b, 2 * b, 10 * b, &sent, false, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 1 + sent.len());
        assert_eq!(stamps[0].flags, BStamp::BOUNDARY);
        assert_eq!(stamps[1].byte, sent[0]);
    }

    #[test]
    fn skewed_break_span_still_qualifies() {
        // A servo break at +1% clock reads 10.1 bits; the bar must take it.
        let b = 18;
        let sent = ping_status(1);
        let mut edges = Vec::new();
        frame_edges(3000, b, 5 * b, 10 * b, &sent, true, &mut edges);
        // Stretch the break rise to 10.1 bits after the fall.
        edges.iter_mut().for_each(|e| {
            if e.tick == (3000 + 10 * b) as u32 && e.level == Level::High {
                e.tick += (b / 10) as u32;
            }
        });
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps[0].flags, BStamp::BOUNDARY);
        assert_eq!(stamps.len(), 1 + sent.len());
    }
}
