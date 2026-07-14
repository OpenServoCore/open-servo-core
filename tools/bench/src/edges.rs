//! Edge-to-stamp decoder: the adapter's captured wire edges reconstructed
//! into the byte+tick stamps [`crate::osc::parse_exchange`] consumes.
//!
//! The capture contract (measured on silicon, own and foreign traffic):
//! data-region edges -- each frame's byte-0 start fall through the last
//! byte's interior edges -- are tick-exact; BREAK edges are never
//! trustworthy. The break fall is never captured, the break rise only
//! sometimes, and every break instead contributes one phantom fall plus
//! one phantom rise at fixed sub-break offsets (rise ~4.0 bits, fall
//! ~2.0-2.4 bits before the frame's byte-0 fall). A frame's final fall can
//! also stamp ~12 ticks late (watch item: if that fall is decode-sensitive
//! the frame fails its CRC and the exchange retries).
//!
//! The decoder therefore anchors every frame on its byte-0 start fall and
//! derives break geometry by law arithmetic: break fall = anchor - 11
//! rx-bits (10-bit law break + 1 stop, protocol sec 3), frame end = last
//! byte's start fall + 10 bits. Characters sample at
//! `start + bit*(3 + 2k)/2` (the integration resampler's math) and each
//! re-anchors on its own captured start fall, so HSI drift never
//! accumulates. Bare breaks (CAL trains, pulses) produce no stamps -- with
//! no data region there is nothing trustworthy to anchor on.

use osc_client::wire::{Level, WireEdge};

// Stamp semantics here: break stamps carry the law-derived fall tick
// (BOUNDARY); data stamps carry their own captured start-fall tick --
// every interior byte is a real capture, not a stride synthesis.
use crate::pirate::BStamp;

/// Decode a time-ordered edge capture into wire-order stamps.
/// `bit_ticks` = capture ticks per bit at the operational baud.
pub fn stamps_from_edges(edges: &[WireEdge], bit_ticks: u32) -> Vec<BStamp> {
    let b = bit_ticks as i64;
    let t = |i: usize| edges[i].tick as i64;
    let mut out = Vec::new();
    let mut from = 0;
    while let Some(a) = find_anchor(edges, from, b) {
        out.push(BStamp {
            tick: (t(a) - 11 * b) as u32,
            byte: 0,
            flags: BStamp::BOUNDARY,
        });
        let mut start = t(a);
        loop {
            out.push(BStamp {
                tick: start as u32,
                byte: sample_char(edges, start, b),
                flags: 0,
            });
            // Back-to-back streaming: the next character's start fall lands
            // one char time out (+/- clock skew); anything later is the
            // frame's end.
            match next_fall_in(edges, start + 19 * b / 2, start + 11 * b) {
                Some(j) => start = t(j),
                None => break,
            }
        }
        from = edges.partition_point(|e| (e.tick as i64) < start + 10 * b);
    }
    out
}

/// Find the next frame anchor (byte-0 start fall) at or after `from`.
///
/// A real break leaves a fixed signature before the anchor: an
/// edge-silent break body (the fall is never captured), then the phantom
/// cluster inside the last ~4.5 bits. The phantom fall itself is a decoy
/// candidate; what separates the true anchor is the cluster BEHIND it --
/// the anchor is preceded by the phantom fall (a fall in its cluster),
/// while the phantom fall is preceded only by the phantom rise, too far
/// out to be a break rise.
fn find_anchor(edges: &[WireEdge], from: usize, b: i64) -> Option<usize> {
    for j in from..edges.len() {
        if edges[j].level != Level::Low {
            continue;
        }
        let f = edges[j].tick as i64;
        let cluster_lo = f - 9 * b / 2;
        let quiet_lo = f - 21 * b / 2;
        let quiet = |e: &WireEdge| {
            let x = e.tick as i64;
            x >= quiet_lo && x < cluster_lo
        };
        if edges[..j].iter().rev().any(quiet) {
            continue;
        }
        let cluster: Vec<&WireEdge> = edges[..j]
            .iter()
            .rev()
            .take_while(|e| (e.tick as i64) >= cluster_lo)
            .collect();
        let has_fall = cluster.iter().any(|e| e.level == Level::Low);
        // Sometimes-captured break rise: alone, one bit before byte 0.
        let lone_break_rise = matches!(cluster.as_slice(), [e]
            if e.level == Level::High && (f - e.tick as i64 - b).abs() <= 2 * b / 5);
        if has_fall || lone_break_rise || cluster.is_empty() {
            return Some(j);
        }
    }
    None
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

    /// Emit a break-framed frame the way the capture actually records it:
    /// no break fall, the phantom pair at the measured sub-break offsets,
    /// optionally the sometimes-captured break rise, then exact data
    /// edges. `anchor` is the byte-0 start fall; `pitch` the char spacing
    /// (10*b nominal; skew models the talker's HSI).
    fn frame_edges(
        anchor: i64,
        b: i64,
        pitch: i64,
        bytes: &[u8],
        break_rise: bool,
        out: &mut Vec<WireEdge>,
    ) {
        out.push(WireEdge {
            tick: (anchor - 4 * b) as u32,
            level: Level::High,
        });
        out.push(WireEdge {
            tick: (anchor - 7 * b / 3) as u32,
            level: Level::Low,
        });
        if break_rise {
            out.push(WireEdge {
                tick: (anchor - b) as u32,
                level: Level::High,
            });
        }
        for (k, &byte) in bytes.iter().enumerate() {
            char_edges(anchor + k as i64 * pitch, b, byte, out);
        }
    }

    /// A valid ping status frame for servo `id` (LEN 6: model + fw + CRC).
    fn ping_status(id: u8) -> Vec<u8> {
        let mut f = vec![id, 0x06, 0x80, 0x34, 0x12, 0x07];
        let crc = osc_crc(&f);
        f.extend_from_slice(&crc.to_le_bytes());
        f
    }

    #[test]
    fn decodes_one_frame_with_law_derived_break() {
        let b = 18; // 1M in the 18 MHz tick domain
        let sent = build_ping(1);
        let mut edges = Vec::new();
        frame_edges(1000, b, 10 * b, &sent, true, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 1 + sent.len());
        assert_eq!(stamps[0].byte, 0);
        assert_eq!(stamps[0].flags, BStamp::BOUNDARY);
        assert_eq!(stamps[0].tick, (1000 - 11 * b) as u32);
        for (k, s) in stamps[1..].iter().enumerate() {
            assert_eq!(s.byte, sent[k], "byte {k}");
            assert_eq!(s.tick, (1000 + k as i64 * 10 * b) as u32, "tick {k}");
        }
    }

    #[test]
    fn phantom_cluster_is_never_a_frame() {
        // A bare break (CAL mark / pulse): phantoms + break rise, no data.
        let b: i64 = 18;
        let edges = vec![
            WireEdge {
                tick: (5000 - 4 * b) as u32,
                level: Level::High,
            },
            WireEdge {
                tick: (5000 - 7 * b / 3) as u32,
                level: Level::Low,
            },
            WireEdge {
                tick: (5000 - b) as u32,
                level: Level::High,
            },
        ];
        assert!(stamps_from_edges(&edges, b as u32).is_empty());
    }

    #[test]
    fn full_exchange_parses_with_exact_turnaround() {
        let b = 6; // 3M
        let sent = build_ping(1);
        let reply = ping_status(1);
        let mut edges = Vec::new();
        let echo_anchor = 2000;
        frame_edges(echo_anchor, b, 10 * b, &sent, false, &mut edges);
        // Instruction wire end = last echo char start + 10 bits.
        let echo_end = echo_anchor + (sent.len() as i64 - 1) * 10 * b + 10 * b;
        let turnaround = 700; // ~39 us at 3M
        let reply_anchor = echo_end + turnaround + 11 * b;
        // Reply rides the servo's HSI: chars 1 tick wide of nominal.
        frame_edges(reply_anchor, b, 10 * b + 1, &reply, true, &mut edges);
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
        frame_edges(1000, b, 10 * b, &f1, true, &mut edges);
        // Next break fall lands half a char after frame 1's wire end
        // (burst chaining gap is under a character time).
        let f1_end = 1000 + f1.len() as i64 * 10 * b;
        let a2 = f1_end + 5 * b + 11 * b;
        frame_edges(a2, b, 10 * b, &f2, false, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 2 * (1 + f1.len()));
        let second = &stamps[1 + f1.len()..];
        assert_eq!(second[0].flags, BStamp::BOUNDARY);
        assert_eq!(second[0].tick, (a2 - 11 * b) as u32);
        assert_eq!(second[1].byte, f2[0]);
    }

    #[test]
    fn payload_zero_bytes_are_data_not_breaks() {
        let b = 18;
        // WRITE id 5 addr 0x0100 data AA (protocol sec 3.2 vector): LEN
        // even, contains 0x00 bytes in addr.
        let frame = [0x02, 0x06, 0x30, 0x00, 0x01, 0xAA, 0x07, 0x0D];
        let mut edges = Vec::new();
        frame_edges(9000, b, 10 * b, &frame, true, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 1 + frame.len());
        let bytes: Vec<u8> = stamps[1..].iter().map(|s| s.byte).collect();
        assert_eq!(bytes, frame);
        assert!(stamps[1..].iter().all(|s| s.flags == 0));
    }

    #[test]
    fn missing_break_rise_still_anchors() {
        let b = 6;
        let sent = build_ping(3);
        let mut edges = Vec::new();
        frame_edges(4000, b, 10 * b, &sent, false, &mut edges);
        // 3M phantom fall offset is -2.0 bits, not -2.33.
        edges[1].tick = (4000 - 2 * b) as u32;
        let stamps = stamps_from_edges(&edges, b as u32);
        assert_eq!(stamps.len(), 1 + sent.len());
        assert_eq!(stamps[1].byte, sent[0]);
    }
}
