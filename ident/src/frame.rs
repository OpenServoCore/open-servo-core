//! Byte-level parsers: the telemetry-region snapshot (bus gread) and the
//! TEL burst stream (`Stream`-result status frames on the main bus). Both
//! mirror the firmware layout - offsets come from [`crate::regs`], the
//! stream shape from the core `tel` module - and are pinned by golden
//! vectors, never imported.

use crate::regs::{Reg, telemetry};

/// One decoded read of the telemetry region. `base` is the gread start
/// address; the slice must cover every field below or parse returns None.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelemetrySnapshot {
    pub fault_flags: u8,
    pub status_flags: u8,
    pub mode_active: u8,
    pub fault_code: u8,
    pub theta_hat_q16: i32,
    pub omega_hat_cps: i32,
    pub tau_d_counts: i16,
    pub i_lim_counts: u16,
    pub t_winding_cc: i16,
    pub vbus_counts: u16,
    pub duty_applied_q15: i16,
    pub omega_bemf_cps: i16,
    pub r_hat_q12: u16,
    pub i_hat_counts: i16,
    pub sample_tick: u32,
    pub pos: u16,
    pub current: u16,
    pub current_trough: u16,
    pub current_bias_counts: u16,
    pub i_mean_counts: i16,
    pub i_min_counts: i16,
    pub i_max_counts: i16,
    pub vdiff_mean: i16,
    pub duty_mean_q15: i16,
    pub agg_seq: u16,
}

fn get<const N: usize>(base: u16, bytes: &[u8], r: Reg) -> Option<[u8; N]> {
    debug_assert_eq!(r.width as usize, N);
    let off = r.addr.checked_sub(base)? as usize;
    bytes.get(off..off + N)?.try_into().ok()
}

impl TelemetrySnapshot {
    pub fn parse(base: u16, bytes: &[u8]) -> Option<Self> {
        use telemetry as t;
        Some(Self {
            fault_flags: u8::from_le_bytes(get(base, bytes, t::FAULT_FLAGS)?),
            status_flags: u8::from_le_bytes(get(base, bytes, t::STATUS_FLAGS)?),
            mode_active: u8::from_le_bytes(get(base, bytes, t::MODE_ACTIVE)?),
            fault_code: u8::from_le_bytes(get(base, bytes, t::FAULT_CODE)?),
            theta_hat_q16: i32::from_le_bytes(get(base, bytes, t::THETA_HAT_Q16)?),
            omega_hat_cps: i32::from_le_bytes(get(base, bytes, t::OMEGA_HAT_CPS)?),
            tau_d_counts: i16::from_le_bytes(get(base, bytes, t::TAU_D_COUNTS)?),
            i_lim_counts: u16::from_le_bytes(get(base, bytes, t::I_LIM_COUNTS)?),
            t_winding_cc: i16::from_le_bytes(get(base, bytes, t::T_WINDING_CC)?),
            vbus_counts: u16::from_le_bytes(get(base, bytes, t::VBUS_COUNTS)?),
            duty_applied_q15: i16::from_le_bytes(get(base, bytes, t::DUTY_APPLIED_Q15)?),
            omega_bemf_cps: i16::from_le_bytes(get(base, bytes, t::OMEGA_BEMF_CPS)?),
            r_hat_q12: u16::from_le_bytes(get(base, bytes, t::R_HAT_Q12)?),
            i_hat_counts: i16::from_le_bytes(get(base, bytes, t::I_HAT_COUNTS)?),
            sample_tick: u32::from_le_bytes(get(base, bytes, t::SAMPLE_TICK)?),
            pos: u16::from_le_bytes(get(base, bytes, t::POS)?),
            current: u16::from_le_bytes(get(base, bytes, t::CURRENT)?),
            current_trough: u16::from_le_bytes(get(base, bytes, t::CURRENT_TROUGH)?),
            current_bias_counts: u16::from_le_bytes(get(base, bytes, t::CURRENT_BIAS_COUNTS)?),
            i_mean_counts: i16::from_le_bytes(get(base, bytes, t::I_MEAN_COUNTS)?),
            i_min_counts: i16::from_le_bytes(get(base, bytes, t::I_MIN_COUNTS)?),
            i_max_counts: i16::from_le_bytes(get(base, bytes, t::I_MAX_COUNTS)?),
            vdiff_mean: i16::from_le_bytes(get(base, bytes, t::VDIFF_MEAN)?),
            duty_mean_q15: i16::from_le_bytes(get(base, bytes, t::DUTY_MEAN_Q15)?),
            agg_seq: u16::from_le_bytes(get(base, bytes, t::AGG_SEQ)?),
        })
    }
}

/// Unwraps a wrapping u16 sequence (agg_seq) into a monotone u64 timebase.
/// Treats any backward step as a wrap, so feed it in arrival order.
#[derive(Default)]
pub struct SeqUnwrap {
    last: Option<u16>,
    epoch: u64,
}

impl SeqUnwrap {
    pub fn push(&mut self, seq: u16) -> u64 {
        if let Some(last) = self.last
            && seq < last
        {
            self.epoch += 1 << 16;
        }
        self.last = Some(seq);
        self.epoch + seq as u64
    }
}

// --- TEL stream ------------------------------------------------------------
// Mirror of firmware/lib/core/src/tel.rs; the golden tests pin the bytes.
// One Stream-result status payload, all LE:
//   [0]    stream_seq  u8, increments per frame, wraps; restarts at 0 per arm
//   [1]    flags       bit 0 = LAST frame of the burst
//   [2..4] valid       u16 bitmap, bit i = sample i window_valid
//   [4..]  up to 16 samples x the tel_mask-selected 2-byte fields in bit order

pub const TEL_BIT_POS: u16 = 1 << 0;
pub const TEL_BIT_CURRENT: u16 = 1 << 1;
pub const TEL_BIT_CURRENT_TROUGH: u16 = 1 << 2;
pub const TEL_BIT_DUTY: u16 = 1 << 3;
pub const TEL_BIT_VDIFF: u16 = 1 << 4;
pub const TEL_BIT_VBUS: u16 = 1 << 5;
pub const TEL_MASK_ALL: u16 = 0x3F;

pub const STREAM_HDR: usize = 4;
pub const STREAM_SAMPLES_MAX: usize = 16;
pub const STREAM_FLAG_LAST: u8 = 1 << 0;

pub const fn sample_len(mask: u16) -> usize {
    2 * (mask & TEL_MASK_ALL).count_ones() as usize
}

/// One decoded TEL sample; unselected fields are None. `tick` is the
/// absolute fast-tick index within one burst (unwrapped stream_seq x 16 +
/// position in frame), so time = tick / tick_hz and a dropped frame shows
/// as a 16-tick hole.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelFrame {
    pub tick: u64,
    pub window_valid: bool,
    pub pos: Option<u16>,
    pub current: Option<i16>,
    pub current_trough: Option<u16>,
    pub duty_q15: Option<i16>,
    pub vdiff: Option<i16>,
    pub vbus: Option<u16>,
}

/// Decode one stream payload into per-tick frames; sample i lands at
/// `tick_base + i`. None when the payload cannot be a `mask` stream frame
/// (short header, non-integral sample remainder, over 16 samples).
pub fn decode_stream_payload(mask: u16, payload: &[u8], tick_base: u64) -> Option<Vec<TelFrame>> {
    let slen = sample_len(mask);
    if payload.len() < STREAM_HDR {
        return None;
    }
    let valid = u16::from_le_bytes([payload[2], payload[3]]);
    let body = &payload[STREAM_HDR..];
    if slen == 0 {
        return body.is_empty().then(Vec::new);
    }
    if !body.len().is_multiple_of(slen) || body.len() / slen > STREAM_SAMPLES_MAX {
        return None;
    }
    let mut out = Vec::with_capacity(body.len() / slen);
    for (i, ch) in body.chunks(slen).enumerate() {
        let mut off = 0;
        let mut take = |bit: u16| {
            if mask & bit != 0 {
                let v = [ch[off], ch[off + 1]];
                off += 2;
                Some(v)
            } else {
                None
            }
        };
        out.push(TelFrame {
            tick: tick_base + i as u64,
            window_valid: valid & (1 << i) != 0,
            pos: take(TEL_BIT_POS).map(u16::from_le_bytes),
            current: take(TEL_BIT_CURRENT).map(i16::from_le_bytes),
            current_trough: take(TEL_BIT_CURRENT_TROUGH).map(u16::from_le_bytes),
            duty_q15: take(TEL_BIT_DUTY).map(i16::from_le_bytes),
            vdiff: take(TEL_BIT_VDIFF).map(i16::from_le_bytes),
            vbus: take(TEL_BIT_VBUS).map(u16::from_le_bytes),
        });
    }
    Some(out)
}

/// Assembles one burst's Stream statuses, in arrival order, into ticked
/// frames: unwraps the u8 stream_seq (wraps every 256 frames = 4096
/// samples) and spreads each frame's samples over its 16-tick slot, so a
/// dropped or corrupt frame leaves a 16-tick hole rather than a time skew.
/// Bursts restart stream_seq at 0, so one assembler serves one burst.
pub struct StreamAssembler {
    mask: u16,
    last_seq: Option<u8>,
    frame_idx: u64,
    holes: u64,
    skipped: u64,
}

impl StreamAssembler {
    /// None if the mask has reserved bits set or selects nothing.
    pub fn new(mask: u16) -> Option<Self> {
        if mask & !TEL_MASK_ALL != 0 || mask == 0 {
            return None;
        }
        Some(Self {
            mask,
            last_seq: None,
            frame_idx: 0,
            holes: 0,
            skipped: 0,
        })
    }

    /// Consume one status: `stream` is true for a Stream-result frame
    /// (anything else is skipped and counted). Decoded samples append to
    /// `out` with correct ticks and per-sample window_valid.
    pub fn push(&mut self, stream: bool, payload: &[u8], out: &mut Vec<TelFrame>) {
        if !stream || payload.len() < STREAM_HDR {
            self.skipped += 1;
            return;
        }
        let seq = payload[0];
        let (idx, expected) = match self.last_seq {
            // bursts arm at seq 0: a nonzero first seq means missed frames
            None => (seq as u64, 0),
            Some(last) => (
                self.frame_idx + seq.wrapping_sub(last) as u64,
                self.frame_idx + 1,
            ),
        };
        self.holes += idx.saturating_sub(expected);
        match decode_stream_payload(self.mask, payload, idx * STREAM_SAMPLES_MAX as u64) {
            Some(frames) => out.extend(frames),
            None => self.skipped += 1,
        }
        self.last_seq = Some(seq);
        self.frame_idx = idx;
    }

    /// Frames missing from stream_seq continuity (dropped or corrupt).
    pub fn holes(&self) -> u64 {
        self.holes
    }

    /// Statuses that were not decodable stream frames.
    pub fn skipped(&self) -> u64 {
        self.skipped
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regs::telemetry as t;

    /// Mirror of the core tel.rs test vector generator: sample(i) with all
    /// six fields, window_valid on even i.
    fn sample_bytes(mask: u16, i: u16) -> Vec<u8> {
        let fields: [(u16, u16); 6] = [
            (TEL_BIT_POS, 0x1000 + i),
            (TEL_BIT_CURRENT, (-(i as i16) - 1) as u16),
            (TEL_BIT_CURRENT_TROUGH, 0xB000 + i),
            (TEL_BIT_DUTY, 0x2000 + i),
            (TEL_BIT_VDIFF, (-300 - i as i16) as u16),
            (TEL_BIT_VBUS, 1800 + i),
        ];
        let mut out = Vec::new();
        for (bit, v) in fields {
            if mask & bit != 0 {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
        out
    }

    fn stream_payload(mask: u16, seq: u8, last: bool, count: u16) -> Vec<u8> {
        let mut valid = 0u16;
        let mut p = vec![seq, if last { STREAM_FLAG_LAST } else { 0 }, 0, 0];
        for i in 0..count {
            if i < 16 && i.is_multiple_of(2) {
                valid |= 1 << i;
            }
            p.extend(sample_bytes(mask, i));
        }
        p[2..4].copy_from_slice(&valid.to_le_bytes());
        p
    }

    #[test]
    fn stream_golden_full_mask_full_batch() {
        // the exact bytes core's encode_golden_full_mask_full_batch pins
        let p = stream_payload(TEL_MASK_ALL, 0x42, false, 16);
        assert_eq!(p.len(), STREAM_HDR + 16 * 12);
        assert_eq!(p[..4], [0x42, 0x00, 0x55, 0x55]);
        assert_eq!(
            p[4..16],
            [
                0x00, 0x10, 0xFF, 0xFF, 0x00, 0xB0, 0x00, 0x20, 0xD4, 0xFE, 0x08, 0x07
            ]
        );
        assert_eq!(
            p[184..196],
            [
                0x0F, 0x10, 0xF0, 0xFF, 0x0F, 0xB0, 0x0F, 0x20, 0xC5, 0xFE, 0x17, 0x07
            ]
        );

        let frames = decode_stream_payload(TEL_MASK_ALL, &p, 320).expect("decodes");
        assert_eq!(frames.len(), 16);
        let f = frames[0];
        assert_eq!(f.tick, 320);
        assert!(f.window_valid);
        assert_eq!(f.pos, Some(0x1000));
        assert_eq!(f.current, Some(-1));
        assert_eq!(f.current_trough, Some(0xB000));
        assert_eq!(f.duty_q15, Some(0x2000));
        assert_eq!(f.vdiff, Some(-300));
        assert_eq!(f.vbus, Some(1800));
        let l = frames[15];
        assert_eq!(l.tick, 335);
        assert!(!l.window_valid);
        assert_eq!(l.pos, Some(0x100F));
        assert_eq!(l.vbus, Some(1815));
    }

    #[test]
    fn stream_golden_ladder_subset_leaves_unselected_none() {
        // core's encode_subset_mask_packs_in_bit_order vector: mask 0x1B
        let mask = 0x1B;
        let p = stream_payload(mask, 7, false, 2);
        assert_eq!(p[..4], [7, 0, 0x01, 0x00]);
        assert_eq!(p[4..12], [0x00, 0x10, 0xFF, 0xFF, 0x00, 0x20, 0xD4, 0xFE]);
        assert_eq!(p[12..20], [0x01, 0x10, 0xFE, 0xFF, 0x01, 0x20, 0xD3, 0xFE]);

        let frames = decode_stream_payload(mask, &p, 0).expect("decodes");
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].pos, Some(0x1000));
        assert_eq!(frames[0].current, Some(-1));
        assert_eq!(frames[0].duty_q15, Some(0x2000));
        assert_eq!(frames[0].vdiff, Some(-300));
        assert_eq!(frames[0].current_trough, None);
        assert_eq!(frames[0].vbus, None);
        assert!(frames[0].window_valid);
        assert!(!frames[1].window_valid);
        assert_eq!(frames[1].tick, 1);
    }

    #[test]
    fn decode_rejects_malformed_payloads() {
        assert!(decode_stream_payload(TEL_MASK_ALL, &[0, 0, 0], 0).is_none());
        // remainder not an integral sample count for the mask
        let mut p = stream_payload(0x1B, 0, false, 2);
        p.pop();
        assert!(decode_stream_payload(0x1B, &p, 0).is_none());
        // over 16 samples cannot come from one frame
        let p = stream_payload(TEL_BIT_POS, 0, false, 17);
        assert!(decode_stream_payload(TEL_BIT_POS, &p, 0).is_none());
    }

    #[test]
    fn assembler_ticks_run_through_the_seq_wrap() {
        // 258 frames of one sample each: seq wraps 255 -> 0 with no hole
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        for k in 0..258u64 {
            let p = stream_payload(TEL_BIT_POS, k as u8, k == 257, 1);
            a.push(true, &p, &mut out);
        }
        assert_eq!(out.len(), 258);
        assert_eq!(out[0].tick, 0);
        assert_eq!(out[255].tick, 255 * 16);
        assert_eq!(out[257].tick, 257 * 16);
        assert_eq!(a.holes(), 0);
        assert_eq!(a.skipped(), 0);
    }

    #[test]
    fn assembler_missing_frame_leaves_a_16_tick_hole() {
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        for seq in [0u8, 1, 3] {
            let p = stream_payload(TEL_BIT_POS, seq, seq == 3, 16);
            a.push(true, &p, &mut out);
        }
        assert_eq!(out.len(), 48);
        assert_eq!(out[31].tick, 31);
        assert_eq!(out[32].tick, 48, "frame 2 dropped: ticks jump 32 -> 48");
        assert_eq!(a.holes(), 1);
    }

    #[test]
    fn assembler_skips_non_stream_statuses() {
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        a.push(false, &[0x55, 0xAA], &mut out); // an OK ack, not a frame
        let p = stream_payload(TEL_BIT_POS, 0, true, 2);
        a.push(true, &p, &mut out);
        assert_eq!(out.len(), 2);
        assert_eq!(a.skipped(), 1);
        assert_eq!(a.holes(), 0);
    }

    #[test]
    fn assembler_rejects_bad_masks() {
        assert!(StreamAssembler::new(0).is_none());
        assert!(StreamAssembler::new(1 << 6).is_none());
        assert!(StreamAssembler::new(TEL_MASK_ALL).is_some());
    }

    #[test]
    fn agg_seq_unwrap_across_wrap() {
        let mut u = SeqUnwrap::default();
        assert_eq!(u.push(65534), 65534);
        assert_eq!(u.push(65535), 65535);
        assert_eq!(u.push(0), 65536);
        assert_eq!(u.push(1), 65537);
    }

    #[test]
    fn telemetry_snapshot_parses_a_synthetic_region() {
        let base = 0x0200u16;
        let mut bytes = vec![0u8; 0x60];
        let put = |b: &mut [u8], r: crate::regs::Reg, v: &[u8]| {
            let off = (r.addr - base) as usize;
            b[off..off + v.len()].copy_from_slice(v);
        };
        put(&mut bytes, t::FAULT_FLAGS, &[0x20]);
        put(&mut bytes, t::FAULT_CODE, &[6]);
        put(&mut bytes, t::THETA_HAT_Q16, &(2421i32 << 16).to_le_bytes());
        put(&mut bytes, t::TAU_D_COUNTS, &(-42i16).to_le_bytes());
        put(&mut bytes, t::VBUS_COUNTS, &1713u16.to_le_bytes());
        put(&mut bytes, t::POS, &2421u16.to_le_bytes());
        put(&mut bytes, t::I_MEAN_COUNTS, &(-33i16).to_le_bytes());
        put(&mut bytes, t::AGG_SEQ, &513u16.to_le_bytes());
        let s = TelemetrySnapshot::parse(base, &bytes).unwrap();
        assert_eq!(s.fault_flags, 0x20);
        assert_eq!(s.fault_code, 6);
        assert_eq!(s.theta_hat_q16, 2421 << 16);
        assert_eq!(s.tau_d_counts, -42);
        assert_eq!(s.vbus_counts, 1713);
        assert_eq!(s.pos, 2421);
        assert_eq!(s.i_mean_counts, -33);
        assert_eq!(s.agg_seq, 513);

        assert!(
            TelemetrySnapshot::parse(base, &bytes[..0x50]).is_none(),
            "short read"
        );
        assert!(
            TelemetrySnapshot::parse(0x0210, &bytes).is_none(),
            "base past a field"
        );
    }
}
