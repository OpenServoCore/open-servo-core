//! Byte-level parsers: the telemetry-region snapshot (bus gread) and the
//! TEL side-channel stream (USART2 at 3 Mbaud). Both mirror the firmware
//! layout - offsets come from [`crate::regs`], the TEL shape from the core
//! `tel` module - and are pinned by golden vectors, never imported.

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
// Mirror of firmware/lib/core/src/tel.rs; the golden test pins the bytes.

pub const TEL_SYNC: u8 = 0xA7;
pub const TEL_HDR_LEN: usize = 3;
pub const TEL_FLAG_WINDOW_VALID: u8 = 1 << 0;

pub const TEL_BIT_POS: u16 = 1 << 0;
pub const TEL_BIT_CURRENT: u16 = 1 << 1;
pub const TEL_BIT_CURRENT_TROUGH: u16 = 1 << 2;
pub const TEL_BIT_DUTY: u16 = 1 << 3;
pub const TEL_BIT_VDIFF: u16 = 1 << 4;
pub const TEL_BIT_VBUS: u16 = 1 << 5;
pub const TEL_MASK_ALL: u16 = 0x3F;

pub const fn tel_frame_len(mask: u16) -> usize {
    TEL_HDR_LEN + 2 * (mask & TEL_MASK_ALL).count_ones() as usize
}

/// One decoded TEL frame; unselected fields are None.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelFrame {
    pub seq: u8,
    pub window_valid: bool,
    pub pos: Option<u16>,
    pub current: Option<i16>,
    pub current_trough: Option<u16>,
    pub duty_q15: Option<i16>,
    pub vdiff: Option<i16>,
    pub vbus: Option<u16>,
}

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelStats {
    /// Frames decoded.
    pub frames: u64,
    /// Lock losses (sync mismatch at a frame boundary).
    pub realigns: u64,
    /// Frames missing from seq continuity - the firmware bumps seq per
    /// attempted frame, so a drop-on-busy surfaces here, not as corruption.
    pub seq_gaps: u64,
}

/// Consecutive sync + seq-continuous frames required to (re)acquire lock.
/// Bench-proven against the live 3 Mbaud stream; a mid-data 0xA7 cannot
/// sustain the predicate.
const LOCK_FRAMES: usize = 4;

/// Stateful deframer for one fixed mask: push raw serial bytes, collect
/// decoded frames. Never panics on garbage; realigns and continues.
pub struct TelDeframer {
    mask: u16,
    flen: usize,
    buf: Vec<u8>,
    cur: usize,
    locked: bool,
    last_seq: Option<u8>,
    stats: TelStats,
}

impl TelDeframer {
    /// None if the mask has reserved bits set or selects nothing.
    pub fn new(mask: u16) -> Option<Self> {
        if mask & !TEL_MASK_ALL != 0 || mask == 0 {
            return None;
        }
        Some(Self {
            mask,
            flen: tel_frame_len(mask),
            buf: Vec::new(),
            cur: 0,
            locked: false,
            last_seq: None,
            stats: TelStats::default(),
        })
    }

    pub fn stats(&self) -> TelStats {
        self.stats
    }

    pub fn push(&mut self, bytes: &[u8], out: &mut Vec<TelFrame>) {
        self.buf.extend_from_slice(bytes);
        loop {
            if !self.locked && !self.acquire() {
                break;
            }
            if self.buf.len() - self.cur < self.flen {
                break;
            }
            if self.buf[self.cur] != TEL_SYNC {
                self.locked = false;
                self.last_seq = None;
                self.stats.realigns += 1;
                continue;
            }
            let frame = self.decode_at(self.cur);
            if let Some(last) = self.last_seq {
                let d = frame.seq.wrapping_sub(last);
                if d != 1 {
                    self.stats.seq_gaps += d.wrapping_sub(1) as u64;
                }
            }
            self.last_seq = Some(frame.seq);
            self.stats.frames += 1;
            out.push(frame);
            self.cur += self.flen;
        }
        // compact the consumed prefix so the buffer stays bounded
        if self.cur > 0 {
            self.buf.drain(..self.cur);
            self.cur = 0;
        }
    }

    /// Slide until LOCK_FRAMES consecutive frames check out at the cursor.
    /// Keeps the tail short when no lock is found.
    fn acquire(&mut self) -> bool {
        let need = LOCK_FRAMES * self.flen;
        while self.cur + need <= self.buf.len() {
            let b = &self.buf[self.cur..];
            let synced = (0..LOCK_FRAMES).all(|k| b[k * self.flen] == TEL_SYNC);
            let continuous = (0..LOCK_FRAMES - 1)
                .all(|k| b[(k + 1) * self.flen + 1] == b[k * self.flen + 1].wrapping_add(1));
            if synced && continuous {
                self.locked = true;
                return true;
            }
            self.cur += 1;
        }
        let keep = need.saturating_sub(1);
        if self.buf.len() - self.cur > keep {
            let drop_to = self.buf.len() - keep;
            self.cur = self.cur.max(drop_to);
        }
        false
    }

    fn decode_at(&self, at: usize) -> TelFrame {
        let b = &self.buf[at..at + self.flen];
        let mut off = TEL_HDR_LEN;
        let mut take = |bit: u16| {
            if self.mask & bit != 0 {
                let v = [b[off], b[off + 1]];
                off += 2;
                Some(v)
            } else {
                None
            }
        };
        TelFrame {
            seq: b[1],
            window_valid: b[2] & TEL_FLAG_WINDOW_VALID != 0,
            pos: take(TEL_BIT_POS).map(u16::from_le_bytes),
            current: take(TEL_BIT_CURRENT).map(i16::from_le_bytes),
            current_trough: take(TEL_BIT_CURRENT_TROUGH).map(u16::from_le_bytes),
            duty_q15: take(TEL_BIT_DUTY).map(i16::from_le_bytes),
            vdiff: take(TEL_BIT_VDIFF).map(i16::from_le_bytes),
            vbus: take(TEL_BIT_VBUS).map(u16::from_le_bytes),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regs::telemetry as t;

    fn tel_encode(mask: u16, seq: u8, window_valid: bool, fields: &[(u16, u16)]) -> Vec<u8> {
        let mut f = vec![TEL_SYNC, seq, if window_valid { 1 } else { 0 }];
        for bit in 0..6 {
            let b = 1u16 << bit;
            if mask & b != 0 {
                let v = fields
                    .iter()
                    .find(|(fb, _)| *fb == b)
                    .map_or(0, |(_, v)| *v);
                f.extend_from_slice(&v.to_le_bytes());
            }
        }
        f
    }

    /// The core tel.rs golden vector: all fields, seq 0x42, window valid.
    fn golden_fields() -> Vec<(u16, u16)> {
        vec![
            (TEL_BIT_POS, 0x1234),
            (TEL_BIT_CURRENT, (-2i16) as u16),
            (TEL_BIT_CURRENT_TROUGH, 0xBEEF),
            (TEL_BIT_DUTY, 0x7FFF),
            (TEL_BIT_VDIFF, (-300i16) as u16),
            (TEL_BIT_VBUS, 1822),
        ]
    }

    #[test]
    fn golden_all_fields_frame_round_trips() {
        // pin the exact wire bytes first (mirrors the core encode test)
        let frame = tel_encode(TEL_MASK_ALL, 0x42, true, &golden_fields());
        let want = [
            0xA7, 0x42, 0x01, 0x34, 0x12, 0xFE, 0xFF, 0xEF, 0xBE, 0xFF, 0x7F, 0xD4, 0xFE, 0x1E,
            0x07,
        ];
        assert_eq!(frame, want);

        let mut d = TelDeframer::new(TEL_MASK_ALL).unwrap();
        let mut stream = Vec::new();
        for k in 0..5u8 {
            stream.extend(tel_encode(
                TEL_MASK_ALL,
                0x42u8.wrapping_add(k),
                true,
                &golden_fields(),
            ));
        }
        let mut out = Vec::new();
        d.push(&stream, &mut out);
        assert_eq!(out.len(), 5);
        let f = out[0];
        assert_eq!(f.seq, 0x42);
        assert!(f.window_valid);
        assert_eq!(f.pos, Some(0x1234));
        assert_eq!(f.current, Some(-2));
        assert_eq!(f.current_trough, Some(0xBEEF));
        assert_eq!(f.duty_q15, Some(0x7FFF));
        assert_eq!(f.vdiff, Some(-300));
        assert_eq!(f.vbus, Some(1822));
        assert_eq!(d.stats().seq_gaps, 0);
    }

    #[test]
    fn subset_mask_leaves_unselected_none() {
        let mask = TEL_BIT_POS | TEL_BIT_VDIFF;
        let mut d = TelDeframer::new(mask).unwrap();
        let mut stream = Vec::new();
        for k in 0..4u8 {
            stream.extend(tel_encode(
                mask,
                k,
                false,
                &[(TEL_BIT_POS, 2048), (TEL_BIT_VDIFF, 1709)],
            ));
        }
        let mut out = Vec::new();
        d.push(&stream, &mut out);
        assert_eq!(out.len(), 4);
        assert_eq!(out[0].pos, Some(2048));
        assert_eq!(out[0].vdiff, Some(1709));
        assert_eq!(out[0].current, None);
        assert_eq!(out[0].vbus, None);
        assert!(!out[0].window_valid);
    }

    #[test]
    fn lock_skips_garbage_prefix() {
        let mask = TEL_BIT_POS;
        let mut d = TelDeframer::new(mask).unwrap();
        let mut stream = vec![0x00, 0xA7, 0x13, 0xFF]; // garbage incl. a stray sync
        for k in 10..16u8 {
            stream.extend(tel_encode(mask, k, false, &[(TEL_BIT_POS, 100)]));
        }
        let mut out = Vec::new();
        d.push(&stream, &mut out);
        assert_eq!(out.len(), 6);
        assert_eq!(out[0].seq, 10);
        assert_eq!(d.stats().seq_gaps, 0);
    }

    #[test]
    fn realign_on_corrupt_byte_recovers() {
        let mask = TEL_BIT_POS;
        let flen = tel_frame_len(mask);
        let mut stream = Vec::new();
        for k in 0..20u8 {
            stream.extend(tel_encode(mask, k, false, &[(TEL_BIT_POS, 300)]));
        }
        stream[8 * flen] = 0x55; // corrupt frame 8's sync
        let mut d = TelDeframer::new(mask).unwrap();
        let mut out = Vec::new();
        d.push(&stream, &mut out);
        assert_eq!(d.stats().realigns, 1);
        // 8 clean before the corruption, relock consumes nothing extra
        assert!(out.len() >= 8 + 8, "recovered after realign: {}", out.len());
        assert_eq!(out.last().unwrap().seq, 19);
    }

    #[test]
    fn seq_gap_counts_dropped_frames() {
        let mask = TEL_BIT_POS;
        let mut stream = Vec::new();
        for k in [0u8, 1, 2, 3, 4, 6, 7, 10, 11, 12] {
            stream.extend(tel_encode(mask, k, false, &[(TEL_BIT_POS, 7)]));
        }
        let mut d = TelDeframer::new(mask).unwrap();
        let mut out = Vec::new();
        d.push(&stream, &mut out);
        assert_eq!(out.len(), 10);
        assert_eq!(d.stats().seq_gaps, 3, "5 and 8, 9 dropped");
        assert_eq!(d.stats().realigns, 0);
    }

    #[test]
    fn deframer_rejects_bad_masks() {
        assert!(TelDeframer::new(0).is_none());
        assert!(TelDeframer::new(1 << 6).is_none());
        assert!(TelDeframer::new(TEL_MASK_ALL).is_some());
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
