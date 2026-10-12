//! Byte-level parsers: the telemetry-region snapshot (bus gread) and the
//! TEL burst stream (`Stream`-result status frames on the main bus). The
//! snapshot mirrors the firmware layout through [`crate::regs`]; TEL rows
//! decode through the protocol's own `tel` module, pinned against the
//! core encoder.

use osc_protocol::tel;

use crate::regs::{Reg, telemetry};

/// One decoded read of the telemetry region. `base` is the gread start
/// address; the slice must cover every field below or parse returns None.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct TelemetrySnapshot {
    /// When the region was read, ms on the host's clock: the driver stamps
    /// it, parse leaves it 0. Every read stalls the servo's kernel for a
    /// few ticks, so under polling `sample_tick` and `agg_seq` run slow and
    /// this is the clock a fit times the shaft on.
    pub host_ms: f64,
    pub fault_flags: u8,
    pub status_flags: u8,
    pub mode_active: u8,
    pub fault_code: u8,
    pub omega_hat_src: u8,
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
    pub limit_flags: u8,
    pub window_floor_q15: u16,
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
            host_ms: 0.0,
            fault_flags: u8::from_le_bytes(get(base, bytes, t::FAULT_FLAGS)?),
            status_flags: u8::from_le_bytes(get(base, bytes, t::STATUS_FLAGS)?),
            mode_active: u8::from_le_bytes(get(base, bytes, t::MODE_ACTIVE)?),
            fault_code: u8::from_le_bytes(get(base, bytes, t::FAULT_CODE)?),
            omega_hat_src: u8::from_le_bytes(get(base, bytes, t::OMEGA_HAT_SRC)?),
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
            limit_flags: u8::from_le_bytes(get(base, bytes, t::LIMIT_FLAGS)?),
            window_floor_q15: u16::from_le_bytes(get(base, bytes, t::WINDOW_FLOOR_Q15)?),
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
// One Stream-result status payload (protocol sec 5.6), split by
// `osc_protocol::tel::StreamFrame`: a head (stream_seq, flags, the first
// row's sample_tick), the rows (the count from LEN), a trailer (valid
// bitmap, fault OR). stream_seq restarts at 0 per arm.

pub use osc_servo_core::tel::duty_counts;

pub const TEL_BIT_POS: u16 = tel::BIT_POS;
pub const TEL_BIT_CURRENT: u16 = tel::BIT_CURRENT;
pub const TEL_BIT_CURRENT_TROUGH: u16 = tel::BIT_CURRENT_TROUGH;
pub const TEL_BIT_DUTY: u16 = tel::BIT_DUTY;
pub const TEL_BIT_VDIFF: u16 = tel::BIT_VDIFF;
pub const TEL_BIT_VBUS: u16 = tel::BIT_VBUS;
pub const TEL_BIT_CURRENT_RAW: u16 = tel::BIT_CURRENT_RAW;
pub const TEL_BIT_VMOTOR_A: u16 = tel::BIT_VMOTOR_A;
pub const TEL_BIT_VMOTOR_B: u16 = tel::BIT_VMOTOR_B;
pub const TEL_BIT_VBUS_RAW: u16 = tel::BIT_VBUS_RAW;
pub const TEL_BIT_NTC_RAW: u16 = tel::BIT_NTC_RAW;
/// The linearized pot the kernel controls on, the Q4 word itself.
pub const TEL_BIT_POS_LIN: u16 = tel::BIT_POS_LIN;
pub const TEL_MASK_ALL: u16 = tel::MASK_ALL;

/// The raw-capture default: the per-tick ADC frame set plus applied duty.
pub const TEL_MASK_RAW: u16 = TEL_BIT_POS
    | TEL_BIT_CURRENT_RAW
    | TEL_BIT_CURRENT_TROUGH
    | TEL_BIT_DUTY
    | TEL_BIT_VMOTOR_A
    | TEL_BIT_VMOTOR_B;

/// One decoded TEL sample; unselected fields are None. `tick` is the
/// fast-tick index within one burst, from the frames' stamps with the
/// first frame's row 0 at 0, so time = tick / tick_hz and a lost frame or
/// a row the servo dropped shows as a jump.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelFrame {
    pub tick: u64,
    pub window_valid: bool,
    pub pos: Option<u16>,
    pub current: Option<i16>,
    pub current_trough: Option<u16>,
    /// Signed PWM compare counts the bridge applied, logical direction;
    /// full scale is the servo's `pwm_period_ticks` ([`TelBurst`]).
    pub duty_counts: Option<i16>,
    pub vdiff: Option<i16>,
    pub vbus: Option<u16>,
    pub current_raw: Option<u16>,
    pub vmotor_a: Option<u16>,
    pub vmotor_b: Option<u16>,
    pub vbus_raw: Option<u16>,
    pub ntc_raw: Option<u16>,
    /// Linearized pot, Q4 (counts x 16): `interp_q4(pos)` over the live
    /// table, `pos << 4` at the identity.
    pub pos_lin: Option<u16>,
}

impl TelFrame {
    /// The counts the kernel controls on: `pos_lin` over the grid when the
    /// stream carried it, else raw `pos`; the two agree at the identity.
    pub fn counts(&self) -> Option<f64> {
        match self.pos_lin {
            Some(q4) => Some(q4 as f64 / crate::lut::GRID as f64),
            None => self.pos.map(f64::from),
        }
    }

    /// The row applied `goal_q15` at `pwm_period_ticks`: its duty is the
    /// goal's own compare count.
    pub fn duty_is(&self, goal_q15: i16, pwm_period_ticks: u16) -> bool {
        self.duty_counts == Some(duty_counts(goal_q15, pwm_period_ticks))
    }
}

/// A duty in compare counts as Q15 of `pwm_period_ticks` (32767 = full
/// drive); 0 for a zero period.
pub fn counts_q15(counts: i16, pwm_period_ticks: u16) -> f64 {
    if pwm_period_ticks == 0 {
        return 0.0;
    }
    counts as f64 * i16::MAX as f64 / pwm_period_ticks as f64
}

/// One TEL burst as decoded: its frames, the rows the servo dropped from
/// it, which the frames are missing, and the PWM period its duty counts
/// are of.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TelBurst {
    pub frames: Vec<TelFrame>,
    pub rows_dropped: u16,
    pub pwm_period_ticks: u16,
}

/// Decode one stream payload into per-tick frames; row i lands at
/// `tick_base + i`. None when the payload cannot be a `mask` stream frame
/// ([`tel::StreamFrame::parse`]).
pub fn decode_stream_payload(mask: u16, payload: &[u8], tick_base: u64) -> Option<Vec<TelFrame>> {
    decode_rows(mask, &tel::StreamFrame::parse(mask, payload)?, tick_base)
}

fn decode_rows(mask: u16, f: &tel::StreamFrame<'_>, tick_base: u64) -> Option<Vec<TelFrame>> {
    let mut out = Vec::with_capacity(f.len());
    for (i, row) in f.rows().enumerate() {
        let w = tel::decode_row(mask, row)?;
        let take = |bit: u16| (mask & bit != 0).then(|| w[bit.trailing_zeros() as usize]);
        out.push(TelFrame {
            tick: tick_base + i as u64,
            window_valid: f.valid & (1 << i) != 0,
            pos: take(TEL_BIT_POS),
            current: take(TEL_BIT_CURRENT).map(|v| v as i16),
            current_trough: take(TEL_BIT_CURRENT_TROUGH),
            duty_counts: take(TEL_BIT_DUTY).map(|v| v as i16),
            vdiff: take(TEL_BIT_VDIFF).map(|v| v as i16),
            vbus: take(TEL_BIT_VBUS),
            current_raw: take(TEL_BIT_CURRENT_RAW),
            vmotor_a: take(TEL_BIT_VMOTOR_A),
            vmotor_b: take(TEL_BIT_VMOTOR_B),
            vbus_raw: take(TEL_BIT_VBUS_RAW),
            ntc_raw: take(TEL_BIT_NTC_RAW),
            pos_lin: take(TEL_BIT_POS_LIN),
        });
    }
    Some(out)
}

/// Assembles one burst's Stream statuses, in arrival order, into ticked
/// frames. Each frame's rows tick on from its stamp, unwrapped against the
/// previous frame's (a u16 wraps every 3.3 s at 20 kHz, far past any gap
/// inside a burst), so a lost frame or dropped rows leave a tick hole
/// rather than a time skew. The u8 stream_seq counts lost frames.
/// Bursts restart stream_seq at 0, so one assembler serves one burst.
pub struct StreamAssembler {
    mask: u16,
    last_seq: Option<u8>,
    frame_idx: u64,
    /// The previous frame's stamp and the tick it unwrapped to.
    stamp: Option<(u16, u64)>,
    holes: u64,
    skipped: u64,
}

impl StreamAssembler {
    /// None if the mask has reserved bits set or selects nothing. Whether
    /// a row fits the line is the servo's call (`tel_row_bytes_max`).
    pub fn new(mask: u16) -> Option<Self> {
        if mask & !TEL_MASK_ALL != 0 || mask == 0 {
            return None;
        }
        Some(Self {
            mask,
            last_seq: None,
            frame_idx: 0,
            stamp: None,
            holes: 0,
            skipped: 0,
        })
    }

    /// Consume one status: `stream` is true for a Stream-result frame
    /// (anything else is skipped and counted). Decoded samples append to
    /// `out` with their ticks and per-sample window_valid.
    pub fn push(&mut self, stream: bool, payload: &[u8], out: &mut Vec<TelFrame>) {
        let Some(f) = stream
            .then(|| tel::StreamFrame::parse(self.mask, payload))
            .flatten()
        else {
            self.skipped += 1;
            return;
        };
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
        match decode_rows(self.mask, &f, tick) {
            Some(frames) => out.extend(frames),
            None => self.skipped += 1,
        }
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

    use osc_protocol::wire::STREAM_FLAG_LAST;
    use osc_servo_core::tel::{ROW_BYTES_MAX, STREAM_PAYLOAD_MAX, TelSample, encode_stream};

    /// The ident and tool masks: raw capture, R/L, ladder, bias, each with
    /// the linearized pot a live table adds.
    const MASKS: [u16; 8] = [
        TEL_MASK_RAW,
        TEL_MASK_RAW | TEL_BIT_POS_LIN,
        crate::exp::rl::TEL_RL_MASK,
        crate::exp::rl::TEL_RL_MASK | TEL_BIT_POS_LIN,
        crate::exp::inertia::TEL_LADDER_MASK,
        crate::exp::inertia::TEL_LADDER_MASK | TEL_BIT_POS_LIN,
        TEL_BIT_POS,
        TEL_BIT_POS | TEL_BIT_POS_LIN,
    ];

    /// Every field distinct and in range, duty negative on odd rows.
    fn sample(i: u16) -> TelSample {
        TelSample {
            pos: 0x100 + i,
            current: -(i as i16) - 1,
            current_trough: 0xB00 + i,
            duty: if i.is_multiple_of(2) {
                293
            } else {
                -1200 + i as i16
            },
            vdiff: -300 - i as i16,
            vbus: 1800 + i,
            current_raw: 0x0100 + i,
            vmotor_a: 0x0A00 + i,
            vmotor_b: 0x0B00 + i,
            vbus_raw: 0x0C00 + i,
            ntc_raw: 0x0D00 + i,
            pos_lin_q4: 0xE000 + i,
            tick: 0,
            window_valid: i.is_multiple_of(2),
            fault: false,
        }
    }

    /// The firmware encoder's payload for `count` rows of `sample`, the
    /// first stamped `tick`.
    fn stream_payload(mask: u16, seq: u8, last: bool, tick: u16, count: u16) -> Vec<u8> {
        let rows: Vec<TelSample> = (0..count)
            .map(|i| TelSample {
                tick: tick.wrapping_add(i),
                ..sample(i)
            })
            .collect();
        let mut buf = [0u8; STREAM_PAYLOAD_MAX];
        let n = encode_stream(mask, seq, last, &rows, &mut buf);
        buf[..n].to_vec()
    }

    /// `sample(i)` as the decoder should see it under `mask`.
    fn want(mask: u16, tick: u64, i: u16) -> TelFrame {
        let s = sample(i);
        let sel = |bit: u16| mask & bit != 0;
        TelFrame {
            tick,
            window_valid: s.window_valid,
            pos: sel(TEL_BIT_POS).then_some(s.pos),
            current: sel(TEL_BIT_CURRENT).then_some(s.current),
            current_trough: sel(TEL_BIT_CURRENT_TROUGH).then_some(s.current_trough),
            duty_counts: sel(TEL_BIT_DUTY).then_some(s.duty),
            vdiff: sel(TEL_BIT_VDIFF).then_some(s.vdiff),
            vbus: sel(TEL_BIT_VBUS).then_some(s.vbus),
            current_raw: sel(TEL_BIT_CURRENT_RAW).then_some(s.current_raw),
            vmotor_a: sel(TEL_BIT_VMOTOR_A).then_some(s.vmotor_a),
            vmotor_b: sel(TEL_BIT_VMOTOR_B).then_some(s.vmotor_b),
            vbus_raw: sel(TEL_BIT_VBUS_RAW).then_some(s.vbus_raw),
            ntc_raw: sel(TEL_BIT_NTC_RAW).then_some(s.ntc_raw),
            pos_lin: sel(TEL_BIT_POS_LIN).then_some(s.pos_lin_q4),
        }
    }

    #[test]
    fn every_tool_mask_round_trips_the_firmware_encoder() {
        for mask in MASKS {
            let p = stream_payload(mask, 3, false, 0, 16);
            assert_eq!(p.len(), tel::frame_len(mask, 16), "{mask:#x}");
            let frames = decode_stream_payload(mask, &p, 48).expect("decodes");
            for (i, f) in frames.iter().enumerate() {
                assert_eq!(*f, want(mask, 48 + i as u64, i as u16), "{mask:#x} row {i}");
            }
        }
    }

    /// Every mask the servo can stream decodes field for field.
    #[test]
    fn every_encodable_mask_round_trips() {
        let mut n = 0;
        for mask in 1..=TEL_MASK_ALL {
            let p = stream_payload(mask, 0, true, 0, 2);
            if p.is_empty() {
                continue;
            }
            n += 1;
            let frames = decode_stream_payload(mask, &p, 0).expect("decodes");
            assert_eq!(frames, [want(mask, 0, 0), want(mask, 1, 1)], "{mask:#x}");
        }
        let admitted = (1..=TEL_MASK_ALL).filter(|&m| tel::row_bytes(m) <= ROW_BYTES_MAX);
        assert_eq!(n, admitted.count());
    }

    #[test]
    fn stream_golden_six_raw_fields() {
        // the bytes core's encode_golden_six_raw_fields_full_batch pins for
        // its own sample(0): pos 0x100, trough 0xB00, duty -1200, raw 0x100,
        // va 0xA00, vb 0xB00
        let row = [0x00, 0x01, 0xB0, 0x50, 0x0B, 0x10, 0x00, 0x0A, 0xB0];
        let mut p = vec![0x42, 0x00, 0x30, 0x12];
        p.extend(row);
        p.extend([0x01, 0x00, 0x00, 0x00, 0x00, 0x00]);
        let f = decode_stream_payload(TEL_MASK_RAW, &p, 320).expect("decodes")[0];
        assert_eq!(f.tick, 320);
        assert!(f.window_valid);
        assert_eq!(f.pos, Some(0x100));
        assert_eq!(f.current_trough, Some(0xB00));
        assert_eq!(f.duty_counts, Some(-1200));
        assert_eq!(f.current_raw, Some(0x100));
        assert_eq!((f.vmotor_a, f.vmotor_b), (Some(0xA00), Some(0xB00)));
        assert_eq!(
            (f.current, f.vdiff, f.vbus, f.pos_lin),
            (None, None, None, None)
        );
    }

    #[test]
    fn a_goal_reads_as_its_compare_count() {
        let f = TelFrame {
            duty_counts: Some(-293),
            ..TelFrame::default()
        };
        assert!(f.duty_is(-8000, 1200));
        assert!(!f.duty_is(8000, 1200));
        // 8520 does not survive a counts round trip: the goal is quantized,
        // never the row
        assert_eq!(duty_counts(8520, 1200), 312);
        assert_ne!(counts_q15(312, 1200).round() as i16, 8520);
        assert_eq!(counts_q15(1200, 1200), 32767.0);
        assert_eq!(counts_q15(5, 0), 0.0);
    }

    #[test]
    fn decode_rejects_malformed_payloads() {
        assert!(decode_stream_payload(TEL_MASK_ALL, &[0, 0, 0], 0).is_none());
        // remainder not an integral row count for the mask
        let mut p = stream_payload(0x1B, 0, false, 0, 2);
        p.pop();
        assert!(decode_stream_payload(0x1B, &p, 0).is_none());
        // more rows than the trailer's bitmap marks cannot be one frame
        let rows = tel::rows_max(TEL_BIT_POS) + 1;
        let p = vec![0; tel::frame_len(TEL_BIT_POS, rows)];
        assert!(decode_stream_payload(TEL_BIT_POS, &p, 0).is_none());
    }

    /// A host takes the row count from LEN, never 16: a 24-row frame and a
    /// 5-row frame run on tick for tick.
    #[test]
    fn assembler_takes_any_row_count_from_the_length() {
        let frame = |seq: u8, last: bool, tick: u16, rows: u16| {
            let mut p = vec![seq, if last { STREAM_FLAG_LAST } else { 0 }];
            p.extend(tick.to_le_bytes());
            for i in 0..rows {
                p.extend((tick + i).to_le_bytes());
            }
            p.extend([0xFF, 0xFF, 0xFF, 0x00, 0, 0]);
            p
        };
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        a.push(true, &frame(0, false, 100, 24), &mut out);
        a.push(true, &frame(1, true, 124, 5), &mut out);
        assert_eq!(out.len(), 29);
        for (k, f) in out.iter().enumerate() {
            assert_eq!((f.tick, f.pos), (k as u64, Some(100 + k as u16)));
            assert!(f.window_valid);
        }
        assert_eq!((a.holes(), a.skipped()), (0, 0));
    }

    #[test]
    fn assembler_ticks_run_through_the_seq_and_stamp_wraps() {
        // 258 one-row frames 16 ticks apart: seq wraps 255 -> 0 with no
        // hole, the stamp wraps past 0xFFFF on the way
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        for k in 0..258u64 {
            let tick = 0xF000u16.wrapping_add(16 * k as u16);
            let p = stream_payload(TEL_BIT_POS, k as u8, k == 257, tick, 1);
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
    fn assembler_missing_frame_leaves_a_seq_hole_and_a_tick_jump() {
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        for seq in [0u8, 1, 3] {
            let p = stream_payload(TEL_BIT_POS, seq, seq == 3, 16 * seq as u16, 16);
            a.push(true, &p, &mut out);
        }
        assert_eq!(out.len(), 48);
        assert_eq!(out[31].tick, 31);
        assert_eq!(out[32].tick, 48, "frame 2 dropped: ticks jump 32 -> 48");
        assert_eq!(a.holes(), 1);
    }

    /// Rows the servo dropped against full buffers leave seq whole: only
    /// the stamp tells.
    #[test]
    fn assembler_dropped_rows_jump_the_ticks_not_the_seq() {
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        a.push(
            true,
            &stream_payload(TEL_BIT_POS, 0, false, 7, 16),
            &mut out,
        );
        a.push(
            true,
            &stream_payload(TEL_BIT_POS, 1, true, 7 + 20, 16),
            &mut out,
        );
        assert_eq!(out.len(), 32);
        assert_eq!((out[15].tick, out[16].tick), (15, 20));
        assert_eq!(a.holes(), 0);
    }

    #[test]
    fn assembler_skips_non_stream_statuses() {
        let mut a = StreamAssembler::new(TEL_BIT_POS).unwrap();
        let mut out = Vec::new();
        a.push(false, &[0x55, 0xAA], &mut out); // an OK ack, not a frame
        let p = stream_payload(TEL_BIT_POS, 0, true, 0, 2);
        a.push(true, &p, &mut out);
        assert_eq!(out.len(), 2);
        assert_eq!(a.skipped(), 1);
        assert_eq!(a.holes(), 0);
    }

    #[test]
    fn assembler_rejects_bad_masks() {
        assert!(StreamAssembler::new(0).is_none());
        assert!(StreamAssembler::new(1 << 12).is_none());
        for mask in MASKS {
            assert!(StreamAssembler::new(mask).is_some());
        }
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
        let mut bytes = vec![0u8; 0x6a];
        let put = |b: &mut [u8], r: crate::regs::Reg, v: &[u8]| {
            let off = (r.addr - base) as usize;
            b[off..off + v.len()].copy_from_slice(v);
        };
        put(&mut bytes, t::FAULT_FLAGS, &[0x20]);
        put(&mut bytes, t::FAULT_CODE, &[6]);
        put(&mut bytes, t::OMEGA_HAT_SRC, &[1]);
        put(&mut bytes, t::THETA_HAT_Q16, &(2421i32 << 16).to_le_bytes());
        put(&mut bytes, t::TAU_D_COUNTS, &(-42i16).to_le_bytes());
        put(&mut bytes, t::VBUS_COUNTS, &1713u16.to_le_bytes());
        put(&mut bytes, t::POS, &2421u16.to_le_bytes());
        put(&mut bytes, t::I_MEAN_COUNTS, &(-33i16).to_le_bytes());
        put(&mut bytes, t::AGG_SEQ, &513u16.to_le_bytes());
        put(&mut bytes, t::LIMIT_FLAGS, &[0x09]);
        put(&mut bytes, t::WINDOW_FLOOR_Q15, &4356u16.to_le_bytes());
        let s = TelemetrySnapshot::parse(base, &bytes).unwrap();
        assert_eq!(s.fault_flags, 0x20);
        assert_eq!(s.fault_code, 6);
        assert_eq!(s.omega_hat_src, 1);
        assert_eq!(s.theta_hat_q16, 2421 << 16);
        assert_eq!(s.tau_d_counts, -42);
        assert_eq!(s.vbus_counts, 1713);
        assert_eq!(s.pos, 2421);
        assert_eq!(s.i_mean_counts, -33);
        assert_eq!(s.agg_seq, 513);
        assert_eq!(s.limit_flags, 0x09);
        assert_eq!(s.window_floor_q15, 4356);

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
