//! Byte-level parsers: the telemetry-region snapshot (bus gread) and the
//! TEL burst stream (`Stream`-result status frames on the main bus). Both
//! mirror the firmware's table layout through [`crate::regs`]: a TEL row is
//! table bytes at their own addresses ([`tel::Chunks`]).

use osc_protocol::tel::{self, Chunk, Chunks};

use crate::regs::{Reg, telemetry};

/// One decoded read of the telemetry region. `base` is the gread start
/// address; the slice must cover every region field below or parse returns
/// None. The window fields are the host's own, filled by
/// [`crate::window::Windows`]: parse leaves them 0.
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
    pub omega_bemf_cps: i16,
    pub r_hat_q12: u16,
    /// The tick block's signed held current, the kernel's per-tick reading.
    pub i_tick_counts: i16,
    /// Signed PWM compare counts the bridge applied; full scale is CALIB
    /// `pwm_period_ticks`.
    pub duty_ticks: i16,
    pub sample_tick: u32,
    pub pos: u16,
    pub current_raw: u16,
    pub current_trough: u16,
    pub current_bias_counts: u16,
    pub limit_flags: u8,
    pub window_floor_q15: u16,
    /// The window streamed just before the read ([`crate::window`]): the
    /// current's mean and extremes, counts, and the drive-window `va - vb`
    /// mean.
    pub i_mean_counts: i16,
    pub i_min_counts: i16,
    pub i_max_counts: i16,
    pub vdiff_mean: i16,
    /// The window's applied duty mean, compare counts of
    /// `pwm_period_ticks`.
    pub duty_mean_counts: i16,
    pub pwm_period_ticks: u16,
    /// The window's number: `sample_tick` in windows at the read that
    /// carried it, repeated by a read that formed none.
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
            omega_bemf_cps: i16::from_le_bytes(get(base, bytes, t::OMEGA_BEMF_CPS)?),
            r_hat_q12: u16::from_le_bytes(get(base, bytes, t::R_HAT_Q12)?),
            i_tick_counts: i16::from_le_bytes(get(base, bytes, t::I_TICK_COUNTS)?),
            duty_ticks: i16::from_le_bytes(get(base, bytes, t::DUTY_TICKS)?),
            sample_tick: u32::from_le_bytes(get(base, bytes, t::SAMPLE_TICK)?),
            pos: u16::from_le_bytes(get(base, bytes, t::POS)?),
            current_raw: u16::from_le_bytes(get(base, bytes, t::CURRENT_RAW)?),
            current_trough: u16::from_le_bytes(get(base, bytes, t::CURRENT_TROUGH)?),
            current_bias_counts: u16::from_le_bytes(get(base, bytes, t::CURRENT_BIAS_COUNTS)?),
            limit_flags: u8::from_le_bytes(get(base, bytes, t::LIMIT_FLAGS)?),
            window_floor_q15: u16::from_le_bytes(get(base, bytes, t::WINDOW_FLOOR_Q15)?),
            ..Default::default()
        })
    }

    /// The window applied `goal_q15`: its duty mean is the goal's compare
    /// count.
    pub fn duty_is(&self, goal_q15: i16) -> bool {
        self.duty_mean_counts == duty_counts(goal_q15, self.pwm_period_ticks)
    }

    /// The window's duty mean as Q15 of the PWM period.
    pub fn duty_mean_q15(&self) -> f64 {
        counts_q15(self.duty_mean_counts, self.pwm_period_ticks)
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
// row's sample_tick), then the rows, each the chunks' table bytes back to
// back (the count from LEN). stream_seq restarts at 0 per arm.

pub use osc_servo_core::estimator::window::duty_counts;
pub use osc_servo_core::regions::telemetry::tick_flag;

/// The chunk covering `first..=last`, table order.
const fn span(first: Reg, last: Reg) -> Chunk {
    Chunk::new(first.addr, last.addr + last.width as u16 - first.addr)
}

const fn one(first: Reg, last: Reg) -> Chunks {
    let c = span(first, last);
    Chunks::one(c.addr, c.len)
}

/// The raw capture: current_raw, vmotor_a, vmotor_b, pos, then duty and
/// current_trough.
pub const TEL_RAW: Chunks = Chunks::two(
    span(telemetry::CURRENT_RAW, telemetry::POS),
    span(telemetry::DUTY_TICKS, telemetry::CURRENT_TROUGH),
);

/// The ladder and step captures: pos, then duty, the held current and
/// vdiff (current_trough riding along).
pub const TEL_LADDER: Chunks = Chunks::two(
    span(telemetry::POS, telemetry::POS),
    span(telemetry::DUTY_TICKS, telemetry::VDIFF_COUNTS),
);

/// The R/L capture: current_raw, vmotor_a, vmotor_b, then vbus_raw and
/// duty (ntc_raw riding along).
pub const TEL_RL: Chunks = Chunks::two(
    span(telemetry::CURRENT_RAW, telemetry::VMOTOR_B),
    span(telemetry::VBUS_RAW, telemetry::DUTY_TICKS),
);

/// The pot alone.
pub const TEL_POS: Chunks = one(telemetry::POS, telemetry::POS);

/// A row's chunks as `addr+len` joined by `,`: `0x232+6,0x23c+6`.
pub fn chunks_text(chunks: &Chunks) -> String {
    let parts: Vec<String> = chunks
        .0
        .iter()
        .filter(|c| c.len != 0)
        .map(|c| format!("{:#x}+{}", c.addr, c.len))
        .collect();
    parts.join(",")
}

/// The shunt read the drive window `duty_ticks` drove: the firmware's own
/// test (`window::select`) against CALIB `i_window_min_ticks`.
pub fn window_valid(duty_ticks: i16, i_window_min_ticks: u16) -> bool {
    duty_ticks != 0 && duty_ticks.unsigned_abs() >= i_window_min_ticks
}

/// One decoded TEL sample; fields the row does not carry are None. `tick`
/// is the fast-tick index within one burst, from the frames' stamps with
/// the first frame's row 0 at 0, so time = tick / tick_hz and a lost frame
/// or a row the servo dropped shows as a jump.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct TelFrame {
    pub tick: u64,
    /// The shunt read this tick's drive window: the row's `tick_flags`
    /// I_VALID, else [`window_valid`] on its duty; false with neither.
    pub window_valid: bool,
    pub pos: Option<u16>,
    /// `i_tick_counts`: the signed held current.
    pub current: Option<i16>,
    pub current_trough: Option<u16>,
    /// Signed PWM compare counts the bridge applied, logical direction;
    /// full scale is the servo's `pwm_period_ticks` ([`TelBurst`]).
    pub duty_counts: Option<i16>,
    /// `vdiff_counts`.
    pub vdiff: Option<i16>,
    /// `vbus_counts`.
    pub vbus: Option<u16>,
    pub current_raw: Option<u16>,
    pub vmotor_a: Option<u16>,
    pub vmotor_b: Option<u16>,
    pub vbus_raw: Option<u16>,
    pub ntc_raw: Option<u16>,
}

impl TelFrame {
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

/// One row of `chunks` at fast tick `tick`; `i_window_min_ticks` is the
/// servo's CALIB floor [`window_valid`] judges a row without `tick_flags`
/// by. A row shorter than the chunks reads as fields absent.
pub fn decode_row(chunks: &Chunks, i_window_min_ticks: u16, row: &[u8], tick: u64) -> TelFrame {
    use telemetry as t;
    let take = |r: Reg| {
        let at = chunks.offset(r.addr, r.width as u16)?;
        let b = row.get(at..at + 2)?;
        Some(u16::from_le_bytes([b[0], b[1]]))
    };
    let duty_counts = take(t::DUTY_TICKS).map(|v| v as i16);
    let valid = match (take(t::TICK_FLAGS), duty_counts) {
        (Some(flags), _) => flags & tick_flag::I_VALID != 0,
        (None, Some(d)) => window_valid(d, i_window_min_ticks),
        (None, None) => false,
    };
    TelFrame {
        tick,
        window_valid: valid,
        pos: take(t::POS),
        current: take(t::I_TICK_COUNTS).map(|v| v as i16),
        current_trough: take(t::CURRENT_TROUGH),
        duty_counts,
        vdiff: take(t::VDIFF_COUNTS).map(|v| v as i16),
        vbus: take(t::VBUS_COUNTS),
        current_raw: take(t::CURRENT_RAW),
        vmotor_a: take(t::VMOTOR_A),
        vmotor_b: take(t::VMOTOR_B),
        vbus_raw: take(t::VBUS_RAW),
        ntc_raw: take(t::NTC_RAW),
    }
}

/// Assembles one burst's Stream statuses, in arrival order, into ticked
/// frames. Each frame's rows tick on from its stamp, unwrapped against the
/// previous frame's (a u16 wraps every 3.3 s at 20 kHz, far past any gap
/// inside a burst), so a lost frame or dropped rows leave a tick hole
/// rather than a time skew. The u8 stream_seq counts lost frames.
/// Bursts restart stream_seq at 0, so one assembler serves one burst.
pub struct StreamAssembler {
    chunks: Chunks,
    i_window_min_ticks: u16,
    last_seq: Option<u8>,
    frame_idx: u64,
    /// The previous frame's stamp and the tick it unwrapped to.
    stamp: Option<(u16, u64)>,
    holes: u64,
    skipped: u64,
    alert: bool,
}

impl StreamAssembler {
    /// None unless the servo could arm `chunks` ([`Chunks::valid`]).
    /// `i_window_min_ticks` is the servo's CALIB floor ([`decode_row`]).
    pub fn new(chunks: Chunks, i_window_min_ticks: u16) -> Option<Self> {
        chunks.valid().then_some(Self {
            chunks,
            i_window_min_ticks,
            last_seq: None,
            frame_idx: 0,
            stamp: None,
            holes: 0,
            skipped: 0,
            alert: false,
        })
    }

    /// Consume one status: `stream` is true for a Stream-result frame
    /// (anything else is skipped and counted), `alert` its INST ALERT bit.
    /// Decoded samples append to `out` with their ticks.
    pub fn push(&mut self, stream: bool, alert: bool, payload: &[u8], out: &mut Vec<TelFrame>) {
        let Some(f) = stream
            .then(|| tel::StreamFrame::parse(self.chunks.row_bytes(), payload))
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
        self.alert |= alert;
        out.extend(f.rows().enumerate().map(|(i, row)| {
            decode_row(&self.chunks, self.i_window_min_ticks, row, tick + i as u64)
        }));
    }

    /// Frames missing from stream_seq continuity (dropped or corrupt).
    pub fn holes(&self) -> u64 {
        self.holes
    }

    /// Statuses that were not decodable stream frames.
    pub fn skipped(&self) -> u64 {
        self.skipped
    }

    /// Some frame's INST carried ALERT: the servo latched a fault (protocol
    /// sec 5.6).
    pub fn alert(&self) -> bool {
        self.alert
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regs::telemetry as t;

    use osc_protocol::tel::encode_stream_head;
    use osc_protocol::wire::STREAM_FLAG_LAST;
    use osc_servo_core::tel::{STREAM_HDR, STREAM_PAYLOAD_MAX};
    use osc_servo_core::{ControlTableCell, RegionStorage, RegionStorageRaw, TelemetryTick};

    const PRESETS: [Chunks; 4] = [TEL_RAW, TEL_LADDER, TEL_RL, TEL_POS];

    /// Every field distinct and in range, duty negative on odd rows, the
    /// shunt window read on even rows.
    fn sample(i: u16) -> (TelemetryTick, u16) {
        let tick = TelemetryTick {
            i_tick_counts: -(i as i16) - 1,
            vdiff_counts: -300 - i as i16,
            duty_ticks: if i.is_multiple_of(2) {
                293
            } else {
                -1200 + i as i16
            },
            pos: 0x100 + i,
            current_trough: 0xB00 + i,
            current_raw: 0x0100 + i,
            vmotor_a: 0x0A00 + i,
            vmotor_b: 0x0B00 + i,
            vbus_raw: 0x0C00 + i,
            ntc_raw: 0x0D00 + i,
            vcal: 0x0500 + i,
            vmotor_a_trough: 0x0700 + i,
            vmotor_b_trough: 0x0800 + i,
            tick_flags: if i.is_multiple_of(2) {
                tick_flag::I_VALID
            } else {
                0
            },
            sample_tick: i as u32,
            _scan_trough: [0x0900 + i; 7],
            _rsvd_align: 0,
        };
        (tick, 1800 + i)
    }

    /// The row the servo copies for `chunks` out of a table holding
    /// `sample(i)`: core's own layout, not this crate's mirror of it.
    fn row(chunks: Chunks, i: u16) -> Vec<u8> {
        let (tick, vbus) = sample(i);
        let table = ControlTableCell::new();
        table.with_mut(|tb| {
            tb.telemetry.tick = tick;
            tb.telemetry.estimates.vbus_counts = vbus;
        });
        let base = table.region_ptr().cast::<u8>();
        let mut out = Vec::new();
        for c in chunks.0 {
            for a in c.addr..c.addr + c.len {
                // SAFETY: a read of the local table; every chunk a test
                // copies is valid, so inside it.
                out.push(unsafe { base.add(a as usize).read() });
            }
        }
        out
    }

    /// One frame of `count` rows of `sample`, the first stamped `tick`.
    fn payload(chunks: Chunks, seq: u8, last: bool, tick: u16, count: u16) -> Vec<u8> {
        let mut p = vec![0; STREAM_HDR];
        encode_stream_head(seq, last, tick, &mut p);
        for i in 0..count {
            p.extend(row(chunks, i));
        }
        assert!(p.len() <= STREAM_PAYLOAD_MAX);
        p
    }

    /// `sample(i)` as a row of `chunks` should decode, the shunt window
    /// judged on duty alone when the row has no `tick_flags`.
    fn want(chunks: Chunks, floor: u16, tick: u64, i: u16) -> TelFrame {
        let (s, vbus) = sample(i);
        let sel = |r: Reg| chunks.offset(r.addr, r.width as u16).is_some();
        let duty = sel(t::DUTY_TICKS).then_some(s.duty_ticks);
        TelFrame {
            tick,
            window_valid: if sel(t::TICK_FLAGS) {
                s.tick_flags & tick_flag::I_VALID != 0
            } else {
                duty.is_some_and(|d| window_valid(d, floor))
            },
            pos: sel(t::POS).then_some(s.pos),
            current: sel(t::I_TICK_COUNTS).then_some(s.i_tick_counts),
            current_trough: sel(t::CURRENT_TROUGH).then_some(s.current_trough),
            duty_counts: duty,
            vdiff: sel(t::VDIFF_COUNTS).then_some(s.vdiff_counts),
            vbus: sel(t::VBUS_COUNTS).then_some(vbus),
            current_raw: sel(t::CURRENT_RAW).then_some(s.current_raw),
            vmotor_a: sel(t::VMOTOR_A).then_some(s.vmotor_a),
            vmotor_b: sel(t::VMOTOR_B).then_some(s.vmotor_b),
            vbus_raw: sel(t::VBUS_RAW).then_some(s.vbus_raw),
            ntc_raw: sel(t::NTC_RAW).then_some(s.ntc_raw),
        }
    }

    fn decode(chunks: Chunks, floor: u16, p: &[u8]) -> Vec<TelFrame> {
        let mut a = StreamAssembler::new(chunks, floor).expect("valid chunks");
        let mut out = Vec::new();
        a.push(true, false, p, &mut out);
        assert_eq!((a.holes(), a.skipped()), (0, 0));
        out
    }

    #[test]
    fn presets_carry_the_fields_their_captures_fit() {
        use t::*;
        let has = |c: Chunks, regs: &[Reg]| {
            regs.iter()
                .all(|r| c.offset(r.addr, r.width as u16).is_some())
        };
        assert!(PRESETS.iter().all(Chunks::valid));
        let raw = [
            DUTY_TICKS,
            POS,
            CURRENT_TROUGH,
            CURRENT_RAW,
            VMOTOR_A,
            VMOTOR_B,
        ];
        assert!(has(TEL_RAW, &raw));
        assert_eq!(
            (TEL_RAW, TEL_RAW.row_bytes()),
            (Chunks::two(Chunk::new(0x232, 8), Chunk::new(0x240, 4)), 12)
        );
        assert!(has(
            TEL_LADDER,
            &[I_TICK_COUNTS, VDIFF_COUNTS, DUTY_TICKS, POS]
        ));
        assert_eq!(
            TEL_LADDER,
            Chunks::two(Chunk::new(0x238, 2), Chunk::new(0x240, 8))
        );
        let rl = [DUTY_TICKS, CURRENT_RAW, VMOTOR_A, VMOTOR_B, VBUS_RAW];
        assert!(has(TEL_RL, &rl));
        assert_eq!(
            (TEL_RL, TEL_RL.row_bytes()),
            (Chunks::two(Chunk::new(0x232, 6), Chunk::new(0x23C, 6)), 12)
        );
        assert_eq!(TEL_POS, Chunks::one(0x238, 2));
    }

    /// The presets and the rail estimate in a second chunk decode, field
    /// for field, the bytes core's table holds at the chunks' addresses.
    #[test]
    fn every_preset_round_trips_the_core_table() {
        let with_vbus = Chunks::two(Chunk::new(0x240, 8), span(t::VBUS_COUNTS, t::VBUS_COUNTS));
        let flags = Chunks::two(Chunk::new(0x238, 4), span(t::TICK_FLAGS, t::TICK_FLAGS));
        for chunks in PRESETS.into_iter().chain([with_vbus, flags]) {
            for floor in [0, 300] {
                let frames = decode(chunks, floor, &payload(chunks, 0, false, 48, 16));
                assert_eq!(frames.len(), 16);
                for (i, f) in frames.iter().enumerate() {
                    assert_eq!(
                        *f,
                        want(chunks, floor, i as u64, i as u16),
                        "{chunks:?} row {i}"
                    );
                }
            }
        }
    }

    /// The window is the firmware's: a nonzero drive at or over the floor,
    /// either way; `tick_flags`, when the row has them, say it outright.
    #[test]
    fn window_valid_is_the_drive_against_the_floor() {
        assert!(window_valid(160, 160));
        assert!(window_valid(-160, 160));
        assert!(!window_valid(159, 160));
        assert!(!window_valid(-159, 160));
        assert!(!window_valid(0, 0));
        assert!(window_valid(1, 0));
        // row 0's 293 is over a 160 floor and its flags agree; with the
        // flags in the row, a 300 floor does not overrule them
        let flags = Chunks::two(Chunk::new(0x240, 2), span(t::TICK_FLAGS, t::TICK_FLAGS));
        let p = payload(flags, 0, true, 0, 1);
        assert!(decode(flags, 300, &p)[0].window_valid);
        let p = payload(TEL_RAW, 0, true, 0, 1);
        assert!(decode(TEL_RAW, 160, &p)[0].window_valid);
        assert!(!decode(TEL_RAW, 300, &p)[0].window_valid);
        let p = payload(TEL_POS, 0, true, 0, 1);
        assert!(!decode(TEL_POS, 0, &p)[0].window_valid, "no duty, no flags");
    }

    #[test]
    fn stream_golden_six_raw_fields() {
        // the default six at row 0 of sample: raw 0x100, va 0xA00, vb
        // 0xB00, pos 0x100, then duty 293 and trough 0xB00
        let p = [
            0x00, 0x00, 0x30, 0x12, 0x00, 0x01, 0x00, 0x0A, 0x00, 0x0B, 0x00, 0x01, 0x25, 0x01,
            0x00, 0x0B,
        ];
        assert_eq!(p[4..], row(TEL_RAW, 0)[..]);
        let f = decode(TEL_RAW, 160, &p)[0];
        assert!(f.window_valid);
        assert_eq!(f.pos, Some(0x100));
        assert_eq!(f.current_trough, Some(0xB00));
        assert_eq!(f.duty_counts, Some(293));
        assert_eq!(f.current_raw, Some(0x100));
        assert_eq!((f.vmotor_a, f.vmotor_b), (Some(0xA00), Some(0xB00)));
        assert_eq!((f.current, f.vdiff, f.vbus), (None, None, None));
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

    /// A payload that cannot be a frame of the chunks' rows is skipped,
    /// never decoded: no head, no row, a ragged remainder.
    #[test]
    fn assembler_skips_malformed_payloads() {
        let mut a = StreamAssembler::new(TEL_LADDER, 0).unwrap();
        let mut out = Vec::new();
        a.push(true, false, &[0, 0, 0], &mut out);
        a.push(true, false, &[0, 0, 0, 0], &mut out);
        let mut p = payload(TEL_LADDER, 0, false, 0, 2);
        p.pop();
        a.push(true, false, &p, &mut out);
        assert!(out.is_empty());
        assert_eq!(a.skipped(), 3);
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
            p
        };
        let mut a = StreamAssembler::new(TEL_POS, 0).unwrap();
        let mut out = Vec::new();
        a.push(true, false, &frame(0, false, 100, 24), &mut out);
        a.push(true, false, &frame(1, true, 124, 5), &mut out);
        assert_eq!(out.len(), 29);
        for (k, f) in out.iter().enumerate() {
            assert_eq!((f.tick, f.pos), (k as u64, Some(100 + k as u16)));
        }
        assert_eq!((a.holes(), a.skipped()), (0, 0));
    }

    #[test]
    fn assembler_ticks_run_through_the_seq_and_stamp_wraps() {
        // 258 one-row frames 16 ticks apart: seq wraps 255 -> 0 with no
        // hole, the stamp wraps past 0xFFFF on the way
        let mut a = StreamAssembler::new(TEL_POS, 0).unwrap();
        let mut out = Vec::new();
        for k in 0..258u64 {
            let tick = 0xF000u16.wrapping_add(16 * k as u16);
            let p = payload(TEL_POS, k as u8, k == 257, tick, 1);
            a.push(true, false, &p, &mut out);
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
        let mut a = StreamAssembler::new(TEL_POS, 0).unwrap();
        let mut out = Vec::new();
        for seq in [0u8, 1, 3] {
            let p = payload(TEL_POS, seq, seq == 3, 16 * seq as u16, 16);
            a.push(true, false, &p, &mut out);
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
        let mut a = StreamAssembler::new(TEL_POS, 0).unwrap();
        let mut out = Vec::new();
        a.push(true, false, &payload(TEL_POS, 0, false, 7, 16), &mut out);
        a.push(
            true,
            false,
            &payload(TEL_POS, 1, true, 7 + 20, 16),
            &mut out,
        );
        assert_eq!(out.len(), 32);
        assert_eq!((out[15].tick, out[16].tick), (15, 20));
        assert_eq!(a.holes(), 0);
    }

    /// A frame's INST ALERT is the servo's latched fault, and the burst
    /// carries it.
    #[test]
    fn assembler_carries_the_frame_alert() {
        for alert in [false, true] {
            let mut a = StreamAssembler::new(TEL_POS, 0).unwrap();
            let mut out = Vec::new();
            a.push(true, false, &payload(TEL_POS, 0, false, 0, 4), &mut out);
            a.push(true, alert, &payload(TEL_POS, 1, true, 4, 4), &mut out);
            assert_eq!(a.alert(), alert);
        }
    }

    #[test]
    fn assembler_skips_non_stream_statuses() {
        let mut a = StreamAssembler::new(TEL_POS, 0).unwrap();
        let mut out = Vec::new();
        a.push(false, false, &[0x55, 0xAA], &mut out); // an OK ack, not a frame
        let p = payload(TEL_POS, 0, true, 0, 2);
        a.push(true, false, &p, &mut out);
        assert_eq!(out.len(), 2);
        assert_eq!(a.skipped(), 1);
        assert_eq!(a.holes(), 0);
    }

    #[test]
    fn assembler_refuses_chunks_the_servo_would_not_arm() {
        assert!(StreamAssembler::new(Chunks::NONE, 0).is_none());
        assert!(StreamAssembler::new(Chunks::one(0x225, 2), 0).is_none());
        assert!(StreamAssembler::new(Chunks::one(0x224, 14), 0).is_none());
        for c in PRESETS {
            assert!(StreamAssembler::new(c, 0).is_some());
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
        let mut bytes = vec![0u8; 0x80];
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
        put(&mut bytes, t::DUTY_TICKS, &(-293i16).to_le_bytes());
        put(&mut bytes, t::I_TICK_COUNTS, &(-57i16).to_le_bytes());
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
        assert_eq!((s.duty_ticks, s.i_tick_counts), (-293, -57));
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
