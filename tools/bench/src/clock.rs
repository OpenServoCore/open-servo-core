//! Servo clock truth from the wire. The adapter's edge capture counts its
//! crystal (the same crystal that sets the host baud and paces CAL trains),
//! so a servo reply's character starts, timed there, read the servo's clock
//! against the bus reference with no servo-side state involved: the
//! independent measurement a CAL trim is judged by (protocol sec 9.3).
//!
//! The servo's UART divisor is integer at every catalog rate, so its char
//! pitch is exactly 10 bits of its own clock. A reply leaves in TX DMA arms
//! with idle seams between them: the fit shares one slope across arms and
//! gives each arm its own intercept, so a seam never reads as clock skew.
//! Sign: + = servo fast, the CAL ruler's convention.

use anyhow::Result;

use crate::edges::BStamp;
use crate::osc::{build_instruction, build_read, parse_exchange};
use crate::run::capture;
use crate::wire::Wire;
use osc_protocol::table::TELEMETRY_COMMON_START;
use osc_protocol::wire::{Inst, Opcode, ResultCode};

/// Bit-times per character: 1 start + 8 data + 1 stop.
const BITS_PER_CHAR: f64 = 10.0;

/// Fewest characters a pitch fit takes: under this the quantization of the
/// capture clock dominates any clock offset worth reading.
const MIN_CHARS: usize = 16;

/// An interval this many bits over the reply's median char interval is an
/// idle seam between TX arms. Arm seams are ISR-paced (>= ~1 us); a quarter
/// bit is far above capture quantization at every catalog rate.
const SEAM_BITS: f64 = 0.25;

/// A char start this many bits off the shared-slope fit was misdecoded:
/// clock skew moves starts by a fraction of a tick per char.
const RESIDUAL_MAX_BITS: f64 = 0.5;

/// Adapter UART kernel clock: its divisor is `round(PCLK / bps)`
/// (firmware/host-ch32 `usart_baud::apply_raw`), floored at 16.
pub const ADAPTER_PCLK_HZ: u32 = 144_000_000;
const ADAPTER_BRR_MIN: u32 = 16;

/// One frame's char pitch against the capture crystal.
#[derive(Clone, Debug, PartialEq)]
pub struct PitchFit {
    /// Talker clock offset, ppm, + = talker fast.
    pub ppm: f64,
    pub chars: usize,
    /// TX arms the frame split into at idle seams.
    pub arms: usize,
    /// Largest char-start residual against the fit, capture ticks.
    pub residual_max: f64,
}

/// Fit one frame's data stamps (break excluded) at `bit_ticks` capture
/// ticks per nominal bit. `None` when too short or misdecoded.
pub fn fit_pitch(chars: &[BStamp], bit_ticks: f64) -> Option<PitchFit> {
    if chars.len() < MIN_CHARS {
        return None;
    }
    let t: Vec<f64> = chars
        .iter()
        .map(|s| s.tick.wrapping_sub(chars[0].tick) as f64)
        .collect();
    let mut steps: Vec<f64> = t.windows(2).map(|w| w[1] - w[0]).collect();
    steps.sort_by(f64::total_cmp);
    let seam = steps[steps.len() / 2] + SEAM_BITS * bit_ticks;

    let mut arms = Vec::new();
    let mut head = 0;
    for i in 1..t.len() {
        if t[i] - t[i - 1] > seam {
            arms.push(&t[head..i]);
            head = i;
        }
    }
    arms.push(&t[head..]);

    let centred = |arm: &[f64]| {
        (
            (arm.len() - 1) as f64 / 2.0,
            arm.iter().sum::<f64>() / arm.len() as f64,
        )
    };
    let (mut sxy, mut sxx) = (0.0, 0.0);
    for arm in &arms {
        let (kbar, tbar) = centred(arm);
        for (k, &ti) in arm.iter().enumerate() {
            sxy += (k as f64 - kbar) * (ti - tbar);
            sxx += (k as f64 - kbar).powi(2);
        }
    }
    if sxx == 0.0 {
        return None;
    }
    let pitch = sxy / sxx;
    let mut residual_max: f64 = 0.0;
    for arm in &arms {
        let (kbar, tbar) = centred(arm);
        for (k, &ti) in arm.iter().enumerate() {
            residual_max = residual_max.max((ti - tbar - pitch * (k as f64 - kbar)).abs());
        }
    }
    if residual_max > RESIDUAL_MAX_BITS * bit_ticks {
        return None;
    }
    Some(PitchFit {
        ppm: (BITS_PER_CHAR * bit_ticks / pitch - 1.0) * 1e6,
        chars: chars.len(),
        arms: arms.len(),
        residual_max,
    })
}

/// The data stamps of the frame ending at stamp index `end` (exclusive):
/// back to the nearest break stamp.
pub fn frame_chars(stamps: &[BStamp], end: usize) -> &[BStamp] {
    let head = stamps[..end]
        .iter()
        .rposition(|s| s.flags & BStamp::BOUNDARY != 0)
        .map_or(0, |i| i + 1);
    &stamps[head..end]
}

/// A CAL train's own spacing as the capture saw it: the last `breaks`
/// break stamps against the `gap_ticks` grid.
#[derive(Clone, Debug, PartialEq)]
pub struct Pacing {
    /// Whole-train spacing error, ppm. Positive = spaced long, which a
    /// servo's ruler reads one-for-one as its own clock running fast.
    pub ppm: f64,
    /// Worst single gap's error, capture ticks.
    pub gap_err_max: i64,
}

/// `None` past one capture wrap per gap: the edge unwrap is exact only for
/// edges under a u16 tick wrap apart.
pub fn train_pacing(stamps: &[BStamp], breaks: usize, gap_ticks: u32) -> Option<Pacing> {
    if gap_ticks > u16::MAX as u32 {
        return None;
    }
    let marks: Vec<u32> = stamps
        .iter()
        .filter(|s| s.flags & BStamp::BOUNDARY != 0)
        .map(|s| s.tick)
        .collect();
    if breaks < 2 || marks.len() < breaks {
        return None;
    }
    let marks = &marks[marks.len() - breaks..];
    let nominal = (breaks - 1) as f64 * gap_ticks as f64;
    let span = marks[breaks - 1].wrapping_sub(marks[0]) as f64;
    let gap_err_max = marks
        .windows(2)
        .map(|w| (w[1].wrapping_sub(w[0]) as i64 - gap_ticks as i64).abs())
        .max()
        .unwrap_or(0);
    Some(Pacing {
        ppm: (span - nominal) / nominal * 1e6,
        gap_err_max,
    })
}

/// Mean and sample sd of repeated readings.
#[derive(Clone, Debug, PartialEq)]
pub struct Summary {
    pub n: usize,
    pub mean: f64,
    pub sd: f64,
}

impl Summary {
    pub fn of(xs: &[f64]) -> Option<Self> {
        if xs.len() < 2 {
            return None;
        }
        let n = xs.len() as f64;
        let mean = xs.iter().sum::<f64>() / n;
        let sd = (xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0)).sqrt();
        Some(Self {
            n: xs.len(),
            mean,
            sd,
        })
    }

    /// Half-width of the normal 95% interval on the mean.
    pub fn half95(&self) -> f64 {
        1.96 * self.sd / (self.n as f64).sqrt()
    }
}

/// Capture ticks per bit at `baud`, fractional.
pub fn bit_ticks(w: &Wire, baud: u32) -> f64 {
    w.hz_per_us() as f64 * 1e6 / baud as f64
}

/// The adapter's real rate when asked for `bps`, and its offset from
/// `nominal` in ppm (+ = adapter fast): the instrument's known answer.
pub fn adapter_offset_ppm(bps: u32, nominal: u32) -> f64 {
    let brr = ((ADAPTER_PCLK_HZ + bps / 2) / bps).max(ADAPTER_BRR_MIN);
    (ADAPTER_PCLK_HZ as f64 / brr as f64 / nominal as f64 - 1.0) * 1e6
}

/// Servo clock truth: `replies` READs of `len` bytes at `addr`, each
/// reply's pitch fit at the servo's catalog `baud` (the host must be on
/// that rate). Returns the per-reply readings and the replies that failed
/// to parse or fit.
pub fn servo_truth(
    w: &mut Wire,
    id: u8,
    baud: u32,
    addr: u16,
    len: u16,
    replies: u32,
) -> Result<(Vec<f64>, u32)> {
    let read = build_read(id, addr, len);
    let settle_ms = wire_ms(read.len() + len as usize, baud);
    let bt = bit_ticks(w, baud);
    let mut ppm = Vec::new();
    let mut rejects = 0;
    for _ in 0..replies {
        let (stamps, bits) = capture(w, &read, settle_ms)?;
        let fit = parse_exchange(&stamps, &read, bits)
            .ok()
            .filter(|ex| ex.status.result == Some(ResultCode::Ok))
            .and_then(|ex| fit_pitch(frame_chars(&stamps, ex.stamps_end), bt));
        match fit {
            Some(f) => ppm.push(f.ppm),
            None => rejects += 1,
        }
    }
    Ok((ppm, rejects))
}

/// Instrument self-check: `frames` NOREPLY WRITEs of `len` bytes to
/// `absent_id` (no servo answers or acts), each echo's pitch fit against
/// the catalog `nominal` rate while the host runs whatever rate it is set
/// to. Reads `adapter_offset_ppm` exactly when the instrument is right.
pub fn adapter_echo(
    w: &mut Wire,
    absent_id: u8,
    nominal: u32,
    len: usize,
    frames: u32,
) -> Result<(Vec<f64>, u32)> {
    // Aimed at the read-only telemetry block: even a servo that did hold the
    // id would refuse the write.
    let mut payload = TELEMETRY_COMMON_START.to_le_bytes().to_vec();
    payload.extend((0..len).map(|i| i as u8));
    let frame = build_instruction(absent_id, Opcode::Write, Inst::FLAG_NOREPLY, &payload);
    let settle_ms = wire_ms(frame.len(), w.current_baud());
    let bt = bit_ticks(w, nominal);
    let mut ppm = Vec::new();
    let mut rejects = 0;
    for _ in 0..frames {
        let (stamps, _) = capture(w, &frame, settle_ms)?;
        let chars = frame_chars(&stamps, stamps.len());
        let echoed =
            chars.len() == frame.len() && chars.iter().zip(&frame).all(|(s, b)| s.byte == *b);
        match fit_pitch(chars, bt).filter(|_| echoed) {
            Some(f) => ppm.push(f.ppm),
            None => rejects += 1,
        }
    }
    Ok((ppm, rejects))
}

/// Settle window for `bytes` of wire at `baud`, plus servo latency and USB
/// slack (the tool-read rule).
fn wire_ms(bytes: usize, baud: u32) -> u64 {
    const SLACK_BYTES: u64 = 16;
    const SLACK_MS: u64 = 4;
    (bytes as u64 + SLACK_BYTES) * 10_000 / baud as u64 + SLACK_MS
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::edges::stamps_from_edges;
    use crate::edges::tests::frame_edges;

    /// Char start stamps of a talker whose clock runs `ppm` fast, quantized
    /// by floor() on the capture grid, with `phase` ticks of start offset
    /// and an idle `seam` after char `seam_at`.
    fn stamps(
        n: usize,
        bit_ticks: f64,
        ppm: f64,
        phase: f64,
        seam_at: usize,
        seam: f64,
    ) -> Vec<BStamp> {
        let pitch = BITS_PER_CHAR * bit_ticks / (1.0 + ppm * 1e-6);
        (0..n)
            .map(|k| {
                let gap = if k > seam_at { seam } else { 0.0 };
                BStamp {
                    tick: (1000.0 + phase + k as f64 * pitch + gap).floor() as u32,
                    byte: 0x55,
                    flags: 0,
                }
            })
            .collect()
    }

    #[test]
    fn reads_a_fast_and_a_slow_talker_with_the_ruler_sign() {
        for ppm in [-12_000.0, -3_000.0, 2_500.0, 30_000.0] {
            let fit = fit_pitch(&stamps(245, 18.0, ppm, 0.37, usize::MAX, 0.0), 18.0).unwrap();
            assert!((fit.ppm - ppm).abs() < 20.0, "want {ppm}, read {}", fit.ppm);
        }
    }

    #[test]
    fn an_arm_seam_never_reads_as_skew() {
        // 3M, 1.7 us idle between a 4-char header arm and the payload arm.
        let s = stamps(245, 6.0, 1_000.0, 0.81, 3, 1.7 * 18.0);
        let fit = fit_pitch(&s, 6.0).unwrap();
        assert_eq!(fit.arms, 2);
        assert!((fit.ppm - 1_000.0).abs() < 100.0, "read {}", fit.ppm);
        // The same capture fit as one arm would read the seam as slowness.
        let naive = (BITS_PER_CHAR * 6.0 * 244.0 / (s[244].tick - s[0].tick) as f64 - 1.0) * 1e6;
        assert!(naive < -500.0, "seam folded in reads {naive}");
    }

    #[test]
    fn quantization_averages_out_over_reply_phases() {
        // Near-commensurate worst case at 3M: the quantization pattern is
        // shared by every char of a reply, so single replies err by tens of
        // ppm, but the mean over start phases is unbiased.
        let truth = 10.0;
        let reads: Vec<f64> = (0..64)
            .map(|i| {
                fit_pitch(
                    &stamps(245, 6.0, truth, i as f64 / 64.0, usize::MAX, 0.0),
                    6.0,
                )
                .unwrap()
                .ppm
            })
            .collect();
        let s = Summary::of(&reads).unwrap();
        assert!((s.mean - truth).abs() < 5.0, "mean {}", s.mean);
        assert!(
            reads.iter().any(|r| (r - truth).abs() > 5.0),
            "worst case not exercised"
        );
    }

    #[test]
    fn a_misdecoded_start_rejects_the_fit() {
        let mut s = stamps(64, 18.0, 0.0, 0.5, usize::MAX, 0.0);
        s[40].tick -= 18; // one bit early: a data edge taken for a start
        assert_eq!(fit_pitch(&s, 18.0), None);
    }

    #[test]
    fn a_short_frame_reads_nothing() {
        assert_eq!(
            fit_pitch(
                &stamps(MIN_CHARS - 1, 18.0, 0.0, 0.0, usize::MAX, 0.0),
                18.0
            ),
            None
        );
    }

    #[test]
    fn decoded_reply_reads_its_talker_through_the_edge_path() {
        // 1M, a talker one tick slow per char: 180 nominal, 181 real.
        let b = 18;
        let bytes: Vec<u8> = (0..64u8).collect();
        let mut edges = Vec::new();
        frame_edges(5_000, b, 5 * b, 10 * b + 1, &bytes, true, &mut edges);
        let stamps = stamps_from_edges(&edges, b as u32);
        let chars = frame_chars(&stamps, stamps.len());
        assert_eq!(chars.len(), bytes.len());
        let fit = fit_pitch(chars, b as f64).unwrap();
        let want = (180.0 / 181.0 - 1.0) * 1e6;
        assert!(
            (fit.ppm - want).abs() < 0.01,
            "want {want}, read {}",
            fit.ppm
        );
    }

    #[test]
    fn train_pacing_reads_a_late_last_break() {
        let gap = 7_200u32; // 400 us at 18 MHz
        let mut s: Vec<BStamp> = (0..9u32)
            .map(|k| BStamp {
                tick: 100 + k * gap,
                byte: 0,
                flags: BStamp::BOUNDARY,
            })
            .collect();
        // The announce's own break leads the train and must not count.
        s.insert(
            0,
            BStamp {
                tick: 7,
                byte: 0,
                flags: BStamp::BOUNDARY,
            },
        );
        s.last_mut().unwrap().tick += 18; // 1 us late
        let p = train_pacing(&s, 9, gap).unwrap();
        assert!((p.ppm - 1e6 / 3200.0).abs() < 1e-6, "ppm {}", p.ppm);
        assert_eq!(p.gap_err_max, 18);
        assert_eq!(
            train_pacing(&s, 9, u16::MAX as u32 + 1),
            None,
            "past one wrap"
        );
    }

    #[test]
    fn adapter_detune_offsets_are_the_divisor_ratio() {
        // One BRR step off 1M: 144 -> 145.
        let slow = adapter_offset_ppm(993_103, 1_000_000);
        assert!((slow - (144.0 / 145.0 - 1.0) * 1e6).abs() < 1e-6);
        assert_eq!(adapter_offset_ppm(3_000_000, 3_000_000), 0.0);
    }

    #[test]
    fn summary_half_width_shrinks_with_n() {
        let s = Summary::of(&[1.0, 3.0, 1.0, 3.0]).unwrap();
        assert_eq!(s.mean, 2.0);
        assert!((s.sd - (4.0f64 / 3.0).sqrt()).abs() < 1e-12);
        assert!((s.half95() - 1.96 * s.sd / 2.0).abs() < 1e-12);
    }
}
