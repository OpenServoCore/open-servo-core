//! `osc cal-replay` -- offline replay + corruption diagnostic for a saved
//! sweep_tel.csv. `osc cal` streams the ripple sweep as a TEL burst and
//! writes the decoded frames; this command reads them back without a servo,
//! classifies which corruption mode (if any) the capture carries, and
//! re-runs the same pot-LUT / motor-rev pipeline so the analysis can be
//! iterated offline. It NEVER connects to the servo (no baud/id).
//!
//! Two corruption modes are distinguished: dropped frames (CRC-failed on
//! the bus; the values that survive are clean, tick continuity has 16-tick
//! holes) versus value-domain bit-errors (pos out of the 12-bit range, or a
//! within-range pos jump no motor could make in one sample - old side-
//! channel captures replayed through the CSV can still carry these).

use anyhow::{Context, Result};
use osc_ident::frame::TelFrame;
use osc_ident::lut::{self, build_multi, stitched_motor_revs};

use super::{RIPPLE_PER_REV, build_sweep, build_sweep_chunks, pos_plausible};
use crate::rig::csvio;

/// A pos delta larger than this across two truly-consecutive samples is
/// physically impossible at the sweep sample rate (the capture traverses at a
/// few thousand counts/s, i.e. well under one count per 20 kHz sample), so it
/// marks a within-range bit-flip rather than real motion.
const MAX_STEP_COUNTS: u32 = 200;

/// `osc cal-replay` args. Offline: only the file and decode parameters, no
/// bus. Old raw side-channel captures (tel-raw.bin) are unsupported - the
/// CDC channel is gone; replay reads the decoded frames CSV.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// The sweep_tel.csv a cal run writes (tel-raw.bin files from the old
    /// CDC side channel are unsupported).
    path: std::path::PathBuf,
    /// Ripple sample rate (Hz); the cal capture uses the servo tick_hz.
    #[arg(long, default_value_t = 20000.0)]
    fs: f64,
    /// Pot count at the min rail; default = min plausible pos observed.
    #[arg(long)]
    raw_min: Option<u16>,
    /// Pot count at the max rail; default = max plausible pos observed.
    #[arg(long)]
    raw_max: Option<u16>,
}

/// Corruption tallies from a decoded frame list.
struct Classification {
    /// Missing samples in tick continuity (dropped frames x 16).
    tick_holes: u64,
    /// Frames with pos > 4095 (12-bit ADC overflow = definite bit corruption).
    out_of_range: usize,
    /// Truly-consecutive (tick delta 1) within-range pos pairs whose |delta|
    /// exceeds MAX_STEP_COUNTS - a within-range bit-flip.
    implausible_jumps: usize,
    /// Largest |pos delta| seen across the same within-range consecutive pairs.
    max_consec_delta: u32,
}

fn classify(frames: &[TelFrame]) -> Classification {
    let out_of_range = frames
        .iter()
        .filter(|f| matches!(f.pos, Some(p) if !pos_plausible(p)))
        .count();
    let mut tick_holes = 0u64;
    let mut implausible_jumps = 0usize;
    let mut max_consec_delta = 0u32;
    for w in frames.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        // only truly consecutive samples (no dropped frame in between)
        if b.tick.saturating_sub(a.tick) != 1 {
            tick_holes += b.tick.saturating_sub(a.tick).saturating_sub(1);
            continue;
        }
        // both ends within range: an out-of-range end is already counted above,
        // and this keeps the jump metric a distinct within-range signal.
        if let (Some(pa), Some(pb)) = (a.pos, b.pos)
            && pos_plausible(pa)
            && pos_plausible(pb)
        {
            let delta = (pa as i32 - pb as i32).unsigned_abs();
            if delta > MAX_STEP_COUNTS {
                implausible_jumps += 1;
            }
            max_consec_delta = max_consec_delta.max(delta);
        }
    }
    Classification {
        tick_holes,
        out_of_range,
        implausible_jumps,
        max_consec_delta,
    }
}

pub fn run(args: &Args) -> Result<()> {
    let frames = csvio::read_tel_frames(&args.path)
        .with_context(|| format!("read {} (a decoded frames CSV)", args.path.display()))?;
    println!("file: {} ({} frames)", args.path.display(), frames.len());

    let cls = classify(&frames);

    // resolve rails: explicit flags win; otherwise fall back to the observed
    // plausible-pos bounds and say so.
    let observed_used = args.raw_min.is_none() || args.raw_max.is_none();
    let obs_min = frames
        .iter()
        .filter_map(|f| f.pos)
        .filter(|&p| pos_plausible(p))
        .min();
    let obs_max = frames
        .iter()
        .filter_map(|f| f.pos)
        .filter(|&p| pos_plausible(p))
        .max();
    let raw_min = args.raw_min.or(obs_min).unwrap_or(0);
    let raw_max = args.raw_max.or(obs_max).unwrap_or(0);

    let sweep = build_sweep(&frames);
    let chunks = build_sweep_chunks(&frames);
    let chunk_samples: usize = chunks.iter().map(|(pos, _)| pos.len()).sum();
    let run_len = sweep.as_ref().map(|(pos, _)| pos.len()).unwrap_or(0);
    let run_cov = sweep
        .as_ref()
        .map(|(pos, _)| lut::span_coverage(pos, raw_min, raw_max))
        .unwrap_or(0.0);

    // significant when >1% of samples went missing (advisory heuristic)
    let significant_holes =
        !frames.is_empty() && cls.tick_holes.saturating_mul(100) > frames.len() as u64;
    let verdict = if cls.out_of_range > 0 || cls.implausible_jumps > 0 {
        "looks like value-domain bit-errors (an old side-channel capture?)"
    } else if significant_holes {
        "looks like dropped frames (values clean, whole 16-sample frames lost)"
    } else {
        "capture looks clean"
    };

    println!("--- classification ---");
    println!("frames:            {}", frames.len());
    println!(
        "tick holes:        {} samples missing (dropped frames x 16)",
        cls.tick_holes
    );
    println!(
        "out-of-range pos:  {} (pos > 4095, bit corruption)",
        cls.out_of_range
    );
    println!(
        "implausible jumps: {} (|pos delta| > {} across consecutive ticks; max consec delta {})",
        cls.implausible_jumps, MAX_STEP_COUNTS, cls.max_consec_delta
    );
    if observed_used {
        println!("note: rails from observed plausible pos bounds (no --raw-min/--raw-max)");
    }
    println!(
        "longest run:       {run_len} samples, span coverage {:.0}% of rails {raw_min}..{raw_max}",
        run_cov * 100.0
    );
    println!(
        "chunks:            {} (samples total {chunk_samples})",
        chunks.len()
    );
    println!("verdict:           {verdict}");

    println!("--- pipeline replay ---");
    if chunks.is_empty() {
        println!("no usable chunks");
    } else {
        println!(
            "stitching {} chunks ({chunk_samples} samples)",
            chunks.len()
        );
        match stitched_motor_revs(&chunks, args.fs, RIPPLE_PER_REV, raw_min, raw_max) {
            Some((m, cov)) => println!(
                "motor revs (full travel): {m:.2} (stitched coverage {:.0}%)",
                cov * 100.0
            ),
            None => println!("motor revs (full travel): unavailable"),
        }
        // same LUT summary wording as cal::run; build_multi decides populated vs
        // identity (stitched coverage / ripple SNR) internally.
        let l = build_multi(&chunks, args.fs, RIPPLE_PER_REV, raw_min, raw_max);
        let populated = l.corr.iter().any(|&c| c != 0);
        if populated {
            let maxc = l.corr.iter().map(|&c| c.unsigned_abs()).max().unwrap_or(0);
            println!("pot LUT: populated, max |corr| {maxc} counts");
        } else {
            let all_pos: Vec<u16> = chunks
                .iter()
                .flat_map(|(pos, _)| pos.iter().copied())
                .collect();
            let cov = lut::span_coverage(&all_pos, raw_min, raw_max);
            println!(
                "pot LUT: identity (stitched coverage {:.0}% of travel)",
                cov * 100.0
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame(tick: u64, pos: u16) -> TelFrame {
        TelFrame {
            tick,
            pos: Some(pos),
            current: Some(0),
            ..Default::default()
        }
    }

    #[test]
    fn classify_counts_out_of_range_holes_and_within_range_jumps() {
        let frames = vec![
            frame(0, 1000),
            frame(1, 1010),  // +10, ok
            frame(2, 5000),  // out-of-range (bit corruption)
            frame(3, 1020),  // 2->3 consecutive but tick 2 out-of-range -> not a jump
            frame(4, 1900),  // 1020->1900 = 880, within-range implausible jump
            frame(21, 1905), // 16-tick hole (a dropped frame) -> not consecutive
        ];
        let c = classify(&frames);
        assert_eq!(c.out_of_range, 1);
        assert_eq!(c.implausible_jumps, 1);
        assert_eq!(c.max_consec_delta, 880);
        assert_eq!(c.tick_holes, 16);
    }

    #[test]
    fn classify_clean_stream_has_no_flags() {
        // small monotonic steps, all consecutive, all in range
        let frames: Vec<TelFrame> = (0..8u64).map(|k| frame(k, 1000 + k as u16)).collect();
        let c = classify(&frames);
        assert_eq!(c.out_of_range, 0);
        assert_eq!(c.implausible_jumps, 0);
        assert_eq!(c.max_consec_delta, 1);
        assert_eq!(c.tick_holes, 0);
    }
}
