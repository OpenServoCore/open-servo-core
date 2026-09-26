//! Whether a recording holds every segment its schedule promises, each clean.

use crate::sweep::{Cfg, Recording};

/// Segments a sweep commits: the baseline, then one per step per direction.
pub(crate) fn expected_segments(baseline: bool, dirs: usize, steps: usize) -> usize {
    baseline as usize + dirs * steps
}

/// The check on a recording `sweep::record` returned Ok (a chain that gave
/// up is already an Err): every segment present, numbered in order, clean,
/// and every direction driven.
pub(crate) fn verdict(rec: &Recording, cfg: &Cfg) -> Result<(), String> {
    let segs = &rec.segments;
    let baseline = cfg.baseline_ms > 0;
    let want = expected_segments(baseline, cfg.dirs.signs().len(), cfg.steps.len());
    if segs.len() != want {
        return Err(format!("{} of {want} segments", segs.len()));
    }
    let first = u32::from(!baseline);
    for (i, s) in segs.iter().enumerate() {
        if s.seg != first + i as u32 {
            return Err(format!(
                "segment {i} is seg {}, not {}",
                s.seg,
                first + i as u32
            ));
        }
        if s.stats.holes > 0 || s.stats.garble > 0 {
            return Err(format!(
                "seg {}: {} holes, {} garble",
                s.seg, s.stats.holes, s.stats.garble
            ));
        }
    }
    for &d in cfg.dirs.signs() {
        if !segs.iter().any(|s| s.dir == d) {
            return Err(format!("no segment drives dir {d:+}"));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rig::pump::BurstStats;
    use crate::sweep::{Decay, Dirs, Segment, Step};

    fn cfg(dirs: Dirs, baseline_ms: u32) -> Cfg {
        Cfg {
            steps: vec![Step::Drive(20, Some(80)), Step::Coast(400)],
            dirs,
            decay: Decay::Slow,
            window_ms: 150,
            rest_ms: 1500,
            baseline_ms,
            seek_duty_pct: 15,
            seek_cap_pct: 45,
            settle_ms: 300,
            stall: false,
            static_load: false,
            guard: (532, 3526),
            tel_mask: 0x1cd,
            rung_tries: 3,
        }
    }

    fn seg(seg: u32, dir: i8, holes: u64) -> Segment {
        Segment {
            seg,
            dir,
            cmd_duty_q15: 0,
            frames: Vec::new(),
            stats: BurstStats {
                frames: 1,
                samples: 16,
                holes,
                garble: 0,
            },
        }
    }

    /// What a clean both-ways run commits: seg 0, then fwd, then rev.
    fn clean(cfg: &Cfg) -> Recording {
        let n = cfg.steps.len() as u32;
        let mut segments = Vec::new();
        if cfg.baseline_ms > 0 {
            segments.push(seg(0, 0, 0));
        }
        for (d, &dir) in cfg.dirs.signs().iter().enumerate() {
            for k in 0..n {
                segments.push(seg(1 + d as u32 * n + k, dir, 0));
            }
        }
        Recording { segments }
    }

    #[test]
    fn clean_recordings_pass() {
        for c in [
            cfg(Dirs::Both, 1000),
            cfg(Dirs::Fwd, 0),
            cfg(Dirs::Rev, 1000),
        ] {
            assert_eq!(verdict(&clean(&c), &c), Ok(()));
        }
    }

    #[test]
    fn a_short_recording_is_rejected() {
        let c = cfg(Dirs::Both, 1000);
        let mut r = clean(&c);
        r.segments.pop();
        assert_eq!(verdict(&r, &c).unwrap_err(), "4 of 5 segments");
    }

    #[test]
    fn a_dirty_segment_is_rejected() {
        let c = cfg(Dirs::Both, 1000);
        let mut r = clean(&c);
        r.segments[3].stats.holes = 2;
        assert!(verdict(&r, &c).unwrap_err().starts_with("seg 3: 2 holes"));
        let mut r = clean(&c);
        r.segments[1].stats.garble = 1;
        assert!(verdict(&r, &c).unwrap_err().contains("1 garble"));
    }

    #[test]
    fn a_missing_direction_is_rejected() {
        let c = cfg(Dirs::Both, 1000);
        let mut r = clean(&c);
        for s in &mut r.segments[3..] {
            s.dir = 1;
        }
        assert_eq!(verdict(&r, &c).unwrap_err(), "no segment drives dir -1");
    }

    #[test]
    fn segments_number_from_the_baseline() {
        let c = cfg(Dirs::Fwd, 1000);
        let mut r = clean(&c);
        r.segments[2].seg = 4;
        assert!(verdict(&r, &c).unwrap_err().contains("is seg 4, not 2"));
        let c = cfg(Dirs::Fwd, 0);
        let mut r = clean(&c);
        r.segments[0].seg = 0;
        assert!(verdict(&r, &c).unwrap_err().contains("is seg 0, not 1"));
    }
}
