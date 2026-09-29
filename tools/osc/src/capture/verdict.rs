//! Whether a recording holds every segment its schedule promises, each clean.

use osc_ident::exp::{GOVERNED, declined, judge};

use crate::sweep::{Cfg, Recording, Step};

/// Segments a sweep commits: the baseline, then one per step per direction.
pub(crate) fn expected_segments(baseline: bool, dirs: usize, steps: usize) -> usize {
    baseline as usize + dirs * steps
}

/// The check on a recording `sweep::record` returned Ok (a chain that gave
/// up is already an Err): every segment present, numbered in order, clean,
/// and every direction driven. A drive segment whose applied duty never
/// once met its goal, because the current limit held it under, is not
/// clean: nothing in it measured the step as commanded. A drive slews from
/// rest, a chained one from the duty the segment before it left applied.
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
        if s.cmd_duty_q15 == 0 {
            continue;
        }
        let step = cfg.steps[i.saturating_sub(baseline as usize) % cfg.steps.len()];
        let start = match (step, i.checked_sub(1)) {
            (Step::Drive(..), _) | (_, None) => 0,
            (_, Some(prev)) => segs[prev]
                .frames
                .iter()
                .rev()
                .find_map(|f| f.duty_q15)
                .unwrap_or(0),
        };
        let duty = s.frames.iter().filter_map(|f| Some((f.tick, f.duty_q15?)));
        if declined(&judge(duty, s.cmd_duty_q15, start)) {
            return Err(format!("seg {}: {GOVERNED}", s.seg));
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
    use crate::sweep::{Decay, Dirs, Segment};
    use osc_ident::frame::TelFrame;

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
            stops: Some((209, 3849)),
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

    /// Applied duty per tick: a slew from `from` at the firmware's rate,
    /// held at `held` once it gets there.
    fn frames(from: i16, held: i16, n: u64) -> Vec<TelFrame> {
        (0..n)
            .map(|t| TelFrame {
                tick: t,
                duty_q15: Some((held as i64).min(from as i64 + 128 * (t as i64 + 1)) as i16),
                ..Default::default()
            })
            .collect()
    }

    /// A 20% rung the limit held at 15% never met its goal: rejected. The
    /// same rung slewing up to its goal passes, and so does a climb the
    /// limit held for a while before the duty met the goal.
    #[test]
    fn a_governed_segment_is_rejected() {
        let c = cfg(Dirs::Fwd, 0);
        let rung = |frames: Vec<TelFrame>| {
            let mut r = clean(&c);
            r.segments[0].cmd_duty_q15 = 6553;
            r.segments[0].frames = frames;
            r
        };
        let held = rung(frames(4369, 4915, 400));
        assert_eq!(
            verdict(&held, &c).unwrap_err(),
            "seg 1: declined: the current limit governed this window"
        );
        assert_eq!(verdict(&rung(frames(4369, 6553, 400)), &c), Ok(()));
        let mut climb = frames(4369, 4915, 200);
        climb.extend(frames(4915, 6553, 200).into_iter().map(|f| TelFrame {
            tick: f.tick + 200,
            ..f
        }));
        assert_eq!(verdict(&rung(climb), &c), Ok(()));
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
