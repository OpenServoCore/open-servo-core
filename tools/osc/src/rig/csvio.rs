//! CSV record and replay. Two tiers: raw per-experiment logs (every parsed
//! snapshot, every TEL frame - the complete record), and derived fit-input
//! CSVs (dwell samples, rungs, step series) that `ident fit <dir>` reads
//! back so fit changes never need rig time. The derived columns mirror the
//! osc-ident structs exactly - the round-trip tests pin it.

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use osc_ident::burst::{self, Capture};
use osc_ident::exp::WindowSample;
use osc_ident::exp::ladder::RungSummary;
use osc_ident::exp::resistance::DwellSample;
use osc_ident::exp::rl::{SegKind, Segment};
use osc_ident::fits::{RungPoint, StepSeries};
use osc_ident::frame::{TelFrame, TelemetrySnapshot};

pub(crate) struct OutDir(pub(crate) PathBuf);

impl OutDir {
    pub(crate) fn create(base: &Path) -> Result<Self> {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let dir = base.join(format!("{stamp}"));
        std::fs::create_dir_all(&dir).with_context(|| format!("mkdir {}", dir.display()))?;
        Ok(Self(dir))
    }

    pub(crate) fn file(&self, name: &str) -> Result<BufWriter<File>> {
        let p = self.0.join(name);
        Ok(BufWriter::new(
            File::create(&p).with_context(|| format!("create {}", p.display()))?,
        ))
    }
}

/// Raw snapshot log: one row per Read, full field dump.
pub(crate) struct SnapshotLog {
    w: BufWriter<File>,
}

impl SnapshotLog {
    pub(crate) fn create(dir: &OutDir, name: &str) -> Result<Self> {
        let mut w = dir.file(name)?;
        writeln!(
            w,
            "host_ms,fault_flags,fault_code,mode_active,theta_hat_q16,omega_hat_cps,\
             tau_d_counts,i_lim_counts,t_winding_cc,vbus_counts,duty_applied_q15,\
             omega_bemf_cps,r_hat_q12,i_hat_counts,sample_tick,pos,current,\
             current_trough,current_bias_counts,i_mean_counts,i_min_counts,\
             i_max_counts,vdiff_mean,duty_mean_q15,agg_seq"
        )?;
        Ok(Self { w })
    }

    pub(crate) fn push(&mut self, host_ms: f64, s: &TelemetrySnapshot) -> Result<()> {
        writeln!(
            self.w,
            "{host_ms:.1},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            s.fault_flags,
            s.fault_code,
            s.mode_active,
            s.theta_hat_q16,
            s.omega_hat_cps,
            s.tau_d_counts,
            s.i_lim_counts,
            s.t_winding_cc,
            s.vbus_counts,
            s.duty_applied_q15,
            s.omega_bemf_cps,
            s.r_hat_q12,
            s.i_hat_counts,
            s.sample_tick,
            s.pos,
            s.current,
            s.current_trough,
            s.current_bias_counts,
            s.i_mean_counts,
            s.i_min_counts,
            s.i_max_counts,
            s.vdiff_mean,
            s.duty_mean_q15,
            s.agg_seq
        )?;
        Ok(())
    }
}

pub(crate) fn write_tel_frames(dir: &OutDir, name: &str, frames: &[TelFrame]) -> Result<()> {
    let mut w = dir.file(name)?;
    writeln!(
        w,
        "tick,window_valid,pos,current,current_trough,duty_q15,vdiff,vbus,current_raw,vmotor_a,vmotor_b,vbus_raw,ntc_raw"
    )?;
    let opt = |v: Option<i32>| v.map(|v| v.to_string()).unwrap_or_default();
    for f in frames {
        writeln!(
            w,
            "{},{},{},{},{},{},{},{},{},{},{},{},{}",
            f.tick,
            f.window_valid as u8,
            opt(f.pos.map(|v| v as i32)),
            opt(f.current.map(|v| v as i32)),
            opt(f.current_trough.map(|v| v as i32)),
            opt(f.duty_q15.map(|v| v as i32)),
            opt(f.vdiff.map(|v| v as i32)),
            opt(f.vbus.map(|v| v as i32)),
            opt(f.current_raw.map(|v| v as i32)),
            opt(f.vmotor_a.map(|v| v as i32)),
            opt(f.vmotor_b.map(|v| v as i32)),
            opt(f.vbus_raw.map(|v| v as i32)),
            opt(f.ntc_raw.map(|v| v as i32)),
        )?;
    }
    Ok(())
}

/// Decoded frames back from a `write_tel_frames` CSV (cal-replay's input);
/// empty cells are unselected fields.
pub(crate) fn read_tel_frames(path: &Path) -> Result<Vec<TelFrame>> {
    fn opt<T: std::str::FromStr>(s: &str) -> Result<Option<T>>
    where
        T::Err: std::error::Error + Send + Sync + 'static,
    {
        if s.is_empty() {
            Ok(None)
        } else {
            Ok(Some(s.parse()?))
        }
    }
    let mut out = Vec::new();
    for parts in rows(path, 13)? {
        out.push(TelFrame {
            tick: parts[0].parse()?,
            window_valid: parts[1] == "1",
            pos: opt(&parts[2])?,
            current: opt(&parts[3])?,
            current_trough: opt(&parts[4])?,
            duty_q15: opt(&parts[5])?,
            vdiff: opt(&parts[6])?,
            vbus: opt(&parts[7])?,
            current_raw: opt(&parts[8])?,
            vmotor_a: opt(&parts[9])?,
            vmotor_b: opt(&parts[10])?,
            vbus_raw: opt(&parts[11])?,
            ntc_raw: opt(&parts[12])?,
        });
    }
    Ok(out)
}

// --- derived fit inputs -----------------------------------------------------

pub(crate) fn write_dwell_samples(dir: &OutDir, samples: &[DwellSample]) -> Result<()> {
    let mut w = dir.file("resistance.csv")?;
    writeln!(w, "dwell,dir,t_ms,i,vdiff,duty_q15")?;
    for s in samples {
        writeln!(
            w,
            "{},{},{},{},{},{}",
            s.dwell, s.dir, s.w.t_ms, s.w.i, s.w.vdiff, s.w.duty_q15
        )?;
    }
    Ok(())
}

pub(crate) fn read_dwell_samples(dir: &Path) -> Result<Vec<DwellSample>> {
    let mut out = Vec::new();
    for parts in rows(&dir.join("resistance.csv"), 6)? {
        out.push(DwellSample {
            dwell: parts[0].parse()?,
            dir: parts[1].parse()?,
            w: WindowSample {
                t_ms: parts[2].parse()?,
                i: parts[3].parse()?,
                vdiff: parts[4].parse()?,
                duty_q15: parts[5].parse()?,
            },
        });
    }
    Ok(out)
}

pub(crate) fn write_rungs(dir: &OutDir, rungs: &[RungSummary]) -> Result<()> {
    let mut w = dir.file("rungs.csv")?;
    writeln!(w, "duty_q15,omega,omega_r2,i,v,windows,used,note")?;
    for r in rungs {
        writeln!(
            w,
            "{},{},{},{},{},{},{},{}",
            r.duty_q15,
            r.omega,
            r.omega_r2,
            r.i,
            r.v,
            r.windows,
            r.used as u8,
            r.note.as_deref().unwrap_or("").replace(',', ";"),
        )?;
    }
    Ok(())
}

/// Used rungs back as fit points (the unused ones only matter to humans).
pub(crate) fn read_rung_points(dir: &Path) -> Result<Vec<RungPoint>> {
    Ok(read_rungs(dir)?
        .iter()
        .filter(|r| r.used)
        .map(|r| RungPoint {
            omega: r.omega,
            i: r.i,
            v: r.v,
        })
        .collect())
}

pub(crate) fn read_rungs(dir: &Path) -> Result<Vec<RungSummary>> {
    let mut out = Vec::new();
    for parts in rows(&dir.join("rungs.csv"), 8)? {
        out.push(RungSummary {
            duty_q15: parts[0].parse()?,
            omega: parts[1].parse()?,
            omega_r2: parts[2].parse()?,
            i: parts[3].parse()?,
            v: parts[4].parse()?,
            windows: parts[5].parse()?,
            used: parts[6] == "1",
            note: if parts[7].is_empty() {
                None
            } else {
                Some(parts[7].clone())
            },
        });
    }
    Ok(out)
}

/// Long form, one row per sample; `src` tags tel/agg per step so the
/// offline fit picks the same smoothing window the live one did.
pub(crate) fn write_step_series(dir: &OutDir, series: &[(StepSeries, bool)]) -> Result<()> {
    let mut w = dir.file("inertia_steps.csv")?;
    writeln!(w, "step,src,duty_q15,t,pos,i,mask")?;
    for (k, (s, tel)) in series.iter().enumerate() {
        let src = if *tel { "tel" } else { "agg" };
        for j in 0..s.t.len() {
            writeln!(
                w,
                "{k},{src},{},{},{},{},{}",
                s.duty_q15, s.t[j], s.pos[j], s.i[j], s.mask[j] as u8
            )?;
        }
    }
    Ok(())
}

pub(crate) fn read_step_series(dir: &Path) -> Result<Vec<(StepSeries, bool)>> {
    let mut out: Vec<(StepSeries, bool)> = Vec::new();
    let mut cur: Option<usize> = None;
    for parts in rows(&dir.join("inertia_steps.csv"), 7)? {
        let k: usize = parts[0].parse()?;
        if cur != Some(k) {
            cur = Some(k);
            out.push((
                StepSeries {
                    t: Vec::new(),
                    pos: Vec::new(),
                    i: Vec::new(),
                    mask: Vec::new(),
                    duty_q15: parts[2].parse()?,
                },
                parts[1] == "tel",
            ));
        }
        let s = &mut out.last_mut().expect("pushed").0;
        s.t.push(parts[3].parse()?);
        s.pos.push(parts[4].parse()?);
        s.i.push(parts[5].parse()?);
        s.mask.push(parts[6] == "1");
    }
    Ok(out)
}

/// One high-rate shunt capture per file, `burst-0.csv` up. The column
/// layout is osc-ident's own, so a bench capture from anywhere replays
/// through the same reader.
pub(crate) fn write_bursts(dir: &OutDir, caps: &[Capture]) -> Result<()> {
    for (k, cap) in caps.iter().enumerate() {
        let mut w = dir.file(&format!("burst-{k}.csv"))?;
        w.write_all(burst::to_csv(cap).as_bytes())?;
    }
    Ok(())
}

/// Read `burst-0.csv` up until one is missing - the write order, so a run
/// cut short reads back in order with no gaps.
pub(crate) fn read_bursts(dir: &Path) -> Result<Vec<Capture>> {
    let mut out = Vec::new();
    for k in 0.. {
        let p = dir.join(format!("burst-{k}.csv"));
        if !p.exists() {
            break;
        }
        let text = std::fs::read_to_string(&p).with_context(|| format!("read {}", p.display()))?;
        out.push(burst::from_csv(&text).map_err(|e| anyhow::anyhow!("{}: {e}", p.display()))?);
    }
    Ok(out)
}

/// Every captured burst of an R/L run, one row per sample: the segment
/// tag then the raw frame. `ident fit` refits R, tau and the gates from
/// this alone.
pub(crate) fn write_rl_segments(dir: &OutDir, segs: &[Segment]) -> Result<()> {
    let mut w = dir.file("rl.csv")?;
    writeln!(
        w,
        "seg,chain,kind,dir,bias,cmd_duty_q15,tick,window_valid,pos,duty_q15,current_raw,vmotor_a,vmotor_b,vbus_raw"
    )?;
    let opt = |v: Option<i32>| v.map(|v| v.to_string()).unwrap_or_default();
    for (k, s) in segs.iter().enumerate() {
        for f in &s.tel {
            writeln!(
                w,
                "{k},{},{},{},{},{},{},{},{},{},{},{},{},{}",
                s.chain,
                s.kind.as_str(),
                s.dir,
                s.bias,
                s.cmd_duty_q15,
                f.tick,
                f.window_valid as u8,
                opt(f.pos.map(|v| v as i32)),
                opt(f.duty_q15.map(|v| v as i32)),
                opt(f.current_raw.map(|v| v as i32)),
                opt(f.vmotor_a.map(|v| v as i32)),
                opt(f.vmotor_b.map(|v| v as i32)),
                opt(f.vbus_raw.map(|v| v as i32)),
            )?;
        }
    }
    Ok(())
}

pub(crate) fn read_rl_segments(dir: &Path) -> Result<Vec<Segment>> {
    fn opt<T: std::str::FromStr>(s: &str) -> Result<Option<T>>
    where
        T::Err: std::error::Error + Send + Sync + 'static,
    {
        if s.is_empty() {
            Ok(None)
        } else {
            Ok(Some(s.parse()?))
        }
    }
    let mut out: Vec<Segment> = Vec::new();
    let mut cur: Option<usize> = None;
    for parts in rows(&dir.join("rl.csv"), 14)? {
        let k: usize = parts[0].parse()?;
        if cur != Some(k) {
            cur = Some(k);
            out.push(Segment {
                chain: parts[1].parse()?,
                kind: SegKind::parse(&parts[2])
                    .with_context(|| format!("unknown rl segment kind {:?}", parts[2]))?,
                dir: parts[3].parse()?,
                bias: parts[4].parse()?,
                cmd_duty_q15: parts[5].parse()?,
                tel: Vec::new(),
            });
        }
        out.last_mut().expect("pushed").tel.push(TelFrame {
            tick: parts[6].parse()?,
            window_valid: parts[7] == "1",
            pos: opt(&parts[8])?,
            duty_q15: opt(&parts[9])?,
            current_raw: opt(&parts[10])?,
            vmotor_a: opt(&parts[11])?,
            vmotor_b: opt(&parts[12])?,
            vbus_raw: opt(&parts[13])?,
            ..Default::default()
        });
    }
    Ok(out)
}

fn rows(path: &Path, cols: usize) -> Result<Vec<Vec<String>>> {
    let f = File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut out = Vec::new();
    for (n, line) in BufReader::new(f).lines().enumerate() {
        let line = line?;
        if n == 0 || line.is_empty() {
            continue;
        }
        let parts: Vec<String> = line.split(',').map(str::to_string).collect();
        anyhow::ensure!(
            parts.len() >= cols,
            "{}:{} has {} cols, want {cols}",
            path.display(),
            n + 1,
            parts.len()
        );
        out.push(parts);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp() -> OutDir {
        let dir = std::env::temp_dir().join(format!("ident-csv-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        OutDir(dir)
    }

    #[test]
    fn bursts_round_trip_in_write_order() {
        let dir = tmp();
        let caps: Vec<Capture> = (0..2)
            .map(|n| Capture {
                samples: (0..osc_ident::burst::SAMPLES)
                    .map(|k| (100 + n * 10 + k % 200) as u16)
                    .collect(),
                meta: osc_ident::burst::Meta {
                    pre_q15: 0,
                    step_q15: 8520,
                    step_index: 485,
                    start_cnt: 1094,
                    pwm_arr: 1200,
                    start_dir: 1,
                    restore_dir: 0,
                    vbus_raw: 2169,
                    bias: 118,
                    chans: osc_ident::burst::CHAN_VMOTOR_A,
                    frame_len: 2,
                    vmotor_bias: 779,
                    pos: 122,
                    seated: true,
                },
            })
            .collect();
        write_bursts(&dir, &caps).unwrap();
        assert_eq!(read_bursts(&dir.0).unwrap(), caps);
    }

    #[test]
    fn tel_frames_round_trip_with_unselected_fields() {
        let dir = tmp();
        let frames = vec![
            TelFrame {
                tick: 0,
                window_valid: true,
                pos: Some(2048),
                current: Some(-33),
                current_trough: Some(500),
                duty_q15: Some(8520),
                vdiff: Some(-1700),
                vbus: Some(1731),
                current_raw: Some(620),
                vmotor_a: Some(1710),
                vmotor_b: Some(12),
                vbus_raw: Some(2290),
                ntc_raw: None,
            },
            TelFrame {
                tick: 17,
                window_valid: false,
                pos: Some(2049),
                current: Some(12),
                current_trough: None,
                duty_q15: Some(0),
                vdiff: Some(0),
                vbus: None,
                current_raw: None,
                vmotor_a: None,
                vmotor_b: Some(3),
                vbus_raw: None,
                ntc_raw: Some(2050),
            },
        ];
        write_tel_frames(&dir, "tel.csv", &frames).unwrap();
        let back = read_tel_frames(&dir.0.join("tel.csv")).unwrap();
        assert_eq!(back, frames);
    }

    #[test]
    fn dwell_samples_round_trip() {
        let dir = tmp();
        let samples = vec![
            DwellSample {
                dwell: 0,
                dir: 1,
                w: WindowSample {
                    t_ms: 12.8,
                    i: 55.5,
                    vdiff: 1700.0,
                    duty_q15: 8520.0,
                },
            },
            DwellSample {
                dwell: 3,
                dir: -1,
                w: WindowSample {
                    t_ms: 900.0,
                    i: -60.25,
                    vdiff: -1690.0,
                    duty_q15: -11468.0,
                },
            },
        ];
        write_dwell_samples(&dir, &samples).unwrap();
        let back = read_dwell_samples(&dir.0).unwrap();
        assert_eq!(back.len(), 2);
        for (a, b) in samples.iter().zip(&back) {
            assert_eq!(a.dwell, b.dwell);
            assert_eq!(a.dir, b.dir);
            assert_eq!(a.w, b.w);
        }
    }

    #[test]
    fn rungs_round_trip_used_only() {
        let dir = tmp();
        let rungs = vec![
            RungSummary {
                duty_q15: 8520,
                omega: 1500.5,
                omega_r2: 0.999,
                i: 40.0,
                v: 450.0,
                windows: 80,
                used: true,
                note: None,
            },
            RungSummary {
                duty_q15: 20971,
                omega: 0.0,
                omega_r2: 0.0,
                i: 0.0,
                v: 0.0,
                windows: 3,
                used: false,
                note: Some("steady segment too short, dropped".into()),
            },
        ];
        write_rungs(&dir, &rungs).unwrap();
        let pts = read_rung_points(&dir.0).unwrap();
        assert_eq!(pts.len(), 1);
        assert_eq!(pts[0].omega, 1500.5);
        assert_eq!(pts[0].i, 40.0);
        assert_eq!(pts[0].v, 450.0);
    }

    #[test]
    fn rl_segments_round_trip() {
        let dir = tmp();
        let frame = |tick: u64, duty: i16, raw: u16| TelFrame {
            tick,
            window_valid: true,
            pos: Some(2048),
            duty_q15: Some(duty),
            current_raw: Some(raw),
            vmotor_a: Some(1800),
            vmotor_b: Some(775),
            vbus_raw: Some(2000),
            ..Default::default()
        };
        let segs = vec![
            Segment {
                chain: 0,
                kind: SegKind::Rest,
                dir: 0,
                bias: 0,
                cmd_duty_q15: 0,
                tel: vec![TelFrame {
                    tick: 0,
                    window_valid: false,
                    current_raw: Some(512),
                    ..Default::default()
                }],
            },
            Segment {
                chain: 1,
                kind: SegKind::Toggle,
                dir: -1,
                bias: 1,
                cmd_duty_q15: -9830,
                tel: vec![frame(0, -9830, 640), frame(1, -9830, 690)],
            },
        ];
        write_rl_segments(&dir, &segs).unwrap();
        let back = read_rl_segments(&dir.0).unwrap();
        assert_eq!(back, segs);
    }

    #[test]
    fn step_series_round_trip() {
        let dir = tmp();
        let series = vec![
            (
                StepSeries {
                    t: vec![0.0, 0.001, 0.002],
                    pos: vec![400.0, 402.5, 407.0],
                    i: vec![300.0, 280.0, 260.0],
                    mask: vec![true, true, false],
                    duty_q15: 14745.0,
                },
                true,
            ),
            (
                StepSeries {
                    t: vec![0.0, 0.0008],
                    pos: vec![3600.0, 3595.0],
                    i: vec![-310.0, -290.0],
                    mask: vec![true, true],
                    duty_q15: -14745.0,
                },
                false,
            ),
        ];
        write_step_series(&dir, &series).unwrap();
        let back = read_step_series(&dir.0).unwrap();
        assert_eq!(back.len(), 2);
        for ((s, tel), (bs, btel)) in series.iter().zip(&back) {
            assert_eq!(tel, btel);
            assert_eq!(s.t, bs.t);
            assert_eq!(s.pos, bs.pos);
            assert_eq!(s.i, bs.i);
            assert_eq!(s.mask, bs.mask);
            assert_eq!(s.duty_q15, bs.duty_q15);
        }
    }
}
