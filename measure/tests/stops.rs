//! The stops step's seated read on recorded bursts: the record's seated
//! set, the reverse groups at the low stop of bench capture B, through the
//! period-pair fit on stage 1's scales and the board's edge lags.

mod common;

use std::path::{Path, PathBuf};

use common::path;
use osc_measure::capture::burst;
use osc_measure::fit::rise::{Lags, Scales};
use osc_measure::fit::seated::{self, Shot};
use serde_json::Value;

/// The seated set of record: B's three seatings, the groups that press
/// into the stop, and its pooled median, ohm.
const SEATS: [&str; 3] = ["seat-1", "seat-2", "seat-3"];
const GROUPS: [(&str, usize); 3] = [("g3-rev-40", 1), ("g4-rev-25", 0), ("g5-rev-25-rested", 0)];
const R_RECORD_OHM: f64 = 4.2174;

fn stage1() -> Scales {
    let s: Value =
        serde_json::from_str(&common::text("testdata/bursts/sources.json")).expect("sources");
    let v = |k: &str| s["scales"][k]["value"].as_f64().expect(k);
    let lag = |k: &str| s["scales"]["lags"][k]["value"].as_f64().expect(k);
    Scales {
        counts_per_a: v("counts_per_a"),
        terminal_counts_per_v: v("terminal_counts_per_v"),
        terminal_offset_counts: v("terminal_offset_counts"),
        window_scale: v("window_scale"),
        lags: Lags {
            on: lag("on"),
            tap_on: lag("tap_on"),
            off: lag("off"),
            settle: lag("settle"),
        },
    }
}

/// Every burst of the seated set under `root`, a group a folder.
fn shots(root: &Path) -> Vec<Shot> {
    let mut out = Vec::new();
    for (s, seat) in SEATS.iter().enumerate() {
        for (g, (name, level)) in GROUPS.iter().enumerate() {
            for k in 0.. {
                let p = root.join(seat).join(name).join(format!("burst-{k}.csv"));
                let Ok(text) = std::fs::read_to_string(&p) else {
                    break;
                };
                out.push(Shot {
                    group: 3 * s + g,
                    stop: 0,
                    level: *level,
                    ntc_c: 25.0,
                    cap: burst::from_csv(&text).unwrap_or_else(|e| panic!("{}: {e}", p.display())),
                });
            }
        }
    }
    out
}

fn pooled(root: &Path) -> f64 {
    let all = shots(root);
    assert_eq!(all.len(), 45, "five bursts a group");
    let r = seated::read(&all, &stage1());
    for b in &r.bursts {
        println!(
            "group {} burst {}: {} R {:.4} ohm, first window {:.4} A ({:+.1}%){}",
            b.group,
            b.k,
            b.step_q15,
            b.fit.as_ref().map_or(f64::NAN, |f| f.r_ohm),
            b.i_first_a,
            100.0 * b.move_share,
            b.aside
                .as_ref()
                .map_or(String::new(), |a| format!(", set aside: {}", a.said()))
        );
    }
    assert_eq!(
        r.kept().count(),
        36,
        "the first of each group set aside, nothing else"
    );
    let p = r.r(|_| true).expect("a pool");
    let l = r.l(|_| true).expect("a pool");
    println!(
        "pooled R {:.4} ohm (record {R_RECORD_OHM}), sd {:.2}%, within groups {:.2}%; L {:.4} mH",
        p.median,
        100.0 * p.sd_share,
        100.0 * r.r_sd_within_groups().expect("groups"),
        l.median * 1e3
    );
    p.median
}

#[test]
fn the_seated_pool_matches_the_record_on_its_bursts() {
    let r = pooled(&path("testdata/seated"));
    assert!(
        (r / R_RECORD_OHM - 1.0).abs() < 0.01,
        "pooled R {r:.4} against the record's {R_RECORD_OHM}"
    );
}

/// The same on the bringup checkout's own capture folder, when
/// `OSC_BRINGUP` names the checkout: the committed bursts are its files.
#[test]
fn the_seated_pool_matches_the_record_on_the_full_capture() {
    let Some(root) = std::env::var_os("OSC_BRINGUP") else {
        println!("OSC_BRINGUP unset: skipped");
        return;
    };
    let full = PathBuf::from(root).join("captures/ident-slice2/B/seated");
    for seat in SEATS {
        for (name, _) in GROUPS {
            for k in 0..5 {
                let rel = format!("{seat}/{name}/burst-{k}.csv");
                assert_eq!(
                    std::fs::read(full.join(&rel)).expect(&rel),
                    std::fs::read(path("testdata/seated").join(&rel)).expect(&rel),
                    "{rel} is the recorded file"
                );
            }
        }
    }
    let r = pooled(&full);
    assert!((r / R_RECORD_OHM - 1.0).abs() < 0.01, "pooled R {r:.4}");
}
