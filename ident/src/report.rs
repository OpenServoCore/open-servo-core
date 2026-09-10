//! The human-readable identification report: per-experiment fit tables and
//! the encoded write set with quantization and saturation called out. Plain
//! text, every section optional - a partial run renders what it has and
//! says "skipped" for the rest.

use core::fmt::Write as _;

use crate::exp::bias::BiasResult;
use crate::exp::breakaway::BreakawayResult;
use crate::exp::inertia::InertiaResult;
use crate::exp::ladder::LadderResult;
use crate::exp::resistance::ResistanceResult;
use crate::exp::rl::RlResult;
use crate::gains::{EncodedGains, GainSet};

/// The board-D rig's hand-eyeballed seeds (kernel band), for the comparison
/// column. b_i 655 predates both coupling rescales (it was inert under the
/// original shift-16 form) and i_ki was eyeballed ~40x low - the rendered
/// note says so instead of bending any formula toward them.
const HAND_SEEDS: [(&str, u16); 8] = [
    ("r_q12", 13800),
    ("recip_ke_q", 5184),
    ("ke_vpc_q", 809),
    ("b_i_q313", 655),
    ("i_kp_q88", 863),
    ("i_ki_q412", 205),
    ("fric_fc_counts", 20),
    ("fric_fv_q016", 66),
];

/// Everything the report can show; every section is optional.
#[derive(Default)]
pub struct ReportInputs<'a> {
    pub bias: Option<&'a BiasResult>,
    pub resistance: Option<&'a ResistanceResult>,
    pub rl: Option<&'a RlResult>,
    pub breakaway: Option<&'a BreakawayResult>,
    pub ladder: Option<&'a LadderResult>,
    pub inertia: Option<&'a InertiaResult>,
    pub gains: Option<(&'a GainSet, &'a EncodedGains)>,
}

fn opt(v: Option<f64>) -> String {
    match v {
        Some(v) => format!("{v:.4}"),
        None => "-".into(),
    }
}

pub fn render(r: &ReportInputs<'_>) -> String {
    let mut s = String::new();
    let _ = writeln!(s, "identification report");
    let _ = writeln!(s, "=====================");

    let _ = writeln!(s, "\n[E0 bias]");
    match r.bias {
        Some(b) => {
            let _ = writeln!(s, "  sigma_theta   {:.3} counts (n={})", b.sigma_theta, b.n);
            let _ = writeln!(s, "  i_noise       {:.3} counts", b.i_noise);
            let _ = writeln!(s, "  i_bias_delta  {:.3} counts", b.i_bias_delta);
            let _ = writeln!(s, "  vbus          {:.1} +/- {:.1}", b.vbus_mean, b.vbus_sd);
        }
        None => {
            let _ = writeln!(s, "  skipped");
        }
    }

    let _ = writeln!(s, "\n[E2 resistance]");
    match r.resistance {
        Some(x) => {
            let _ = writeln!(
                s,
                "  R             {:.4} vcounts/ccount (r2 {:.4}, n={})",
                x.r_vpc, x.r2, x.n
            );
            let _ = writeln!(s, "  fwd/rev       {} / {}", opt(x.r_fwd), opt(x.r_rev));
            let _ = writeln!(s, "  heat drift    {:.5} vpc/s", x.drift_vpc_per_s);
        }
        None => {
            let _ = writeln!(s, "  skipped");
        }
    }

    let _ = writeln!(
        s,
        "\n[E7 winding R/L] (free shaft, duty toggles; NOT the table's R)"
    );
    match r.rl {
        Some(x) => {
            let _ = writeln!(
                s,
                "  R             {:.4} ohm [{:.4}, {:.4}] (origin {:.4}, r2 {:.4}, n={})",
                x.r_ohm, x.r_bracket.0, x.r_bracket.1, x.r_origin_ohm, x.r2, x.transitions
            );
            let _ = writeln!(
                s,
                "                {:.4} vcounts/ccount; rail-referenced {:.4} ohm (adds the bridge)",
                x.r_vpc, x.r_rail_ohm
            );
            let _ = writeln!(
                s,
                "  tau           {:.1} us [{:.1}, {:.1}]  ->  L {:.4} mH [{:.4}, {:.4}]",
                x.tau_us,
                x.tau_bracket.0,
                x.tau_bracket.1,
                x.l_henries * 1e3,
                x.l_bracket.0 * 1e3,
                x.l_bracket.1 * 1e3
            );
            let _ = writeln!(
                s,
                "  fwd/rev       {} / {}   bias lo/hi {} / {}   up/down {} / {}",
                opt(x.r_fwd),
                opt(x.r_rev),
                opt(x.r_bias_lo),
                opt(x.r_bias_hi),
                opt(x.r_up),
                opt(x.r_down)
            );
            let _ = writeln!(
                s,
                "  supply        {:.3} ohm source{}   v0 {:.3} V   bias {:.1} counts",
                x.src_ohm,
                if x.supply_soft { " (SOFT)" } else { "" },
                x.v0_volts,
                x.bias_counts
            );
            let gates: Vec<String> = x
                .gates
                .iter()
                .map(|g| {
                    format!(
                        "{} {} ({})",
                        if g.pass { "pass" } else { "FAIL" },
                        g.name,
                        g.detail
                    )
                })
                .collect();
            let _ = writeln!(s, "  gates         {}", gates.join(", "));
            let _ = writeln!(
                s,
                "  verdict       {}",
                if x.ok {
                    "gates pass; advisory only - E2 is the table's R"
                } else {
                    "GATES FAILED - numbers unusable"
                }
            );
            for w in &x.warnings {
                let _ = writeln!(s, "  warn: {w}");
            }
        }
        None => {
            let _ = writeln!(s, "  skipped");
        }
    }

    let _ = writeln!(s, "\n[E1 breakaway] (model-derived from R and vbus)");
    match r.breakaway {
        Some(x) => {
            let _ = writeln!(
                s,
                "  duty_bk       fwd {} / rev {}",
                x.duty_bk_fwd.map_or("-".into(), |d| d.to_string()),
                x.duty_bk_rev.map_or("-".into(), |d| d.to_string()),
            );
            let _ = writeln!(
                s,
                "  fric counts   fwd {} / rev {} (asym {})",
                opt(x.fric_fwd_counts),
                opt(x.fric_rev_counts),
                opt(x.asymmetry)
            );
        }
        None => {
            let _ = writeln!(s, "  skipped");
        }
    }

    let _ = writeln!(s, "\n[E3 ladder]");
    match r.ladder {
        Some(x) => {
            let _ = writeln!(
                s,
                "  Ke            {:.5} vcounts per c/s (r2 {:.4}, n={})",
                x.ke.ke_vpc, x.ke.r2, x.ke.n
            );
            for (name, f) in [("fwd", &x.fric_fwd), ("rev", &x.fric_rev)] {
                match f {
                    Some(f) => {
                        let _ = writeln!(
                            s,
                            "  fric {name}      fc {:.2} cc, fv {:.6} cc per c/s (r2 {:.4})",
                            f.fc, f.fv, f.r2
                        );
                    }
                    None => {
                        let _ = writeln!(s, "  fric {name}      -");
                    }
                }
            }
            let _ = writeln!(
                s,
                "  rungs used    {}",
                x.rungs.iter().filter(|r| r.used).count()
            );
            for w in &x.warnings {
                let _ = writeln!(s, "  warn: {w}");
            }
        }
        None => {
            let _ = writeln!(s, "  skipped");
        }
    }

    let _ = writeln!(s, "\n[E4 inertia]");
    match r.inertia {
        Some(x) => {
            if let Some(d) = &x.b_direct {
                let _ = writeln!(s, "  B direct      {:.5} (r2 {:.4}, n={})", d.b, d.r2, d.n);
            }
            if let Some(e) = &x.b_exp {
                let _ = writeln!(
                    s,
                    "  B exp-rise    {:.5} (spread {:.3}, steps={})",
                    e.b,
                    e.spread,
                    e.steps.len()
                );
            }
            let _ = writeln!(
                s,
                "  B best        {:.5} -> j_ff {:.2} (tel steps {})",
                x.b_best, x.j_ff, x.tel_steps
            );
            for w in &x.warnings {
                let _ = writeln!(s, "  warn: {w}");
            }
        }
        None => {
            let _ = writeln!(s, "  skipped");
        }
    }

    let _ = writeln!(s, "\n[gains]");
    match r.gains {
        Some((g, e)) => {
            let _ = writeln!(
                s,
                "  projected omega noise {:.1} c/s (l2 * sigma_theta)",
                g.omega_noise_cps
            );
            let _ = writeln!(
                s,
                "  {:<22} {:>12} {:>7} {:>8} {:>5}  vs hand seed",
                "field", "physical", "raw", "quant%", "sat"
            );
            for (name, f) in e.fields() {
                let seed = HAND_SEEDS.iter().find(|(n, _)| *n == name).map(|(_, v)| *v);
                let cmp = match seed {
                    Some(sv) if sv != 0 => format!("{:.2}x of {sv}", f.raw as f64 / sv as f64),
                    _ => String::new(),
                };
                let _ = writeln!(
                    s,
                    "  {:<22} {:>12.5} {:>7} {:>7.2}% {:>5}  {}",
                    name,
                    f.physical,
                    f.raw,
                    f.quantization_pct,
                    if f.saturated { "SAT" } else { "" },
                    cmp
                );
            }
            let _ = writeln!(
                s,
                "  note: hand-seed b_i predates the per-tick rescale (was inert) and\n  \
                 the hand i_ki was eyeballed ~40x low - large ratios there are expected."
            );
        }
        None => {
            let _ = writeln!(s, "  skipped");
        }
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exp::rl::RlResult;
    use crate::gains::{BwTargets, PlantParams, encode, synthesize};

    #[test]
    fn rl_section_shows_the_verdict() {
        let mut x = RlResult {
            r_ohm: 4.0,
            r_origin_ohm: 4.01,
            r_rail_ohm: 4.3,
            r_bracket: (3.9, 4.1),
            r_vpc: 2.87,
            v0_volts: 0.01,
            r2: 0.999,
            r_fwd: Some(4.0),
            r_rev: Some(4.02),
            r_bias_lo: Some(3.99),
            r_bias_hi: Some(4.01),
            r_up: Some(3.95),
            r_down: Some(4.05),
            tau_us: 150.0,
            tau_bracket: (145.0, 155.0),
            l_henries: 0.6e-3,
            l_bracket: (0.55e-3, 0.65e-3),
            src_ohm: 0.27,
            supply_soft: false,
            bias_counts: 512.0,
            null_step_counts: Some(0.3),
            transitions: 156,
            gates: vec![crate::exp::rl::Gate {
                name: "null",
                pass: true,
                detail: "0.3 counts of step".into(),
            }],
            ok: true,
            warnings: Vec::new(),
        };
        let s = render(&ReportInputs {
            rl: Some(&x),
            ..Default::default()
        });
        assert!(s.contains("0.6000 mH"), "L missing:\n{s}");
        assert!(s.contains("advisory only"));

        x.ok = false;
        x.supply_soft = true;
        x.gates[0].pass = false;
        let s = render(&ReportInputs {
            rl: Some(&x),
            ..Default::default()
        });
        assert!(s.contains("GATES FAILED"), "verdict missing:\n{s}");
        assert!(s.contains("FAIL null"));
        assert!(s.contains("(SOFT)"));
    }

    #[test]
    fn renders_empty_and_partial_inputs() {
        let all_skipped = render(&ReportInputs::default());
        assert_eq!(all_skipped.matches("skipped").count(), 7);

        let p = PlantParams {
            r_vpc: 3.37,
            ke_vpc: 0.2,
            fc: 20.0,
            fv: 0.001,
            b: 0.1,
            sigma_theta: 1.0,
            l_cd: 3.58e-4,
            tick_hz: 20_100.0,
            f_med: 2_010.0,
        };
        let g = synthesize(&p, &BwTargets::default());
        let e = encode(&g);
        let s = render(&ReportInputs {
            gains: Some((&g, &e)),
            ..Default::default()
        });
        assert!(s.contains("b_i_q313"));
        assert!(s.contains("x of 13800"), "hand-seed column missing:\n{s}");
        assert!(s.contains("eyeballed ~40x low"));
    }
}
