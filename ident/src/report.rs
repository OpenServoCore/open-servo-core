//! The human-readable identification report: per-experiment fit tables and
//! the encoded write set with quantization and saturation called out. Plain
//! text, every section optional - a partial run renders what it has and
//! says "skipped" for the rest.

use core::fmt::Write as _;

use crate::exp::bias::BiasResult;
use crate::exp::breakaway::BreakawayResult;
use crate::exp::held::HeldRun;
use crate::exp::inductance::{BurstRoute, InductanceResult};
use crate::exp::inertia::InertiaResult;
use crate::exp::ladder::LadderResult;
use crate::exp::resistance::ResistanceResult;
use crate::exp::rl::RlResult;
use crate::gains::{EncodedGains, GainSet, PlantParams};
use crate::sources::{Source, Winding};

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
    pub inductance: Option<&'a InductanceResult>,
    pub breakaway: Option<&'a BreakawayResult>,
    pub ladder: Option<&'a LadderResult>,
    pub inertia: Option<&'a InertiaResult>,
    pub gains: Option<(&'a GainSet, &'a EncodedGains)>,
    /// The plant the gains were synthesized from and where its winding
    /// terms and noise floor came from.
    pub plant: Option<PlantInputs<'a>>,
}

#[derive(Copy, Clone)]
pub struct PlantInputs<'a> {
    pub plant: &'a PlantParams,
    pub winding: &'a Winding,
    pub sigma_from: Source,
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
        None if r
            .plant
            .is_some_and(|p| matches!(p.winding.r_from, Source::Burst | Source::BurstHeld)) =>
        {
            let _ = writeln!(s, "  not run: E8 supplied R");
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

    let _ = writeln!(s, "\n[E8 winding R/L] (high-rate shunt burst)");
    match r.inductance {
        Some(x) => render_e8(&mut s, x),
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
    if let Some(p) = r.plant {
        let w = p.winding;
        let _ = writeln!(s, "  plant inputs");
        let _ = writeln!(
            s,
            "    r_vpc        {:>10.4} vcounts/ccount{}  {}",
            p.plant.r_vpc,
            w.r_ohm.map_or(String::new(), |r| format!(" ({r:.3} ohm)")),
            w.r_from.as_str()
        );
        let _ = writeln!(
            s,
            "    l            {:>10.4} mH                {}",
            w.l_h * 1e3,
            w.l_from.as_str()
        );
        let _ = writeln!(
            s,
            "    ke, fc, fv   {:>10.5} {:.2} {:.6}   {}",
            p.plant.ke_vpc,
            p.plant.fc,
            p.plant.fv,
            Source::Ladder.as_str()
        );
        let _ = writeln!(
            s,
            "    b            {:>10.5}                   {}",
            p.plant.b,
            Source::Inertia.as_str()
        );
        let _ = writeln!(
            s,
            "    sigma_theta  {:>10.3} counts            {}",
            p.plant.sigma_theta,
            p.sigma_from.as_str()
        );
    }
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

fn gate_line(gates: &[crate::exp::rl::Gate]) -> String {
    gates
        .iter()
        .map(|g| {
            format!(
                "{} {} ({})",
                if g.pass { "pass" } else { "FAIL" },
                g.name,
                g.detail
            )
        })
        .collect::<Vec<_>>()
        .join(", ")
}

fn render_held(s: &mut String, h: &HeldRun) {
    let duties: Vec<String> = h
        .duties
        .iter()
        .map(|d| format!("{:.0}", d * 100.0))
        .collect();
    let _ = writeln!(
        s,
        "  held route    {} seated captures stepping to {}%; voltage {}; taps {}",
        h.captures,
        duties.join("/"),
        h.route.map_or("not measured", |r| r.as_str()),
        if h.rest_zeroed {
            "zeroed on the rest reference"
        } else {
            "not zeroed (no rest reference: their split rides in c)"
        }
    );
    for seat in &h.seats {
        let _ = writeln!(
            s,
            "  seat          {} stop at pos {}, hold {:.0}% draws {} A ({} captures)",
            if seat.dir < 0 { "low" } else { "high" },
            seat.pos,
            seat.hold_duty * 100.0,
            seat.i_hold_a.map_or("-".into(), |i| format!("{i:.3}")),
            seat.captures
        );
    }
    match h.reg {
        Some(g) => {
            let _ = writeln!(
                s,
                "  R regression  {:.3} ohm   L_env {:.4} mH   tau {:.1} us   c {:.3} V  \
                 ({} rows, {} hold levels)",
                g.r_ohm,
                g.l_h * 1e3,
                g.tau_us,
                g.c_volts,
                g.n,
                h.hold_rows
            );
        }
        None => {
            let _ = writeln!(s, "  R regression  - (degenerate)");
        }
    }
    let _ = writeln!(s, "  held gates    {}", gate_line(&h.gates));
}

fn render_e8(s: &mut String, x: &InductanceResult) {
    if x.held.captures > 0 {
        render_held(s, &x.held);
    }
    if x.rest_captures + x.hold_captures > 0 {
        render_free(s, x);
    }
    let _ = writeln!(
        s,
        "  verdict       {}",
        match x.route() {
            Some(BurstRoute::Held) => {
                "PROMOTED via the held route - the gains take R and L_env from its regression"
                    .to_string()
            }
            Some(BurstRoute::Free) => format!(
                "PROMOTED via the free-shaft route{} - the gains take R and L_env from its \
                 regression",
                match x.held.captures {
                    0 => String::new(),
                    _ => format!(" (held declined: {})", x.held.blocking().join(", ")),
                }
            ),
            None => format!(
                "declined (held: {}; free: {}) - E2 supplies R, L stays at the default",
                match x.held.captures {
                    0 => "no seated captures".to_string(),
                    _ => x.held.blocking().join(", "),
                },
                x.blocking().join(", ")
            ),
        }
    );
    for w in &x.warnings {
        let _ = writeln!(s, "  warn: {w}");
    }
}

fn render_free(s: &mut String, x: &InductanceResult) {
    let v = &x.volts;
    let _ = writeln!(
        s,
        "  free shaft    {} from rest, {} from a hold",
        x.rest_captures, x.hold_captures
    );
    let _ = writeln!(
        s,
        "  voltage       {}",
        match v.route {
            Some(r) => format!(
                "measured in the burst: {} ({} captures)",
                r.as_str(),
                v.captures
            ),
            None => format!(
                "pre-arm rail less an estimated bridge drop, not measured ({} captures)",
                v.captures
            ),
        }
    );
    let _ = writeln!(
        s,
        "  R pairs       {} ohm from {} pair(s){}  settled asymptotes against the settled mean V",
        v.r_pair_ohm.map_or("-".into(), |r| format!("{r:.3}")),
        v.pairs.len(),
        v.r_pair_bracket
            .map(|(lo, hi)| format!(" [{lo:.3}, {hi:.3}]"))
            .unwrap_or_default()
    );
    match v.reg {
        Some(g) => {
            let _ = writeln!(
                s,
                "  R regression  {:.3} ohm   L_env {:.4} mH   tau {:.1} us   c {:.3} V{}  ({} periods)",
                g.r_ohm,
                g.l_h * 1e3,
                g.tau_us,
                g.c_volts,
                g.emf_v_per_ms
                    .map(|e| format!("   back-EMF {e:+.3} V/ms (time term earned its place)"))
                    .unwrap_or_default(),
                g.n
            );
        }
        None => {
            let _ = writeln!(s, "  R regression  - (degenerate)");
        }
    }
    let ripple = if x.l_ripple_ok {
        format!(
            "{:.4} mH [{:.4}, {:.4}]",
            x.l_ripple_h * 1e3,
            x.l_ripple_bracket.0 * 1e3,
            x.l_ripple_bracket.1 * 1e3
        )
    } else {
        format!("declined ({:.4} mH recorded)", x.l_ripple_h * 1e3)
    };
    let _ = writeln!(
        s,
        "  L ripple      {ripple}  incremental, one ON window (~25 us); brake decay {} mH",
        x.l_off_h.map_or("-".into(), |v| format!("{:.4}", v * 1e3))
    );
    let _ = writeln!(
        s,
        "  envelope      tau {:.1} us [{:.1}, {:.1}] charge balance; ON window {:.1} us, tau_off {:.1} us",
        x.tau_cb_us, x.tau_cb_bracket.0, x.tau_cb_bracket.1, x.tau_us, x.tau_off_us
    );
    let _ = writeln!(
        s,
        "  shunt charge  shunt_on_share {} inside the ON window",
        x.shunt_on_share.map_or("-".into(), |v| format!("{v:.3}"))
    );
    let _ = writeln!(
        s,
        "  pre-arm rail  R pairs {} ohm ON window, {} charge balance (bridge included); \
         asymptote route {:.3}; L_env {:.4} mH",
        x.r_pair_ohm.map_or("-".into(), |v| format!("{v:.3}")),
        x.r_pair_cb_ohm.map_or("-".into(), |v| format!("{v:.3}")),
        x.r_asym_ohm,
        x.l_env_h * 1e3
    );
    let _ = writeln!(
        s,
        "  supply        pre-arm source {}",
        x.src_prearm_ohm
            .map_or("- (no from-a-hold control)".into(), |z| format!(
                "{z:.3} ohm"
            ))
    );
    if let Some(d) = &v.diag {
        let _ = writeln!(
            s,
            "  in-burst      source {} ohm ({}), open-circuit {} V",
            d.z_src_ohm.map_or("-".into(), |z| format!("{z:.3}")),
            if d.z_from_rail {
                "rail"
            } else {
                "driven terminal less R_hs"
            },
            d.rail_open_v.map_or("-".into(), |v| format!("{v:.3}"))
        );
        let _ = writeln!(
            s,
            "  OFF phase     chopping terminal median {} V, min {} V{}",
            d.off_median_v.map_or("-".into(), |v| format!("{v:+.3}")),
            d.off_min_v.map_or("-".into(), |v| format!("{v:+.3}")),
            if d.body_diode {
                "  BODY DIODE: the terminal sits below ground through the brake"
            } else {
                ""
            }
        );
        let _ = writeln!(
            s,
            "  ON window     {} ({} over {:.1} us); effective duty {} x commanded",
            match d.on_flat {
                Some(true) => "flat",
                Some(false) => "SAGS",
                None => "-",
            },
            d.on_sag_v
                .map_or("-".into(), |v| format!("{:+.1} mV", v * 1e3)),
            d.on_span_us,
            d.duty_ratio.map_or("-".into(), |v| format!("{v:.3}"))
        );
    }
    let _ = writeln!(
        s,
        "  V0            {:.3} V ({})",
        x.v0_volts,
        if x.v0_measured {
            "from the from-a-hold control"
        } else {
            "pre-registered default - no usable control"
        }
    );
    let by_duty: Vec<String> = x
        .l_by_duty
        .iter()
        .map(|(d, l)| format!("{:.0}% {:.3} mH", d * 100.0, l * 1e3))
        .collect();
    let _ = writeln!(s, "  L by duty     {}", by_duty.join(", "));
    let _ = writeln!(
        s,
        "  trace         cadence {:.2} samples/period, {:.1}-sample ON windows, \
         {:.2} us amplifier settling, bias {:.1} counts",
        x.cadence_samples, x.window_samples, x.settle_us, x.bias_counts
    );
    let _ = writeln!(s, "  gates         {}", gate_line(&x.gates));
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

    /// The E8 section off the real bench captures - the CSV, the fit and
    /// the render in one chain, so a column or field rename shows up here.
    #[test]
    fn inductance_section_renders_the_bench_captures() {
        use crate::burst::from_csv;
        use crate::exp::inductance::{FitCfg, fit_captures};
        use crate::exp::rl::Scales;
        use crate::units::SenseParams;

        const BOARD_D: SenseParams = SenseParams {
            shunt_r_mohm: 60,
            gain_milli: 15_000,
            vmotor_div_top: 6_800,
            vmotor_div_bot: 3_300,
            vdd_mv: 3_300,
            tick_hz: 20_100,
        };
        let sc = Scales::from_sense(&BOARD_D, 15_000, 10_000).unwrap();
        let caps: Vec<_> = [
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/rest-to-20.csv"
            )),
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/rest-to-40.csv"
            )),
        ]
        .iter()
        .map(|t| from_csv(t).expect("fixture"))
        .collect();
        let r = fit_captures(&caps, &sc, &FitCfg::default()).expect("fit");
        let s = render(&ReportInputs {
            inductance: Some(&r),
            ..Default::default()
        });
        assert!(s.contains("[E8 winding R/L]"), "{s}");
        assert!(s.contains("L by duty     20%"), "{s}");
        assert!(s.contains("L ripple"), "{s}");
        assert!(s.contains("R regression"), "{s}");
        assert!(s.contains("shunt_on_share"), "{s}");
        assert!(s.contains("pass cadence"), "{s}");
        assert!(
            s.contains("pre-arm rail, not measured") || s.contains("not measured"),
            "{s}"
        );
        assert!(s.contains("verdict       "), "{s}");
    }

    #[test]
    fn the_held_route_renders_its_seat_and_feeds_the_verdict() {
        use crate::burst::from_csv;
        use crate::exp::inductance::{FitCfg, fit_captures};
        use crate::exp::rl::Scales;
        use crate::units::SenseParams;

        let sense = SenseParams {
            shunt_r_mohm: 60,
            gain_milli: 15_000,
            vmotor_div_top: 6_800,
            vmotor_div_bot: 3_300,
            vdd_mv: 3_300,
            tick_hz: 20_100,
        };
        let sc = Scales::from_sense(&sense, 15_000, 10_000).unwrap();
        let caps: Vec<_> = [
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/held/c7-n20-0.csv"
            )),
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/held/c7-n30-0.csv"
            )),
            include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/testdata/burst/held/c7-n40-0.csv"
            )),
        ]
        .iter()
        .map(|t| from_csv(t).expect("fixture"))
        .collect();
        let r = fit_captures(&caps, &sc, &FitCfg::default()).expect("fit");
        let s = render(&ReportInputs {
            inductance: Some(&r),
            ..Default::default()
        });
        assert!(
            s.contains("held route    3 seated captures stepping to 20/30/40%"),
            "{s}"
        );
        assert!(
            s.contains("seat          low stop at pos 122, hold 12% draws"),
            "{s}"
        );
        assert!(s.contains("held gates    pass captures"), "{s}");
        assert!(!s.contains("  free shaft    "), "{s}");
        // one capture per duty leaves the spread nothing to judge
        assert!(s.contains("FAIL l-spread"), "{s}");
        assert!(
            s.contains("verdict       declined (held: l-spread; free: captures"),
            "{s}"
        );
    }

    #[test]
    fn every_winding_input_names_its_source() {
        use crate::sources::{Source, Winding};
        let p = PlantParams {
            r_vpc: 2.87,
            ke_vpc: 0.2,
            fc: 20.0,
            fv: 0.001,
            b: 0.1,
            sigma_theta: 1.0,
            l_cd: 3.58e-4,
            tick_hz: 20_100.0,
            f_med: 2_010.0,
        };
        let burst = Winding {
            r_ohm: Some(4.0),
            r_vpc: 2.87,
            r_from: Source::Burst,
            l_h: 0.6e-3,
            l_from: Source::Burst,
        };
        let s = render(&ReportInputs {
            plant: Some(PlantInputs {
                plant: &p,
                winding: &burst,
                sigma_from: Source::Bias,
            }),
            ..Default::default()
        });
        assert!(s.contains("not run: E8 supplied R"), "{s}");
        assert!(s.contains("(4.000 ohm)  E8 burst"), "{s}");
        assert!(s.contains("0.6000 mH                E8 burst"), "{s}");
        assert!(s.contains("E0 bias"), "{s}");
        let stall = Winding {
            r_ohm: None,
            r_from: Source::StallFallback,
            l_h: 0.5e-3,
            l_from: Source::Default,
            ..burst
        };
        let s = render(&ReportInputs {
            plant: Some(PlantInputs {
                plant: &p,
                winding: &stall,
                sigma_from: Source::Default,
            }),
            ..Default::default()
        });
        assert!(s.contains("E2 fallback"), "{s}");
        assert!(s.contains("0.5000 mH                default"), "{s}");
    }

    #[test]
    fn renders_empty_and_partial_inputs() {
        let all_skipped = render(&ReportInputs::default());
        assert_eq!(all_skipped.matches("skipped").count(), 8);

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
