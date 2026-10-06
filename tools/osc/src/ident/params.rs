//! params.json: serde mirrors of the fit results, the synthesized plant,
//! and the encoded write set. The osc-ident types stay serde-free (wasm
//! lib keeps zero deps); these mirrors are the CLI's file format.

use std::path::Path;

use crate::rig::plant::Lut;
use anyhow::{Context, Result};
use osc_ident::exp::anchor::AnchorResult;
use osc_ident::exp::bias::BiasResult;
use osc_ident::exp::breakaway::BreakawayResult;
use osc_ident::exp::held::HeldRun;
use osc_ident::exp::inductance::InductanceResult;
use osc_ident::exp::inertia::{InertiaResult, StoredMotion};
use osc_ident::exp::resistance::ResistanceResult;
use osc_ident::exp::rl::{RlResult, Scales};
use osc_ident::exp::wavefit::WaveRun;
use osc_ident::exp::winding::VoltRun;
use osc_ident::gains::{BwTargets, Encoded, EncodedGains, PlantParams};
use osc_ident::sources::{Source, Winding};
use osc_ident::thermometer;
use osc_ident::units::SenseParams;
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Default)]
pub struct ParamsFile {
    pub bias: Option<BiasJson>,
    pub resistance: Option<ResistanceJson>,
    pub rl: Option<RlJson>,
    pub inductance: Option<InductanceJson>,
    pub breakaway: Option<BreakawayJson>,
    /// The winding the run took off the servo because nothing in it
    /// measured R: the fit's R and L, as they were stored.
    pub stored_winding: Option<StoredWindingJson>,
    /// The Ke and friction line the servo carried into the run: what
    /// inertia reads B against when the ladder declines.
    pub stored_motion: Option<StoredMotionJson>,
    pub ladder: Option<LadderJson>,
    pub inertia: Option<InertiaJson>,
    /// CalibSense scales read off the table - the offline fit's l_cd input.
    pub sense: Option<SenseJson>,
    pub plant: Option<PlantJson>,
    /// The position table the run fitted through: which counts the plant is
    /// in.
    pub pot: Option<PotJson>,
    /// The thermometer's anchor the run read, and the rest temperature the
    /// host paired with it; the fit encodes them into `gains`.
    #[serde(default)]
    pub thermometer: Option<ThermometerJson>,
    #[serde(default)]
    pub gains: Vec<GainJson>,
}

/// The anchor hold as recorded ([`osc_ident::exp::anchor::AnchorResult`])
/// with the ambient the host supplied, degrees C.
#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct ThermometerJson {
    pub ambient_c: f64,
    pub r_vpc: f64,
    pub spread: f64,
    pub n: usize,
    pub i_counts: f64,
    pub duty: f64,
}

impl ThermometerJson {
    pub fn new(a: &AnchorResult, ambient_c: f64) -> Self {
        Self {
            ambient_c,
            r_vpc: a.r_vpc,
            spread: a.spread,
            n: a.n,
            i_counts: a.i_counts,
            duty: a.duty,
        }
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct PotJson {
    pub lut_state: String,
    /// `linearized` while the table was LIVE, else `raw`.
    pub counts: String,
    /// CRC-16/ARC over the effective points, hex.
    pub lut_crc: String,
    pub nonzero_points: usize,
    /// Raw counts of the first and last nonzero point.
    pub band: Option<[u16; 2]>,
}

impl From<&Lut> for PotJson {
    fn from(l: &Lut) -> Self {
        Self {
            lut_state: l.state_name(),
            counts: l.pot().label().into(),
            lut_crc: format!("{:#06x}", l.crc()),
            nonzero_points: l.nonzero(),
            band: l.band().map(|(lo, hi)| [lo, hi]),
        }
    }
}

impl PotJson {
    /// The one line a refit prints about the counts it works in.
    pub fn describe(&self) -> String {
        match self.counts.as_str() {
            "linearized" => format!(
                "pot counts: linearized (lut LIVE, crc {}, {} nonzero calibration points)",
                self.lut_crc, self.nonzero_points
            ),
            _ => format!("pot counts: raw (lut {})", self.lut_state),
        }
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct BiasJson {
    pub sigma_theta: f64,
    pub sigma_raw: f64,
    pub gain: f64,
    pub rest: f64,
    pub tel_n: usize,
    pub pos_mean: f64,
    pub i_noise: f64,
    pub i_bias_delta: f64,
    pub vbus_mean: f64,
    pub vbus_sd: f64,
    pub n: usize,
    pub warnings: Vec<String>,
}

impl From<&BiasResult> for BiasJson {
    fn from(b: &BiasResult) -> Self {
        Self {
            sigma_theta: b.sigma_theta,
            sigma_raw: b.sigma_raw,
            gain: b.gain,
            rest: b.rest,
            tel_n: b.tel_n,
            pos_mean: b.pos_mean,
            i_noise: b.i_noise,
            i_bias_delta: b.i_bias_delta,
            vbus_mean: b.vbus_mean,
            vbus_sd: b.vbus_sd,
            n: b.n,
            warnings: b.warnings.clone(),
        }
    }
}

impl BiasJson {
    pub fn result(&self) -> BiasResult {
        BiasResult {
            sigma_theta: self.sigma_theta,
            sigma_raw: self.sigma_raw,
            gain: self.gain,
            rest: self.rest,
            tel_n: self.tel_n,
            pos_mean: self.pos_mean,
            i_noise: self.i_noise,
            i_bias_delta: self.i_bias_delta,
            vbus_mean: self.vbus_mean,
            vbus_sd: self.vbus_sd,
            n: self.n,
            warnings: self.warnings.clone(),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct ResistanceJson {
    pub r_vpc: f64,
    pub r_fwd: Option<f64>,
    pub r_rev: Option<f64>,
    pub r2: f64,
    pub n: usize,
    pub drift_vpc_per_s: f64,
}

impl From<&ResistanceResult> for ResistanceJson {
    fn from(r: &ResistanceResult) -> Self {
        Self {
            r_vpc: r.r_vpc,
            r_fwd: r.r_fwd,
            r_rev: r.r_rev,
            r2: r.r2,
            n: r.n,
            drift_vpc_per_s: r.drift_vpc_per_s,
        }
    }
}

/// The R/L run as recorded. `ok` is the gate verdict: false means the run
/// may not feed gain synthesis.
#[derive(Serialize, Deserialize, Clone)]
pub struct RlJson {
    pub r_ohm: f64,
    pub r_vpc: f64,
    pub r_bracket: (f64, f64),
    pub r_origin_ohm: f64,
    pub r_rail_ohm: f64,
    pub r2: f64,
    pub v0_volts: f64,
    pub r_fwd: Option<f64>,
    pub r_rev: Option<f64>,
    pub r_bias_lo: Option<f64>,
    pub r_bias_hi: Option<f64>,
    pub r_up: Option<f64>,
    pub r_down: Option<f64>,
    pub tau_us: f64,
    pub tau_bracket: (f64, f64),
    pub l_henries: f64,
    pub l_bracket: (f64, f64),
    pub src_ohm: f64,
    pub supply_soft: bool,
    pub null_step_counts: Option<f64>,
    pub transitions: usize,
    pub gates: Vec<(String, bool, String)>,
    pub ok: bool,
}

impl From<&RlResult> for RlJson {
    fn from(x: &RlResult) -> Self {
        Self {
            r_ohm: x.r_ohm,
            r_vpc: x.r_vpc,
            r_bracket: x.r_bracket,
            r_origin_ohm: x.r_origin_ohm,
            r_rail_ohm: x.r_rail_ohm,
            r2: x.r2,
            v0_volts: x.v0_volts,
            r_fwd: x.r_fwd,
            r_rev: x.r_rev,
            r_bias_lo: x.r_bias_lo,
            r_bias_hi: x.r_bias_hi,
            r_up: x.r_up,
            r_down: x.r_down,
            tau_us: x.tau_us,
            tau_bracket: x.tau_bracket,
            l_henries: x.l_henries,
            l_bracket: x.l_bracket,
            src_ohm: x.src_ohm,
            supply_soft: x.supply_soft,
            null_step_counts: x.null_step_counts,
            transitions: x.transitions,
            gates: x
                .gates
                .iter()
                .map(|g| (g.name.to_string(), g.pass, g.detail.clone()))
                .collect(),
            ok: x.ok,
        }
    }
}

/// The high-rate burst run as recorded. `promoted` is the verdict that
/// decides whether the gains used it (either route); `gates`, `ok` and
/// `blocking` are the free shaft's, its trace gates and the waveform fit's
/// (`wave`). The ON-window, pairs and regression numbers are diagnostics,
/// checked in `checks` and deciding nothing. L_ripple and L_env are
/// different quantities, not two estimates of one - see osc-ident's
/// `exp::inductance`. Fields after `ok` postdate the voltage channels and
/// default when an older recording is refitted.
#[derive(Serialize, Deserialize, Clone)]
pub struct InductanceJson {
    pub l_ripple_h: f64,
    pub l_ripple_bracket: (f64, f64),
    pub l_off_h: Option<f64>,
    pub l_env_h: f64,
    pub l_env_bracket: (f64, f64),
    pub tau_us: f64,
    pub tau_bracket: (f64, f64),
    pub tau_off_us: f64,
    pub r_pair_ohm: Option<f64>,
    pub r_pair_bracket: Option<(f64, f64)>,
    pub r_asym_ohm: f64,
    pub v0_volts: f64,
    pub v0_measured: bool,
    /// (step duty as a fraction of full scale, L_ripple in henries).
    pub l_by_duty: Vec<(f64, f64)>,
    pub ripple_spread: f64,
    pub env_spread: f64,
    pub bias_counts: f64,
    pub settle_us: f64,
    pub window_samples: f64,
    pub cadence_samples: f64,
    pub rest_captures: usize,
    pub hold_captures: usize,
    pub gates: Vec<(String, bool, String)>,
    pub ok: bool,
    #[serde(default)]
    pub promoted: bool,
    #[serde(default)]
    pub blocking: Vec<String>,
    #[serde(default)]
    pub tau_cb_us: f64,
    #[serde(default)]
    pub shunt_on_share: Option<f64>,
    #[serde(default)]
    pub l_ripple_ok: bool,
    #[serde(default)]
    pub r_pair_cb_ohm: Option<f64>,
    #[serde(default)]
    pub src_prearm_ohm: Option<f64>,
    #[serde(default)]
    pub volts: VoltsJson,
    /// The route that fed the gains; None when both declined.
    #[serde(default)]
    pub route: Option<String>,
    #[serde(default)]
    pub held: HeldJson,
    #[serde(default)]
    pub wave: Option<WaveJson>,
    #[serde(default)]
    pub checks: Vec<(String, bool, String)>,
}

/// The whole-waveform fit of the from-rest captures: the line, where it
/// was read, and the captures set aside.
#[derive(Serialize, Deserialize, Clone)]
pub struct WaveJson {
    /// Winding R, its formal standard error, L and V0: the fitted line.
    pub r_ohm: f64,
    pub r_sd_ohm: f64,
    pub l_h: f64,
    pub v0_volts: f64,
    pub tau_a_us: f64,
    pub delta_us: f64,
    pub rms_counts: f64,
    pub captures: usize,
    /// The servo's current limit the line was read at, amps; V/I there is
    /// what r_q12 and the stall-safe duties take, the slope there with the
    /// bridge what the current loop takes. Duty x rail over current.
    pub i_lim_a: Option<f64>,
    pub v_over_i_lim_ohm: Option<f64>,
    pub slope_lim_ohm: Option<f64>,
    pub r_median_ohm: f64,
    pub r_spread: f64,
    /// (capture index, duty, forward, pos, R ohms) of each capture set
    /// aside.
    pub set_aside: Vec<(usize, f64, bool, u16, f64)>,
    /// The even and the odd captures' slope R, and their V/I at the limit,
    /// which the split-halves gate compares.
    pub halves_ohm: Option<(f64, f64)>,
    pub halves_v_over_i_ohm: Option<(f64, f64)>,
    /// Captures whose half window has an edge no terminal sample caught.
    pub half_blind: usize,
    pub on_loss_us: f64,
    pub rail_v: f64,
    /// Fitted on the terminals' difference: the ON line is the
    /// difference's, the low side and the brake measured, and the driven
    /// terminal's own R and V/I at the limit beside it.
    pub both_terminals: bool,
    pub v_on_open: f64,
    pub z_on_ohm: f64,
    pub on_drop_ohm: f64,
    pub off_ohm: f64,
    pub lo_side_ohm: Option<f64>,
    pub driven_r_ohm: Option<f64>,
    pub driven_v_over_i_lim_ohm: Option<f64>,
    pub tau_us: f64,
    pub emf_end_v: f64,
    /// Gates with nothing to judge, and why.
    pub skipped: Vec<(String, String)>,
}

impl From<&WaveRun> for WaveJson {
    fn from(w: &WaveRun) -> Self {
        Self {
            r_ohm: w.fit.r_ohm,
            r_sd_ohm: w.fit.r_sd_ohm,
            l_h: w.fit.l_h,
            v0_volts: w.fit.v0_volts,
            tau_a_us: w.fit.tau_a_us,
            delta_us: w.fit.delta_us,
            rms_counts: w.fit.rms_counts,
            captures: w.fit.captures,
            i_lim_a: w.at_limit.map(|a| a.i_a),
            v_over_i_lim_ohm: w.at_limit.map(|a| a.v_over_i_ohm),
            slope_lim_ohm: w.at_limit.map(|a| a.slope_ohm),
            r_median_ohm: w.r_median_ohm,
            r_spread: w.r_spread,
            set_aside: w
                .set_aside()
                .map(|c| (c.index, c.duty, c.forward, c.pos, c.r_ohm))
                .collect(),
            halves_ohm: w.halves.map(|(a, b)| (a.r_ohm, b.r_ohm)),
            halves_v_over_i_ohm: w.halves_v_over_i(),
            half_blind: w.half_blind,
            on_loss_us: w.on_loss_us,
            rail_v: w.rail_v,
            both_terminals: w.both_terminals,
            v_on_open: w.v_on_open,
            z_on_ohm: w.z_on_ohm,
            on_drop_ohm: w.on_drop_ohm,
            off_ohm: w.off_ohm,
            lo_side_ohm: w.lo_side_ohm,
            driven_r_ohm: w.driven.map(|d| d.0.r_ohm),
            driven_v_over_i_lim_ohm: w.driven.and_then(|d| d.1).map(|a| a.v_over_i_ohm),
            tau_us: w.tau_us,
            emf_end_v: w.emf_end_v,
            skipped: w
                .skipped
                .iter()
                .map(|(n, why)| (n.to_string(), why.clone()))
                .collect(),
        }
    }
}

/// The held-at-a-stop route as recorded.
#[derive(Serialize, Deserialize, Clone, Default)]
pub struct HeldJson {
    pub captures: usize,
    /// (drive sign toward the stop, pos, hold duty, hold current in amps).
    pub seats: Vec<(i8, u16, f64, Option<f64>)>,
    pub rest_zeroed: bool,
    pub r_ohm: Option<f64>,
    pub l_h: Option<f64>,
    pub tau_us: Option<f64>,
    pub c_volts: Option<f64>,
    pub rows: usize,
    pub hold_rows: usize,
    pub l_spread: f64,
    pub gates: Vec<(String, bool, String)>,
    pub promoted: bool,
}

impl From<&HeldRun> for HeldJson {
    fn from(h: &HeldRun) -> Self {
        Self {
            captures: h.captures,
            seats: h
                .seats
                .iter()
                .map(|s| (s.dir, s.pos, s.hold_duty, s.i_hold_a))
                .collect(),
            rest_zeroed: h.rest_zeroed,
            r_ohm: h.reg.map(|g| g.r_ohm),
            l_h: h.reg.map(|g| g.l_h),
            tau_us: h.reg.map(|g| g.tau_us),
            c_volts: h.reg.map(|g| g.c_volts),
            rows: h.reg.map_or(0, |g| g.n),
            hold_rows: h.hold_rows,
            l_spread: h.l_spread,
            gates: h
                .gates
                .iter()
                .map(|g| (g.name.to_string(), g.pass, g.detail.clone()))
                .collect(),
            promoted: h.promotable(),
        }
    }
}

/// The run's voltage source and its two R routes, winding referenced.
#[derive(Serialize, Deserialize, Clone, Default)]
pub struct VoltsJson {
    /// None when the pre-arm rail stood in for a measurement.
    pub route: Option<String>,
    pub captures: usize,
    pub r_pair_ohm: Option<f64>,
    pub r_pair_bracket: Option<(f64, f64)>,
    pub r_reg_ohm: Option<f64>,
    pub l_env_h: Option<f64>,
    pub tau_reg_us: Option<f64>,
    pub c_volts: Option<f64>,
    pub emf_v_per_ms: Option<f64>,
    pub periods: usize,
    pub z_src_ohm: Option<f64>,
    pub rail_open_v: Option<f64>,
    pub off_median_v: Option<f64>,
    pub off_min_v: Option<f64>,
    pub body_diode: Option<bool>,
    pub on_sag_v: Option<f64>,
    pub on_flat: Option<bool>,
    pub duty_ratio: Option<f64>,
}

impl From<&VoltRun> for VoltsJson {
    fn from(v: &VoltRun) -> Self {
        let d = v.diag.as_ref();
        Self {
            route: v.route.map(|r| r.as_str().to_string()),
            captures: v.captures,
            r_pair_ohm: v.r_pair_ohm,
            r_pair_bracket: v.r_pair_bracket,
            r_reg_ohm: v.reg.map(|g| g.r_ohm),
            l_env_h: v.reg.map(|g| g.l_h),
            tau_reg_us: v.reg.map(|g| g.tau_us),
            c_volts: v.reg.map(|g| g.c_volts),
            emf_v_per_ms: v.reg.and_then(|g| g.emf_v_per_ms),
            periods: v.reg.map_or(0, |g| g.n),
            z_src_ohm: d.and_then(|d| d.z_src_ohm),
            rail_open_v: d.and_then(|d| d.rail_open_v),
            off_median_v: d.and_then(|d| d.off_median_v),
            off_min_v: d.and_then(|d| d.off_min_v),
            body_diode: d.map(|d| d.body_diode),
            on_sag_v: d.and_then(|d| d.on_sag_v),
            on_flat: d.and_then(|d| d.on_flat),
            duty_ratio: d.and_then(|d| d.duty_ratio),
        }
    }
}

impl From<&InductanceResult> for InductanceJson {
    fn from(x: &InductanceResult) -> Self {
        Self {
            l_ripple_h: x.l_ripple_h,
            l_ripple_bracket: x.l_ripple_bracket,
            l_off_h: x.l_off_h,
            l_env_h: x.l_env_h,
            l_env_bracket: x.l_env_bracket,
            tau_us: x.tau_us,
            tau_bracket: x.tau_bracket,
            tau_off_us: x.tau_off_us,
            r_pair_ohm: x.r_pair_ohm,
            r_pair_bracket: x.r_pair_bracket,
            r_asym_ohm: x.r_asym_ohm,
            v0_volts: x.v0_volts,
            v0_measured: x.v0_measured,
            l_by_duty: x.l_by_duty.clone(),
            ripple_spread: x.ripple_spread,
            env_spread: x.env_spread,
            bias_counts: x.bias_counts,
            settle_us: x.settle_us,
            window_samples: x.window_samples,
            cadence_samples: x.cadence_samples,
            rest_captures: x.rest_captures,
            hold_captures: x.hold_captures,
            gates: x
                .gates
                .iter()
                .map(|g| (g.name.to_string(), g.pass, g.detail.clone()))
                .collect(),
            ok: x.ok,
            promoted: x.promotable(),
            blocking: x.blocking().iter().map(|b| b.to_string()).collect(),
            tau_cb_us: x.tau_cb_us,
            shunt_on_share: x.shunt_on_share,
            l_ripple_ok: x.l_ripple_ok,
            r_pair_cb_ohm: x.r_pair_cb_ohm,
            src_prearm_ohm: x.src_prearm_ohm,
            volts: VoltsJson::from(&x.volts),
            route: x.route().map(|r| r.as_str().to_string()),
            held: HeldJson::from(&x.held),
            wave: x.wave.as_ref().ok().map(WaveJson::from),
            checks: x
                .checks
                .iter()
                .map(|g| (g.name.to_string(), g.pass, g.detail.clone()))
                .collect(),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct BreakawayJson {
    pub duty_bk_fwd: Option<i16>,
    pub duty_bk_rev: Option<i16>,
    pub fric_fwd_counts: Option<f64>,
    pub fric_rev_counts: Option<f64>,
    pub model_derived: bool,
    pub asymmetry: Option<f64>,
}

impl From<&BreakawayResult> for BreakawayJson {
    fn from(b: &BreakawayResult) -> Self {
        Self {
            duty_bk_fwd: b.duty_bk_fwd,
            duty_bk_rev: b.duty_bk_rev,
            fric_fwd_counts: b.fric_fwd_counts,
            fric_rev_counts: b.fric_rev_counts,
            model_derived: b.model_derived,
            asymmetry: b.asymmetry,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct LadderJson {
    pub ke_vpc: f64,
    pub ke_r2: f64,
    pub fc_fwd: Option<f64>,
    pub fv_fwd: Option<f64>,
    pub fc_rev: Option<f64>,
    pub fv_rev: Option<f64>,
    pub rungs_used: usize,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct InertiaJson {
    /// The current decay's B, what the gains take.
    pub b_best: f64,
    pub b_decay_spread: f64,
    pub b_decay_steps: usize,
    pub b_climb: Option<f64>,
    /// Where its Ke and friction came from.
    pub priors: String,
    pub b_direct: Option<f64>,
    pub b_exp: Option<f64>,
    pub j_ff: f64,
    pub tel_steps: usize,
}

impl InertiaJson {
    pub fn new(r: &InertiaResult, priors: &str) -> Self {
        Self {
            b_best: r.b_best,
            b_decay_spread: r.b_decay.spread,
            b_decay_steps: r.b_decay.steps.len(),
            b_climb: r.b_climb.map(|c| c.b),
            priors: priors.into(),
            b_direct: r.b_direct.as_ref().map(|d| d.b),
            b_exp: r.b_exp.as_ref().map(|e| e.b),
            j_ff: r.j_ff,
            tel_steps: r.tel_steps,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct SenseJson {
    pub shunt_r_mohm: u16,
    pub gain_milli: u16,
    pub vmotor_div_top: u16,
    pub vmotor_div_bot: u16,
    pub tick_hz: u16,
    /// The R/L band's additions; a run recorded before it has them zero,
    /// which the scales below reject rather than guess around.
    #[serde(default)]
    pub vdd_mv: u16,
    #[serde(default)]
    pub vbus_div_top_ohm: u16,
    #[serde(default)]
    pub vbus_div_bot_ohm: u16,
}

impl SenseJson {
    pub fn params(&self) -> SenseParams {
        SenseParams {
            shunt_r_mohm: self.shunt_r_mohm,
            gain_milli: self.gain_milli,
            vmotor_div_top: self.vmotor_div_top,
            vmotor_div_bot: self.vmotor_div_bot,
            vdd_mv: self.vdd_mv,
            tick_hz: self.tick_hz,
        }
    }

    pub fn scales(&self) -> Option<Scales> {
        Scales::from_sense(&self.params(), self.vbus_div_top_ohm, self.vbus_div_bot_ohm)
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct PlantJson {
    /// What r_q12 takes: the winding between the terminal taps.
    pub r_vpc: f64,
    /// The current loop's plant R, synthesized into i_ki. Zero means
    /// absent: `ident synth` then closes the loop on `r_vpc`. The fit path
    /// always writes it.
    #[serde(default)]
    pub r_loop_vpc: f64,
    pub ke_vpc: f64,
    pub fc: f64,
    pub fv: f64,
    pub b: f64,
    pub sigma_theta: f64,
    /// Zero means absent: `ident synth` then derives it from `l_henries`
    /// through the sense block. The fit path always writes it.
    #[serde(default)]
    pub l_cd: f64,
    pub tick_hz: f64,
    pub f_med: f64,
    /// The targets the gains were synthesized against. Zero means absent,
    /// which `ident synth` fills from the CLI flags.
    #[serde(default)]
    pub f_ci: f64,
    #[serde(default)]
    pub f_cv: f64,
    #[serde(default)]
    pub f_cp: f64,
    #[serde(default)]
    pub f_o: f64,
    /// Which experiment each input came from.
    #[serde(default)]
    pub r_source: String,
    /// `r_vpc` and `r_loop_vpc` in ohms when the burst supplied them.
    #[serde(default)]
    pub r_ohm: Option<f64>,
    #[serde(default)]
    pub r_loop_ohm: Option<f64>,
    /// Which R went where, in words.
    #[serde(default)]
    pub winding_use: String,
    #[serde(default)]
    pub l_source: String,
    #[serde(default)]
    pub l_henries: f64,
    #[serde(default)]
    pub sigma_source: String,
}

impl PlantJson {
    pub fn new(p: &PlantParams, t: &BwTargets, w: &Winding, sigma_from: &str) -> Self {
        let ohm = |vpc: f64| w.r_ohm.filter(|_| w.r_vpc > 0.0).map(|r| r * vpc / w.r_vpc);
        let (r_ohm, r_loop_ohm) = (ohm(p.r_vpc), ohm(p.r_loop_vpc));
        let winding_use = match (w.r_from, r_ohm, w.r_ohm, r_loop_ohm) {
            (Source::Burst, Some(r), Some(plan), Some(lo)) => format!(
                "r_q12 takes r_vpc, the waveform's winding R between the terminal taps \
                 ({r:.3} ohm); every stall-safe duty takes V/I at the current limit ({plan:.3} \
                 ohm); the current loop's i_ki takes r_loop_vpc, the V-I line's slope with the \
                 bridge ({lo:.3} ohm); i_kp takes L"
            ),
            _ => "r_q12, every stall-safe duty and the current loop's i_ki take r_vpc; i_kp \
                  takes L"
                .into(),
        };
        Self {
            r_vpc: p.r_vpc,
            r_loop_vpc: p.r_loop_vpc,
            ke_vpc: p.ke_vpc,
            fc: p.fc,
            fv: p.fv,
            b: p.b,
            sigma_theta: p.sigma_theta,
            l_cd: p.l_cd,
            tick_hz: p.tick_hz,
            f_med: p.f_med,
            f_ci: t.f_ci,
            f_cv: t.f_cv,
            f_cp: t.f_cp,
            f_o: t.f_o,
            r_source: w.r_from.as_str().into(),
            r_ohm,
            r_loop_ohm,
            winding_use,
            l_source: w.l_from.as_str().into(),
            l_henries: w.l_h,
            sigma_source: sigma_from.into(),
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct StoredMotionJson {
    pub ke_vpc: f64,
    pub fc: f64,
    pub fv: f64,
}

impl From<&StoredMotion> for StoredMotionJson {
    fn from(m: &StoredMotion) -> Self {
        Self {
            ke_vpc: m.ke_vpc,
            fc: m.fc,
            fv: m.fv,
        }
    }
}

impl StoredMotionJson {
    pub fn motion(&self) -> StoredMotion {
        StoredMotion {
            ke_vpc: self.ke_vpc,
            fc: self.fc,
            fv: self.fv,
        }
    }
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct StoredWindingJson {
    pub r_vpc: f64,
    pub r_loop_vpc: f64,
    pub r_ohm: Option<f64>,
    pub l_henries: f64,
}

impl From<&Winding> for StoredWindingJson {
    fn from(w: &Winding) -> Self {
        Self {
            r_vpc: w.r_vpc,
            r_loop_vpc: w.r_loop_vpc,
            r_ohm: w.r_ohm,
            l_henries: w.l_h,
        }
    }
}

impl StoredWindingJson {
    pub fn winding(&self) -> Winding {
        Winding {
            r_ohm: self.r_ohm,
            r_vpc: self.r_vpc,
            r_taps_vpc: self.r_vpc,
            r_from: Source::Stored,
            r_loop_vpc: self.r_loop_vpc,
            l_h: self.l_henries,
            l_from: Source::Stored,
        }
    }
}

#[derive(Serialize, Deserialize, Clone)]
pub struct GainJson {
    pub name: String,
    pub physical: f64,
    pub raw: u16,
    pub quantization_pct: f64,
    pub saturated: bool,
}

impl GainJson {
    pub fn set(e: &EncodedGains) -> Vec<Self> {
        Self::of(&e.fields())
    }

    /// The thermometer anchor's four fields, in table order.
    pub fn anchor(a: &thermometer::Anchor) -> Vec<Self> {
        Self::of(&a.fields())
    }

    fn of(fields: &[(&str, Encoded)]) -> Vec<Self> {
        fields
            .iter()
            .map(|(name, f)| Self {
                name: name.to_string(),
                physical: f.physical,
                raw: f.raw,
                quantization_pct: f.quantization_pct,
                saturated: f.saturated,
            })
            .collect()
    }
}

impl ParamsFile {
    pub fn save(&self, path: &Path) -> Result<()> {
        std::fs::write(path, serde_json::to_string_pretty(self)?)
            .with_context(|| format!("write {}", path.display()))
    }

    pub fn load(path: &Path) -> Result<Self> {
        serde_json::from_str(
            &std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?,
        )
        .with_context(|| format!("parse {}", path.display()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn params_json_round_trips() {
        let p = ParamsFile {
            resistance: Some(ResistanceJson {
                r_vpc: 3.37,
                r_fwd: Some(3.36),
                r_rev: Some(3.38),
                r2: 0.9995,
                n: 140,
                drift_vpc_per_s: 0.001,
            }),
            gains: vec![GainJson {
                name: "r_q12".into(),
                physical: 3.37,
                raw: 13804,
                quantization_pct: 0.002,
                saturated: false,
            }],
            ..Default::default()
        };
        let dir = std::env::temp_dir().join(format!("ident-params-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("params.json");
        p.save(&path).unwrap();
        let back = ParamsFile::load(&path).unwrap();
        assert_eq!(back.resistance.unwrap().r_vpc, 3.37);
        assert_eq!(back.gains[0].raw, 13804);
        assert!(back.bias.is_none());
    }
}
