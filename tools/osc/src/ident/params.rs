//! params.json: serde mirrors of the fit results, the synthesized plant,
//! and the encoded write set. The osc-ident types stay serde-free (wasm
//! lib keeps zero deps); these mirrors are the CLI's file format.

use std::path::Path;

use anyhow::{Context, Result};
use osc_ident::exp::bias::BiasResult;
use osc_ident::exp::breakaway::BreakawayResult;
use osc_ident::exp::resistance::ResistanceResult;
use osc_ident::gains::{BwTargets, Encoded, EncodedGains, PlantParams};
use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Default)]
pub struct ParamsFile {
    pub bias: Option<BiasJson>,
    pub resistance: Option<ResistanceJson>,
    pub breakaway: Option<BreakawayJson>,
    pub ladder: Option<LadderJson>,
    pub inertia: Option<InertiaJson>,
    /// CalibSense scales read off the table - the offline fit's l_cd input.
    pub sense: Option<SenseJson>,
    pub plant: Option<PlantJson>,
    pub gains: Vec<GainJson>,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct BiasJson {
    pub sigma_theta: f64,
    pub pos_mean: f64,
    pub i_noise: f64,
    pub i_bias_delta: f64,
    pub vbus_mean: f64,
    pub vbus_sd: f64,
    pub n: usize,
}

impl From<&BiasResult> for BiasJson {
    fn from(b: &BiasResult) -> Self {
        Self {
            sigma_theta: b.sigma_theta,
            pos_mean: b.pos_mean,
            i_noise: b.i_noise,
            i_bias_delta: b.i_bias_delta,
            vbus_mean: b.vbus_mean,
            vbus_sd: b.vbus_sd,
            n: b.n,
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

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct InertiaJson {
    pub b_best: f64,
    pub b_direct: Option<f64>,
    pub b_exp: Option<f64>,
    pub j_ff: f64,
    pub tel_steps: usize,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct SenseJson {
    pub shunt_r_mohm: u16,
    pub gain_milli: u16,
    pub vmotor_div_top: u16,
    pub vmotor_div_bot: u16,
    pub tick_hz: u16,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
pub struct PlantJson {
    pub r_vpc: f64,
    pub ke_vpc: f64,
    pub fc: f64,
    pub fv: f64,
    pub b: f64,
    pub sigma_theta: f64,
    pub l_cd: f64,
    pub tick_hz: f64,
    pub f_med: f64,
    pub f_ci: f64,
    pub f_cv: f64,
    pub f_cp: f64,
    pub f_o: f64,
}

impl PlantJson {
    pub fn new(p: &PlantParams, t: &BwTargets) -> Self {
        Self {
            r_vpc: p.r_vpc,
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
        e.fields()
            .iter()
            .map(|(name, f): &(&str, Encoded)| Self {
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
