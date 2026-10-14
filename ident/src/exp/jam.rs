//! The jam check's knobs, in physical units and derived per servo: volts
//! for the drive, so a rung means the same on USB and 2S; milliamps for
//! what a stop may take, from the gear class; the winding R the servo
//! carries when it carries one.

use crate::exp::rl::Scales;
use crate::limits::{GearClass, ServoLimits};

/// The probe's rung step and its backstop, mV: 3% of a 7.4 V rail a step,
/// 5.4% of 3.7 V. The backstop is reached only while the current reads low.
pub const PROBE_STEP_MV: f64 = 200.0;
pub const PROBE_MAX_MV: f64 = 2500.0;

/// The probe's current cap over the class's stop hold: 1.5x over the
/// breakaway of the families the classes cover (MG90 <= 245 mA, SG90 <=
/// 160 mA).
pub const PROBE_OF_HOLD: f64 = 0.75;

/// Under this a drive draws nothing, mA: an open winding reads only the
/// shunt's noise, a few mA, and the class's windings (3 to 7 ohm) draw
/// over 70 mA at the lowest rung of a 3.7 V rail.
pub const OPEN_MA: f64 = 30.0;

#[derive(Clone, Debug)]
pub struct JamCfg {
    pub class: GearClass,
    pub mv_per_vcount: f64,
    pub ma_per_ccount: f64,
    /// The rung step and the backstop, vcounts.
    pub step: f64,
    pub v_max: f64,
    /// The probe's cap and the stop hold, both at most the servo's limit,
    /// and the open-winding floor: ccounts.
    pub i_probe: f64,
    pub i_hold: f64,
    pub i_open: f64,
    /// Winding R, vcounts per ccount, when the servo carries one.
    pub r_vpc: Option<f64>,
    pub press_budget_ms: f64,
    pub poll_ms: u32,
    /// Half-width of the band around mid travel the check ends in, counts.
    pub margin: u16,
    /// Poll budget: 400 x 20 ms is 8 s.
    pub polls_max: u32,
}

impl JamCfg {
    /// `i_lim`: the servo's current limit, ccounts.
    pub fn new(
        class: GearClass,
        mv_per_vcount: f64,
        ma_per_ccount: f64,
        i_lim: u16,
        r_vpc: Option<f64>,
    ) -> Self {
        let ccounts = |ma: f64| (ma / ma_per_ccount).min(i_lim as f64);
        Self {
            class,
            mv_per_vcount,
            ma_per_ccount,
            step: PROBE_STEP_MV / mv_per_vcount,
            v_max: PROBE_MAX_MV / mv_per_vcount,
            i_probe: ccounts(PROBE_OF_HOLD * class.hold_ma()),
            i_hold: ccounts(class.hold_ma()),
            i_open: OPEN_MA / ma_per_ccount,
            r_vpc,
            press_budget_ms: class.press_budget_ms(),
            poll_ms: 20,
            margin: 300,
            polls_max: 400,
        }
    }

    pub fn for_servo(lim: &ServoLimits, sc: &Scales, class: GearClass) -> Self {
        Self::new(
            class,
            sc.v_term_per_count * 1000.0,
            sc.amps_per_count * 1000.0,
            lim.i_lim,
            lim.r_vpc(),
        )
    }

    pub fn ma(&self, ccounts: f64) -> f64 {
        ccounts * self.ma_per_ccount
    }

    pub fn mv(&self, vcounts: f64) -> f64 {
        vcounts * self.mv_per_vcount
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{BENCH_MA_PER_CCOUNT, BENCH_MV_PER_VCOUNT};
    use super::*;

    /// The bench MG90's scales, its 500 mA limit: plastic probes to 225 mA
    /// and holds 300, metal 375 and 500; a lower limit caps both; a rung is
    /// 200 mV on every rail.
    #[test]
    fn knobs_in_physical_units() {
        let lim = (500.0 / BENCH_MA_PER_CCOUNT) as u16;
        let cfg = |class| JamCfg::new(class, BENCH_MV_PER_VCOUNT, BENCH_MA_PER_CCOUNT, lim, None);
        let ma = |c: &JamCfg, i: f64| c.ma(i).round();
        let p = cfg(GearClass::Plastic);
        assert_eq!((ma(&p, p.i_probe), ma(&p, p.i_hold)), (225.0, 300.0));
        assert_eq!(p.press_budget_ms, 250.0);
        let u = cfg(GearClass::Unknown);
        assert_eq!((u.i_probe, u.i_hold), (p.i_probe, p.i_hold));
        let m = cfg(GearClass::Metal);
        assert_eq!((ma(&m, m.i_probe), ma(&m, m.i_hold)), (375.0, 500.0));
        assert_eq!(
            (m.mv(m.step).round(), m.mv(m.v_max).round()),
            (200.0, 2500.0)
        );
        let low = JamCfg::new(
            GearClass::Metal,
            BENCH_MV_PER_VCOUNT,
            BENCH_MA_PER_CCOUNT,
            280,
            None,
        );
        assert_eq!((low.i_probe, low.i_hold), (280.0, 280.0));
    }
}
