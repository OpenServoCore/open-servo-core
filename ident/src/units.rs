//! Count -> real-world-unit conversion factors and a text report. Derives
//! amps/volts/degrees per count and the torque constants from the CalibSense
//! constants the earlier bands pinned plus the calibration outputs
//! (kinematics + motor Ke). Pure math; matches gains.rs's sense-scale
//! reasoning (both current and vmotor channels reference VDD as the ADC ref).

use core::f64::consts::PI;
use core::fmt::Write as _;

use crate::kinematics::{self, KinematicsResult};

/// Wire-readable CalibSense fields (mirrors regs::calib). tick_hz is carried
/// for completeness but no factor here needs it: firmware reports velocity in
/// counts/second, so per-tick conversion never enters (see [`derive`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SenseParams {
    pub shunt_r_mohm: u16,
    pub gain_milli: u16,
    pub vmotor_div_top: u16,
    pub vmotor_div_bot: u16,
    pub vdd_mv: u16,
    pub tick_hz: u16,
}

/// Real-unit conversion factors. Non-optional factors are 0.0 on a degenerate
/// input (zero denominator) rather than an error. The velocity factor is
/// deg_per_count itself: firmware's omega_hat is counts/second, so
/// (deg/s per c/s) == deg/count with no tick_hz term - hence no separate field.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UnitFactors {
    pub amps_per_count: f64,
    pub volts_per_count: f64,
    pub deg_per_count: f64,
    pub kt_output_ideal_nm_per_a: Option<f64>,
    pub kt_motor_nm_per_a: Option<f64>,
}

/// ADC lsb in volts: 4096 = 12-bit full scale, ref = VDD.
pub fn adc_lsb_v(sense: &SenseParams) -> f64 {
    sense.vdd_mv as f64 / 1000.0 / 4096.0
}

/// Shunt + PGA scale; 0.0 on a degenerate front end.
pub fn amps_per_count(sense: &SenseParams) -> f64 {
    let denom = sense.gain_milli as f64 / 1000.0 * (sense.shunt_r_mohm as f64 / 1000.0);
    if denom > 0.0 {
        adc_lsb_v(sense) / denom
    } else {
        0.0
    }
}

/// Volts per count behind any resistive divider on the same ADC ref. The
/// vmotor taps return to one common bias node, so a terminal DIFFERENCE
/// scales by exactly this factor - the bias cancels.
pub fn div_volts_per_count(sense: &SenseParams, top: u16, bot: u16) -> f64 {
    if bot == 0 {
        return 0.0;
    }
    adc_lsb_v(sense) * (top as f64 + bot as f64) / bot as f64
}

pub fn volts_per_count(sense: &SenseParams) -> f64 {
    div_volts_per_count(sense, sense.vmotor_div_top, sense.vmotor_div_bot)
}

/// The bias capacitance on the terminal taps' bias node: Cb1 10 uF + Cb2
/// 100 nF, rev 2A BOM nominal. With the 79.5 ohm node it sets a 0.80 ms
/// low pass, not yet measured on a board.
pub const BIAS_CAP_F: f64 = 10.1e-6;

/// One terminal leg's ON-phase level for [`BiasNode::steady`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum Leg {
    /// The leg's tap code, counts.
    Tap(f64),
    /// The terminal itself, volts.
    Volts(f64),
}

/// The node the terminal taps' bottom legs return to (protocol sec 5.5):
/// VDD through the bias divider, loaded by every terminal leg. Leg current
/// under drive moves it, by up to 4x that move on one terminal read
/// absolutely; a terminal difference cancels it.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BiasNode {
    /// Source impedance over a leg's bottom resistor, and over the whole leg.
    c_t: f64,
    c_v: f64,
    /// A tap's share of its terminal.
    k: f64,
    lsb_v: f64,
    tau_us: f64,
}

impl BiasNode {
    /// None without a published bias divider (firmware before it, or a
    /// recording that predates it).
    pub fn new(sense: &SenseParams, top_ohm: u16, bot_ohm: u16) -> Option<Self> {
        let (rt, rb) = (sense.vmotor_div_top as f64, sense.vmotor_div_bot as f64);
        if top_ohm == 0 || bot_ohm == 0 || rb == 0.0 || sense.vdd_mv == 0 {
            return None;
        }
        let zs = top_ohm as f64 * bot_ohm as f64 / (top_ohm as f64 + bot_ohm as f64);
        let node = 1.0 / (1.0 / zs + 2.0 / (rt + rb));
        Some(Self {
            c_t: zs / rb,
            c_v: zs / (rt + rb),
            k: rb / (rt + rb),
            lsb_v: adc_lsb_v(sense),
            tau_us: node * BIAS_CAP_F * 1e6,
        })
    }

    /// The settled bias, counts, at ON fraction `d` (the bridge's measured
    /// width where known) with each leg's ON level; `vb0` is the node with
    /// no leg current. Slow decay: every terminal sits low in the OFF phase.
    /// A leg missing from `legs` is open.
    pub fn steady(&self, vb0: f64, d: f64, legs: &[Leg]) -> f64 {
        let mut den = 1.0;
        let mut num = vb0;
        for leg in legs {
            den += self.c_v;
            match *leg {
                Leg::Tap(t) => {
                    den += d * self.c_t * (1.0 - self.k);
                    num += d * self.c_t * t;
                }
                Leg::Volts(v) => num += d * self.c_v * v / self.lsb_v,
            }
        }
        num / den
    }

    /// The bias `dt_us` after its input steps from `from`'s level to `to`'s.
    pub fn settle(&self, from: f64, to: f64, dt_us: f64) -> f64 {
        to + (from - to) * (-dt_us.max(0.0) / self.tau_us).exp()
    }
}

/// Derive every conversion factor. 1000 = milli scaling, 100 = centi,
/// 180/PI = deg<->rad.
pub fn derive(
    sense: &SenseParams,
    kin: &KinematicsResult,
    pos_min_phys: i32,
    pos_max_phys: i32,
    ke_vpc_q: u16,
) -> UnitFactors {
    let amps_per_count = amps_per_count(sense);
    let volts_per_count = volts_per_count(sense);

    let deg_per_count = kinematics::deg_per_count(
        kin.angle_min_cdeg,
        kin.angle_max_cdeg,
        pos_min_phys,
        pos_max_phys,
    );

    // ke_vpc_q is Q4.12 vcounts per OUTPUT c/s: it was fit against pot (output)
    // speed, so the gear ratio is already baked in and the OUTPUT torque
    // constant needs no explicit ratio. This is the lossless upper bound - real
    // output torque is lower by the unknown gear efficiency.
    let ke_vpc_phys = ke_vpc_q as f64 / 4096.0;
    let ke_out_v_per_cps = ke_vpc_phys * volts_per_count;
    let out_rad_per_cps = deg_per_count.abs() * PI / 180.0;
    let kt_output_ideal_nm_per_a = if out_rad_per_cps > 0.0 && ke_out_v_per_cps.is_finite() {
        let kt = ke_out_v_per_cps / out_rad_per_cps;
        kt.is_finite().then_some(kt)
    } else {
        None
    };

    // Motor-side torque constant divides the output constant back down by the
    // measured gear ratio. gear_ratio_centi == 0 is the "unset" sentinel.
    let kt_motor_nm_per_a = match kt_output_ideal_nm_per_a {
        Some(kt) if kin.gear_ratio_centi != 0 => {
            let gear_ratio = kin.gear_ratio_centi as f64 / 100.0;
            Some(kt / gear_ratio)
        }
        _ => None,
    };

    UnitFactors {
        amps_per_count,
        volts_per_count,
        deg_per_count,
        kt_output_ideal_nm_per_a,
        kt_motor_nm_per_a,
    }
}

fn opt(v: Option<f64>) -> String {
    match v {
        Some(v) => format!("{v:.6}"),
        None => "unset".into(),
    }
}

/// Render the `[units]` report section: each factor with its real unit, plus a
/// caveat that output torque is a lossless upper bound and that angle/torque
/// accuracy ride on the position table and the measured gear ratio.
pub fn render(f: &UnitFactors) -> String {
    let mut s = String::new();
    render_into(&mut s, f);
    s
}

/// [`render`] appending into a caller-owned buffer.
pub fn render_into(s: &mut String, f: &UnitFactors) {
    let _ = writeln!(s, "[units]");
    let _ = writeln!(s, "  amps_per_count   {:.9} A/count", f.amps_per_count);
    let _ = writeln!(s, "  volts_per_count  {:.9} V/count", f.volts_per_count);
    let _ = writeln!(
        s,
        "  deg_per_count    {:.6} deg/count (also deg/s per c/s)",
        f.deg_per_count
    );
    let _ = writeln!(
        s,
        "  kt_output_ideal  {} N.m/A (lossless upper bound)",
        opt(f.kt_output_ideal_nm_per_a)
    );
    let _ = writeln!(s, "  kt_motor         {} N.m/A", opt(f.kt_motor_nm_per_a));
    let _ = writeln!(
        s,
        "  note: output torque is a lossless upper bound - real is lower by the\n  \
         unknown gear efficiency; angle and torque accuracy depend on the pot\n  \
         LUT and the measured gear ratio."
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    const REV_2A: SenseParams = SenseParams {
        shunt_r_mohm: 60,
        gain_milli: 14_884,
        vmotor_div_top: 6_400,
        vmotor_div_bot: 1_600,
        vdd_mv: 3300,
        tick_hz: 20_000,
    };

    fn rev_2a_bias() -> (BiasNode, f64) {
        let lsb = adc_lsb_v(&REV_2A);
        (
            BiasNode::new(&REV_2A, 430, 100).unwrap(),
            3.3 * 100.0 / 530.0 / lsb,
        )
    }

    /// The bias plan's sec 2.2 "today 430/100" rows: slow decay, the driven
    /// terminal at the rail and the other low, so the terminal sum is D x Vr.
    #[test]
    fn bias_shift_matches_the_430_100_rows_at_both_rails() {
        let (node, vb0) = rev_2a_bias();
        let lsb = adc_lsb_v(&REV_2A);
        let duties = [0.0, 0.10, 0.15, 0.25, 0.50, 0.75, 0.87, 1.00];
        let rows = [
            (4.5, [-12.4, -7.9, -5.7, -1.2, 10.0, 21.2, 26.5, 32.4]),
            (8.4, [-12.4, -4.0, 0.1, 8.5, 29.4, 50.2, 60.3, 71.1]),
        ];
        for (vr, mv) in rows {
            for (d, want) in duties.iter().zip(mv) {
                let vb = node.steady(vb0, *d, &[Leg::Volts(vr), Leg::Volts(0.0)]);
                let got = (vb - vb0) * lsb * 1e3;
                assert!((got - want).abs() < 0.06, "{vr} V, D {d}: {got:.2} mV");
            }
        }
    }

    /// A tap reads its terminal as 0.2 Vt + 0.8 VB, so reading one terminal
    /// against the boot bias carries 4x the shift: +129 mV at 4.5 V and +284
    /// mV at 8.4 V, full duty.
    #[test]
    fn one_terminal_read_against_the_boot_bias_carries_four_times_the_shift() {
        let (node, vb0) = rev_2a_bias();
        let (lsb, vpc) = (adc_lsb_v(&REV_2A), volts_per_count(&REV_2A));
        for (vr, want) in [(4.5, 129.4), (8.4, 284.5)] {
            let vb = node.steady(vb0, 1.0, &[Leg::Volts(vr), Leg::Volts(0.0)]);
            let tap = 0.2 * vr / lsb + 0.8 * vb;
            let at = |b: f64| vpc * (tap - b) + lsb * b;
            assert!((at(vb) - vr).abs() < 1e-9);
            assert!(((at(vb0) - vr) * 1e3 - want).abs() < 0.3, "{vr} V");
        }
    }

    /// Both legs as taps give the same node as both legs as terminal volts.
    #[test]
    fn tap_and_terminal_legs_solve_to_one_node() {
        let (node, vb0) = rev_2a_bias();
        let lsb = adc_lsb_v(&REV_2A);
        let d = 0.6;
        let vb = node.steady(vb0, d, &[Leg::Volts(7.4), Leg::Volts(0.3)]);
        let tap = |v: f64| 0.2 * v / lsb + 0.8 * vb;
        let by_taps = node.steady(vb0, d, &[Leg::Tap(tap(7.4)), Leg::Tap(tap(0.3))]);
        assert!((by_taps - vb).abs() < 1e-9);
    }

    /// Rev 2A board #1, motor unplugged, OpenLoop slow decay at +/-100%
    /// from a 8.40 and a 4.50 V supply, three power-cycled repeats in each
    /// of two runs: per rung the parked taps' mean (VB0) and the median
    /// crest taps (A, B), snapshots whose low tap read high dropped. With no
    /// current the low terminal sits at 0 V, so its tap / 0.8 is VB itself.
    /// The equation lands on it within 1.5 mV, mean over the repeats.
    #[test]
    fn bias_shift_matches_the_open_load_bench_rungs() {
        // (VB0, tap A, tap B) per repeat
        type Reps = [(f64, f64, f64); 3];
        const RUNGS: [(&str, Reps); 8] = [
            (
                "A 8.40 fwd",
                [
                    (773.833, 2707.0, 687.0),
                    (774.000, 2704.5, 687.5),
                    (775.000, 2705.0, 687.0),
                ],
            ),
            (
                "A 8.40 rev",
                [
                    (773.833, 688.0, 2703.5),
                    (774.000, 687.0, 2705.0),
                    (775.000, 686.0, 2704.5),
                ],
            ),
            (
                "A 4.50 fwd",
                [
                    (771.000, 1698.5, 647.0),
                    (772.000, 1701.5, 647.0),
                    (773.167, 1702.0, 648.5),
                ],
            ),
            (
                "A 4.50 rev",
                [
                    (771.000, 647.0, 1698.0),
                    (772.000, 647.0, 1699.0),
                    (773.167, 648.0, 1702.0),
                ],
            ),
            (
                "B 8.40 fwd",
                [
                    (773.833, 2706.0, 686.5),
                    (773.000, 2702.0, 685.0),
                    (773.500, 2705.0, 686.5),
                ],
            ),
            (
                "B 8.40 rev",
                [
                    (773.833, 687.0, 2708.0),
                    (773.000, 685.5, 2702.0),
                    (773.500, 687.0, 2700.5),
                ],
            ),
            (
                "B 4.50 fwd",
                [
                    (770.500, 1700.0, 647.0),
                    (772.833, 1701.5, 648.5),
                    (772.833, 1699.5, 648.0),
                ],
            ),
            (
                "B 4.50 rev",
                [
                    (770.500, 647.0, 1698.0),
                    (772.833, 649.0, 1698.0),
                    (772.833, 648.0, 1699.0),
                ],
            ),
        ];
        // VDD from the parked bias: 771 counts is 0.619 V
        let sense = SenseParams {
            vdd_mv: 3288,
            ..REV_2A
        };
        let node = BiasNode::new(&sense, 430, 100).unwrap();
        let lsb = adc_lsb_v(&sense);
        for (rung, reps) in RUNGS {
            let fwd = rung.ends_with("fwd");
            let resid: f64 = reps
                .iter()
                .map(|&(vb0, a, b)| {
                    let eq = node.steady(vb0, 1.0, &[Leg::Tap(a), Leg::Tap(b)]);
                    let measured = if fwd { b } else { a } / 0.8;
                    (eq - measured) * lsb * 1e3
                })
                .sum::<f64>()
                / reps.len() as f64;
            assert!(resid.abs() <= 1.5, "{rung}: {resid:+.2} mV");
        }
    }

    /// Cb against the 79.5 ohm node: 0.80 ms, 0.060 of a step per 50 us
    /// tick.
    #[test]
    fn bias_follows_a_step_through_its_0_8_ms_low_pass() {
        let (node, _) = rev_2a_bias();
        let tick = node.settle(0.0, 1.0, 50.0);
        assert!((tick - 0.060).abs() < 0.001, "{tick}");
        let tau = node.settle(0.0, 1.0, 803.0);
        assert!((tau - (1.0 - (-1.0f64).exp())).abs() < 0.002, "{tau}");
        assert!((node.settle(0.3, 1.0, -5.0) - 0.3).abs() < 1e-12);
    }

    #[test]
    fn no_bias_node_without_a_published_divider() {
        assert!(BiasNode::new(&REV_2A, 0, 0).is_none());
        assert!(BiasNode::new(&REV_2A, 430, 0).is_none());
    }

    fn kin(gear_ratio_centi: u16) -> KinematicsResult {
        // 190 deg over a 3800-count span -> deg_per_count = 0.05
        KinematicsResult {
            angle_min_cdeg: 0,
            angle_max_cdeg: 19000,
            gear_ratio_centi,
        }
    }

    #[test]
    fn amps_and_volts_per_count_hand_computed() {
        // vdd 3300 mV -> lsb = 3.3/4096 = 805.664 uV; shunt 0.033 ohm, G 15.0
        let sense = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };
        let f = derive(&sense, &kin(0), 100, 3900, 0);
        let lsb = 3.3 / 4096.0;
        assert!((f.amps_per_count - lsb / (15.0 * 0.033)).abs() < 1e-12);
        let vdiv = 28_200.0 / 10_000.0;
        assert!((f.volts_per_count - lsb * vdiv).abs() < 1e-12);
    }

    #[test]
    fn deg_per_count_passthrough_and_torque_derivation() {
        // Choose values so kt_output_ideal is a round number.
        // deg_per_count = 0.05 -> out_rad_per_cps = 0.05*PI/180.
        // volts_per_count = lsb*vdiv, ke_vpc_q/4096 * volts_per_count = ke_out_v.
        let sense = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };
        let ke_vpc_q = 4096; // ke_vpc_phys = 1.0
        let f = derive(&sense, &kin(15000), 100, 3900, ke_vpc_q);
        assert!((f.deg_per_count - 0.05).abs() < 1e-12);

        let lsb = 3.3 / 4096.0;
        let vpc = lsb * 28_200.0 / 10_000.0;
        let out_rad = 0.05 * PI / 180.0;
        let expect_out = vpc / out_rad;
        assert!((f.kt_output_ideal_nm_per_a.unwrap() - expect_out).abs() < 1e-9);

        // kt_motor = kt_output_ideal / (gear_ratio_centi/100)
        let gear = 15000.0 / 100.0;
        assert!((f.kt_motor_nm_per_a.unwrap() - expect_out / gear).abs() < 1e-9);
        assert!(
            (f.kt_motor_nm_per_a.unwrap() - f.kt_output_ideal_nm_per_a.unwrap() / gear).abs()
                < 1e-12
        );
    }

    #[test]
    fn kt_motor_none_when_gear_unset() {
        let sense = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };
        let f = derive(&sense, &kin(0), 100, 3900, 4096);
        assert!(f.kt_output_ideal_nm_per_a.is_some());
        assert!(f.kt_motor_nm_per_a.is_none());
    }

    #[test]
    fn degenerate_guards_no_panic() {
        let base = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };

        // zero shunt -> amps_per_count 0
        let f = derive(
            &SenseParams {
                shunt_r_mohm: 0,
                ..base
            },
            &kin(15000),
            100,
            3900,
            4096,
        );
        assert_eq!(f.amps_per_count, 0.0);

        // zero gain -> amps_per_count 0
        let f = derive(
            &SenseParams {
                gain_milli: 0,
                ..base
            },
            &kin(15000),
            100,
            3900,
            4096,
        );
        assert_eq!(f.amps_per_count, 0.0);

        // zero divider bottom -> volts_per_count 0 and torque None (no volts)
        let f = derive(
            &SenseParams {
                vmotor_div_bot: 0,
                ..base
            },
            &kin(15000),
            100,
            3900,
            4096,
        );
        // zero volts scale propagates to a zero torque constant (the division
        // denominator out_rad is still finite, so it is Some(0.0), not None)
        assert_eq!(f.volts_per_count, 0.0);
        assert_eq!(f.kt_output_ideal_nm_per_a, Some(0.0));
        assert_eq!(f.kt_motor_nm_per_a, Some(0.0));

        // zero deg_per_count (zero phys span) -> torque None
        let f = derive(&base, &kin(15000), 512, 512, 4096);
        assert_eq!(f.deg_per_count, 0.0);
        assert!(f.kt_output_ideal_nm_per_a.is_none());
        assert!(f.kt_motor_nm_per_a.is_none());
    }

    #[test]
    fn render_shows_section_and_unset() {
        let f = UnitFactors {
            amps_per_count: 1.6e-3,
            volts_per_count: 2.27e-3,
            deg_per_count: 0.05,
            kt_output_ideal_nm_per_a: Some(0.5),
            kt_motor_nm_per_a: None,
        };
        let s = render(&f);
        assert!(s.contains("[units]"));
        assert!(s.contains("amps_per_count"));
        assert!(s.contains("kt_motor"));
        assert!(
            s.contains("unset"),
            "kt_motor None should render unset:\n{s}"
        );
        assert!(s.contains("lossless upper bound"));
    }
}
