use control_table::{Block, Section};

use crate::estimator::window::SettleGain;
use crate::regions::config::ProtectionDefaults;

/// The pot's mechanical stops in raw ADC counts: the ends of the angle map
/// (`CalibKinematics`) and the domain a position table is validated against.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibPot {
    pub raw_min: u16,
    pub raw_max: u16,
}

/// Sense-chain primitives the host converts raw counts with (protocol sec
/// 5.5): divider legs in ohms, amplifier gain x1000. `vdd_mv` is the
/// host-measured VDD at the chip pin -- the v006 ADC reference is VDD itself.
/// `tick_hz` is stamped at install from the same chip constant that programs
/// TIM1; the window floors are board data in TIM1 ticks.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibSense {
    #[ct_field(access = ro)]
    pub shunt_r_mohm: u16,
    #[ct_field(access = ro)]
    pub gain_milli: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_top: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_bot: u16,
    pub vdd_mv: u16,
    #[ct_field(access = ro)]
    pub tick_hz: u16,
    #[ct_field(access = ro)]
    pub i_window_min_ticks: u16,
    #[ct_field(access = ro)]
    pub v_window_min_ticks: u16,
}

/// Motor identification, firmware-shaped: the winding's resistance at
/// 25.00 C (Q4.12 vcounts per ccount, never the value as measured: the
/// host converts its measurement to 25.00 C with the board NTC's reading
/// along copper's slope, `r(T) = r_q12 x (1 + ALPHA x (T - 25.00 C))`,
/// `estimator::thermal::ALPHA_Q26`; 0 = never measured),
/// reciprocal Ke (c/s per vcount Q6.10, estimator::bemf convention), fusion
/// current gain (bakes Ts/J, Q3.13: rig B runs ~3.4 so Q0.16 saturated), the
/// friction model (Coulomb ccounts, viscous Q0.16; the whole line is the
/// observer's drag on its own speed and the velocity loop's feed-forward
/// on the profile's; the Coulomb level is also the load past which the
/// position hold keeps the current instead of parking), and forward Ke
/// (vcounts per c/s Q4.12, the current loop's bemf decoupling feedforward).
/// Forward and reciprocal Ke are both host-written: the chip has no divide
/// to derive one from the other. SG90 scale ~0.28 vcounts per c/s (~1150
/// stored); the u16 caps at 16, 57x headroom.
///
/// Then the brush term (`estimator::bemf`): `r_brush_q12` the spinning
/// brushes' resistance to a change in current beyond the winding's, in
/// `r_q12`'s units and never raised along copper's slope, at most `r_q12`;
/// `brush_cps` the speed it was identified at, c/s, where its weight is
/// full. A 0 size, or a speed under `bemf::BRUSH_SPAN_CPS`, turns the term
/// off.
///
/// Then the window term (`estimator::window::WindowTerm`): `g_window_q016`
/// the motor's loss conductance across its inductance, counts of current
/// per vcount, Q0.16, which the shunt's drive-window sample reads times
/// `(1 - D) x V_bus` on top of the winding's current. 0 turns it off, as
/// does a value over `g x r_q12 <= 2^25` or an unidentified winding.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibMotor {
    pub r_q12: u16,
    pub recip_ke_q: u16,
    pub b_i_q313: u16,
    pub fric_fc_counts: u16,
    pub fric_fv_q016: u16,
    pub ke_vpc_q: u16,
    #[ct_field(le = &addr::motor::R_Q12)]
    pub r_brush_q12: u16,
    pub brush_cps: u16,
    pub g_window_q016: u16,
    #[ct_field(skip)]
    pub _rsvd_align: u16,
}

/// The identification record firmware never reads: Ke as the host states
/// it and the breakaway friction, ccounts.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibMotorExt {
    pub ke_uvs_per_rad: u16,
    pub fric_breakaway_counts: u16,
}

/// Count<->angle scale and gearing, pure host-facing metadata: firmware never
/// reads it, the ISR stays in counts. Angles are centi-degrees at the pot
/// stops raw_min/raw_max (which equal pos_min/max_phys); `gear_ratio_centi`
/// is motor revs per output rev x100. All-zero = unset.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibKinematics {
    pub angle_min_cdeg: i16,
    pub angle_max_cdeg: i16,
    pub gear_ratio_centi: u16,
}

/// Supply-sense and thermistor board data (protocol sec 5.5): the direct
/// rail divider legs, the NTC pull-up / R25 / beta, the nominal
/// motor-terminal divider bias (the boot-measured value publishes in
/// telemetry), and the VDD divider that makes that bias: terminal leg
/// current moves the bias node under drive, and a host reading one terminal
/// absolutely corrects for it from these legs. RO: install re-stamps every
/// board fact after the overlay.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibSenseExt {
    #[ct_field(access = ro)]
    pub vbus_div_top_ohm: u16,
    #[ct_field(access = ro)]
    pub vbus_div_bot_ohm: u16,
    #[ct_field(access = ro)]
    pub ntc_pullup_ohm: u16,
    #[ct_field(access = ro)]
    pub ntc_r25_ohm: u16,
    #[ct_field(access = ro)]
    pub ntc_beta: u16,
    #[ct_field(access = ro)]
    pub vmotor_bias_nom_counts: u16,
    /// Supply connector -> VSYS drop across the input Schottky at rest, mV,
    /// so a host can estimate the pack voltage from `vbus_raw`.
    #[ct_field(access = ro)]
    pub rail_drop_mv: u16,
    #[ct_field(access = ro)]
    pub vmotor_bias_top_ohm: u16,
    #[ct_field(access = ro)]
    pub vmotor_bias_bot_ohm: u16,
}

/// Winding estimate constants, host-written per servo and board:
/// `th_rise_q016` the rotor node's rise over the board NTC per count
/// squared of its heat and `th_rise2_q016` the body node's (centi-C,
/// Q0.16 each: a thermal resistance times the winding's resistance, in
/// the board's current counts; with the NTC on the can the body gain is
/// 0 and the rotor node alone rides on it), then the board NTC's
/// counts-to-centi-C quadratic about its reference point, derived by the
/// host from `CalibSenseExt`'s beta constants (`estimator::thermal::
/// NtcCfg`). No curve = no estimate (`t_winding_cc` reads its sentinel,
/// under every cutoff); a curve with both gains 0 reads the NTC alone.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibThermal {
    pub th_rise_q016: u16,
    pub th_rise2_q016: u16,
    pub ntc_raw_ref: u16,
    pub ntc_t_ref_cc: i16,
    pub ntc_k1_q88: i16,
    pub ntc_k2_q24: u16,
}

/// TIM1's period in compare counts, stamped at install from the constant
/// that programs it: the full scale of TEL `duty` (sec 5.6) and of the
/// window floors.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibPwm {
    #[ct_field(access = ro)]
    pub pwm_period_ticks: u16,
}

/// The gear train's material, host-written by cal for hosts to pick limits
/// from (a stop hold's current); firmware never reads it. 0 = unknown, the
/// state of a servo never calibrated; 1 = plastic, 2 = metal.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibGear {
    pub gear_class: u8,
}

/// The ADC's wide codes, per board since the converter is the chip's:
/// slot `k` belongs to code `(k << 8) | 0xff` on any channel and holds half
/// the span the converter skips after that code, Q4 counts; 0 = no wide
/// code there (`adc` module). Host-written by calibration.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibAdc {
    pub adc_wide_mid_q4: [u8; crate::adc::ADC_WIDE_SLOTS],
}

/// Where the board's NTC sits, the code CALIB `ntc_site` publishes. 0 is
/// no site: an image older than the board facts block.
#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum NtcSite {
    /// In room air beside the bridge.
    Air = 1,
    /// On the motor can.
    Can = 2,
}

/// [`NtcSite`] as its code.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibNtcSite {
    #[ct_field(access = ro)]
    pub ntc_site: u8,
}

/// The board's stage 1 characterization, published for the host; the
/// firmware reads none of it. Q15 ratio, Q4 ADC counts, milliohms and TIM1
/// ticks (the Q4 lags too). `window_scale_q15` 0 states no characterization,
/// and then every word is 0 (`CalibBoard::violation`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Characterization {
    /// The drive window's V_on / i over the 100% window's.
    pub window_scale_q15: u16,
    /// The driven tap difference's offset at zero current.
    pub terminal_offset_q4: i16,
    /// The low-side loop a braking winding closes through.
    pub brake_loop_mohm: u16,
    /// Where the shunt's crest sample sits in the drive window.
    pub crest_sample_ticks: u16,
    /// The slow-decay burst edges' lags: the shunt's and the tap's on
    /// edges, the off edge, and the shunt's settle.
    pub shunt_on_lag_q4: u16,
    pub tap_on_lag_q4: u16,
    pub off_lag_q4: u16,
    pub shunt_settle_q4: u16,
}

/// The settle table's words in CALIB.
pub const SETTLE_WORDS: usize = 24;

/// The board facts a host takes off the servo that no other register holds
/// (protocol sec 5.5): the bridge's shortest pulse, the settle table (0 past
/// its last band), the protection defaults CONFIG is seeded from, and the
/// stage 1 characterization. Stamped at install after the calib overlay;
/// the firmware never reads the block.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibBoard {
    #[ct_field(access = ro)]
    pub bridge_min_ticks: u16,
    #[ct_field(access = ro)]
    pub i_settle_start_ticks: u16,
    #[ct_field(access = ro)]
    pub i_settle_q15: [u16; SETTLE_WORDS],
    #[ct_field(access = ro)]
    pub oc_trip_ma: u16,
    #[ct_field(access = ro)]
    pub oc_trip_us: u16,
    #[ct_field(access = ro)]
    pub rail_source_mohm: u16,
    #[ct_field(access = ro)]
    pub window_scale_q15: u16,
    #[ct_field(access = ro)]
    pub terminal_offset_q4: i16,
    #[ct_field(access = ro)]
    pub brake_loop_mohm: u16,
    #[ct_field(access = ro)]
    pub crest_sample_ticks: u16,
    #[ct_field(access = ro)]
    pub shunt_on_lag_q4: u16,
    #[ct_field(access = ro)]
    pub tap_on_lag_q4: u16,
    #[ct_field(access = ro)]
    pub off_lag_q4: u16,
    #[ct_field(access = ro)]
    pub shunt_settle_q4: u16,
}

impl CalibBoard {
    /// The block from the board's values, which [`Self::assert_valid`]
    /// holds to CALIB's words. Const-eval only.
    pub const fn from_board(
        bridge_min_ticks: u16,
        settle: &SettleGain,
        protection: &ProtectionDefaults,
        ch: &Characterization,
    ) -> Self {
        let mut i_settle_q15 = [0u16; SETTLE_WORDS];
        let mut i = 0;
        while i < settle.q15.len() && i < SETTLE_WORDS {
            i_settle_q15[i] = settle.q15[i];
            i += 1;
        }
        Self {
            bridge_min_ticks,
            i_settle_start_ticks: settle.start_ticks,
            i_settle_q15,
            oc_trip_ma: protection.oc_trip_ma,
            oc_trip_us: protection.oc_trip_us,
            rail_source_mohm: protection.rail_source_mohm as u16,
            window_scale_q15: ch.window_scale_q15,
            terminal_offset_q4: ch.terminal_offset_q4,
            brake_loop_mohm: ch.brake_loop_mohm,
            crest_sample_ticks: ch.crest_sample_ticks,
            shunt_on_lag_q4: ch.shunt_on_lag_q4,
            tap_on_lag_q4: ch.tap_on_lag_q4,
            off_lag_q4: ch.off_lag_q4,
            shunt_settle_q4: ch.shunt_settle_q4,
        }
    }

    /// The rule the board's values break as CALIB words, or None.
    /// Const-eval only.
    pub const fn violation(
        settle: &SettleGain,
        protection: &ProtectionDefaults,
        ch: &Characterization,
    ) -> Option<&'static str> {
        if settle.q15.len() > SETTLE_WORDS {
            return Some(
                "Calibration: calibration.i_settle_gain.q15 must hold at most 24 bands, \
                 CALIB's settle words",
            );
        }
        let mut i = 0;
        while i < settle.q15.len() {
            if settle.q15[i] == 0 {
                return Some(
                    "Calibration: calibration.i_settle_gain.q15 must hold no 0, \
                     which ends the table in CALIB",
                );
            }
            i += 1;
        }
        if protection.rail_source_mohm > u16::MAX as u32 {
            return Some("ProtectionDefaults: rail_source_mohm must fit CALIB's u16 word");
        }
        let rest = ch.terminal_offset_q4 != 0
            || ch.brake_loop_mohm != 0
            || ch.crest_sample_ticks != 0
            || ch.shunt_on_lag_q4 != 0
            || ch.tap_on_lag_q4 != 0
            || ch.off_lag_q4 != 0
            || ch.shunt_settle_q4 != 0;
        if ch.window_scale_q15 == 0 && rest {
            return Some(
                "Characterization: window_scale_q15 0 states no characterization, \
                 so every word must be 0",
            );
        }
        None
    }

    /// Compile-time call site: the board's `run!` and the sim's mirror.
    pub const fn assert_valid(
        settle: &SettleGain,
        protection: &ProtectionDefaults,
        ch: &Characterization,
    ) {
        if let Some(rule) = Self::violation(settle, protection, ch) {
            panic!("{}", rule);
        }
    }
}

/// Calibration section: always writable (normal field validation applies),
/// volatile until persisted -- persistence is SAVE's job, not a write gate.
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CALIB_BASE_ADDR,
    size = crate::regions::CALIB_REGION_SIZE,
)]
pub struct CalibRegs {
    pub motor: CalibMotor,
    pub thermal: CalibThermal,
    pub motor_ext: CalibMotorExt,
    pub pot: CalibPot,
    pub kinematics: CalibKinematics,
    pub sense: CalibSense,
    pub sense_ext: CalibSenseExt,
    pub pwm: CalibPwm,
    pub gear: CalibGear,
    pub adc: CalibAdc,
    pub ntc_site: CalibNtcSite,
    pub board: CalibBoard,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 82],
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regions::config::TEST_PROTECTION;

    const SETTLE: SettleGain = SettleGain {
        start_ticks: 40,
        q15: &[
            37138, 35303, 34342, 33918, 33643, 33470, 33374, 33259, 33213, 33156, 33103, 33052,
            33020, 32982, 32938, 32900, 32868, 32841, 32824, 32806, 32785,
        ],
    };

    const STAGE_1: Characterization = Characterization {
        window_scale_q15: 32975,
        terminal_offset_q4: 38,
        brake_loop_mohm: 301,
        crest_sample_ticks: 31,
        shunt_on_lag_q4: 704,
        tap_on_lag_q4: 632,
        off_lag_q4: 224,
        shunt_settle_q4: 1152,
    };

    const NONE: Characterization = Characterization {
        window_scale_q15: 0,
        terminal_offset_q4: 0,
        brake_loop_mohm: 0,
        crest_sample_ticks: 0,
        shunt_on_lag_q4: 0,
        tap_on_lag_q4: 0,
        off_lag_q4: 0,
        shunt_settle_q4: 0,
    };

    fn dev(settle: &SettleGain, ch: &Characterization) -> Option<&'static str> {
        CalibBoard::violation(settle, &TEST_PROTECTION, ch)
    }

    #[test]
    fn the_dev_board_and_a_board_with_no_stage_1_pass() {
        assert_eq!(dev(&SETTLE, &STAGE_1), None);
        assert_eq!(dev(&SETTLE, &NONE), None);
        assert_eq!(dev(&SettleGain::UNITY, &STAGE_1), None);
        CalibBoard::assert_valid(&SETTLE, &TEST_PROTECTION, &STAGE_1);
    }

    #[test]
    fn the_block_holds_the_boards_words_and_zeros_past_the_settle_table() {
        let b = CalibBoard::from_board(27, &SETTLE, &TEST_PROTECTION, &STAGE_1);
        assert_eq!(b.bridge_min_ticks, 27);
        assert_eq!(b.i_settle_start_ticks, 40);
        assert_eq!(b.i_settle_q15[..21], *SETTLE.q15);
        assert_eq!(b.i_settle_q15[21..], [0; 3]);
        assert_eq!(
            (b.oc_trip_ma, b.oc_trip_us, b.rail_source_mohm),
            (2200, 400, 800)
        );
        assert_eq!(
            [
                b.window_scale_q15,
                b.terminal_offset_q4 as u16,
                b.brake_loop_mohm,
                b.crest_sample_ticks,
                b.shunt_on_lag_q4,
                b.tap_on_lag_q4,
                b.off_lag_q4,
                b.shunt_settle_q4,
            ],
            [32975, 38, 301, 31, 704, 632, 224, 1152]
        );
    }

    #[test]
    fn a_settle_table_over_calibs_words_or_with_a_zero_is_refused() {
        const LONG: [u16; SETTLE_WORDS + 1] = [32768; SETTLE_WORDS + 1];
        let over = SettleGain {
            start_ticks: 40,
            q15: &LONG,
        };
        assert_eq!(
            dev(&over, &STAGE_1),
            Some(
                "Calibration: calibration.i_settle_gain.q15 must hold at most 24 bands, \
                 CALIB's settle words"
            )
        );
        let full = SettleGain {
            start_ticks: 40,
            q15: &LONG[..SETTLE_WORDS],
        };
        assert_eq!(dev(&full, &STAGE_1), None);
        let zero = SettleGain {
            start_ticks: 40,
            q15: &[37138, 0, 34342],
        };
        assert_eq!(
            dev(&zero, &STAGE_1),
            Some(
                "Calibration: calibration.i_settle_gain.q15 must hold no 0, \
                 which ends the table in CALIB"
            )
        );
    }

    #[test]
    fn a_source_resistance_over_the_word_is_refused() {
        let p = |mohm| ProtectionDefaults {
            rail_source_mohm: mohm,
            ..TEST_PROTECTION
        };
        assert_eq!(CalibBoard::violation(&SETTLE, &p(65_535), &STAGE_1), None);
        assert_eq!(
            CalibBoard::violation(&SETTLE, &p(65_536), &STAGE_1),
            Some("ProtectionDefaults: rail_source_mohm must fit CALIB's u16 word")
        );
    }

    #[test]
    fn a_half_stated_characterization_is_refused() {
        let refused = Some(
            "Characterization: window_scale_q15 0 states no characterization, \
             so every word must be 0",
        );
        assert_eq!(
            dev(
                &SETTLE,
                &Characterization {
                    window_scale_q15: 0,
                    ..STAGE_1
                }
            ),
            refused
        );
        assert_eq!(
            dev(
                &SETTLE,
                &Characterization {
                    terminal_offset_q4: -1,
                    ..NONE
                }
            ),
            refused
        );
    }

    #[test]
    #[should_panic(expected = "Characterization: window_scale_q15 0 states no characterization")]
    fn assert_valid_panics_with_the_rule() {
        let ch = Characterization {
            shunt_settle_q4: 1,
            ..NONE
        };
        CalibBoard::assert_valid(&SETTLE, &TEST_PROTECTION, &ch);
    }
}
