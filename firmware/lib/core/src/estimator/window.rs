//! Drive-window selection for the twin per-period ADC scans. Center-aligned
//! PWMMODE1 with the chip-side CCR mapping (Slow: STATIC_HIGH + arr-ticks,
//! Fast: ticks + 0) puts the drive window at the counter PEAK under Slow
//! decay and at the TROUGH under Fast, for either drive direction - decay
//! alone picks the scan half. The shunt is direction-blind, so the signed
//! current takes its sign from the commanded duty.

use crate::SensorFrame;
use crate::estimator::bemf::RECIP_ARR_SHIFT;
use crate::math::q_mul_u;
use crate::traits::DecayMode;

/// Which scan half carries the drive window and whether it is wide enough
/// for each sense path to have settled.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct WindowSel {
    pub i_valid: bool,
    pub v_valid: bool,
    pub use_trough: bool,
}

/// `drive_ticks` is the TIM1 compare width the motor write used; the caller
/// passes 0 for Coast/Brake/Disabled or zero duty, which invalidates both
/// paths even against a zero floor. Floors are board-data minimum widths
/// (`i_window_min_ticks` / `v_window_min_ticks`).
pub fn select(decay: DecayMode, drive_ticks: u32, i_floor: u16, v_floor: u16) -> WindowSel {
    WindowSel {
        i_valid: drive_ticks != 0 && drive_ticks >= i_floor as u32,
        v_valid: drive_ticks != 0 && drive_ticks >= v_floor as u32,
        use_trough: matches!(decay, DecayMode::Fast),
    }
}

/// Signed bias-subtracted shunt current from the drive-window scan, or `None`
/// when the window is too narrow for a settled sample.
pub fn i_from_frame(
    frame: &SensorFrame,
    sel: WindowSel,
    duty_sign_positive: bool,
    bias: u16,
) -> Option<i32> {
    if !sel.i_valid {
        return None;
    }
    let sample = if sel.use_trough {
        frame.current_trough
    } else {
        frame.current
    };
    let mag = sample as i32 - bias as i32;
    Some(if duty_sign_positive { mag } else { -mag })
}

/// Drive-window terminal differential `va - vb`. Positive duty drives IN1
/// (TIM1 CH3, PC6) -> DRV8212P OUT1 -> MOT_A -> VSNA -> `vmotor_a`, so
/// `va - vb` is positive under forward drive, negative under reverse - the
/// sign convention downstream bemf math relies on. Both taps share the
/// board's terminal bias, so it cancels here.
pub fn vdiff_from_frame(frame: &SensorFrame, sel: WindowSel) -> Option<i32> {
    if !sel.v_valid {
        return None;
    }
    let (va, vb) = if sel.use_trough {
        (frame.vmotor_a_trough, frame.vmotor_b_trough)
    } else {
        (frame.vmotor_a, frame.vmotor_b)
    };
    Some(va as i32 - vb as i32)
}

/// A settled brake trough carries no drive current: both low-side FETs on,
/// the winding current recirculating inside the bridge and none of it
/// crossing the return-path shunt, so the reading is pure amplifier offset.
/// Fast decay puts the drive window at the trough and zero ticks is
/// Coast/Brake/Disabled (no PWM edge), so both are out.
///
/// The brake half-width must clear MUCH more than the drive window's floor.
/// The amplifier leaves the drive pulse saturated and its tail decays over
/// far longer than the floor allows for: measured on a duty grid, the trough
/// sits a flat 5 counts above the disabled-driver rest bias up to 20% duty,
/// then climbs with drive current as the window shrinks - 7 counts at 30%,
/// 16 at 60%, 38 at 85%. At the drive floor itself the tracker would learn a
/// bias tens of counts high and poison every current reading. Six times the
/// floor keeps the feed inside the flat part; above it the tracker holds,
/// which is all a slow thermal drift needs.
const BIAS_SETTLE_FLOORS: u32 = 6;

pub fn trough_is_brake(decay: DecayMode, drive_ticks: u32, pwm_arr: u16, i_floor: u16) -> bool {
    let brake_ticks = (pwm_arr as u32).saturating_sub(drive_ticks);
    matches!(decay, DecayMode::Slow)
        && drive_ticks != 0
        && brake_ticks != 0
        && brake_ticks >= (i_floor as u32).saturating_mul(BIAS_SETTLE_FLOORS)
}

/// Mirrors chip-side `effort_to_ticks` (`mag * arr / 32767` approximated as
/// round(`mag * arr >> 15`)) so validity floors compare against the same
/// width the motor write programs.
pub fn drive_ticks(duty: i16, pwm_arr: u16) -> u32 {
    let mag = duty.unsigned_abs().min(i16::MAX as u16) as u32;
    (mag * pwm_arr as u32 + (1 << 14)) >> 15
}

/// Inverse of `drive_ticks`: the smallest duty magnitude whose window
/// `select` accepts against the `min_ticks` floor; full duty when none does.
/// `recip_arr_q24` per `bemf::RECIP_ARR_SHIFT`. The floored reciprocal reads
/// the quotient low by under two while `min_ticks <= 512`; the remainder
/// closes the gap, so no divide.
pub fn floor_duty(min_ticks: u16, pwm_arr: u16, recip_arr_q24: u32) -> u16 {
    // drive_ticks(d) >= t  <=>  d * arr >= (t << 15) - (1 << 14)
    let need = ((min_ticks.max(1) as u32) << 15) - (1 << 14);
    let est = q_mul_u(need, recip_arr_q24, RECIP_ARR_SHIFT);
    let short = need.saturating_sub(est * pwm_arr as u32);
    let d = est + (short != 0) as u32 + (short > pwm_arr as u32) as u32;
    d.min(i16::MAX as u32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn frame() -> SensorFrame {
        SensorFrame {
            current: 2500,
            current_trough: 1500,
            vmotor_a: 3000,
            vmotor_a_trough: 2900,
            vmotor_b: 100,
            vmotor_b_trough: 90,
            ..Default::default()
        }
    }

    fn valid(use_trough: bool) -> WindowSel {
        WindowSel {
            i_valid: true,
            v_valid: true,
            use_trough,
        }
    }

    #[test]
    fn slow_decay_selects_peak_fast_selects_trough() {
        assert!(!select(DecayMode::Slow, 500, 240, 220).use_trough);
        assert!(select(DecayMode::Fast, 500, 240, 220).use_trough);
    }

    #[test]
    fn floor_boundaries() {
        let s = select(DecayMode::Slow, 239, 240, 220);
        assert!(!s.i_valid);
        assert!(s.v_valid);
        let s = select(DecayMode::Slow, 240, 240, 220);
        assert!(s.i_valid);
        assert!(s.v_valid);
        let s = select(DecayMode::Slow, 219, 240, 220);
        assert!(!s.i_valid);
        assert!(!s.v_valid);
    }

    #[test]
    fn zero_ticks_invalid_even_with_zero_floors() {
        let s = select(DecayMode::Slow, 0, 0, 0);
        assert!(!s.i_valid);
        assert!(!s.v_valid);
    }

    #[test]
    fn current_peak_vs_trough_pick() {
        let f = frame();
        assert_eq!(i_from_frame(&f, valid(false), true, 0), Some(2500));
        assert_eq!(i_from_frame(&f, valid(true), true, 0), Some(1500));
    }

    #[test]
    fn current_bias_and_sign() {
        let f = frame();
        assert_eq!(i_from_frame(&f, valid(false), true, 2048), Some(452));
        assert_eq!(i_from_frame(&f, valid(false), false, 2048), Some(-452));
        // sample below bias with positive duty reads negative
        assert_eq!(i_from_frame(&f, valid(true), true, 2048), Some(-548));
    }

    #[test]
    fn current_invalid_window_is_none() {
        let f = frame();
        let sel = WindowSel {
            i_valid: false,
            v_valid: true,
            use_trough: false,
        };
        assert_eq!(i_from_frame(&f, sel, true, 0), None);
    }

    #[test]
    fn vdiff_peak_vs_trough_pick() {
        let f = frame();
        assert_eq!(vdiff_from_frame(&f, valid(false)), Some(2900));
        assert_eq!(vdiff_from_frame(&f, valid(true)), Some(2810));
    }

    #[test]
    fn vdiff_sign_tracks_drive_direction() {
        assert!(vdiff_from_frame(&frame(), valid(false)).unwrap() > 0);
        let rev = SensorFrame {
            vmotor_a: 100,
            vmotor_b: 3000,
            ..Default::default()
        };
        assert_eq!(vdiff_from_frame(&rev, valid(false)), Some(-2900));
    }

    #[test]
    fn vdiff_invalid_window_is_none() {
        let f = frame();
        let sel = WindowSel {
            i_valid: true,
            v_valid: false,
            use_trough: false,
        };
        assert_eq!(vdiff_from_frame(&f, sel), None);
    }

    #[test]
    fn trough_is_brake_only_inside_a_settled_slow_brake_phase() {
        assert!(trough_is_brake(DecayMode::Slow, 1, 1200, 160));
        // 960 brake ticks = 6 x the floor, the last duty whose trough is
        // settled; one tick more of drive and the feed shuts off.
        assert!(trough_is_brake(DecayMode::Slow, 240, 1200, 160));
        assert!(!trough_is_brake(DecayMode::Slow, 241, 1200, 160));
        assert!(!trough_is_brake(DecayMode::Slow, 600, 1200, 160));
        assert!(!trough_is_brake(DecayMode::Fast, 600, 1200, 160));
        assert!(!trough_is_brake(DecayMode::Slow, 0, 1200, 160));
        assert!(!trough_is_brake(DecayMode::Slow, 1200, 1200, 0));
        assert!(!trough_is_brake(
            DecayMode::Slow,
            drive_ticks(i16::MAX, 1200),
            1200,
            0
        ));
    }

    #[test]
    fn drive_ticks_pins_chip_mapping() {
        assert_eq!(drive_ticks(0, 1200), 0);
        assert_eq!(drive_ticks(i16::MAX, 1200), 1200);
        assert_eq!(drive_ticks(-i16::MAX, 1200), 1200);
        assert_eq!(drive_ticks(i16::MIN, 1200), 1200);
    }

    #[test]
    fn drive_ticks_monotonic() {
        let mut last = 0;
        for duty in (0..=i16::MAX).step_by(317) {
            let t = drive_ticks(duty, 1200);
            assert!(t >= last, "non-monotonic at duty={duty}: {last} -> {t}");
            last = t;
        }
    }

    fn recip(arr: u16) -> u32 {
        (1 << RECIP_ARR_SHIFT) / arr as u32
    }

    fn is_floor(d: u16, floor: u16, arr: u16) -> bool {
        let valid = |d: u16| select(DecayMode::Slow, drive_ticks(d as i16, arr), floor, 0).i_valid;
        valid(d) && (d == 0 || !valid(d - 1))
    }

    #[test]
    fn floor_duty_is_window_valid() {
        for floor in [100, 160, 240] {
            let d = floor_duty(floor, 1200, recip(1200));
            assert!(drive_ticks(d as i16, 1200) >= floor as u32);
            assert!(is_floor(d, floor, 1200), "floor {floor}: duty {d}");
        }
        assert_eq!(floor_duty(160, 1200, recip(1200)), 4356);
    }

    #[test]
    fn floor_duty_is_the_smallest_valid_duty() {
        for floor in 0..=1200 {
            let d = floor_duty(floor, 1200, recip(1200));
            assert!(is_floor(d, floor, 1200), "floor {floor}: duty {d}");
        }
        for arr in [600, 1000, 1199, 2400, 4800] {
            for floor in 0..=512.min(arr) {
                let d = floor_duty(floor, arr, recip(arr));
                assert!(is_floor(d, floor, arr), "arr {arr} floor {floor}: duty {d}");
            }
        }
    }

    #[test]
    fn floor_duty_saturates_past_full_duty() {
        assert_eq!(floor_duty(1201, 1200, recip(1200)), i16::MAX as u16);
        assert_eq!(floor_duty(u16::MAX, 1200, recip(1200)), i16::MAX as u16);
    }
}
