//! STAT lamp policy: the servo's health at a glance. Bus traffic has its
//! own passive lamp on the `DATA` line, so STAT shows state, never traffic.
//! In precedence order:
//!
//! - lit for the lamp test at boot,
//! - solid while a fault is latched (`fault_code` nonzero until the
//!   torque ack clears it),
//! - a ~1 Hz blink while `data_flags` names any reason closed loop is
//!   refused (not calibrated, plant unset, stamp mismatch),
//! - dark when healthy.

use osc_servo_core::kernel::faults::CODE_NONE;
use osc_servo_drivers::led::Pattern;
use osc_servo_drivers::traits::Monotonic as _;

use crate::providers::monotonic::Monotonic;

const LAMP_TEST_TICKS: u32 = 150_000 * Monotonic::TICKS_PER_US;
const SETUP_BLINK: Pattern = Pattern::Blink {
    period_us: 1_000_000,
};

pub fn pattern(lamp_test: bool, fault_code: u8, data_flags: u8) -> Pattern {
    if lamp_test || fault_code != CODE_NONE {
        Pattern::SolidOn
    } else if data_flags != 0 {
        SETUP_BLINK
    } else {
        Pattern::SolidOff
    }
}

/// The boot lamp test, timed off the main loop's clock. Once over it stays
/// over: the tick counter wraps, so a bare elapsed-time compare would
/// re-run the test every lap.
pub struct LampTest {
    from: Option<u32>,
}

impl LampTest {
    pub fn new(now: u32) -> Self {
        Self { from: Some(now) }
    }

    pub fn running(&mut self, now: u32) -> bool {
        let running = self
            .from
            .is_some_and(|from| now.wrapping_sub(from) < LAMP_TEST_TICKS);
        if !running {
            self.from = None;
        }
        running
    }
}

#[cfg(test)]
mod tests {
    use osc_servo_core::data_state::{CALIB_VIRGIN, PLANT_UNSET};
    use osc_servo_core::kernel::faults::{CODE_DATA, CODE_STALL};

    use super::*;

    #[test]
    fn healthy_is_dark() {
        assert!(pattern(false, CODE_NONE, 0) == Pattern::SolidOff);
    }

    #[test]
    fn a_latched_fault_is_solid_and_wins_over_setup() {
        assert!(pattern(false, CODE_STALL, 0) == Pattern::SolidOn);
        assert!(pattern(false, CODE_DATA, PLANT_UNSET) == Pattern::SolidOn);
    }

    #[test]
    fn any_setup_reason_blinks_at_1_hz() {
        for flags in [PLANT_UNSET, CALIB_VIRGIN, PLANT_UNSET | CALIB_VIRGIN] {
            assert!(
                pattern(false, CODE_NONE, flags)
                    == Pattern::Blink {
                        period_us: 1_000_000
                    }
            );
        }
    }

    #[test]
    fn the_lamp_test_lights_a_healthy_servo() {
        assert!(pattern(true, CODE_NONE, 0) == Pattern::SolidOn);
    }

    #[test]
    fn the_lamp_test_ends_and_stays_over_across_a_tick_wrap() {
        let start = u32::MAX - 10;
        let mut test = LampTest::new(start);
        assert!(test.running(start));
        assert!(test.running(start.wrapping_add(LAMP_TEST_TICKS - 1)));
        assert!(!test.running(start.wrapping_add(LAMP_TEST_TICKS)));
        assert!(!test.running(start));
        assert!(!test.running(start.wrapping_add(1)));
    }
}
