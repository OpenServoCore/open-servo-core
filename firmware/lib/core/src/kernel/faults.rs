//! Fault latch + detectors (spec "Fault raise points"). Any set bit forces
//! MotorCmd::Disabled regardless of torque_enable; the torque_enable 0->1
//! edge is the only ack - it clears the mask and the detector state, and a
//! still-present condition re-latches through the normal detectors.

pub const BIT_OVER_CURRENT: u8 = 1 << 0;
pub const BIT_OVER_TEMP: u8 = 1 << 1;
pub const BIT_STALL: u8 = 1 << 2;
pub const BIT_POSITION_ERROR: u8 = 1 << 3;
pub const BIT_SENSOR: u8 = 1 << 4;
pub const BIT_UNDER_VOLT: u8 = 1 << 5;

/// `fault_code` values: bit index + 1, 0 = no fault since the last ack.
pub const CODE_NONE: u8 = 0;
pub const CODE_OVER_CURRENT: u8 = 1;
pub const CODE_OVER_TEMP: u8 = 2;
pub const CODE_STALL: u8 = 3;
pub const CODE_POSITION_ERROR: u8 = 4;
pub const CODE_SENSOR: u8 = 5;
pub const CODE_UNDER_VOLT: u8 = 6;

/// Latched mask + the LATEST newly-latched kind (`fault_code`). Re-raising
/// an already-set bit does not touch the code, so a persisting first fault
/// cannot mask a later second one.
pub struct FaultLatch {
    mask: u8,
    last_code: u8,
}

impl FaultLatch {
    pub const fn new() -> Self {
        Self {
            mask: 0,
            last_code: CODE_NONE,
        }
    }

    pub fn raise(&mut self, bit: u8, code: u8) {
        if self.mask & bit == 0 {
            self.mask |= bit;
            self.last_code = code;
        }
    }

    /// Ack (torque_enable 0->1): mask and code both clear.
    pub fn clear(&mut self) {
        self.mask = 0;
        self.last_code = CODE_NONE;
    }

    pub fn mask(&self) -> u8 {
        self.mask
    }

    pub fn code(&self) -> u8 {
        self.last_code
    }
}

impl Default for FaultLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Detector counters, kept together so `kernel/mod.rs` stays orchestration:
/// OC consecutive-sample count (FAST), position-error persistence timer
/// (MEDIUM), sensor-delta screen with its last raw sample (MEDIUM).
pub struct Detectors {
    oc_ticks: u8,
    pos_err_ticks: u32,
    sensor_bad: u8,
    sensor_last_pos: u16,
    sensor_seen: bool,
}

impl Detectors {
    pub const fn new() -> Self {
        Self {
            oc_ticks: 0,
            pos_err_ticks: 0,
            sensor_bad: 0,
            sensor_last_pos: 0,
            sensor_seen: false,
        }
    }

    /// Ack: every counter re-arms; the sensor screen re-primes off the next
    /// sample instead of comparing across the disabled gap.
    pub fn reset(&mut self) {
        self.oc_ticks = 0;
        self.pos_err_ticks = 0;
        self.sensor_bad = 0;
        self.sensor_seen = false;
    }

    /// FAST: `over` = Some(|i_meas| > oc_trip_counts) for a VALID window,
    /// None when the window is invalid. Invalid samples hold the count - a
    /// masked window must not launder a live overcurrent - and a valid
    /// below-trip sample re-arms. Returns true when the window fills.
    pub fn oc_sample(&mut self, over: Option<bool>, trip_ticks: u8) -> bool {
        match over {
            Some(true) => {
                self.oc_ticks = self.oc_ticks.saturating_add(1);
                self.oc_ticks >= trip_ticks
            }
            Some(false) => {
                self.oc_ticks = 0;
                false
            }
            None => false,
        }
    }

    /// MEDIUM: raw pot delta screen; `bad_count` consecutive jumps trip.
    pub fn sensor_sample(&mut self, pos: u16, delta_max: u16, bad_count: u8) -> bool {
        let delta = (pos as i32 - self.sensor_last_pos as i32).unsigned_abs();
        let jump = self.sensor_seen && delta > delta_max as u32;
        self.sensor_last_pos = pos;
        self.sensor_seen = true;
        if jump {
            self.sensor_bad = self.sensor_bad.saturating_add(1);
            self.sensor_bad >= bad_count
        } else {
            self.sensor_bad = 0;
            false
        }
    }

    /// MEDIUM: tracking-error persistence; trips after `trip_ticks`
    /// consecutive over-threshold medium ticks.
    pub fn pos_err_sample(&mut self, over: bool, trip_ticks: u32) -> bool {
        if over {
            self.pos_err_ticks = self.pos_err_ticks.saturating_add(1);
            self.pos_err_ticks >= trip_ticks
        } else {
            self.pos_err_ticks = 0;
            false
        }
    }
}

impl Default for Detectors {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn latch_latest_code_first_bit_wins_nothing() {
        let mut l = FaultLatch::new();
        assert_eq!((l.mask(), l.code()), (0, CODE_NONE));
        l.raise(BIT_OVER_CURRENT, CODE_OVER_CURRENT);
        assert_eq!((l.mask(), l.code()), (BIT_OVER_CURRENT, CODE_OVER_CURRENT));
        // re-raise of a set bit: code untouched
        l.raise(BIT_OVER_CURRENT, CODE_OVER_CURRENT);
        l.raise(BIT_STALL, CODE_STALL);
        assert_eq!(l.mask(), BIT_OVER_CURRENT | BIT_STALL);
        assert_eq!(l.code(), CODE_STALL);
        // persisting first fault does not clobber the latest latch
        l.raise(BIT_OVER_CURRENT, CODE_OVER_CURRENT);
        assert_eq!(l.code(), CODE_STALL);
        l.clear();
        assert_eq!((l.mask(), l.code()), (0, CODE_NONE));
    }

    #[test]
    fn oc_consecutive_valid_only() {
        let mut d = Detectors::new();
        for _ in 0..3 {
            assert!(!d.oc_sample(Some(true), 4));
        }
        // invalid holds the count, does not reset
        assert!(!d.oc_sample(None, 4));
        assert!(d.oc_sample(Some(true), 4));
        // valid below-trip re-arms
        let mut d = Detectors::new();
        for _ in 0..3 {
            d.oc_sample(Some(true), 4);
        }
        assert!(!d.oc_sample(Some(false), 4));
        for _ in 0..3 {
            assert!(!d.oc_sample(Some(true), 4));
        }
        assert!(d.oc_sample(Some(true), 4));
    }

    #[test]
    fn sensor_screen_primes_and_counts() {
        let mut d = Detectors::new();
        // first sample only primes, no matter how large
        assert!(!d.sensor_sample(4000, 10, 2));
        assert!(!d.sensor_sample(4020, 10, 2));
        assert!(d.sensor_sample(4000, 10, 2));
        // clean delta re-arms
        let mut d = Detectors::new();
        d.sensor_sample(100, 10, 2);
        assert!(!d.sensor_sample(200, 10, 2));
        assert!(!d.sensor_sample(205, 10, 2));
        assert!(!d.sensor_sample(300, 10, 2));
        assert!(d.sensor_sample(400, 10, 2));
        // reset re-primes
        d.reset();
        assert!(!d.sensor_sample(0, 10, 2));
    }

    #[test]
    fn pos_err_timer() {
        let mut d = Detectors::new();
        for _ in 0..9 {
            assert!(!d.pos_err_sample(true, 10));
        }
        assert!(!d.pos_err_sample(false, 10));
        for _ in 0..9 {
            assert!(!d.pos_err_sample(true, 10));
        }
        assert!(d.pos_err_sample(true, 10));
    }
}
