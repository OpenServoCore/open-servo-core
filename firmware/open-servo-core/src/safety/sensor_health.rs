//! Sensor health tracking for fault detection.

use open_servo_math::CentiDeg;

/// Tracks sensor health for fault detection.
///
/// Strategy: When a bad reading is detected (position delta exceeds threshold),
/// skip this tick and increment counter. If counter exceeds threshold,
/// trigger a hard fault. Good readings reset the counter.
///
/// This allows the system to tolerate occasional noise or glitches
/// while still detecting persistent sensor failures.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorHealth {
    /// Last known good position reading
    last_good_position: CentiDeg,

    /// Consecutive bad position readings
    bad_position_count: u8,

    /// Whether sensor health has been initialized (first reading accepted)
    initialized: bool,
}

impl Default for SensorHealth {
    fn default() -> Self {
        Self {
            last_good_position: CentiDeg::from_cdeg(0),
            bad_position_count: 0,
            initialized: false,
        }
    }
}

impl SensorHealth {
    /// Create a new SensorHealth instance.
    pub fn new() -> Self {
        Self::default()
    }

    /// Validate a position reading against the previous good reading.
    ///
    /// Returns:
    /// - `Ok(position)` if the reading is valid (use this value)
    /// - `Err(count)` if the reading is invalid (count = consecutive bad reads)
    ///
    /// On a bad reading, the counter is incremented but the last good position
    /// is preserved. The caller should check if count >= threshold to trigger
    /// a fault.
    #[inline]
    pub fn validate_position(
        &mut self,
        reading: CentiDeg,
        max_delta: CentiDeg,
    ) -> Result<CentiDeg, u8> {
        if !self.initialized {
            // First reading - accept it and initialize
            self.last_good_position = reading;
            self.initialized = true;
            self.bad_position_count = 0;
            return Ok(reading);
        }

        // Check if delta is reasonable
        let delta = (reading.as_cdeg() - self.last_good_position.as_cdeg()).abs();

        if delta <= max_delta.as_cdeg() {
            // Good reading - update last good and reset counter
            self.last_good_position = reading;
            self.bad_position_count = 0;
            Ok(reading)
        } else {
            // Bad reading - increment counter
            self.bad_position_count = self.bad_position_count.saturating_add(1);
            Err(self.bad_position_count)
        }
    }

    /// Reset sensor health state.
    ///
    /// Called after fault clear to re-initialize on next reading.
    pub fn reset(&mut self) {
        self.bad_position_count = 0;
        self.initialized = false;
    }

    /// Get the last known good position.
    #[inline]
    pub fn last_good_position(&self) -> CentiDeg {
        self.last_good_position
    }

    /// Get the current bad reading count.
    #[inline]
    pub fn bad_count(&self) -> u8 {
        self.bad_position_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MAX_DELTA: CentiDeg = CentiDeg(500); // 5° max change per tick

    #[test]
    fn test_first_reading_always_valid() {
        let mut health = SensorHealth::new();
        let result = health.validate_position(CentiDeg::from_cdeg(9000), MAX_DELTA);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().as_cdeg(), 9000);
        assert_eq!(health.bad_count(), 0);
    }

    #[test]
    fn test_small_delta_accepted() {
        let mut health = SensorHealth::new();
        // Initialize
        health.validate_position(CentiDeg::from_cdeg(9000), MAX_DELTA).unwrap();

        // Small change (100 cdeg = 1°) should be accepted
        let result = health.validate_position(CentiDeg::from_cdeg(9100), MAX_DELTA);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().as_cdeg(), 9100);
        assert_eq!(health.bad_count(), 0);
    }

    #[test]
    fn test_large_delta_rejected() {
        let mut health = SensorHealth::new();
        // Initialize at 90°
        health.validate_position(CentiDeg::from_cdeg(9000), MAX_DELTA).unwrap();

        // Large jump (1000 cdeg = 10°) should be rejected
        let result = health.validate_position(CentiDeg::from_cdeg(10000), MAX_DELTA);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), 1);
        assert_eq!(health.bad_count(), 1);

        // Another bad reading increments counter
        let result = health.validate_position(CentiDeg::from_cdeg(10000), MAX_DELTA);
        assert!(result.is_err());
        assert_eq!(result.unwrap_err(), 2);
        assert_eq!(health.bad_count(), 2);
    }

    #[test]
    fn test_good_reading_resets_counter() {
        let mut health = SensorHealth::new();
        health.validate_position(CentiDeg::from_cdeg(9000), MAX_DELTA).unwrap();

        // Cause some bad readings
        health.validate_position(CentiDeg::from_cdeg(15000), MAX_DELTA).unwrap_err();
        health.validate_position(CentiDeg::from_cdeg(15000), MAX_DELTA).unwrap_err();
        assert_eq!(health.bad_count(), 2);

        // Good reading (back near last good position) should reset counter
        let result = health.validate_position(CentiDeg::from_cdeg(9100), MAX_DELTA);
        assert!(result.is_ok());
        assert_eq!(health.bad_count(), 0);
    }

    #[test]
    fn test_reset_clears_state() {
        let mut health = SensorHealth::new();
        health.validate_position(CentiDeg::from_cdeg(9000), MAX_DELTA).unwrap();
        health.validate_position(CentiDeg::from_cdeg(15000), MAX_DELTA).unwrap_err();

        health.reset();

        // After reset, next reading should be accepted as first reading
        assert_eq!(health.bad_count(), 0);
        let result = health.validate_position(CentiDeg::from_cdeg(15000), MAX_DELTA);
        assert!(result.is_ok()); // First reading always accepted
    }

    #[test]
    fn test_last_good_position_preserved_on_bad_reading() {
        let mut health = SensorHealth::new();
        health.validate_position(CentiDeg::from_cdeg(9000), MAX_DELTA).unwrap();

        // Bad reading should not update last_good_position
        health.validate_position(CentiDeg::from_cdeg(15000), MAX_DELTA).unwrap_err();
        assert_eq!(health.last_good_position().as_cdeg(), 9000);
    }
}
