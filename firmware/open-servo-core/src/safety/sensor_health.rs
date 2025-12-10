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
