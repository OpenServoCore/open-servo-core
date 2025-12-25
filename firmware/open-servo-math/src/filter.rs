//! Low-pass filters for servo sensor data
//!
//! Exponential moving average filters using fixed-point math,
//! optimized for servo control system values.

/// Low-pass filter for u16 ADC values (position, current, voltage sensors)
#[derive(Clone, Debug)]
pub struct FilterU16 {
    /// Right shift for alpha (alpha = 1 / 2^shift)
    alpha_shift: u8,
    /// Current filtered value in shifted representation
    /// None = uninitialized, Some = value << alpha_shift
    state: Option<u32>,
}

impl FilterU16 {
    /// Create filter with alpha = 1/2^shift
    /// - shift=0: no filtering (alpha=1.0)
    /// - shift=3: default (alpha=0.125)
    /// - shift=5: heavy filtering (alpha=0.03125)
    pub const fn new(alpha_shift: u8) -> Self {
        Self {
            alpha_shift,
            state: None,
        }
    }

    /// Get the current alpha shift value
    pub fn alpha_shift(&self) -> u8 {
        self.alpha_shift
    }

    /// Update filter with new sample, returns filtered value
    pub fn update(&mut self, sample: u16) -> u16 {
        let sample_u32 = sample as u32;

        match self.state {
            None => {
                // Initialize on first sample
                self.state = Some(sample_u32 << self.alpha_shift);
                sample
            }
            Some(state) => {
                // EMA: y = y_prev + alpha * (x - y_prev)
                let sample_shifted = sample_u32 << self.alpha_shift;
                let diff = sample_shifted.wrapping_sub(state);
                let divisor = 1u32 << self.alpha_shift;
                let delta = diff / divisor;
                let new_state = state.wrapping_add(delta);
                self.state = Some(new_state);
                (new_state >> self.alpha_shift) as u16
            }
        }
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.state = None;
    }

    /// Get current filtered value without updating
    pub fn value(&self) -> Option<u16> {
        self.state.map(|s| (s >> self.alpha_shift) as u16)
    }
}

/// Low-pass filter for i16 values (CentiDeg, MilliAmp, etc.)
#[derive(Clone, Debug)]
pub struct FilterI16 {
    /// Right shift for alpha (alpha = 1 / 2^shift)
    alpha_shift: u8,
    /// Current filtered value in shifted representation
    state: Option<i32>,
}

impl FilterI16 {
    /// Create filter with alpha = 1/2^shift
    pub const fn new(alpha_shift: u8) -> Self {
        Self {
            alpha_shift,
            state: None,
        }
    }

    /// Get the current alpha shift value
    pub fn alpha_shift(&self) -> u8 {
        self.alpha_shift
    }

    /// Update filter with new sample, returns filtered value
    pub fn update(&mut self, sample: i16) -> i16 {
        let sample_i32 = sample as i32;

        match self.state {
            None => {
                self.state = Some(sample_i32 << self.alpha_shift);
                sample
            }
            Some(state) => {
                let sample_shifted = sample_i32 << self.alpha_shift;
                let diff = sample_shifted - state;
                let divisor = 1i32 << self.alpha_shift;
                let delta = diff / divisor;
                let new_state = state + delta;
                self.state = Some(new_state);
                (new_state >> self.alpha_shift) as i16
            }
        }
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.state = None;
    }

    /// Get current filtered value without updating
    pub fn value(&self) -> Option<i16> {
        self.state.map(|s| (s >> self.alpha_shift) as i16)
    }
}

/// Low-pass filter for i32 values
/// Used for filtering position values in the control loop
#[derive(Clone, Debug)]
pub struct FilterI32 {
    /// Right shift for alpha (alpha = 1 / 2^shift)
    alpha_shift: u8,
    /// Current filtered value in shifted representation
    state: Option<i64>,
}

impl FilterI32 {
    /// Create filter with alpha = 1/2^shift
    pub const fn new(alpha_shift: u8) -> Self {
        Self {
            alpha_shift,
            state: None,
        }
    }

    /// Get the current alpha shift value
    pub fn alpha_shift(&self) -> u8 {
        self.alpha_shift
    }

    /// Update filter with new sample, returns filtered value
    pub fn update(&mut self, sample: i32) -> i32 {
        let sample_i64 = sample as i64;

        match self.state {
            None => {
                self.state = Some(sample_i64 << self.alpha_shift);
                sample
            }
            Some(state) => {
                let sample_shifted = sample_i64 << self.alpha_shift;
                let diff = sample_shifted - state;
                let divisor = 1i64 << self.alpha_shift;
                let delta = diff / divisor;
                let new_state = state + delta;
                self.state = Some(new_state);
                (new_state >> self.alpha_shift) as i32
            }
        }
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.state = None;
    }

    /// Get current filtered value without updating
    pub fn value(&self) -> Option<i32> {
        self.state.map(|s| (s >> self.alpha_shift) as i32)
    }
}

/// Recommended filter strengths for different sensors
pub mod presets {
    /// Position sensor (potentiometer): moderate filtering
    pub const POSITION_FILTER_SHIFT: u8 = 3; // alpha = 0.125

    /// Current sensor: light filtering (fast response needed)
    pub const CURRENT_FILTER_SHIFT: u8 = 2; // alpha = 0.25

    /// Temperature sensor: heavy filtering (slow changing)
    pub const TEMP_FILTER_SHIFT: u8 = 5; // alpha = 0.03125

    /// Voltage sensor: moderate filtering
    pub const VOLTAGE_FILTER_SHIFT: u8 = 3; // alpha = 0.125
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_u16_filter() {
        let mut filter = FilterU16::new(2); // alpha = 0.25

        // First sample initializes
        assert_eq!(filter.update(1000), 1000);

        // Step change should be filtered
        let result = filter.update(2000);
        assert!(result > 1000 && result < 2000);

        // Should converge towards new value
        for _ in 0..10 {
            filter.update(2000);
        }
        assert!(filter.value().unwrap() > 1900);
    }

    #[test]
    fn test_i16_filter() {
        let mut filter = FilterI16::new(2);

        // Works with negative values
        assert_eq!(filter.update(-1000), -1000);
        let result = filter.update(-2000);
        assert!(result < -1000 && result > -2000);
    }

    #[test]
    fn test_i32_filter() {
        let mut filter = FilterI32::new(2);

        // First sample initializes
        assert_eq!(filter.update(10000), 10000);

        // Step change should be filtered
        let result = filter.update(20000);
        assert!(result > 10000 && result < 20000);

        // Test with large values
        filter.reset();
        assert_eq!(filter.update(100000), 100000);
        let result = filter.update(200000);
        assert!(result > 100000 && result < 200000);
    }
}
