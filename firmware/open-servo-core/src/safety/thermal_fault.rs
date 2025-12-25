//! Thermal fault detection logic.
//!
//! Monitors temperature from the thermal model and triggers faults
//! when limits are exceeded. Implements hysteresis to prevent oscillation.

use crate::fault::FaultKind;

/// Thermal fault detector with hysteresis.
///
/// Monitors temperature and triggers fault when exceeded.
/// Separate from the thermal model to ensure physics state
/// cannot be accidentally reset.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ThermalFaultDetector {
    /// Maximum allowed temperature in centi-degrees (10000 = 100°C)
    max_temp_cdeg: i16,
    
    /// Temperature for fault recovery in centi-degrees
    /// Provides hysteresis to prevent oscillation
    recovery_temp_cdeg: i16,
    
    /// Track if currently in thermal fault state
    is_faulted: bool,
}

impl ThermalFaultDetector {
    /// Create detector with specified limits.
    ///
    /// # Arguments
    /// * `max_temp_cdeg` - Maximum temperature before fault (centi-degrees)
    /// * `hysteresis_cdeg` - Temperature drop required for recovery (centi-degrees)
    pub fn new(max_temp_cdeg: i16, hysteresis_cdeg: i16) -> Self {
        Self {
            max_temp_cdeg,
            recovery_temp_cdeg: max_temp_cdeg - hysteresis_cdeg,
            is_faulted: false,
        }
    }
    
    /// Check if temperature exceeds safe limits.
    ///
    /// Implements hysteresis:
    /// - Fault triggers when temp > max_temp
    /// - Fault clears when temp < recovery_temp
    ///
    /// # Arguments
    /// * `temp_cdeg` - Current temperature in centi-degrees
    ///
    /// # Returns
    /// Some(FaultKind::MotorOverTemp) if faulted, None otherwise
    pub fn check_fault(&mut self, temp_cdeg: i16) -> Option<FaultKind> {
        if !self.is_faulted {
            // Not faulted - check if we should fault
            if temp_cdeg > self.max_temp_cdeg {
                self.is_faulted = true;
                return Some(FaultKind::MotorOverTemp);
            }
        } else {
            // Already faulted - check if we can recover
            if temp_cdeg < self.recovery_temp_cdeg {
                self.is_faulted = false;
                return None;
            }
            // Still too hot
            return Some(FaultKind::MotorOverTemp);
        }
        None
    }
    
    /// Try to reset fault state.
    ///
    /// Only succeeds if temperature has cooled below recovery threshold.
    ///
    /// # Arguments
    /// * `temp_cdeg` - Current temperature in centi-degrees
    ///
    /// # Returns
    /// true if reset was successful, false if still too hot
    pub fn try_reset(&mut self, temp_cdeg: i16) -> bool {
        if temp_cdeg < self.recovery_temp_cdeg {
            self.is_faulted = false;
            true
        } else {
            false
        }
    }
    
    /// Check if currently faulted.
    pub fn is_faulted(&self) -> bool {
        self.is_faulted
    }
    
    /// Get the maximum temperature limit.
    pub fn max_temp_cdeg(&self) -> i16 {
        self.max_temp_cdeg
    }
    
    /// Get the recovery temperature threshold.
    pub fn recovery_temp_cdeg(&self) -> i16 {
        self.recovery_temp_cdeg
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_fault_trigger() {
        let mut detector = ThermalFaultDetector::new(10000, 1000);
        
        // Below limit - no fault
        assert_eq!(detector.check_fault(9900), None);
        assert!(!detector.is_faulted());
        
        // Above limit - fault triggered
        assert_eq!(detector.check_fault(10100), Some(FaultKind::MotorOverTemp));
        assert!(detector.is_faulted());
    }
    
    #[test]
    fn test_hysteresis() {
        let mut detector = ThermalFaultDetector::new(10000, 1000);
        
        // Trigger fault
        detector.check_fault(10100); // 101°C
        assert!(detector.is_faulted());
        
        // Cool to 95°C - still faulted (above 90°C recovery)
        assert_eq!(detector.check_fault(9500), Some(FaultKind::MotorOverTemp));
        assert!(detector.is_faulted());
        
        // Cool to 89°C - fault clears
        assert_eq!(detector.check_fault(8900), None);
        assert!(!detector.is_faulted());
        
        // Heat to 99°C - no fault yet (below 100°C threshold)
        assert_eq!(detector.check_fault(9900), None);
        assert!(!detector.is_faulted());
    }
    
    #[test]
    fn test_try_reset() {
        let mut detector = ThermalFaultDetector::new(10000, 1000);
        
        // Trigger fault
        detector.check_fault(10100);
        assert!(detector.is_faulted());
        
        // Try reset while hot - fails
        assert!(!detector.try_reset(9500)); // 95°C
        assert!(detector.is_faulted());
        
        // Try reset when cool enough - succeeds
        assert!(detector.try_reset(8900)); // 89°C
        assert!(!detector.is_faulted());
    }
}