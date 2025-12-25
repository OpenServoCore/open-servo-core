//! Kinematics and limit configuration for servo mechanics.

use open_servo_math::CentiDeg32;

/// Describes whether the angle measurement domain is bounded or wrapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AngleDomain {
    /// Angle is bounded between min and max (e.g., potentiometer)
    Bounded,
    /// Angle wraps around (e.g., magnetic encoder 0-360°)
    Wrapping,
}

/// Motor rotation direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// Normal direction
    Normal,
    /// Reversed direction
    Reversed,
}

/// Raw sensor limits in ADC or encoder counts.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SensorLimits {
    /// Minimum raw sensor value
    pub raw_min: u16,
    /// Maximum raw sensor value  
    pub raw_max: u16,
}

impl SensorLimits {
    /// Create new sensor limits with explicit values.
    pub fn new(raw_min: u16, raw_max: u16) -> Self {
        Self { raw_min, raw_max }
    }
}


/// Physical/mechanical limits of the servo mechanism.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MechanicalLimits {
    /// Minimum physical position in centidegrees
    pub min_cdeg: CentiDeg32,
    /// Maximum physical position in centidegrees
    pub max_cdeg: CentiDeg32,
}

impl MechanicalLimits {
    /// Create new mechanical limits with explicit values.
    pub fn new(min_cdeg: i32, max_cdeg: i32) -> Self {
        Self {
            min_cdeg: CentiDeg32::from_cdeg32(min_cdeg),
            max_cdeg: CentiDeg32::from_cdeg32(max_cdeg),
        }
    }
}


/// User-configurable soft limits (must be within mechanical limits).
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UserLimits {
    /// Minimum user-configured position in centidegrees
    pub min_cdeg: CentiDeg32,
    /// Maximum user-configured position in centidegrees
    pub max_cdeg: CentiDeg32,
}

impl UserLimits {
    /// Create new user limits with explicit values.
    pub fn new(min_cdeg: i32, max_cdeg: i32) -> Self {
        Self {
            min_cdeg: CentiDeg32::from_cdeg32(min_cdeg),
            max_cdeg: CentiDeg32::from_cdeg32(max_cdeg),
        }
    }
}


/// Servo kinematics configuration.
///
/// Describes the mechanical characteristics and constraints of the servo.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Kinematics {
    /// Whether angle is bounded or wrapping
    pub angle_domain: AngleDomain,
    /// Physical mechanism limits
    pub mechanical_limits: MechanicalLimits,
    /// Calibration zero offset in centidegrees
    pub zero_offset: CentiDeg32,
    /// Motor rotation direction
    pub direction: Direction,
}

impl Kinematics {
    /// Create new kinematics with explicit values.
    pub fn new(
        angle_domain: AngleDomain,
        mechanical_limits: MechanicalLimits,
        zero_offset: i32,
        direction: Direction,
    ) -> Self {
        Self {
            angle_domain,
            mechanical_limits,
            zero_offset: CentiDeg32::from_cdeg32(zero_offset),
            direction,
        }
    }
}

impl Kinematics {
    /// Apply direction and zero offset to a raw angle.
    pub fn apply_calibration(&self, angle: CentiDeg32) -> CentiDeg32 {
        let mut result = angle;
        
        // Apply direction
        if self.direction == Direction::Reversed {
            result = -result;
        }
        
        // Apply zero offset
        result = result + self.zero_offset;
        
        result
    }
    
    /// Check if angle is within mechanical limits.
    pub fn is_within_mechanical_limits(&self, angle: CentiDeg32) -> bool {
        angle >= self.mechanical_limits.min_cdeg && angle <= self.mechanical_limits.max_cdeg
    }
    
    /// Clamp angle to mechanical limits.
    pub fn clamp_to_mechanical(&self, angle: CentiDeg32) -> CentiDeg32 {
        if angle < self.mechanical_limits.min_cdeg {
            self.mechanical_limits.min_cdeg
        } else if angle > self.mechanical_limits.max_cdeg {
            self.mechanical_limits.max_cdeg
        } else {
            angle
        }
    }
}

/// Complete three-layer limit system.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct LimitSystem {
    /// Raw sensor limits
    pub sensor: SensorLimits,
    /// Mechanical limits
    pub mechanical: MechanicalLimits,
    /// User-configured limits
    pub user: UserLimits,
}

impl LimitSystem {
    /// Create new limit system with explicit values.
    pub fn new(sensor: SensorLimits, mechanical: MechanicalLimits, user: UserLimits) -> Self {
        Self {
            sensor,
            mechanical,
            user,
        }
    }
    
    /// Validate that user limits are within mechanical limits.
    pub fn validate(&self) -> bool {
        self.user.min_cdeg >= self.mechanical.min_cdeg &&
        self.user.max_cdeg <= self.mechanical.max_cdeg &&
        self.user.min_cdeg < self.user.max_cdeg
    }
    
    /// Clamp angle to user limits.
    pub fn clamp_to_user(&self, angle: CentiDeg32) -> CentiDeg32 {
        if angle < self.user.min_cdeg {
            self.user.min_cdeg
        } else if angle > self.user.max_cdeg {
            self.user.max_cdeg
        } else {
            angle
        }
    }
}