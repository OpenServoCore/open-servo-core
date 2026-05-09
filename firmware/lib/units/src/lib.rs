//! OSC native engineering units.

#![no_std]

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Microrads(pub i32);

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct MilliradsPerSec(pub i32);

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Milliamps(pub i16);

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Millivolts(pub i16);

/// 0.1 °C; e.g. 250 = 25.0 °C.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct CentiCelsius(pub i16);

/// Dimensionless normalized actuator command. Negative = reverse, positive = forward.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Effort(pub i16);

impl Effort {
    pub const ZERO: Self = Self(0);
    pub const FULL: Self = Self(i16::MAX);
    pub const MIN: Self = Self(-i16::MAX);
}
