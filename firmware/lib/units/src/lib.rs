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

/// 0.01 degC; e.g. 2500 = 25.0 degC.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct CentiCelsius(pub i16);

/// Normalized actuator command. Range is symmetric (`-i16::MAX..=i16::MAX`) so `-x` can't overflow.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
#[repr(transparent)]
pub struct Effort(pub i16);

impl Effort {
    pub const ZERO: Self = Self(0);
    pub const FULL: Self = Self(i16::MAX);
    pub const MIN: Self = Self(-i16::MAX);
}
