use core::ops::{Add, Sub};

#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct SimTime(u64);

impl SimTime {
    pub const ZERO: Self = Self(0);

    pub const fn from_ns(ns: u64) -> Self {
        Self(ns)
    }

    pub const fn from_us(us: u64) -> Self {
        Self(us * 1_000)
    }

    pub const fn from_ms(ms: u64) -> Self {
        Self(ms * 1_000_000)
    }

    pub const fn as_ns(self) -> u64 {
        self.0
    }
}

impl Add<u64> for SimTime {
    type Output = Self;
    fn add(self, ns: u64) -> Self {
        Self(self.0 + ns)
    }
}

impl Sub for SimTime {
    type Output = u64;
    fn sub(self, other: Self) -> u64 {
        self.0 - other.0
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct ClockRatio {
    pub num: u32,
    pub den: u32,
}

impl ClockRatio {
    pub const IDENTITY: Self = Self { num: 1, den: 1 };

    pub const fn new(num: u32, den: u32) -> Self {
        Self { num, den }
    }

    pub fn to_local(self, t: SimTime) -> u64 {
        (t.as_ns() as u128 * self.num as u128 / self.den as u128) as u64
    }

    pub fn from_local(self, local_ns: u64) -> SimTime {
        SimTime((local_ns as u128 * self.den as u128 / self.num as u128) as u64)
    }
}

impl Default for ClockRatio {
    fn default() -> Self {
        Self::IDENTITY
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn from_units_compose() {
        assert_eq!(SimTime::from_us(1).as_ns(), 1_000);
        assert_eq!(SimTime::from_ms(1).as_ns(), 1_000_000);
        assert_eq!(SimTime::from_ms(2) - SimTime::from_us(500), 1_500_000);
    }

    #[test]
    fn identity_ratio_round_trips() {
        let t = SimTime::from_us(123);
        assert_eq!(ClockRatio::IDENTITY.to_local(t), t.as_ns());
        assert_eq!(ClockRatio::IDENTITY.from_local(t.as_ns()), t);
    }

    #[test]
    fn drift_ratio_scales_local_clock() {
        let r = ClockRatio::new(98, 100);
        let t = SimTime::from_ms(1);
        let local = r.to_local(t);
        assert_eq!(local, 980_000);
        assert_eq!(r.from_local(local), t);
    }
}
